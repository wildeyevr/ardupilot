#include "Copter.h"

#if MODE_FOLLOW_ENABLED

#ifndef FOLLOW_DEBUG_HZ
#define FOLLOW_DEBUG_HZ 1
#endif

/*
 * mode_follow.cpp - follow another mavlink-enabled vehicle by system id
 *
 * Notes for 4.6.x backport:
 *  - AP_Follow provides target state in NED meters (pos), m/s (vel), m/s^2 (accel)
 *  - AC_PosControl expects NEU centimeters (pos), cm/s (vel), cm/s/s (accel)
 *  - AC_PosControl input_*_xy() takes pos and vel by NON-const reference (it may modify them)
 *  - MUST call g2.follow.update_estimates() each loop
 *
 * Current demo-oriented tuning:
 *  - position-only follow input (vel/accel feedforward disabled)
 *  - manual pilot yaw in YAW_BEHAVE_NONE with deadband + heading latch
 */

// manual yaw state for YAW_BEHAVE_NONE
static float follow_manual_yaw_target_rad = 0.0f;
static bool  follow_manual_yaw_init = false;
static bool  follow_manual_yaw_was_active = false;

// Return true if this mode is enabled, used by MAVLink available modes
bool ModeFollow::enabled() const
{
    return g2.follow.enabled();
}

// initialise follow mode
bool ModeFollow::init(const bool ignore_checks)
{
    if (!enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }

#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP_Mount::get_singleton();
    // follow the lead vehicle using sysid
    if (g2.follow.option_is_enabled(AP_Follow::Option::MOUNT_FOLLOW_ON_ENTER) && mount != nullptr) {
        mount->set_target_sysid(g2.follow.get_target_sysid());
    }
#endif

    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                       wp_nav->get_default_speed_up(),
                                       wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                              wp_nav->get_default_speed_up(),
                                              wp_nav->get_accel_z());

    // initialise controller state
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
    // Force yaw control to accept external yaw targets in Follow
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // initialise manual yaw hold target
    follow_manual_yaw_target_rad = ahrs.get_yaw();
    follow_manual_yaw_init = true;
    follow_manual_yaw_was_active = false;

    return true;
}

// perform cleanup required when leaving follow mode
void ModeFollow::exit()
{
    g2.follow.clear_offsets_if_required();
    follow_manual_yaw_init = false;
    follow_manual_yaw_was_active = false;
}

void ModeFollow::run()
{
    // debug rate limit
    static uint32_t last_dbg_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();
#if FOLLOW_DEBUG_HZ > 0
    const bool do_dbg = (now_ms - last_dbg_ms) >= (1000U / FOLLOW_DEBUG_HZ);
#else
    const bool do_dbg = false;
#endif
    if (do_dbg) {
        last_dbg_ms = now_ms;
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // Initialize follow offset if not yet set.
    // Prevents vehicle from starting directly on top of the lead vehicle.
    g2.follow.init_offsets_if_required();

    // IMPORTANT for 4.6.x backport: update AP_Follow internal estimate every loop
    // Do this AFTER offsets are initialized so the estimate uses the correct relative offset
    g2.follow.update_estimates();

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    float yaw_rad = attitude_control->get_att_target_euler_rad().z;
    float yaw_rate_rads = 0.0f;

    // safe dt for manual yaw integration
    float dt = pos_control->get_dt();
    if (!is_positive(dt) || dt > 0.1f) {
        dt = 0.01f;
    }

    // pilot yaw input
    const float pilot_yaw_rate_cds_raw = get_pilot_desired_yaw_rate();
    float pilot_yaw_rate_rads = radians(pilot_yaw_rate_cds_raw * 0.01f);

    // deadband to prevent slow residual yaw creep after stick release
    if (fabsf(pilot_yaw_rate_rads) < radians(5.0f)) {
        pilot_yaw_rate_rads = 0.0f;
    }
    const bool pilot_yaw_active = fabsf(pilot_yaw_rate_rads) > 0.0f;

    if (!follow_manual_yaw_init) {
        follow_manual_yaw_target_rad = ahrs.get_yaw();
        follow_manual_yaw_init = true;
    }

    // --- Get target in NED (meters, m/s, m/s^2) from AP_Follow ---
    Vector3p pos_ofs_ned_p;     // position of lead + offset (NED, meters)
    Vector3f vel_ofs_ned_ms;    // velocity (NED, m/s)
    Vector3f accel_ofs_ned_mss; // accel (NED, m/s^2)

    const bool have_target = g2.follow.get_ofs_pos_vel_accel_NED_m(pos_ofs_ned_p,
                                                                   vel_ofs_ned_ms,
                                                                   accel_ofs_ned_mss);

    if (have_target) {

        float target_heading_deg = 0.0f;
        float target_heading_rate_degs = 0.0f;
        g2.follow.get_target_heading_deg(target_heading_deg);
        g2.follow.get_target_heading_rate_degs(target_heading_rate_degs);

        // --------------------------------------------------------------------
        // Convert NED meters -> NEU centimeters
        //
        // NED (m):  N, E, D
        // NEU (cm): N*100, E*100, U*100 where U = -D
        // --------------------------------------------------------------------

        const Vector3f pos_ofs_ned_m = pos_ofs_ned_p.tofloat();

        // XY pos/vel/accel in NE, cm-based
        Vector2p pos_ne_cm;            // NON-const lvalue (AC_PosControl may modify)
        pos_ne_cm.x = (postype_t)(pos_ofs_ned_m.x * 100.0f);
        pos_ne_cm.y = (postype_t)(pos_ofs_ned_m.y * 100.0f);

        Vector2f vel_ne_cms;           // NON-const lvalue (AC_PosControl may modify)
        vel_ne_cms.x = vel_ofs_ned_ms.x * 100.0f;
        vel_ne_cms.y = vel_ofs_ned_ms.y * 100.0f;
        //vel_ne_cms.x = 0.0f;
        //vel_ne_cms.y = 0.0f;

        //const Vector2f accel_ne_cmss(accel_ofs_ned_mss.x * 100.0f, accel_ofs_ned_mss.y * 100.0f);
        const Vector2f accel_ne_cmss(0.0f, 0.0f);

        // Z pos/vel/accel in Up, cm-based
        float pos_up_cm   = (-pos_ofs_ned_m.z) * 100.0f;       // NON-const lvalue

        float vel_up_cms  = (-vel_ofs_ned_ms.z) * 100.0f;     // NON-const lvalue
        //float vel_up_cms  = 0.0f;

        const float accel_up_cmss = (-accel_ofs_ned_mss.z) * 100.0f;
        //const float accel_up_cmss = 0.0f;

        // Feed into AC_PosControl
        pos_control->input_pos_vel_accel_xy(pos_ne_cm, vel_ne_cms, accel_ne_cmss, false);
        pos_control->input_pos_vel_accel_z(pos_up_cm, vel_up_cms, accel_up_cmss, false);

        if (do_dbg) {
            const auto pos_tgt_cm = pos_control->get_pos_target_cm();   // NEU cm
            const uint8_t yaw_behave = (uint8_t)g2.follow.get_yaw_behave();

            gcs().send_text(
                MAV_SEVERITY_INFO,
                "FOLL ok yawB=%u pos_ofs(NED)m=%.2f %.2f %.2f vel(NED)m/s=%.2f %.2f %.2f acc(NED)m/s2=%.2f %.2f %.2f",
                (unsigned)yaw_behave,
                (double)pos_ofs_ned_m.x, (double)pos_ofs_ned_m.y, (double)pos_ofs_ned_m.z,
                (double)vel_ofs_ned_ms.x, (double)vel_ofs_ned_ms.y, (double)vel_ofs_ned_ms.z,
                (double)accel_ofs_ned_mss.x, (double)accel_ofs_ned_mss.y, (double)accel_ofs_ned_mss.z
            );

            gcs().send_text(
                MAV_SEVERITY_INFO,
                "FOLL in(NEU cm)=pos(%.1f %.1f %.1f) vel(%.1f %.1f %.1f) acc(%.1f %.1f %.1f)",
                (double)pos_ne_cm.x, (double)pos_ne_cm.y, (double)pos_up_cm,
                (double)vel_ne_cms.x, (double)vel_ne_cms.y, (double)vel_up_cms,
                (double)accel_ne_cmss.x, (double)accel_ne_cmss.y, (double)accel_up_cmss
            );

            gcs().send_text(
                MAV_SEVERITY_INFO,
                "FOLL pos_tgt(m)=%.2f %.2f %.2f hdg=%.1f hdgRate=%.2f",
                (double)pos_tgt_cm.x * 0.01,
                (double)pos_tgt_cm.y * 0.01,
                (double)pos_tgt_cm.z * 0.01,
                (double)target_heading_deg,
                (double)target_heading_rate_degs
            );

            gcs().send_text(
                MAV_SEVERITY_INFO,
                "FOLL yaw dbg: curYaw=%.1fdeg pilotYawRate=%.2fdeg/s holdYaw=%.1fdeg active=%u",
                (double)degrees(ahrs.get_yaw()),
                (double)degrees(pilot_yaw_rate_rads),
                (double)degrees(follow_manual_yaw_target_rad),
                (unsigned)pilot_yaw_active
            );
        }

        // Yaw behavior
        switch (g2.follow.get_yaw_behave()) {

            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                // Face the lead vehicle position (NOT the offset position)
                Vector3f pos_lead_ned_p;
                Vector3f vel_lead_ned_ms;
                Vector3f accel_lead_ned_mss;

                if (g2.follow.get_target_pos_vel_accel_NED_m(pos_lead_ned_p, vel_lead_ned_ms, accel_lead_ned_mss)) {
                    Vector3f my_pos_ned_m;
                    if (AP::ahrs().get_relative_position_NED_origin(my_pos_ned_m)) {
                        const Vector3f lead_ned_m = pos_lead_ned_p.tofloat();
                        const Vector2f to_lead_ne = (lead_ned_m - my_pos_ned_m).xy();
                        if (to_lead_ne.length_squared() > 0.25f) { // ~0.5m
                            yaw_rad = to_lead_ne.angle();
                        }
                    }
                }
                yaw_rate_rads = 0.0f;
                follow_manual_yaw_was_active = false;
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE:
                yaw_rad = radians(target_heading_deg);
                yaw_rate_rads = 0.0f;
                follow_manual_yaw_was_active = false;
                break;

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT:
                if (vel_ofs_ned_ms.xy().length_squared() > 0.25f) { // ~0.5m/s
                    yaw_rad = vel_ofs_ned_ms.xy().angle();
                }
                yaw_rate_rads = 0.0f;
                follow_manual_yaw_was_active = false;
                break;

            case AP_Follow::YAW_BEHAVE_NONE:
            default:
                if (pilot_yaw_active) {
                    if (!follow_manual_yaw_was_active) {
                        follow_manual_yaw_target_rad = ahrs.get_yaw();
                        follow_manual_yaw_was_active = true;
                    }
                    follow_manual_yaw_target_rad = wrap_PI(follow_manual_yaw_target_rad + pilot_yaw_rate_rads * dt);
                    yaw_rad = follow_manual_yaw_target_rad;
                    yaw_rate_rads = pilot_yaw_rate_rads;
                } else {
                    follow_manual_yaw_was_active = false;
                    yaw_rad = follow_manual_yaw_target_rad;
                    yaw_rate_rads = 0.0f;
                }
                break;
        }

    } else {

        // Target data invalid; hold using zero velocity and acceleration inputs
        Vector2f vel_ne_zero{};
        const Vector2f accel_ne_zero{};
        pos_control->input_vel_accel_xy(vel_ne_zero, accel_ne_zero, false);

        float vel_up_zero = 0.0f;
        float accel_up_zero = 0.0f;
        pos_control->input_vel_accel_z(vel_up_zero, accel_up_zero, false);

        // still allow manual yaw hold behavior if no target
        if (pilot_yaw_active) {
            if (!follow_manual_yaw_was_active) {
                follow_manual_yaw_target_rad = ahrs.get_yaw();
                follow_manual_yaw_was_active = true;
            }
            follow_manual_yaw_target_rad = wrap_PI(follow_manual_yaw_target_rad + pilot_yaw_rate_rads * dt);
            yaw_rad = follow_manual_yaw_target_rad;
            yaw_rate_rads = pilot_yaw_rate_rads;
        } else {
            follow_manual_yaw_was_active = false;
            yaw_rad = follow_manual_yaw_target_rad;
            yaw_rate_rads = 0.0f;
        }

        if (do_dbg) {
            gcs().send_text(MAV_SEVERITY_WARNING, "FOLL no target -> hold (zero vel/accel)");
        }
    }

    // update the position controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    if (do_dbg) {
        const Vector3f thrust = pos_control->get_thrust_vector();
        gcs().send_text(
            MAV_SEVERITY_INFO,
            "FOLL thrust=%.2f %.2f %.2f yaw=%.1fdeg yawr=%.2fdeg/s",
            (double)thrust.x, (double)thrust.y, (double)thrust.z,
            (double)degrees(yaw_rad),
            (double)degrees(yaw_rate_rads)
        );
    }

    // call attitude controller
    // 4.6.x input_thrust_vector_heading expects centi-deg and centi-deg/s
    const float yaw_cd       = degrees(yaw_rad) * 100.0f;
    const float yaw_rate_cds = degrees(yaw_rate_rads) * 100.0f;

    attitude_control->input_thrust_vector_heading(
        pos_control->get_thrust_vector(),
        yaw_cd,
        yaw_rate_cds
    );
}

float ModeFollow::wp_distance_m() const
{
    return g2.follow.get_distance_to_target_m();
}

float ModeFollow::wp_bearing_deg() const
{
    return g2.follow.get_bearing_to_target_deg();
}

// Returns target location with offset applied, for MAVLink reporting
bool ModeFollow::get_wp(Location &loc) const
{
    Vector3f vel_ned_ms;
    return g2.follow.get_target_location_and_velocity_ofs(loc, vel_ned_ms);
}

#endif // MODE_FOLLOW_ENABLED