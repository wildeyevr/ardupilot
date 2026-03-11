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
 *  - MUST call g2.follow.update_estimates() each loop (you already discovered this!)
 */

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
    return true;
}

// perform cleanup required when leaving follow mode
void ModeFollow::exit()
{
    g2.follow.clear_offsets_if_required();
}

void ModeFollow::run()
{
    // debug rate limit
    static uint32_t last_dbg_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();
    const bool do_dbg = (now_ms - last_dbg_ms) >= (1000U / FOLLOW_DEBUG_HZ);
    if (do_dbg) {
        last_dbg_ms = now_ms;
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // IMPORTANT: update AP_Follow internal estimate every loop
    g2.follow.update_estimates();

    // Initialize follow offset if not yet set (prevents starting on top of lead)
    g2.follow.init_offsets_if_required();

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    float yaw_rad = attitude_control->get_att_target_euler_rad().z;
    float yaw_rate_rads = 0.0f;

    // --- Stationary deadband state ---
    static bool hold_active = false;
    static Vector3f held_pos_ofs_ned_m{};     // meters, NED
    //static uint32_t hold_enter_ms = 0;

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

                // ------------------------------------------------------------
        // Stationary deadband / "sticky target" to prevent GPS wander chase
        // ------------------------------------------------------------
        const float STATIONARY_VEL_MS = 0.1f;   // lead considered stopped below this
        const float DEADBAND_M        = 0.75f;   // don't chase target jitter within this radius
        const float REACQUIRE_M       = 1.25f;   // hysteresis radius to drop hold (>= DEADBAND_M)

        const float lead_speed = vel_ofs_ned_ms.xy().length(); // m/s (use lead+ofs vel)
        const bool lead_stationary = (lead_speed < STATIONARY_VEL_MS);

        if (!hold_active) {
            // Enter hold when lead stationary
            if (lead_stationary) {
                hold_active = true;
                //hold_enter_ms = now_ms;
                held_pos_ofs_ned_m = pos_ofs_ned_m;
            }
        } else {
            // Exit hold when lead moving again
            if (!lead_stationary) {
                hold_active = false;
            } else {
                // If target moves significantly, update held target (or optionally exit hold)
                const float err_m = (pos_ofs_ned_m.xy() - held_pos_ofs_ned_m.xy()).length();

                if (err_m > REACQUIRE_M) {
                    // big jump: snap hold point to new target
                    held_pos_ofs_ned_m = pos_ofs_ned_m;
                } else if (err_m > DEADBAND_M) {
                    // moderate drift: slowly walk the held point toward it (optional smoothing)
                    // If you want hard deadband, comment this block and keep held_pos fixed.
                    const float alpha = 0.1f; // 0..1, small = very "balloon"
                    held_pos_ofs_ned_m.xy() = held_pos_ofs_ned_m.xy() + (pos_ofs_ned_m.xy() - held_pos_ofs_ned_m.xy()) * alpha;
                    held_pos_ofs_ned_m.z = pos_ofs_ned_m.z; // keep Z following normally (or also deadband if desired)
                }
            }
        }

        // If holding, override the target fed into pos_control
        Vector3f use_pos_ofs_ned_m = pos_ofs_ned_m;
        Vector3f use_vel_ofs_ned_ms = vel_ofs_ned_ms;
        Vector3f use_accel_ofs_ned_mss = accel_ofs_ned_mss;

        if (hold_active) {
            use_pos_ofs_ned_m = held_pos_ofs_ned_m;
            use_vel_ofs_ned_ms.zero();
            use_accel_ofs_ned_mss.zero();
        }

        // XY pos/vel/accel in NE, cm-based
        Vector2p pos_ne_cm;
        pos_ne_cm.x = (postype_t)(use_pos_ofs_ned_m.x * 100.0f);
        pos_ne_cm.y = (postype_t)(use_pos_ofs_ned_m.y * 100.0f);

        Vector2f vel_ne_cms;
        vel_ne_cms.x = use_vel_ofs_ned_ms.x * 100.0f;
        vel_ne_cms.y = use_vel_ofs_ned_ms.y * 100.0f;

        const Vector2f accel_ne_cmss(use_accel_ofs_ned_mss.x * 100.0f,
                                     use_accel_ofs_ned_mss.y * 100.0f);

        // Z pos/vel/accel in Up, cm-based
        float pos_up_cm   = (-use_pos_ofs_ned_m.z) * 100.0f;
        float vel_up_cms  = (-use_vel_ofs_ned_ms.z) * 100.0f;
        const float accel_up_cmss = (-use_accel_ofs_ned_mss.z) * 100.0f;

        // Feed into AC_PosControl
        pos_control->input_pos_vel_accel_xy(pos_ne_cm, vel_ne_cms, accel_ne_cmss, false);
        pos_control->input_pos_vel_accel_z(pos_up_cm, vel_up_cms, accel_up_cmss, false);

        if (do_dbg) {
            const auto pos_tgt_cm = pos_control->get_pos_target_cm();   // NEU cm
            const uint8_t yaw_behave = (uint8_t)g2.follow.get_yaw_behave();

            gcs().send_text(MAV_SEVERITY_INFO,
                "FOLL hold=%u leadSpd=%.2f err=%.2f",
                (unsigned)hold_active,
                (double)lead_speed,
                (double)(pos_ofs_ned_m.xy() - held_pos_ofs_ned_m.xy()).length());

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
        }

        if (do_dbg) {
            bool ok_hdg = g2.follow.get_target_heading_deg(target_heading_deg);
            bool ok_hr  = g2.follow.get_target_heading_rate_degs(target_heading_rate_degs);

            gcs().send_text(MAV_SEVERITY_INFO,
                            "FOLL yaw dbg: yawBeh=%u okHdg=%u hdg=%.1f okRate=%u hdgRate=%.2f",
                            (unsigned)g2.follow.get_yaw_behave(),
                            (unsigned)ok_hdg, (double)target_heading_deg,
                            (unsigned)ok_hr,  (double)target_heading_rate_degs);

            gcs().send_text(MAV_SEVERITY_INFO,
                            "FOLL yaw dbg: curYaw=%.1fdeg",
                            (double)degrees(ahrs.get_yaw()));
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
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE:
                yaw_rad = radians(target_heading_deg);
                yaw_rate_rads = radians(target_heading_rate_degs);
                break;

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT:
                if (vel_ofs_ned_ms.xy().length_squared() > 0.25f) { // ~0.5m/s
                    yaw_rad = vel_ofs_ned_ms.xy().angle();
                }
                break;

            case AP_Follow::YAW_BEHAVE_NONE:
            default:
            // Allow pilot yaw while following:
            // - keep yaw angle at current target (yaw_rad already set above)
            // - apply pilot yaw RATE as the commanded yaw_rate_rads

            const float pilot_yaw_rate_cds = get_pilot_desired_yaw_rate();   // cd/s
            yaw_rate_rads = radians(pilot_yaw_rate_cds * 0.01f);                 // -> rad/s
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

        yaw_rate_rads = 0.0f;

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

    // call attitude controller (4.6 API expects centi-deg)
    const float yaw_cd = degrees(yaw_rad) * 100.0f;
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