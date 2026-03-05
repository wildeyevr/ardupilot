#include "Copter.h"

#if MODE_FOLLOW_ENABLED

#ifndef FOLLOW_DEBUG_HZ
#define FOLLOW_DEBUG_HZ 2
#endif

/*
 * mode_follow.cpp - follow another mavlink-enabled vehicle by system id
 *
 * TODO: stick control to move around on sphere
 * TODO: stick control to change sphere diameter
 * TODO: "channel 7 option" to lock onto "pointed at" target
 * TODO: do better in terms of loitering around the moving point; may need a PID?  Maybe use loiter controller somehow?
 * TODO: extrapolate target vehicle position using its velocity and acceleration
 * TODO: ensure AP_AVOIDANCE_ENABLED is true because we rely on it velocity limiting functions
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
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise velocity controller
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    // initialise yaw
    auto_yaw.set_mode_to_default(false);

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

    g2.follow.update_estimates();

    // Initialize follow offset if not yet set.
    // Prevents vehicle from starting directly on top of the lead vehicle.
    g2.follow.init_offsets_if_required();

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    float yaw_rad = attitude_control->get_att_target_euler_rad().z;
    float yaw_rate_rads = 0.0f;

    Vector3p pos_ofs_ned_m;        // NED position in meters
    Vector3f vel_ofs_ned_ms;       // NED velocity in m/s
    Vector3f accel_ofs_ned_mss;    // NED accel in m/s^2

    const bool have_target = g2.follow.get_ofs_pos_vel_accel_NED_m(pos_ofs_ned_m, vel_ofs_ned_ms, accel_ofs_ned_mss);

    if (have_target) {

        float target_heading_deg = 0.0f;
        float target_heading_rate_degs = 0.0f;
        g2.follow.get_target_heading_deg(target_heading_deg);
        g2.follow.get_target_heading_rate_degs(target_heading_rate_degs);

        // Convert NED meters -> NEU centimeters for AC_PosControl
        Vector2p pos_ne_cm;
        pos_ne_cm.x = (postype_t)pos_ofs_ned_m.x * 100.0;
        pos_ne_cm.y = (postype_t)pos_ofs_ned_m.y * 100.0;

        Vector2f vel_ne_cms;
        vel_ne_cms.x = vel_ofs_ned_ms.x * 100.0f;
        vel_ne_cms.y = vel_ofs_ned_ms.y * 100.0f;

        Vector2f accel_ne_cmss;
        accel_ne_cmss.x = accel_ofs_ned_mss.x * 100.0f;
        accel_ne_cmss.y = accel_ofs_ned_mss.y * 100.0f;

        pos_control->input_pos_vel_accel_xy(pos_ne_cm, vel_ne_cms, accel_ne_cmss, false);

        // Z: NED down (m) -> NEU up (cm)
        float pos_up_cm   = -(float)pos_ofs_ned_m.z * 100.0f;
        float vel_up_cms  = -vel_ofs_ned_ms.z * 100.0f;
        float accel_up_cmss = -accel_ofs_ned_mss.z * 100.0f;

        pos_control->input_pos_vel_accel_z(pos_up_cm, vel_up_cms, accel_up_cmss, false);

        if (do_dbg) {
            const auto pos_tgt_cm = pos_control->get_pos_target_cm(); // NEU cm
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
        }

        // yaw behavior (unchanged)
        switch (g2.follow.get_yaw_behave()) {

            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                Vector3p pos_ned_m;
                Vector3f vel_ned_ms;
                Vector3f accel_ned_mss;
                if (g2.follow.get_target_pos_vel_accel_NED_m(pos_ned_m, vel_ned_ms, accel_ned_mss))
                if (pos_ned_m.xy().length_squared() > 1.0f) {
                    yaw_rad = (pos_ned_m.xy() - pos_control->get_pos_target_cm().xy() / 100.0f).tofloat().angle();
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                yaw_rad = radians(target_heading_deg);
                yaw_rate_rads = radians(target_heading_rate_degs);
                break;
            }

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                if (vel_ofs_ned_ms.xy().length_squared() > 1.0f) {
                    yaw_rad = vel_ofs_ned_ms.xy().angle();
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_NONE:
            default:
                break;
        }

    } else {

        // Target data is invalid; hold using zero velocity and acceleration inputs
        Vector2f vel_ne_zero{};
        Vector2f accel_ne_zero{};
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

    // call attitude controller
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), yaw_rad, yaw_rate_rads);
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