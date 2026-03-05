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

    // Initialize follow offset if not yet set.
    // Prevents vehicle from starting directly on top of the lead vehicle.
    g2.follow.init_offsets_if_required();

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    float yaw_rad = attitude_control->get_att_target_euler_rad().z;
    float yaw_rate_rads = 0.0f;

    Vector3p pos_ofs_ned;         // position of lead + offset, in NED frame (from AP_Follow)
    Vector3f vel_ofs_ned;         // velocity of lead + offset, NED
    Vector3f accel_ofs_ned;       // accel of lead + offset, NED
    const bool have_target = g2.follow.get_ofs_pos_vel_accel_NED_m(pos_ofs_ned, vel_ofs_ned, accel_ofs_ned);

    if (have_target) {

        float target_heading_deg = 0.0f;
        float target_heading_rate_degs = 0.0f;
        g2.follow.get_target_heading_deg(target_heading_deg);
        g2.follow.get_target_heading_rate_degs(target_heading_rate_degs);

        // XY: NED north/east matches what Copter expects for horizontal (north/east)
        pos_control->input_pos_vel_accel_xy(pos_ofs_ned.xy(), vel_ofs_ned.xy(), accel_ofs_ned.xy(), false);

        // Z: AP_Follow provides NED (+Down). Copter position controller Z is +Up.
        // Convert NED -> Up by negating position/velocity/accel Z.
        float pos_ofs_up = -float(pos_ofs_ned.z);
        float vel_ofs_up = -vel_ofs_ned.z;
        const float accel_ofs_up = -accel_ofs_ned.z;

        pos_control->input_pos_vel_accel_z(pos_ofs_up, vel_ofs_up, accel_ofs_up, false);

        if (do_dbg) {
            const auto pos_tgt_cm = pos_control->get_pos_target_cm();   // internal pos target in cm
            const uint8_t yaw_behave = (uint8_t)g2.follow.get_yaw_behave();

            gcs().send_text(
                MAV_SEVERITY_INFO,
                "FOLL ok yawB=%u pos_ofs(NED)=%.2f %.2f %.2f vel(NED)=%.2f %.2f %.2f acc(NED)=%.2f %.2f %.2f",
                (unsigned)yaw_behave,
                (double)pos_ofs_ned.x, (double)pos_ofs_ned.y, (double)pos_ofs_ned.z,
                (double)vel_ofs_ned.x, (double)vel_ofs_ned.y, (double)vel_ofs_ned.z,
                (double)accel_ofs_ned.x, (double)accel_ofs_ned.y, (double)accel_ofs_ned.z
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
                "FOLL zconv pos_up=%.2f vel_up=%.2f acc_up=%.2f",
                (double)pos_ofs_up,
                (double)vel_ofs_up,
                (double)accel_ofs_up
            );
        }

        // Determine desired yaw behavior based on configured follow mode
        switch (g2.follow.get_yaw_behave()) {

            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                // Face the target directly
                Vector3p pos_ned;       // lead position (NED)
                Vector3f vel_ned;
                Vector3f accel_ned;
                if (g2.follow.get_target_pos_vel_accel_NED_m(pos_ned, vel_ned, accel_ned)) {
                    if (pos_ned.xy().length_squared() > 1.0) {
                        yaw_rad = (pos_ned.xy() - pos_control->get_pos_target_cm().xy() / 100.0).tofloat().angle();
                    }
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                yaw_rad = radians(target_heading_deg);
                yaw_rate_rads = radians(target_heading_rate_degs);
                break;
            }

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                if (vel_ofs_ned.xy().length_squared() > 1.0) {
                    yaw_rad = vel_ofs_ned.xy().angle();
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_NONE:
            default:
                // do nothing
                break;
        }

    } else {

        // Target data is invalid; hold using zero velocity and acceleration inputs
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