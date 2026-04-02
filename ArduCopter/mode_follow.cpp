#include "Copter.h"

#if MODE_FOLLOW_ENABLED

#ifndef FOLLOW_DEBUG_HZ
#define FOLLOW_DEBUG_HZ 1
#endif

/*
 * mode_follow.cpp - follow another mavlink-enabled vehicle by system id
 *
 * Minimal 4.6.x backport test:
 *  - AP_Follow provides offset-applied target position in NED meters
 *  - AC_PosControl expects NEU centimeters
 *  - feed position only; zero velocity/acceleration feed-forward
 *  - MUST call g2.follow.update_estimates() each loop
 */

// Return true if this mode is enabled, used by MAVLink available modes
bool ModeFollow::enabled() const
{
    return g2.follow.enabled();
}

static bool s_none_yaw_initialized = false;
static float s_none_yaw_target_rad = 0.0f;

// initialise follow mode
bool ModeFollow::init(const bool ignore_checks)
{
    if (!enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }

	s_none_yaw_initialized = false;
	s_none_yaw_target_rad = 0.0f;

    g2.follow.update_estimates();

#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP_Mount::get_singleton();
    if (g2.follow.option_is_enabled(AP_Follow::Option::MOUNT_FOLLOW_ON_ENTER) && mount != nullptr) {
        mount->set_target_sysid(g2.follow.get_target_sysid());
    }
#endif

    const float xy_speed = wp_nav->get_default_speed_xy();
    const float xy_accel = wp_nav->get_wp_acceleration();

    pos_control->set_max_speed_accel_xy(xy_speed, xy_accel);
    pos_control->set_correction_speed_accel_xy(xy_speed, xy_accel);

    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                       wp_nav->get_default_speed_up(),
                                       wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                              wp_nav->get_default_speed_up(),
                                              wp_nav->get_accel_z());

    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    auto_yaw.set_mode_to_default(false);
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // Let AP_Follow establish its normal target behavior every time.
    g2.follow.init_offsets_if_required();

    return true;
}

// perform cleanup required when leaving follow mode
void ModeFollow::exit()
{
    g2.follow.clear_offsets_if_required();
	s_none_yaw_initialized = false;
	s_none_yaw_target_rad = 0.0f;
}

void ModeFollow::run()
{
    const bool armed = motors->armed();

    // disarmed: safe ground handling only
    if (!armed) {
        s_none_yaw_initialized = false;
        s_none_yaw_target_rad = 0.0f;
        make_safe_ground_handling();
        return;
    }

    // stay out while landed
    if (copter.ap.land_complete) {
        s_none_yaw_initialized = false;
        s_none_yaw_target_rad = 0.0f;
        make_safe_ground_handling();
        return;
    }

    // update AP_Follow estimates each loop
    g2.follow.update_estimates();

    // ensure offsets are initialized if needed
    g2.follow.init_offsets_if_required();

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    float yaw_rad = ahrs.get_yaw();
    float yaw_rate_rads = 0.0f;

    // Offset-applied target state
    Vector3p pos_ofs_ned_p;
    Vector3f vel_ofs_ned_ms;
    Vector3f accel_ofs_ned_mss;
    const bool have_target = g2.follow.get_ofs_pos_vel_accel_NED_m(pos_ofs_ned_p,
                                                                   vel_ofs_ned_ms,
                                                                   accel_ofs_ned_mss);

    // Raw lead target state (translation only)
    Vector3f lead_pos_ned_m;
    Vector3f lead_vel_ned_ms;
    Vector3f lead_accel_ned_mss;
    const bool have_lead_target = g2.follow.get_target_pos_vel_accel_NED_m(lead_pos_ned_m,
                                                                           lead_vel_ned_ms,
                                                                           lead_accel_ned_mss);

    if (have_target) {
        float target_heading_deg = 0.0f;
        float target_heading_rate_degs = 0.0f;
        g2.follow.get_target_heading_deg(target_heading_deg);
        g2.follow.get_target_heading_rate_degs(target_heading_rate_degs);

        const Vector3f pos_ofs_ned_m = pos_ofs_ned_p.tofloat();

        // Use the offset-applied position target,
        // but only the lead's translational velocity/acceleration feed-forward.
        Vector3f use_vel_ned_ms{};
        Vector3f use_accel_ned_mss{};

        if (have_lead_target) {
            use_vel_ned_ms = lead_vel_ned_ms;
            use_accel_ned_mss = lead_accel_ned_mss;
        }

        // Convert NED meters to NEU centimeters for 4.6 pos_control API
        Vector2p pos_ne_cm;
        pos_ne_cm.x = (postype_t)(pos_ofs_ned_m.x * 100.0f);
        pos_ne_cm.y = (postype_t)(pos_ofs_ned_m.y * 100.0f);

        Vector2f vel_ne_cms;
        vel_ne_cms.x = use_vel_ned_ms.x * 100.0f;
        vel_ne_cms.y = use_vel_ned_ms.y * 100.0f;

        const Vector2f accel_ne_cmss(use_accel_ned_mss.x * 100.0f,
                                     use_accel_ned_mss.y * 100.0f);

        float pos_up_cm = (-pos_ofs_ned_m.z) * 100.0f;
        float vel_up_cms = (-use_vel_ned_ms.z) * 100.0f;
        const float accel_up_cmss = (-use_accel_ned_mss.z) * 100.0f;

        pos_control->input_pos_vel_accel_xy(pos_ne_cm, vel_ne_cms, accel_ne_cmss, false);
        pos_control->input_pos_vel_accel_z(pos_up_cm, vel_up_cms, accel_up_cmss, false);

        switch (g2.follow.get_yaw_behave()) {

        case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
            s_none_yaw_initialized = false;

            if (have_lead_target) {
                Vector3f my_pos_ned_m;
                if (AP::ahrs().get_relative_position_NED_origin(my_pos_ned_m)) {
                    const Vector2f to_lead_ne = (lead_pos_ned_m - my_pos_ned_m).xy();
                    if (to_lead_ne.length_squared() > 0.25f) {
                        yaw_rad = to_lead_ne.angle();
                    }
                }
            }
            break;
        }

        case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE:
            s_none_yaw_initialized = false;
            yaw_rad = radians(target_heading_deg);
            yaw_rate_rads = radians(target_heading_rate_degs);
            break;

        case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT:
            s_none_yaw_initialized = false;
            if (use_vel_ned_ms.xy().length_squared() > 0.25f) {
                yaw_rad = use_vel_ned_ms.xy().angle();
            }
            break;

        case AP_Follow::YAW_BEHAVE_NONE:
        default: {
            // Latch a yaw target on entry and only move it from pilot stick input.
            if (!s_none_yaw_initialized) {
                s_none_yaw_target_rad = ahrs.get_yaw();
                s_none_yaw_initialized = true;
            }

            const float pilot_yaw_rate_cds = get_pilot_desired_yaw_rate();
            yaw_rate_rads = radians(pilot_yaw_rate_cds * 0.01f);

            // Update the held yaw target only from pilot input.
            s_none_yaw_target_rad = wrap_PI(s_none_yaw_target_rad + yaw_rate_rads * G_Dt);
            yaw_rad = s_none_yaw_target_rad;
            break;
        }
        }

    } else {
        s_none_yaw_initialized = false;
        s_none_yaw_target_rad = 0.0f;

        Vector2f vel_ne_zero{};
        const Vector2f accel_ne_zero{};
        pos_control->input_vel_accel_xy(vel_ne_zero, accel_ne_zero, false);

        float vel_up_zero = 0.0f;
        float accel_up_zero = 0.0f;
        pos_control->input_vel_accel_z(vel_up_zero, accel_up_zero, false);

        yaw_rate_rads = 0.0f;
    }

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

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