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
 * Launch overlay behavior:
 *  - If entering Follow while landed, capture the current deck/home offset:
 *        deck_home_offset = follower_pos - lead_pos
 *  - During climb, use:
 *        XY = captured deck/home XY
 *        Z  = normal Follow Z (FOLL_OFS_Z)
 *  - Once relative altitude reaches the normal Follow Z target, blend XY from
 *    deck/home XY to the normal Follow XY.
 *  - After the blend, revert to ordinary Follow behavior completely.
 *
 * This keeps the original working Follow Z behavior intact and only modifies
 * launch/recovery XY geometry.
 */

// -----------------------------------------------------------------------------
// Quick-test launch config (move to params later if this works well)
// -----------------------------------------------------------------------------
static constexpr bool  FOLLOW_LAUNCH_ENABLE   = true;
static constexpr float FOLLOW_LAUNCH_BLEND_S  = 3.0f;   // seconds to blend XY to mission offset
static constexpr float FOLLOW_Z_REACHED_TOL_M = 1.0f;   // consider climb complete within this Z tolerance

enum class FollowLaunchState : uint8_t {
    INACTIVE = 0,
    CLIMB,
    BLEND_TO_FOLLOW,
    COMPLETE
};

// File-scope state for quick testing.
// If this works well, move these into ModeFollow members later.
static FollowLaunchState s_follow_launch_state = FollowLaunchState::INACTIVE;
static uint32_t s_follow_blend_start_ms = 0;

// Captured deck/home point relative to lead reference
static Vector3f s_follow_deck_home_ofs_ned_m{};

// Blend endpoints
static Vector3f s_follow_blend_from_ofs_ned_m{};
static Vector3f s_follow_blend_to_ofs_ned_m{};

// Sticky hold state
static bool s_hold_active = false;
static Vector3f s_held_pos_ofs_ned_m{};

static bool s_launch_takeoff_started = false;
static bool s_prev_armed = false;

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
    pos_control->set_correction_speed_accel_xy(xy_speed, xy_accel * 1.5f);

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

    s_hold_active = false;
    s_held_pos_ofs_ned_m.zero();

    s_follow_blend_start_ms = 0;
    s_follow_deck_home_ofs_ned_m.zero();
    s_follow_blend_from_ofs_ned_m.zero();
    s_follow_blend_to_ofs_ned_m.zero();
    s_launch_takeoff_started = false;
    s_prev_armed = motors->armed();

    // Let AP_Follow establish its normal target behavior every time.
    g2.follow.init_offsets_if_required();

    // If we enter Follow while landed, pre-arm capture deck/home XY.
    // We'll also re-capture on actual arm edge in run(), which is the more important one.
    if (FOLLOW_LAUNCH_ENABLE && copter.ap.land_complete) {
        Vector3f lead_pos_ned_m;
        Vector3f lead_vel_ned_ms;
        Vector3f lead_accel_ned_mss;
        Vector3f my_pos_ned_m;

        const bool have_lead_target =
            g2.follow.get_target_pos_vel_accel_NED_m(lead_pos_ned_m,
                                                     lead_vel_ned_ms,
                                                     lead_accel_ned_mss);
        const bool have_my_pos =
            AP::ahrs().get_relative_position_NED_origin(my_pos_ned_m);

        if (have_lead_target && have_my_pos) {
            s_follow_deck_home_ofs_ned_m = my_pos_ned_m - lead_pos_ned_m;
            s_follow_launch_state = FollowLaunchState::CLIMB;
        } else {
            s_follow_launch_state = FollowLaunchState::COMPLETE;
        }
    } else {
        s_follow_launch_state = FollowLaunchState::COMPLETE;
    }

    return true;
}

// perform cleanup required when leaving follow mode
void ModeFollow::exit()
{
    g2.follow.clear_offsets_if_required();

    s_follow_launch_state = FollowLaunchState::INACTIVE;
    s_follow_blend_start_ms = 0;
    s_follow_deck_home_ofs_ned_m.zero();
    s_follow_blend_from_ofs_ned_m.zero();
    s_follow_blend_to_ofs_ned_m.zero();

    s_hold_active = false;
    s_held_pos_ofs_ned_m.zero();

    s_launch_takeoff_started = false;
    s_prev_armed = false;
}

void ModeFollow::run()
{
    const bool do_dbg = false;
    const bool armed = motors->armed();

    // disarmed: safe ground handling only
    if (!armed) {
        // reset ground-launch state so the next arm in Follow gets a fresh capture
        if (copter.ap.land_complete) {
            s_follow_launch_state = FollowLaunchState::INACTIVE;
            s_follow_blend_start_ms = 0;
            s_follow_deck_home_ofs_ned_m.zero();
            s_follow_blend_from_ofs_ned_m.zero();
            s_follow_blend_to_ofs_ned_m.zero();
            s_launch_takeoff_started = false;
            s_hold_active = false;
            s_held_pos_ofs_ned_m.zero();
        }

        s_prev_armed = false;
        make_safe_ground_handling();
        return;
    }

    // update AP_Follow estimates first so arm-edge capture uses fresh data
    g2.follow.update_estimates();

    // Detect arm edge while landed in Follow and re-capture launch geometry fresh.
    // This is what allows launch from the current drone-vs-boat deck position.
    if (!s_prev_armed && armed && FOLLOW_LAUNCH_ENABLE && copter.ap.land_complete) {
        Vector3f lead_pos_ned_m;
        Vector3f lead_vel_ned_ms;
        Vector3f lead_accel_ned_mss;
        Vector3f my_pos_ned_m;

        const bool have_lead_target =
            g2.follow.get_target_pos_vel_accel_NED_m(lead_pos_ned_m,
                                                     lead_vel_ned_ms,
                                                     lead_accel_ned_mss);
        const bool have_my_pos =
            AP::ahrs().get_relative_position_NED_origin(my_pos_ned_m);

        if (have_lead_target && have_my_pos) {
            s_follow_deck_home_ofs_ned_m = my_pos_ned_m - lead_pos_ned_m;
            s_follow_launch_state = FollowLaunchState::CLIMB;
            s_follow_blend_start_ms = 0;
            s_follow_blend_from_ofs_ned_m.zero();
            s_follow_blend_to_ofs_ned_m.zero();
            s_launch_takeoff_started = false;
            s_hold_active = false;
            s_held_pos_ofs_ned_m.zero();
        } else {
            s_follow_launch_state = FollowLaunchState::COMPLETE;
        }
    }

    s_prev_armed = true;

    // Allow Follow to run while armed+landed only during launch sequence.
    const bool launch_active =
        (s_follow_launch_state == FollowLaunchState::CLIMB) ||
        (s_follow_launch_state == FollowLaunchState::BLEND_TO_FOLLOW);

    // If we are landed and NOT actively in the launch sequence,
    // do not keep loading/updating follow offsets from Lua params.
    if (copter.ap.land_complete && !launch_active) {
        make_safe_ground_handling();
        return;
    }

    // Only once we've decided we're actually flying the mode do we allow
    // AP_Follow to apply the current relative offsets.
    g2.follow.init_offsets_if_required();

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    float yaw_rad = attitude_control->get_att_target_euler_rad().z;
    float yaw_rate_rads = 0.0f;

    // --- Get target in NED (meters, m/s, m/s^2) from AP_Follow ---
    Vector3p pos_ofs_ned_p;
    Vector3f vel_ofs_ned_ms;
    Vector3f accel_ofs_ned_mss;

    const bool have_target = g2.follow.get_ofs_pos_vel_accel_NED_m(pos_ofs_ned_p,
                                                                   vel_ofs_ned_ms,
                                                                   accel_ofs_ned_mss);

    // --- Get raw lead target state (without offset) ---
    Vector3f pos_lead_ned_m;
    Vector3f vel_lead_ned_ms;
    Vector3f accel_lead_ned_mss;

    const bool have_lead_target = g2.follow.get_target_pos_vel_accel_NED_m(pos_lead_ned_m,
                                                                           vel_lead_ned_ms,
                                                                           accel_lead_ned_mss);

    if (have_target) {

        float target_heading_deg = 0.0f;
        float target_heading_rate_degs = 0.0f;
        g2.follow.get_target_heading_deg(target_heading_deg);
        g2.follow.get_target_heading_rate_degs(target_heading_rate_degs);

        const Vector3f pos_ofs_ned_m = pos_ofs_ned_p.tofloat();

        // Default to stock follow target
        Vector3f use_pos_ofs_ned_m = pos_ofs_ned_m;
        Vector3f use_vel_ofs_ned_ms = vel_ofs_ned_ms;
        Vector3f use_accel_ofs_ned_mss = accel_ofs_ned_mss;

        bool using_launch_target = false;

        if (have_lead_target) {
            const Vector3f lead_pos_ned = pos_lead_ned_m;

            // Normal follow offset from AP_Follow
            const Vector3f normal_follow_ofs_ned_m = pos_ofs_ned_m - lead_pos_ned;

            // current follower position relative to EKF origin
            Vector3f my_pos_ned_m;
            const bool have_my_pos = AP::ahrs().get_relative_position_NED_origin(my_pos_ned_m);

            // current follower relative position to lead
            Vector3f rel_to_lead_ned_m{};
            if (have_my_pos) {
                rel_to_lead_ned_m = my_pos_ned_m - lead_pos_ned;
            }

            switch (s_follow_launch_state) {
            case FollowLaunchState::CLIMB:
                // NED z: more negative = higher altitude
                if (have_my_pos && (rel_to_lead_ned_m.z <= (normal_follow_ofs_ned_m.z + FOLLOW_Z_REACHED_TOL_M))) {
                    s_follow_launch_state = FollowLaunchState::BLEND_TO_FOLLOW;
                    s_follow_blend_start_ms = AP_HAL::millis();

                    s_follow_blend_from_ofs_ned_m = Vector3f(s_follow_deck_home_ofs_ned_m.x,
                                                             s_follow_deck_home_ofs_ned_m.y,
                                                             normal_follow_ofs_ned_m.z);
                    s_follow_blend_to_ofs_ned_m = normal_follow_ofs_ned_m;
                }
                break;

            case FollowLaunchState::BLEND_TO_FOLLOW: {
                const float t = constrain_float(
                    (AP_HAL::millis() - s_follow_blend_start_ms) * 0.001f /
                    MAX(FOLLOW_LAUNCH_BLEND_S, 0.01f),
                    0.0f,
                    1.0f);

                if (t >= 1.0f) {
                    s_follow_launch_state = FollowLaunchState::COMPLETE;
                }
                break;
            }

            case FollowLaunchState::COMPLETE:
            case FollowLaunchState::INACTIVE:
            default:
                break;
            }

            switch (s_follow_launch_state) {
            case FollowLaunchState::CLIMB: {
                // Hold deck-home XY, but use normal Follow Z
                Vector3f effective_ofs_ned_m(s_follow_deck_home_ofs_ned_m.x,
                                             s_follow_deck_home_ofs_ned_m.y,
                                             normal_follow_ofs_ned_m.z);

                use_pos_ofs_ned_m = lead_pos_ned + effective_ofs_ned_m;
                use_vel_ofs_ned_ms = vel_lead_ned_ms;
                use_accel_ofs_ned_mss = accel_lead_ned_mss;
                using_launch_target = true;
                break;
            }

            case FollowLaunchState::BLEND_TO_FOLLOW: {
                const float t = constrain_float(
                    (AP_HAL::millis() - s_follow_blend_start_ms) * 0.001f /
                    MAX(FOLLOW_LAUNCH_BLEND_S, 0.01f),
                    0.0f,
                    1.0f);

                Vector3f effective_ofs_ned_m;
                effective_ofs_ned_m.x = s_follow_blend_from_ofs_ned_m.x +
                                        (s_follow_blend_to_ofs_ned_m.x - s_follow_blend_from_ofs_ned_m.x) * t;
                effective_ofs_ned_m.y = s_follow_blend_from_ofs_ned_m.y +
                                        (s_follow_blend_to_ofs_ned_m.y - s_follow_blend_from_ofs_ned_m.y) * t;
                effective_ofs_ned_m.z = normal_follow_ofs_ned_m.z;

                use_pos_ofs_ned_m = lead_pos_ned + effective_ofs_ned_m;
                use_vel_ofs_ned_ms = vel_lead_ned_ms;
                use_accel_ofs_ned_mss = accel_lead_ned_mss;
                using_launch_target = true;
                break;
            }

            case FollowLaunchState::COMPLETE:
            case FollowLaunchState::INACTIVE:
            default:
                break;
            }

            if (do_dbg) {
                gcs().send_text(MAV_SEVERITY_INFO,
                                "FOLL launch=%u deckXY=%.1f %.1f relZ=%.1f normOfs=%.1f %.1f %.1f use=%.1f %.1f %.1f",
                                (unsigned)s_follow_launch_state,
                                (double)s_follow_deck_home_ofs_ned_m.x,
                                (double)s_follow_deck_home_ofs_ned_m.y,
                                (double)rel_to_lead_ned_m.z,
                                (double)normal_follow_ofs_ned_m.x,
                                (double)normal_follow_ofs_ned_m.y,
                                (double)normal_follow_ofs_ned_m.z,
                                (double)(use_pos_ofs_ned_m.x - lead_pos_ned.x),
                                (double)(use_pos_ofs_ned_m.y - lead_pos_ned.y),
                                (double)(use_pos_ofs_ned_m.z - lead_pos_ned.z));
            }
        }

        // If we're launching and still landed, explicitly leave landed state.
        if (s_follow_launch_state == FollowLaunchState::CLIMB && copter.ap.land_complete) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

            if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
                if (!s_launch_takeoff_started) {
                    set_land_complete(false);
                    pos_control->init_z_controller();
                    s_launch_takeoff_started = true;
                }
            }
        }

        const float STATIONARY_VEL_MS = 0.1f;
        const float DEADBAND_M        = 0.75f;
        const float REACQUIRE_M       = 1.25f;

        const float lead_speed = use_vel_ofs_ned_ms.xy().length();
        const bool lead_stationary = (lead_speed < STATIONARY_VEL_MS);

        if (using_launch_target) {
            s_hold_active = false;
        } else {
            if (!s_hold_active) {
                if (lead_stationary) {
                    s_hold_active = true;
                    s_held_pos_ofs_ned_m = use_pos_ofs_ned_m;
                }
            } else {
                if (!lead_stationary) {
                    s_hold_active = false;
                } else {
                    const float err_m = (use_pos_ofs_ned_m.xy() - s_held_pos_ofs_ned_m.xy()).length();

                    if (err_m > REACQUIRE_M) {
                        s_held_pos_ofs_ned_m = use_pos_ofs_ned_m;
                    } else if (err_m > DEADBAND_M) {
                        const float alpha = 0.1f;
                        s_held_pos_ofs_ned_m.xy() =
                            s_held_pos_ofs_ned_m.xy() + (use_pos_ofs_ned_m.xy() - s_held_pos_ofs_ned_m.xy()) * alpha;
                        s_held_pos_ofs_ned_m.z = use_pos_ofs_ned_m.z;
                    }
                }
            }
        }

        if (s_hold_active) {
            use_pos_ofs_ned_m = s_held_pos_ofs_ned_m;
            use_pos_ofs_ned_m.z = pos_ofs_ned_m.z;
            use_vel_ofs_ned_ms.zero();
            use_accel_ofs_ned_mss.zero();
        }

        Vector2p pos_ne_cm;
        pos_ne_cm.x = (postype_t)(use_pos_ofs_ned_m.x * 100.0f);
        pos_ne_cm.y = (postype_t)(use_pos_ofs_ned_m.y * 100.0f);

        Vector2f vel_ne_cms;
        vel_ne_cms.x = use_vel_ofs_ned_ms.x * 100.0f;
        vel_ne_cms.y = use_vel_ofs_ned_ms.y * 100.0f;

        const Vector2f accel_ne_cmss(use_accel_ofs_ned_mss.x * 100.0f,
                                     use_accel_ofs_ned_mss.y * 100.0f);

        float pos_up_cm   = (-use_pos_ofs_ned_m.z) * 100.0f;
        float vel_up_cms  = (-use_vel_ofs_ned_ms.z) * 100.0f;
        const float accel_up_cmss = (-use_accel_ofs_ned_mss.z) * 100.0f;

        pos_control->input_pos_vel_accel_xy(pos_ne_cm, vel_ne_cms, accel_ne_cmss, false);
        pos_control->input_pos_vel_accel_z(pos_up_cm, vel_up_cms, accel_up_cmss, false);

        switch (g2.follow.get_yaw_behave()) {

            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                Vector3f lead_pos_ned_tmp;
                Vector3f vel_lead_ned_tmp;
                Vector3f accel_lead_ned_tmp;

                if (g2.follow.get_target_pos_vel_accel_NED_m(lead_pos_ned_tmp, vel_lead_ned_tmp, accel_lead_ned_tmp)) {
                    Vector3f my_pos_ned_m;
                    if (AP::ahrs().get_relative_position_NED_origin(my_pos_ned_m)) {
                        const Vector2f to_lead_ne = (lead_pos_ned_tmp - my_pos_ned_m).xy();
                        if (to_lead_ne.length_squared() > 0.25f) {
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
                if (use_vel_ofs_ned_ms.xy().length_squared() > 0.25f) {
                    yaw_rad = use_vel_ofs_ned_ms.xy().angle();
                }
                break;

            case AP_Follow::YAW_BEHAVE_NONE:
            default: {
                const float pilot_yaw_rate_cds = get_pilot_desired_yaw_rate();
                yaw_rate_rads = radians(pilot_yaw_rate_cds * 0.01f);
                break;
            }
        }

    } else {
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