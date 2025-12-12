#include "Copter.h"

// ------------------------------------------------------------------
// Hard-coded configuration (for now; later these can become params)
// ------------------------------------------------------------------

// Offset (in NE frame, meters) from the precision-landing target that
// Wingman will try to hold.  Positive N is "north", positive E is "east".
static const float WINGMAN_OFFSET_N_M     = 10.0f;
static const float WINGMAN_OFFSET_E_M     = 10.0f;   // 10 m west of target

// Altitudes in meters above EKF origin (NEU frame z)
static const float WINGMAN_LAUNCH_ALT_M   = 15.0f;    // initial launch altitude
static const float WINGMAN_CRUISE_ALT_M   = 25.0f;    // cruise/escort altitude

// Simple vertical P-gain: climb_rate = K * alt_error
static const float WINGMAN_ALT_P          = 0.5f;

/*
 * Wingman mode: follow a moving precision-landing target with an
 * offset in NE, similar to LOITER + PrecLand, but:
 *  - we always fly off the PLND target position with a fixed offset
 *  - we have internal stages:
 *       LAUNCH -> climb to WINGMAN_LAUNCH_ALT_M
 *       CRUISE -> climb/descend to WINGMAN_CRUISE_ALT_M
 *  - pilot throttle is *ignored* in flight (altitude is held by autopilot)
 */

// initialise wingman controller
bool ModeWingman::init(bool ignore_checks)
{
    float target_roll_rad, target_pitch_rad;

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad,
                                      loiter_nav->get_angle_max_rad(),
                                      attitude_control->get_althold_lean_angle_max_rad());

    // process pilot's roll and pitch input
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, target_pitch_rad);

    loiter_nav->init_target();

    // initialise the vertical position controller
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_m(get_pilot_speed_dn_ms(),
                                         get_pilot_speed_up_ms(),
                                         get_pilot_accel_U_mss());
    pos_control->set_correction_speed_accel_U_m(get_pilot_speed_dn_ms(),
                                                get_pilot_speed_up_ms(),
                                                get_pilot_accel_U_mss());

    // Initial stage: if we're basically on the ground, go through LAUNCH,
    // otherwise just assume CRUISE (e.g. switched into WINGMAN while already flying)
    if (copter.ap.land_complete_maybe || copter.ap.land_complete) {
        _stage = Stage::LAUNCH;
    } else {
        _stage = Stage::CRUISE;
    }

#if AC_PRECLAND_ENABLED
    _precision_wingman_enabled = true;   // for now: always on
    _precision_wingman_active  = false;
#endif

    return true;
}

#if AC_PRECLAND_ENABLED
bool ModeWingman::do_precision_wingman()
{
    if (!_precision_wingman_enabled) {
        return false;
    }

    if (copter.ap.land_complete_maybe) {
        // don't move on the ground
        return false;
    }

    // If the pilot really wants to move the vehicle in XY, let them.
    // (same heuristic as LOITER's precision loiter)
    if (loiter_nav->get_pilot_desired_acceleration_NE_mss().length() > 0.5f) {
        return false;
    }

    if (!copter.precland.target_acquired()) {
        return false;    // no good vector
    }

    return true;
}

void ModeWingman::precision_wingman_xy()
{
    loiter_nav->clear_pilot_desired_acceleration();

    // Base target position from PrecLand
    Vector2p target_pos_ne_m;
    Vector2f target_vel_ne_ms;
    if (!copter.precland.get_target_position_m(target_pos_ne_m)) {
        // fall back to current vehicle position if we don't have a good target
        target_pos_ne_m = pos_control->get_pos_estimate_NEU_m().xy();
    }

    // Apply hard-coded offset in NE frame
    target_pos_ne_m.x += WINGMAN_OFFSET_N_M;
    target_pos_ne_m.y += WINGMAN_OFFSET_E_M;

    // Get estimated target velocity (absolute NE)
    copter.precland.get_target_velocity_ms(pos_control->get_vel_estimate_NEU_ms().xy(),
                                           target_vel_ne_ms);

    Vector2f zero;
    pos_control->input_pos_vel_accel_NE_m(target_pos_ne_m, target_vel_ne_ms, zero);

    // Run NE position controller
    pos_control->update_NE_controller();
}
#endif  // AC_PRECLAND_ENABLED

// main Wingman control loop (should be called at 100 Hz or more)
void ModeWingman::run()
{
    float target_roll_rad, target_pitch_rad;
    float target_yaw_rate_rads = 0.0f;

    // Throttle / climb rate handling:
    //  - we still use pilot throttle only for the AltHold state machine
    //  - but we IGNORE it for the actual altitude command while Flying
    float pilot_climb_rate_ms = 0.0f;
    float target_climb_rate_ms = 0.0f;    // what we actually send to pos_control

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_m(get_pilot_speed_dn_ms(),
                                         get_pilot_speed_up_ms(),
                                         get_pilot_accel_U_mss());

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad,
                                      loiter_nav->get_angle_max_rad(),
                                      attitude_control->get_althold_lean_angle_max_rad());

    // process pilot's roll and pitch input (XY)
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, target_pitch_rad);

    // pilot yaw rate
    target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

#if AC_PRECLAND_ENABLED
    // yaw-following of precision-landing target, same idea as LOITER
    static uint32_t last_plnd_yaw_dbg_ms = 0;

    if (_precision_wingman_enabled && copter.precland.enabled()) {
        const uint32_t now_ms = AP_HAL::millis();

        if (!copter.precland.target_acquired()) {
            if (now_ms - last_plnd_yaw_dbg_ms > 1000) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Wingman PLND yaw: target NOT acquired");
                last_plnd_yaw_dbg_ms = now_ms;
            }
        } else {
            float target_yaw_rad;
            const bool have_yaw = copter.precland.get_target_yaw_rad(target_yaw_rad);

            if (!have_yaw) {
                if (now_ms - last_plnd_yaw_dbg_ms > 1000) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Wingman PLND yaw: no valid target yaw");
                    last_plnd_yaw_dbg_ms = now_ms;
                }
            } else {
                const float curr_yaw_rad = ahrs.get_yaw_rad();
                const float yaw_err = wrap_PI(target_yaw_rad - curr_yaw_rad);

                // P-controller from yaw error -> yaw rate
                const float k_yaw = 2.0f;   // tune later
                float yaw_rate_cmd = k_yaw * yaw_err;

                // simple yaw-rate limit, ~60 deg/s
                const float yaw_rate_max = radians(60.0f);
                yaw_rate_cmd = constrain_float(yaw_rate_cmd,
                                               -yaw_rate_max,
                                               yaw_rate_max);

                // combine pilot yaw stick + PLND yaw tracking
                target_yaw_rate_rads += yaw_rate_cmd;
            }
        }
    }
#endif // AC_PRECLAND_ENABLED

    // Pilot climb rate is *only* used for the AltHold state machine
    pilot_climb_rate_ms = get_pilot_desired_climb_rate_ms();
    pilot_climb_rate_ms = constrain_float(pilot_climb_rate_ms,
                                          -get_pilot_speed_dn_ms(),
                                          get_pilot_speed_up_ms());

    // Determine high-level altitude-hold state
    AltHoldModeState wingman_state = get_alt_hold_state_U_ms(pilot_climb_rate_ms);

    // We ignore pilot throttle for actual altitude command in flight
    target_climb_rate_ms = 0.0f;

    switch (wingman_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Takeoff:
        // Takeoff: we behave like LOITER but with a fixed launch altitude
        if (!takeoff.running()) {
            takeoff.start_m(WINGMAN_LAUNCH_ALT_M);
        }

        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(pilot_climb_rate_ms);

        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);

        // Run NE loiter / wingman control during takeoff
        loiter_nav->update();
        break;

    case AltHoldModeState::Flying: {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // --- XY logic: precision wingman with offset when enabled ---
#if AC_PRECLAND_ENABLED
        {
            const bool precision_old = _precision_wingman_active;
            if (do_precision_wingman()) {
                precision_wingman_xy();
                _precision_wingman_active = true;
            } else {
                _precision_wingman_active = false;
            }

            if (precision_old && !_precision_wingman_active) {
                // precision was active but is no longer; re-init nav target
                loiter_nav->init_target();
            }

            // Run standard loiter-nav if not doing precision wingman
            if (!_precision_wingman_active) {
                loiter_nav->update();
            }
        }
#else
        loiter_nav->update();
#endif

        // --- Z logic: internal stage-based altitude control ---

        // Current altitude in NEU frame (z is "up")
        const Vector3p &pos_neu = pos_control->get_pos_estimate_NEU_m();
        const float curr_alt_m = pos_neu.z;

        float desired_alt_m = curr_alt_m;   // default, in case we do nothing

        if (_stage == Stage::LAUNCH) {
            desired_alt_m = WINGMAN_LAUNCH_ALT_M;

            // Once we are basically at launch altitude *and* have the target,
            // we can transition into CRUISE.
#if AC_PRECLAND_ENABLED
            const bool have_target = copter.precland.target_acquired();
#else
            const bool have_target = true;
#endif
            if (have_target && (fabsf(curr_alt_m - WINGMAN_LAUNCH_ALT_M) < 1.0f)) {
                _stage = Stage::CRUISE;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Wingman: reached launch alt, entering CRUISE");
            }

        } else { // Stage::CRUISE
            desired_alt_m = WINGMAN_CRUISE_ALT_M;
        }

        // Simple P-only vertical controller to generate climb rate
        const float alt_err = desired_alt_m - curr_alt_m;
        target_climb_rate_ms = WINGMAN_ALT_P * alt_err;

        // Respect max up/down speeds
        target_climb_rate_ms = constrain_float(target_climb_rate_ms,
                                               -get_pilot_speed_dn_ms(),
                                               get_pilot_speed_up_ms());

        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_U_from_climb_rate_ms(target_climb_rate_ms);
        break;
    }
    }

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading_rads(
        loiter_nav->get_thrust_vector(),
        target_yaw_rate_rads,
        false);

    // run the vertical position controller and set output throttle
    pos_control->update_U_controller();
}

float ModeWingman::wp_distance_m() const
{
    return loiter_nav->get_distance_to_target_m();
}

float ModeWingman::wp_bearing_deg() const
{
    return degrees(loiter_nav->get_bearing_to_target_rad());
}
