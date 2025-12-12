#pragma once

#include "mode.h"

// WINGMAN mode: follow a moving precision-landing target (e.g., a "boat")
// using the PrecLand library, flying with a fixed NE offset from the
// target. Conceptually similar to LOITER + PrecLand, but intended to
// be used as an autonomous escort / station-keeping mode.

class ModeWingman : public Mode {

public:
    using Mode::Mode;

    // ------------------------------------------------------------------
    // Mode interface
    // ------------------------------------------------------------------

    // unique mode number
    Number mode_number() const override { return Number::WINGMAN; }

    // initialise wingman controller
    bool init(bool ignore_checks) override;
    // main controller function
    void run() override;

    // GPS is required (we're flying in NE frame)
    bool requires_GPS() const override { return true; }

    // throttle is controlled by the autopilot (alt-hold style)
    bool has_manual_throttle() const override { return false; }

    // allow arming in this mode (same as LOITER)
    bool allows_arming(AP_Arming::Method method) const override { return true; }

    // treat WINGMAN like a "manual" navigation mode from a failsafe
    // perspective (same as LOITER)
    bool is_autopilot() const override { return false; }

    // allow user-initiated takeoff (re-uses the standard AltHold/Loiter
    // takeoff handling when entering from the ground)
    bool has_user_takeoff(bool must_navigate) const override { return true; }

    // allow autotune in this mode
    bool allows_autotune() const override { return true; }

#if AC_PRECLAND_ENABLED
    // external toggles (e.g. from an RC function) can enable/disable
    // precision Wingman behaviour (following the PLND target)
    void set_precision_wingman_enabled(bool value) { _precision_wingman_enabled = value; }
#endif

protected:

    const char *name() const override { return "WINGMAN"; }
    const char *name4() const override { return "WING"; }

    float wp_distance_m() const override;
    float wp_bearing_deg() const override;
    float crosstrack_error_m() const override { return pos_control->crosstrack_error_m(); }

#if AC_PRECLAND_ENABLED
    bool do_precision_wingman();
    void precision_wingman_xy();
#endif

private:
    // High-level altitude / mission stage inside Wingman
    enum class Stage : uint8_t {
        LAUNCH,   // climb to launch altitude
        CRUISE    // hold cruise altitude while following target
    };

    Stage _stage;

#if AC_PRECLAND_ENABLED
    bool _precision_wingman_enabled;
    bool _precision_wingman_active;
#endif
};
