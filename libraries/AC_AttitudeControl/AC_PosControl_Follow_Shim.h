#pragma once

#include "AC_PosControl.h"

// -----------------------------------------------------------------------------
// 4.7-style Follow helpers implemented on top of 4.6 AC_PosControl
// - 4.7 uses NED meters (NE, D)
// - 4.6 uses NEU centimeters (NE, U)
// -----------------------------------------------------------------------------

static inline Vector3f pos_target_NED_m_from_poscontrol(const AC_PosControl& pc)
{
    // 4.6: get_pos_target_cm() is NEU cm (relative to EKF origin)
    const Vector3p neu_cm = pc.get_pos_target_cm();

    Vector3f ned_m;
    ned_m.x = neu_cm.x * 0.01f;       // N: cm -> m
    ned_m.y = neu_cm.y * 0.01f;       // E: cm -> m
    ned_m.z = (-neu_cm.z) * 0.01f;    // D: -(U)  and cm -> m
    return ned_m;
}

static inline void input_pos_vel_accel_NE_m(AC_PosControl& pc,
                                            const Vector2p& pos_ne_m,   // meters, N/E
                                            Vector2f& vel_ne_ms,        // m/s
                                            const Vector2f& accel_ne_mss,
                                            bool limit_output)
{
    // Convert pos: m -> cm
    Vector2p pos_ne_cm;
    pos_ne_cm.x = pos_ne_m.x * 100.0f;
    pos_ne_cm.y = pos_ne_m.y * 100.0f;

    // Convert vel/accel: m/s -> cm/s, m/s^2 -> cm/s^2
    Vector2f vel_ne_cms = vel_ne_ms * 100.0f;
    const Vector2f accel_ne_cmss = accel_ne_mss * 100.0f;

    // 4.6 input expects by-ref vel, so preserve writeback into vel_ne_ms
    pc.input_pos_vel_accel_xy(pos_ne_cm, vel_ne_cms, accel_ne_cmss, limit_output);

    // write back shaped vel
    vel_ne_ms = vel_ne_cms * 0.01f;
}

static inline void input_pos_vel_accel_D_m(AC_PosControl& pc,
                                           float pos_d_m,        // meters, +Down
                                           float& vel_d_ms,      // m/s, +Down
                                           float accel_d_mss,    // m/s^2, +Down
                                           bool limit_output)
{
    // 4.6 Z API is "Up" in cm:
    //   U = -D
    float pos_up_cm = (-pos_d_m) * 100.0f;
    float vel_up_cms = (-vel_d_ms) * 100.0f;
    const float accel_up_cmss = (-accel_d_mss) * 100.0f;

    pc.input_pos_vel_accel_z(pos_up_cm, vel_up_cms, accel_up_cmss, limit_output);

    // write back shaped vel (back to +Down m/s)
    vel_d_ms = (-vel_up_cms) * 0.01f;
}

static inline void input_vel_accel_NE_m(AC_PosControl& pc, Vector2f& vel_ne_ms, const Vector2f& accel_ne_mss, bool limit_output)
{
    Vector2f vel_ne_cms = vel_ne_ms * 100.0f;
    const Vector2f accel_ne_cmss = accel_ne_mss * 100.0f;

    pc.input_vel_accel_xy(vel_ne_cms, accel_ne_cmss, limit_output);

    vel_ne_ms = vel_ne_cms * 0.01f;
}

static inline void input_vel_accel_D_m(AC_PosControl& pc, float& vel_d_ms, float accel_d_mss, bool limit_output)
{
    float vel_up_cms = (-vel_d_ms) * 100.0f;
    const float accel_up_cmss = (-accel_d_mss) * 100.0f;

    pc.input_vel_accel_z(vel_up_cms, accel_up_cmss, limit_output);

    vel_d_ms = (-vel_up_cms) * 0.01f;
}

// Init/update wrappers matching master naming
static inline void NE_init_controller(AC_PosControl& pc) { pc.init_xy_controller(); }
static inline void D_init_controller(AC_PosControl& pc)  { pc.init_z_controller();  }

static inline void NE_update_controller(AC_PosControl& pc) { pc.update_xy_controller(); }
static inline void D_update_controller(AC_PosControl& pc)  { pc.update_z_controller();  }