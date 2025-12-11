#include "AC_PrecLand_config.h"

#if AC_PRECLAND_MAVLINK_ENABLED

#include "AC_PrecLand_MAVLink.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

// perform any required initialisation of backend
void AC_PrecLand_MAVLink::init()
{
    // set healthy
    _state.healthy = true;
}

// retrieve updates from sensor
void AC_PrecLand_MAVLink::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    // LOS vector timeout
    _los_meas.valid = _los_meas.valid && (now_ms - _los_meas.time_ms <= 1000);

    // OPTIONAL: also age-out target yaw if it goes stale
    if (_state.target_yaw_valid && (now_ms - _state.target_yaw_time_ms > 1000)) {
        _state.target_yaw_valid = false;
    }
}

void AC_PrecLand_MAVLink::handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
    // check frame is supported
    if (packet.frame != MAV_FRAME_BODY_FRD && packet.frame != MAV_FRAME_LOCAL_FRD) {
        if (!_wrong_frame_msg_sent) {
            _wrong_frame_msg_sent = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Plnd: Frame not supported");
        }
        return;
    }

    // --- Position / LOS vector handling ---
    if (packet.position_valid == 1) {
        if (packet.distance > 0) {
            _los_meas.vec_unit = Vector3f{packet.x, packet.y, packet.z};
            _los_meas.vec_unit /= packet.distance;
            _los_meas.frame = (packet.frame == MAV_FRAME_BODY_FRD)
                                ? AC_PrecLand::VectorFrame::BODY_FRD
                                : AC_PrecLand::VectorFrame::LOCAL_FRD;
        } else {
            // distance to target must be positive
            return;
        }
    } else {
        // compute unit vector towards target from angles
        _los_meas.vec_unit = Vector3f{-tanf(packet.angle_y), tanf(packet.angle_x), 1.0f};
        _los_meas.vec_unit /= _los_meas.vec_unit.length();
        _los_meas.frame = (packet.frame == MAV_FRAME_BODY_FRD)
                            ? AC_PrecLand::VectorFrame::BODY_FRD
                            : AC_PrecLand::VectorFrame::LOCAL_FRD;
    }

    _distance_to_target = MAX(0, packet.distance);
    _los_meas.time_ms = timestamp_ms;
    _los_meas.valid = true;

    // store target yaw from quaternion if provided
    // q is (w, x, y, z) and is part of the "position_valid" block.
    if (packet.position_valid == 1) {
        const float qw = packet.q[0];
        const float qx = packet.q[1];
        const float qy = packet.q[2];
        const float qz = packet.q[3];

        const float q_norm2 = sq(qw) + sq(qx) + sq(qy) + sq(qz);

        if (!is_zero(q_norm2)) {
            // AP_Math Quaternion is also (w, x, y, z)
            Quaternion q_target(qw, qx, qy, qz);

            float roll, pitch, yaw;
            q_target.to_euler(roll, pitch, yaw);

            _state.target_yaw_rad     = wrap_PI(yaw);
            _state.target_yaw_time_ms = timestamp_ms;
            _state.target_yaw_valid   = true;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            // Debug in SITL so we can see what we're getting
            GCS_SEND_TEXT(
                MAV_SEVERITY_INFO,
                "PLND RX q->yaw: yaw=%.1f deg (qw=%.3f qz=%.3f)",
                (double)degrees(_state.target_yaw_rad),
                (double)qw,
                (double)qz
            );
#endif
        } else {
            // Explicitly clear validity if sender stops providing orientation
            _state.target_yaw_valid = false;
        }
    } else {
        // No valid position block => no valid orientation
        _state.target_yaw_valid = false;
    }
}

#endif // AC_PRECLAND_MAVLINK_ENABLED
