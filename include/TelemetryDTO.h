#ifndef TELEMETRY_DTO_H
#define TELEMETRY_DTO_H

#include "imu_sensor.h"
#include "DroneConstants.h"
#include <stdint.h>

// -----------------------------------------------------------------------------
// TelemetryDTO — unified payload sent to the ground station over ESP-NOW.
// -----------------------------------------------------------------------------
// Bundles a full IMU snapshot (taken atomically by the imu-lib seqlock) with
// the current motor speeds, so a single ESP-NOW packet gives the ground
// station everything it needs to render attitude + thrust state at the same
// instant.
//
// Depends on the abstract IMUSensor interface rather than a specific chip
// type — swapping MPU9250 for an ICM-* in the future does not change the
// telemetry layout.
//
// ----------------------- Wire format / size ----------------------------------
// The receiver (controller / ground station) dispatches incoming ESP-NOW
// packets BY SIZE (see EspNowHandler::onDataRecv). Any field change here is
// a BREAKING CHANGE for the receiver — it must be updated in lockstep.
//
// Current layout (no padding on ESP32, all members 4-byte aligned):
//   accel        12  (Vector3)
//   gyro         12
//   mag          12
//   orientation  12  (Orientation)
//   quaternion   16  (Quaternion)
//   temperature   4
//   motorSpeeds  16  (4 floats)
//   timestampUs   8
//   -------------
//   total:       92 bytes  (well below ESP-NOW 250-byte payload limit)
// -----------------------------------------------------------------------------

struct TelemetryDTO
{
    IMUSensor::Vector3     accel;                  // g
    IMUSensor::Vector3     gyro;                   // deg/s
    IMUSensor::Vector3     mag;                    // uT (zero on sensors without mag)
    IMUSensor::Orientation orientation;            // roll/pitch/yaw, deg
    IMUSensor::Quaternion  quaternion;             // body-to-world, identity if filter inactive
    float                  temperature;            // degC
    float                  motorSpeeds[NUM_MOTORS]; // pulse ticks (0..PULSE_RANGE)
    uint64_t               timestampUs;            // esp_timer_get_time() at IMU sample
};

#endif // TELEMETRY_DTO_H
