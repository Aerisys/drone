#ifndef DRONE_CONSTANTS_H
#define DRONE_CONSTANTS_H

// -----------------------------------------------------------------------------
// DroneConstants — single source of truth for cross-module constants.
// -----------------------------------------------------------------------------
// Anything used by more than one feature (e.g. the motor count, referenced by
// both the motor manager and the telemetry DTO) belongs here so the value
// never diverges. Feature-local constants stay in their own headers.
// -----------------------------------------------------------------------------

// Number of motors in the airframe. Currently fixed to 4 (X quadcopter).
// Used by MotorManager (ESC arrays) and TelemetryDTO (telemetry payload).
#define NUM_MOTORS 4

#endif // DRONE_CONSTANTS_H
