#include <features/PidManager/PidManager.h>

PidManager::PidManager(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), previousError(0), integral(0) {}

float PidManager::calculate(float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    // Protect against zero or negative dt which would explode the derivative.
    // We still update `previousError` so the next valid call computes a
    // sane derivative — previously this branch returned early without
    // updating, causing a stale-baseline spike at the next dt > 0 call.
    if (dt <= 0.0f) {
        previousError = error;
        return kp * error;
    }

    // integrate with anti-windup: clamp integral term to a reasonable range
    const float maxIntegral = 1000.0f; // tune according to application
    integral += error * dt;
    if (integral > maxIntegral) integral = maxIntegral;
    else if (integral < -maxIntegral) integral = -maxIntegral;

    float derivative = (error - previousError) / dt;
    previousError = error;

    return kp * error + ki * integral + kd * derivative;
}

void PidManager::reset()
{
    previousError = 0.0f;
    integral = 0.0f;
}