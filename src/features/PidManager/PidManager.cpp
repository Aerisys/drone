#include <features/PidManager/PidManager.h>

PidManager::PidManager(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), previousError(0), integral(0) {}

float PidManager::calculate(float setpoint, float measured, float dt)
{
    // Protect against zero or negative dt which can cause huge derivative values
    if (dt <= 0.0f) {
        // simply compute proportional term and skip derivative/integral update
        float error = setpoint - measured;
        return kp * error;
    }

    float error = setpoint - measured;

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