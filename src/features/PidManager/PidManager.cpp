#include <features/PidManager/PidManager.h>

PidManager::PidManager(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), previousError(0), integral(0) {}

float PidManager::calculate(float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    integral += error * dt;
    float derivative = (error - previousError) / dt;
    previousError = error;
    return kp * error + ki * integral + kd * derivative;
}

void PidManager::reset()
{
    previousError = 0.0f;
    integral = 0.0f;
}