#ifndef PIDMANAGER_H
#define PIDMANAGER_H

class PidManager
{
public:
    PidManager(float kp, float ki, float kd);

    float calculate(float setpoint, float measured, float dt);
    
    /**
     * @brief Reset the PID controller state (clears integral and previous error)
     * Should be called when starting a new control cycle to avoid integral windup
     */
    void reset();

private:
    float kp, ki, kd;
    float previousError;
    float integral;
};

#endif // PIDMANAGER_H
