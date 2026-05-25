#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "features/PidManager/PidManager.h"
#include <ControllerRequestDTO.h>
#include "esp_log.h"
#include "esp_err.h"
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu9250.h"
#include "DroneConstants.h"

#define TAG_MOTOR_MANAGER "MotorManager"
#define CONTROL_LOOP_HZ 100
#define CONTROL_LOOP_DT (1.0f / CONTROL_LOOP_HZ)

/**
 * @class MotorManager
 * @brief Cascaded PID flight stabilization controller
 * 
 * Architecture:
 *   - Outer Loop (Angle): PID(angle_error) → rate_target
 *   - Inner Loop (Rate):  PID(rate_error)   → motor_corrections
 *   - Motor Mixer (X-Config): Apply corrections to throttle
 * 
 * This ensures fast gyro feedback stabilization while smooth angle tracking.
 */
class MotorManager
{
public:
    bool modeHIL;

    MotorManager(bool modeHIL = false);
    ~MotorManager();

    bool init(MPU9250 *imu, class ControllerUSB *usb = nullptr);

    void setMotorSpeed(int motorIndex, u_int32_t speed);
    void setMotorSpeedsZero();
    void disarmMotors();
    void armMotors();
    void Task();

    void getMotorSpeeds(float output[NUM_MOTORS]);
    
    SemaphoreHandle_t xMotorSpeedMutex = nullptr;
    static SemaphoreHandle_t xControllerRequestMutex;
    static ControllerRequestDTO currentControllerRequestDTO;

private:
    struct MotorPwmConfig
    {
        mcpwm_timer_handle_t timer;
        mcpwm_oper_handle_t operator_handle;
        mcpwm_cmpr_handle_t comparator;
        mcpwm_gen_handle_t generator;
    };

    // ==================== HARDWARE CONFIG ====================
    const int escPins[NUM_MOTORS] = {
        26, // Front-Left (M0)
        25, // Front-Right (M1)
        33, // Rear-Right (M2)
        32  // Rear-Left (M3)
    };
    MotorPwmConfig motorPwmConfigs[NUM_MOTORS];

    static constexpr int PWM_FREQ_HZ = 50;
    static constexpr uint32_t TIMER_RESOLUTION_HZ = 1000000;
    static constexpr uint32_t PERIOD_TICKS = TIMER_RESOLUTION_HZ / PWM_FREQ_HZ;
    static constexpr uint32_t MIN_PULSE_TICKS = 1000;  // 1ms (idle)
    static constexpr uint32_t MAX_PULSE_TICKS = 2000;  // 2ms (full throttle)
    static constexpr uint32_t PULSE_RANGE = MAX_PULSE_TICKS - MIN_PULSE_TICKS;

    // ==================== CONTROL LIMITS ====================
    static constexpr float MAX_ROLL_ANGLE_DEG = 45.0f;
    static constexpr float MAX_PITCH_ANGLE_DEG = 45.0f;
    static constexpr float MAX_YAW_RATE_DEG_S = 180.0f;
    static constexpr float MAX_ROLL_RATE_DEG_S = 360.0f;
    static constexpr float MAX_PITCH_RATE_DEG_S = 360.0f;
    static constexpr float THRUST_HEADROOM_RATIO = 0.15f;  // Reserve 15% for attitude corrections

    // ==================== OUTER LOOP (Angle PID) ====================
    // Converts stick input (angle setpoint) → rate target
    // These should be conservative (slow response to avoid oscillation)
    static constexpr float ANGLE_PITCH_KP = 4.5f;   // angle_error → rate_target
    static constexpr float ANGLE_PITCH_KI = 0.08f;
    static constexpr float ANGLE_PITCH_KD = 0.05f;

    static constexpr float ANGLE_ROLL_KP = 4.5f;
    static constexpr float ANGLE_ROLL_KI = 0.08f;
    static constexpr float ANGLE_ROLL_KD = 0.05f;

    static constexpr float ANGLE_YAW_KP = 2.0f;
    static constexpr float ANGLE_YAW_KI = 0.05f;
    static constexpr float ANGLE_YAW_KD = 0.02f;

    // ==================== INNER LOOP (Rate PID) ====================
    // Converts rate error → motor corrections
    // These should be aggressive (fast gyro feedback)
    static constexpr float RATE_PITCH_KP = 0.15f;   // rate_error → motor_correction
    static constexpr float RATE_PITCH_KI = 0.05f;
    static constexpr float RATE_PITCH_KD = 0.002f;

    static constexpr float RATE_ROLL_KP = 0.15f;
    static constexpr float RATE_ROLL_KI = 0.05f;
    static constexpr float RATE_ROLL_KD = 0.002f;

    static constexpr float RATE_YAW_KP = 0.08f;
    static constexpr float RATE_YAW_KI = 0.02f;
    static constexpr float RATE_YAW_KD = 0.001f;

    // ==================== SAFETY & TUNING ====================
    static constexpr float DT_MIN = 0.005f;   // Minimum dt (5ms) - prevents derivative spikes
    static constexpr float DT_MAX = 0.02f;    // Maximum dt (20ms) - detects timing overruns
    static constexpr float THROTTLE_DEADZONE = 0.02f;  // % of range - disarm if below this
    static constexpr float MAX_SETPOINT_SLEW_RATE_DEG_S = 200.0f; // max °/s for setpoint ramp

    // ==================== STATE ====================
    float motorSpeeds[NUM_MOTORS] = {0};
    bool isMotorArmed = false;
    int64_t lastLoopTime = 0;

    // Outer loop PIDs (angle feedback)
    PidManager pidAnglePitch{ANGLE_PITCH_KP, ANGLE_PITCH_KI, ANGLE_PITCH_KD};
    PidManager pidAngleRoll{ANGLE_ROLL_KP, ANGLE_ROLL_KI, ANGLE_ROLL_KD};
    PidManager pidAngleYaw{ANGLE_YAW_KP, ANGLE_YAW_KI, ANGLE_YAW_KD};

    // Inner loop PIDs (rate feedback)
    PidManager pidRatePitch{RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD};
    PidManager pidRateRoll{RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD};
    PidManager pidRateYaw{RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD};

    MPU9250 *imu = nullptr;
    class ControllerUSB *usbController = nullptr;

    // Previous orientation for rate estimation (via differentiation)
    float prevPitch = 0.0f;
    float prevRoll = 0.0f;
    float prevYaw = 0.0f;

    // Previous smoothed setpoints for slew-rate limiting
    float prevTargetPitch = 0.0f;
    float prevTargetRoll  = 0.0f;

    // Last known arming button state — edge detection to avoid calling arm/disarm every tick
    bool prevArmingState = true;

    // ==================== HELPER FUNCTIONS ====================
    float normalizeAngle(float angle);
    float clampValue(float value, float minVal, float maxVal);
    float clampRateOutput(float value);
};

#endif