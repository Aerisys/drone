#include "features/motorManager/MotorManager.h"
#include <inttypes.h>
#include "esp_timer.h"
#include <features/espNowHandler/EspNowHandler.h>
#include "MotorManager.h"

// when running in HIL mode we rely on the USB controller to receive orientation
// and to send motor outputs back to the host.
#include "features/controllerUSB/ControllerUSB.h"

// Static member initializations
SemaphoreHandle_t MotorManager::xControllerRequestMutex = xSemaphoreCreateMutex();
ControllerRequestDTO MotorManager::currentControllerRequestDTO;

MotorManager::MotorManager(bool modeHIL)
{
    this->modeHIL = modeHIL;
    xMotorSpeedMutex = xSemaphoreCreateMutex();
    if(this->modeHIL){
        return;
    }
    
    // Initialize motor speeds to zero
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (xSemaphoreTake(xMotorSpeedMutex, portMAX_DELAY) == pdTRUE) {
            motorSpeeds[i] = 0.0f;
            xSemaphoreGive(xMotorSpeedMutex);
        }
        
    }
}

MotorManager::~MotorManager()
{
    imu = nullptr;
}

// Function to initialize the motor manager
bool MotorManager::init(IMUSensor *imu, ControllerUSB *usb)
{
    // Mutex sanity check — both must have been created at construction time
    // (xSemaphoreCreateMutex returns nullptr if the heap is exhausted, which
    // is virtually impossible at boot but the resulting nullptr deref would
    // crash hard, so we surface the error cleanly here).
    if (xMotorSpeedMutex == nullptr || xControllerRequestMutex == nullptr)
    {
        ESP_LOGE(TAG_MOTOR_MANAGER,
                 "Mutex creation failed (motorSpeed=%p, controllerReq=%p)",
                 xMotorSpeedMutex, xControllerRequestMutex);
        return false;
    }

    // in HIL mode we rely on the USB controller for orientation and motor
    // output.  the caller has responsibility to pass a valid pointer.
    if (this->modeHIL) {
        // isMotorArmed = true;
        usbController = usb;
        if (usbController) {
            usbController->init();
        }
        return true;
    }

    ESP_LOGI(TAG_MOTOR_MANAGER, "Initializing MCPWM...");
    this->imu = imu;

    // Define shared timers for each group
    mcpwm_timer_handle_t shared_timers[2] = {nullptr, nullptr};

    for (int group_id = 0; group_id < 2; ++group_id)
    {
        // Create a shared timer for each group
        mcpwm_timer_config_t timer_config = {
            .group_id = group_id,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = TIMER_RESOLUTION_HZ,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = PERIOD_TICKS,
            .intr_priority = 0,
            .flags = {
                .update_period_on_empty = true,
                .update_period_on_sync = false,
                .allow_pd = false,
            }};
        if (mcpwm_new_timer(&timer_config, &shared_timers[group_id]) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to create timer for group %d", group_id);
            return false;
        }
    }

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        int group_id = i / 2; // Use group 0 for motors 0,1 and group 1 for motors 2,3

        // Create operator configuration
        mcpwm_operator_config_t operator_config = {
            .group_id = group_id,
            .intr_priority = 0,
            .flags = {
                .update_gen_action_on_tez = false,
                .update_gen_action_on_tep = false,
                .update_gen_action_on_sync = false,
                .update_dead_time_on_tez = false,
                .update_dead_time_on_tep = false,
                .update_dead_time_on_sync = false,
            }};
        if (mcpwm_new_operator(&operator_config, &motorPwmConfigs[i].operator_handle) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to create operator for motor %d", i);
            return false;
        }

        if (mcpwm_operator_connect_timer(motorPwmConfigs[i].operator_handle, shared_timers[group_id]) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to connect operator to timer for motor %d", i);
            return false;
        }

        // Create comparator configuration
        mcpwm_comparator_config_t comparator_config = {
            .intr_priority = 0,
            .flags = {
                .update_cmp_on_tez = true,
                .update_cmp_on_tep = false,
                .update_cmp_on_sync = false,
            }};
        if (mcpwm_new_comparator(motorPwmConfigs[i].operator_handle, &comparator_config, &motorPwmConfigs[i].comparator) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to create comparator for motor %d", i);
            return false;
        }

        // Create generator configuration
        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = escPins[i],
            .flags = {
                .invert_pwm = false,
                .io_loop_back = false,
                .io_od_mode = false,
                .pull_up = false,
                .pull_down = false,
            }};
        if (mcpwm_new_generator(motorPwmConfigs[i].operator_handle, &generator_config, &motorPwmConfigs[i].generator) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to create generator for motor %d", i);
            return false;
        }

        // Set generator actions
        if (mcpwm_generator_set_action_on_timer_event(
                motorPwmConfigs[i].generator,
                MCPWM_GEN_TIMER_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP,
                    MCPWM_TIMER_EVENT_EMPTY,
                    MCPWM_GEN_ACTION_HIGH)) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to set generator action on timer event for motor %d", i);
            return false;
        }

        if (mcpwm_generator_set_action_on_compare_event(
                motorPwmConfigs[i].generator,
                MCPWM_GEN_COMPARE_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP,
                    motorPwmConfigs[i].comparator,
                    MCPWM_GEN_ACTION_LOW)) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to set generator action on compare event for motor %d", i);
            return false;
        }

        // Set initial duty cycle (idle)
        if (mcpwm_comparator_set_compare_value(motorPwmConfigs[i].comparator, (MIN_PULSE_TICKS + MAX_PULSE_TICKS) * 0.5) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to set initial compare value for motor %d", i);
            return false;
        }
    }

    // Synchronize the timers
    // Source timer for synchronization (Master timer, group 0)
    // mcpwm_sync_handle_t sync_src = NULL;
    // mcpwm_timer_sync_src_config_t sync_src_config = {
    //     .timer_event = MCPWM_TIMER_EVENT_EMPTY, // Trigger when timer count is zero
    //     .flags = {
    //         .propagate_input_sync = false,
    //     }};

    // ESP_ERROR_CHECK(mcpwm_new_timer_sync_src(shared_timers[0], &sync_src_config, &sync_src));

    // // Phase timer synchronization (Slave timer, group 1)
    // mcpwm_timer_sync_phase_config_t sync_config = {
    //     .sync_src = sync_src,
    //     .count_value = 0, // Start counting from 0 upon synchronization
    //     .direction = MCPWM_TIMER_DIRECTION_UP,
    // };
    // ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(shared_timers[1], &sync_config));

    // Enable and start timers
    for (int group_id = 0; group_id < 2; ++group_id)
    {
        if (mcpwm_timer_enable(shared_timers[group_id]) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to enable timer for group %d", group_id);
            return false;
        }
        if (mcpwm_timer_start_stop(shared_timers[group_id], MCPWM_TIMER_START_NO_STOP) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to start timer for group %d", group_id);
            return false;
        }
    }

    // Send initial idle signal to arm ESCs
    vTaskDelay(pdMS_TO_TICKS(2000));

    // armMotors();

    ESP_LOGI(TAG_MOTOR_MANAGER, "Initialization complete");
    return true;
}

inline u_int32_t constrain(u_int32_t val, u_int32_t min, u_int32_t max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

// Function to set motor speed
void MotorManager::setMotorSpeed(int motorIndex, u_int32_t pulse_ticks)
{
    if (!isMotorArmed || motorIndex < 0 || motorIndex >= NUM_MOTORS)
    {
        return;
    }

    uint32_t pulse = MIN_PULSE_TICKS + constrain(pulse_ticks, 0, MAX_PULSE_TICKS - MIN_PULSE_TICKS);

    if(modeHIL){
        if (usbController) {
            usbController->setData(motorIndex, pulse);
        } else {
            ESP_LOGE(TAG_MOTOR_MANAGER, "HIL mode but usbController is null!");
        }
    }
    else{
        ESP_ERROR_CHECK(
            mcpwm_comparator_set_compare_value(
                motorPwmConfigs[motorIndex].comparator,
                pulse));
    }

    

    // ESP_LOGI(
    //     TAG_MOTOR_MANAGER,
    //     "Motor %d set to speed %.4f (pulse width: %" PRIu32 " µs)",
    //     motorIndex,
    //     pulse,
    //     pulse_ticks);
}

void MotorManager::setMotorSpeedsZero()
{
    if (xSemaphoreTake(xMotorSpeedMutex, portMAX_DELAY) == pdTRUE) {
        motorSpeeds[0] = 0; // Moteur 0 : avant-gauche
        motorSpeeds[1] = 0; // Moteur 1 : avant-droit
        motorSpeeds[2] = 0; // Moteur 2 : arrière-droit
        motorSpeeds[3] = 0; // Moteur 3 : arrière-gauche
        xSemaphoreGive(xMotorSpeedMutex);
    }
}

// Function to handle emergency stop
void MotorManager::disarmMotors()
{
    isMotorArmed = false;
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (xSemaphoreTake(xMotorSpeedMutex, portMAX_DELAY) == pdTRUE) {
            motorSpeeds[i] = 0;
            if(modeHIL){
                if (usbController) {
                    usbController->setData(i, 0);
                } else {
                    ESP_LOGE(TAG_MOTOR_MANAGER, "HIL mode but usbController is null!");
                }
            }
            else{
                mcpwm_comparator_set_compare_value(motorPwmConfigs[i].comparator, MIN_PULSE_TICKS);
            }
            
            xSemaphoreGive(xMotorSpeedMutex);
        }
        
    }
    ESP_LOGW(TAG_MOTOR_MANAGER, "disable Motor Arming");
}

// Function to reset the emergency stop
void MotorManager::armMotors()
{
    // ===== Real-mode IMU readiness guard =====
    // Skipped in HIL where the sensor is virtual (no calibration, no I2C).
    // These checks run BEFORE the radio failsafe guard so we never arm with
    // a dead/uncalibrated IMU, even if the radio link is perfect.
    if (!modeHIL)
    {
        if (imu == nullptr)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Arm refused: IMU pointer is null");
            return;
        }
        if (imu->getCalibrationStatus() != IMUSensor::CALIBRATED)
        {
            ESP_LOGW(TAG_MOTOR_MANAGER,
                     "Arm refused: IMU not calibrated (status=%d, still in progress or never run)",
                     (int)imu->getCalibrationStatus());
            return;
        }
        if (!imu->isSensorHealthy())
        {
            ESP_LOGW(TAG_MOTOR_MANAGER, "Arm refused: IMU reports unhealthy (I2C error threshold reached)");
            return;
        }
    }

    // ===== Radio failsafe latch guard =====
    // After a failsafe disarm (ping lost > 2 s OR sensor unhealthy), the
    // drone refuses to re-arm until the underlying cause has cleared. Once
    // the latch can be released, the next user arm press clears it.
    if (failsafeEngaged)
    {
        if (EspNowHandler::pingLost)
        {
            ESP_LOGW(TAG_MOTOR_MANAGER,
                     "Arm refused: radio failsafe latched and ping still lost");
            return;
        }
        ESP_LOGI(TAG_MOTOR_MANAGER, "Failsafe cleared (radio + IMU OK) — arming");
        failsafeEngaged = false;
    }

    isMotorArmed = true;
    ESP_LOGI(TAG_MOTOR_MANAGER, "enable Motor Arming; motors zeroed");

    // Reset ALL PID controllers (inner + outer) to avoid integral windup.
    // No pidAngleYaw — yaw is stick-to-rate direct, no angle PID.
    pidAnglePitch.reset();
    pidAngleRoll.reset();
    pidRatePitch.reset();
    pidRateRoll.reset();
    pidRateYaw.reset();

    // Send idle pulse to ESCs
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        setMotorSpeed(i, 0);
    }
}

// ==================== HELPER FUNCTIONS ====================

float MotorManager::normalizeAngle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float MotorManager::clampValue(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

float MotorManager::clampRateOutput(float value)
{
    return clampValue(value, -PULSE_RANGE * 0.5f, PULSE_RANGE * 0.5f);
}

// ==================== CONTROL LOOP (100Hz) ====================
void MotorManager::Task()
{
    ControllerRequestDTO lastControllerRequestDTO;
    int64_t lastTime         = esp_timer_get_time(); // HIL dt source
    uint64_t lastSnapshotUs  = 0;                    // real-mode dt source (0 = first iter)

    ESP_LOGI(TAG_MOTOR_MANAGER, "[%lu ms] MotorManager Task running...", esp_log_timestamp());

    if(modeHIL){
        // esp-lib v1.1.0: assignation par valeur (plus de `new`).
        // Initialise un FlightController neutre (tous axes a 0) pour que les
        // accès `lastControllerRequestDTO.flightController.xxx` soient
        // toujours valides en mode HIL avant le premier paquet reçu.
        lastControllerRequestDTO.flightController     = FlightController();
        lastControllerRequestDTO.has_flightController = true;
    }

    while (true)
    {
        // ===== STEP 1: READ SENSORS =====
        // Real mode: one atomic snapshot from the imu-lib seqlock — gives us
        // a coherent set (orientation + RAW gyro + accel + quaternion + ts)
        // taken at the same IMU iteration. Inner-rate PID uses snap.gyro
        // directly (no derivation noise, no 1-sample lag).
        //
        // HIL mode: orientation only via USB. Gyro rate is derived from
        // orientation differentiation (legacy path, kept further below).
        IMUSensor::SampleBundle snap = {};
        IMUSensor::Orientation currentOrientation = {};
        float dt = 0.0f;

        if (modeHIL) {
            if (usbController) currentOrientation = usbController->getOrientation();
            int64_t now = esp_timer_get_time();
            dt = (now - lastTime) * 1e-6f;
            lastTime = now;
        } else {
            snap = imu->getSnapshot();
            currentOrientation = snap.orientation;
            // dt from the IMU sample timestamp (true inter-sample interval).
            // First iteration: assume 1 ms (nominal 1 kHz INT rate) to seed
            // the derivative terms without a giant first-step error.
            if (lastSnapshotUs == 0) {
                dt = 0.001f;
            } else {
                dt = (snap.timestampUs - lastSnapshotUs) * 1e-6f;
            }
            lastSnapshotUs = snap.timestampUs;
        }

        // ===== STEP 2: UPDATE CONTROLLER COMMANDS =====
        // HIL emergency stop via serial (Unity sends "STOP\n" / "ARM\n")
        if (modeHIL && usbController) {
            if (usbController->isEmergencyStop()) {
                disarmMotors();
            }
        }

        // Radio failsafe (real mode only) — EspNowHandler sets pingLost = true
        // when no ESP-NOW ping has been received for > 2 s. We disarm
        // immediately and latch `failsafeEngaged` so re-arming requires an
        // explicit user button press AND the ping to have recovered (cf.
        // guard in armMotors). prevArmingState is forced to the "disarmed"
        // value so the next button press takes the arm branch.
        if (!modeHIL && EspNowHandler::pingLost && isMotorArmed)
        {
            ESP_LOGW(TAG_MOTOR_MANAGER, "RADIO FAILSAFE: ping lost > 2 s — auto-disarming");
            disarmMotors();
            failsafeEngaged  = true;
            prevArmingState  = false;
        }

        // IMU health failsafe (real mode only) — the imu-lib tracks I2C
        // success/error counts; once the failure threshold is crossed the
        // sensor is marked unhealthy. Continuing to feed PIDs with stale
        // attitude after the I2C bus has died would be catastrophic, so
        // disarm and latch the failsafe (same semantics as radio loss).
        if (!modeHIL && imu && !imu->isSensorHealthy() && isMotorArmed)
        {
            ESP_LOGW(TAG_MOTOR_MANAGER, "IMU FAILSAFE: sensor unhealthy — auto-disarming");
            disarmMotors();
            failsafeEngaged  = true;
            prevArmingState  = false;
        }

        if (xSemaphoreTake(xControllerRequestMutex, portMAX_DELAY))
        {
            if (currentControllerRequestDTO.has_buttonMotorArming)
            {
                // Consume the toggle event so we don't re-arm/disarm every tick.
                currentControllerRequestDTO.has_buttonMotorArming = false;
                if (!prevArmingState) {
                    armMotors();
                    if (modeHIL && usbController) usbController->clearEmergencyStop();
                } else {
                    disarmMotors();
                }
                prevArmingState = !prevArmingState;
            }
            if (currentControllerRequestDTO.has_buttonMotorState)
            {
                // Kill switch: latch — do NOT consume the flag, drone stays
                // disarmed as long as the controller asserts the kill state.
                disarmMotors();
            }
            if (currentControllerRequestDTO.has_flightController)
            {
                // POD copy (esp-lib v1.1) — memberwise, no heap, no race.
                lastControllerRequestDTO = currentControllerRequestDTO;
            }
            xSemaphoreGive(xControllerRequestMutex);
        }

        // ===== STEP 3: CONTROL LOOP (only if armed) =====
        if (isMotorArmed && lastControllerRequestDTO.has_flightController)
        {
            // dt already computed in step 1 from the appropriate source
            // (IMU snapshot timestamp in real mode, esp_timer in HIL).
            // Clamp to prevent derivative spikes from timing jitter.
            dt = clampValue(dt, DT_MIN, DT_MAX);

            // ===== 3.1: PARSE STICK INPUTS =====
            float stickPitch    = lastControllerRequestDTO.flightController.pitch;    // [-1..+1]
            float stickRoll     = lastControllerRequestDTO.flightController.roll;     // [-1..+1]
            float stickYaw      = lastControllerRequestDTO.flightController.yaw;      // [-1..+1]
            float stickThrottle = lastControllerRequestDTO.flightController.throttle; // [0..+1]

            // Raw setpoints from stick
            float targetPitchAngle = stickPitch * MAX_PITCH_ANGLE_DEG;
            float targetRollAngle  = stickRoll  * MAX_ROLL_ANGLE_DEG;
            float targetYawRate    = clampValue(stickYaw * MAX_YAW_RATE_DEG_S, -MAX_YAW_RATE_DEG_S, MAX_YAW_RATE_DEG_S);

            // Slew-rate limit: setpoint cannot jump faster than MAX_SETPOINT_SLEW_RATE_DEG_S
            // Prevents brutal stick inputs from demanding a step change the drone cannot follow
            float slewLimit  = MAX_SETPOINT_SLEW_RATE_DEG_S * dt;
            targetPitchAngle = prevTargetPitch + clampValue(targetPitchAngle - prevTargetPitch, -slewLimit, slewLimit);
            targetRollAngle  = prevTargetRoll  + clampValue(targetRollAngle  - prevTargetRoll,  -slewLimit, slewLimit);

            // Vector magnitude clamp: combined tilt (pitch²+roll²) cannot exceed MAX_PITCH_ANGLE_DEG.
            // Per-axis clamping alone allows √2×45° ≈ 63° when both axes are at maximum (e.g. circle input).
            float combinedTilt = sqrtf(targetPitchAngle * targetPitchAngle + targetRollAngle * targetRollAngle);
            if (combinedTilt > MAX_PITCH_ANGLE_DEG) {
                float scale    = MAX_PITCH_ANGLE_DEG / combinedTilt;
                targetPitchAngle *= scale;
                targetRollAngle  *= scale;
            }

            prevTargetPitch = targetPitchAngle;
            prevTargetRoll  = targetRollAngle;

            // Throttle (0 to PULSE_RANGE)
            float throttleOutput = stickThrottle * PULSE_RANGE;

            // ===== 3.2: SAFETY CHECKS =====
            // Low throttle = reset PIDs to avoid integral windup
            if (stickThrottle < THROTTLE_DEADZONE) {
                pidAnglePitch.reset();
                pidAngleRoll.reset();
                pidRatePitch.reset();
                pidRateRoll.reset();
                pidRateYaw.reset();
                prevTargetPitch = 0.0f;
                prevTargetRoll  = 0.0f;
                throttleOutput = 0.0f;
            }

            // Reserve thrust margin for attitude corrections
            float usableThrottle = throttleOutput * (1.0f - THRUST_HEADROOM_RATIO);
            float thrustHeadroom = PULSE_RANGE * THRUST_HEADROOM_RATIO;

            // ===== 3.3: NORMALIZE ANGLES & GET RATES =====
            currentOrientation.pitch = normalizeAngle(currentOrientation.pitch);
            currentOrientation.roll  = normalizeAngle(currentOrientation.roll);
            currentOrientation.yaw   = normalizeAngle(currentOrientation.yaw);

            float gyroPitchRate, gyroRollRate, gyroYawRate;
            if (modeHIL) {
                // HIL mode: USB controller only ships orientation. Derive
                // body rate via angle differentiation. Introduces ~1-sample
                // lag + numerical noise; acceptable for sim purposes.
                float angleDiffPitch = normalizeAngle(currentOrientation.pitch - prevPitch);
                float angleDiffRoll  = normalizeAngle(currentOrientation.roll  - prevRoll);
                float angleDiffYaw   = normalizeAngle(currentOrientation.yaw   - prevYaw);

                gyroPitchRate = (dt > 0.0f) ? angleDiffPitch / dt : 0.0f;  // deg/s
                gyroRollRate  = (dt > 0.0f) ? angleDiffRoll  / dt : 0.0f;
                gyroYawRate   = (dt > 0.0f) ? angleDiffYaw   / dt : 0.0f;

                prevPitch = currentOrientation.pitch;
                prevRoll  = currentOrientation.roll;
                prevYaw   = currentOrientation.yaw;
            } else {
                // Real mode: raw gyro from the IMU snapshot — DLPF-filtered
                // (and notch-filtered if `imu->setGyroNotch(...)` was called).
                // Already calibrated (gyroOffset + temperature comp applied).
                //
                // Axis mapping assumes standard aerospace body convention:
                //   X forward  -> rotation around X = ROLL rate
                //   Y right    -> rotation around Y = PITCH rate
                //   Z down     -> rotation around Z = YAW rate
                //
                // If your physical mounting differs, either:
                //   - call imu->setInvertAxis(...) at boot to flip signs, or
                //   - swap the assignments below.
                gyroPitchRate = snap.gyro.y;
                gyroRollRate  = snap.gyro.x;
                gyroYawRate   = snap.gyro.z;
            }

            // ===== 3.4: OUTER LOOP - ANGLE TO RATE =====
            // Converts (stick_angle_target - measured_angle) → rate_target
            
            // Outer PIDs output: desired rate setpoints (pitch + roll only).
            // Yaw skips the outer loop — `targetYawRate` (computed once in
            // 3.1) feeds the inner rate PID directly. This avoids a
            // duplicate `desiredYawRate = stickYaw * MAX_YAW_RATE_DEG_S`
            // computation and removes the dead `pidAngleYaw` instance.
            float desiredPitchRate = pidAnglePitch.calculate(targetPitchAngle, currentOrientation.pitch, dt);
            float desiredRollRate  = pidAngleRoll.calculate(targetRollAngle, currentOrientation.roll, dt);

            // Clamp rate targets
            desiredPitchRate = clampValue(desiredPitchRate, -MAX_PITCH_RATE_DEG_S, MAX_PITCH_RATE_DEG_S);
            desiredRollRate  = clampValue(desiredRollRate,  -MAX_ROLL_RATE_DEG_S,  MAX_ROLL_RATE_DEG_S);

            // ===== 3.5: INNER LOOP - RATE TO MOTOR CORRECTIONS =====
            // Converts (desired_rate - measured_rate) → motor_correction
            float motorCorrectionPitch = pidRatePitch.calculate(desiredPitchRate, gyroPitchRate, dt);
            float motorCorrectionRoll  = pidRateRoll.calculate(desiredRollRate,  gyroRollRate,  dt);
            float motorCorrectionYaw   = pidRateYaw.calculate(targetYawRate,     gyroYawRate,   dt);

            // Clamp motor corrections to reserved thrust headroom
            motorCorrectionPitch = clampValue(motorCorrectionPitch, -thrustHeadroom, thrustHeadroom);
            motorCorrectionRoll  = clampValue(motorCorrectionRoll, -thrustHeadroom, thrustHeadroom);
            motorCorrectionYaw   = clampValue(motorCorrectionYaw, -thrustHeadroom, thrustHeadroom);

            // ===== 3.5b: ANGLE SAFETY LIMIT =====
            // Hard cut when physical tilt exceeds the allowed range.
            // Zeroing throttle AND all corrections forces the mixer to output
            // exactly 0 on every motor — asymmetric corrections alone would
            // otherwise keep feeding the rotation.
            float physicalTilt = sqrtf(currentOrientation.pitch * currentOrientation.pitch
                                     + currentOrientation.roll  * currentOrientation.roll);
            if (physicalTilt > MAX_PITCH_ANGLE_DEG) {
                usableThrottle       = 0.0f;
                motorCorrectionPitch = 0.0f;
                motorCorrectionRoll  = 0.0f;
                motorCorrectionYaw   = 0.0f;
                pidAnglePitch.reset();
                pidAngleRoll.reset();
                pidRatePitch.reset();
                pidRateRoll.reset();
            }

            // Send compact one-line telemetry to Unity at 20Hz (every 5 control loops).
            if (modeHIL && usbController) {
                usbController->sendTelemetry(stickPitch,
                                             stickRoll,
                                             stickYaw,
                                             stickThrottle,
                                             targetPitchAngle,
                                             targetRollAngle,
                                             targetYawRate,
                                             motorCorrectionPitch,
                                             motorCorrectionRoll,
                                             motorCorrectionYaw,
                                             currentOrientation.roll,
                                             currentOrientation.pitch,
                                             currentOrientation.yaw);
            }

            // ===== 3.6: MOTOR MIXER (X-Config) =====
            // Apply motor corrections to throttle setpoint
            // M0 (Front-Left):    T - pitch + roll + yaw
            // M1 (Front-Right):   T - pitch - roll - yaw
            // M2 (Rear-Right):    T + pitch - roll - yaw
            // M3 (Rear-Left):     T + pitch + roll + yaw

            // ===== 3.6+3.7: MIXER + SEND TO ESCs (single mutex hold) =====
            // Previously took the mutex 5 times (1 mixer + 4× ESC send). Now
            // a single critical section wraps both compute and dispatch:
            //   - motorSpeeds[] is only written by this task (the mutex only
            //     guards cross-task READS from EspNowHandler::getMotorSpeeds);
            //   - setMotorSpeed() pokes a single MCPWM comparator register
            //     (~µs), so holding the mutex during the 4-call loop is
            //     cheap and gives the telemetry reader a fully coherent
            //     snapshot of all 4 motors at once.
            if (xSemaphoreTake(xMotorSpeedMutex, portMAX_DELAY) == pdTRUE)
            {
                auto clampMotor = [](float val) -> float {
                    if (val < 0.0f) return 0.0f;
                    if (val > (float)PULSE_RANGE) return (float)PULSE_RANGE;
                    return val;
                };

                // Force zero if throttle too low
                if (usableThrottle < 5.0f) {
                    motorSpeeds[0] = motorSpeeds[1] = motorSpeeds[2] = motorSpeeds[3] = 0.0f;
                } else {
                    motorSpeeds[0] = clampMotor(usableThrottle - motorCorrectionPitch + motorCorrectionRoll + motorCorrectionYaw);
                    motorSpeeds[1] = clampMotor(usableThrottle - motorCorrectionPitch - motorCorrectionRoll - motorCorrectionYaw);
                    motorSpeeds[2] = clampMotor(usableThrottle + motorCorrectionPitch - motorCorrectionRoll - motorCorrectionYaw);
                    motorSpeeds[3] = clampMotor(usableThrottle + motorCorrectionPitch + motorCorrectionRoll + motorCorrectionYaw);
                }

                // Dispatch within the same critical section.
                for (int i = 0; i < NUM_MOTORS; i++) {
                    setMotorSpeed(i, motorSpeeds[i]);
                }
                xSemaphoreGive(xMotorSpeedMutex);
            }
        }
        else if (!isMotorArmed)
        {
            setMotorSpeedsZero();
            for (int i = 0; i < NUM_MOTORS; i++) setMotorSpeed(i, 0);
        }

        // ===== STEP 4: WAIT FOR NEXT TICK =====
        // Real mode: block on the IMU sample semaphore. The sensor task
        //   publishes at ~1 kHz (DATA_READY INT), so this loop runs in
        //   lockstep with the sensor — no phase drift, no missed samples.
        //   20 ms timeout = watchdog (sensor stuck -> we still iterate
        //   to keep failsafes alive; C2 will add an explicit disarm).
        //
        // HIL mode: no INT, no semaphore -> classic 100 Hz tick delay.
        if (modeHIL) {
            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            if (imu->waitForNewSample(20) != ESP_OK) {
                ESP_LOGW(TAG_MOTOR_MANAGER, "IMU sample timeout (>20 ms) — sensor stuck?");
                // continue with the (stale) last snapshot. Failsafe disarm
                // on prolonged stuck sensor will be added in task C2.
            }
        }
    }
}

void MotorManager::getMotorSpeeds(float output[NUM_MOTORS]) {
    if (xSemaphoreTake(xMotorSpeedMutex, portMAX_DELAY) == pdTRUE) {
        memcpy(output, motorSpeeds, sizeof(float) * NUM_MOTORS);
        xSemaphoreGive(xMotorSpeedMutex);
    }
}
