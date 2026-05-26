#include "features/espNowHandler/EspNowHandler.h"
#include "features/motorManager/MotorManager.h"
#include "features/controllerUSB/ControllerUSB.h"

#include "mpu9250.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "driver/i2c_master.h"

EspNowHandler *espNowHandler;
MotorManager *motorManager;
MPU9250 *imu;
static i2c_master_bus_handle_t i2cBus = nullptr;

static const char *TAG_MAIN = "MAIN";

extern "C" void app_main(void)
{
    // ---- 1. NVS (single init point for the whole firmware) ----------------
    // The IMU library auto-loads its persisted calibration from NVS during
    // init(), and EspNowHandler stores/loads the paired MAC. Initialise once
    // here with the standard ESP-IDF recovery pattern; downstream modules
    // must not re-init NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else
    {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // ---- 2. I2C bus (owned by the application, shared across drivers) -----
    // The new imu-lib v1.1 takes a bus handle instead of installing the I2C
    // driver itself. Creating the bus here lets us share it with future
    // peripherals (baro, OSD, ...) on the same SDA/SCL.
    i2c_master_bus_config_t busCfg = {};
    busCfg.i2c_port           = I2C_NUM_0;
    busCfg.sda_io_num         = GPIO_NUM_21;
    busCfg.scl_io_num         = GPIO_NUM_22;
    busCfg.clk_source         = I2C_CLK_SRC_DEFAULT;
    busCfg.glitch_ignore_cnt  = 7;
    busCfg.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&busCfg, &i2cBus));

    // ---- 3. Object construction --------------------------------------------
    espNowHandler = new EspNowHandler();
    imu = new MPU9250();
    motorManager = new MotorManager(true);

    // prepare USB controller for HIL mode
    ControllerUSB *usbController = nullptr;
    if (motorManager->modeHIL) {
        usbController = new ControllerUSB();
    }
    if (!espNowHandler->init(imu, motorManager))
    {
        ESP_LOGE(TAG_MAIN, "ESP-NOW init failed!");
        return;
    }

    // ---- 4. IMU init with new API -----------------------------------------
    // 1 kHz INT-driven path: MPU9250 INT pin wired to GPIO 19 + sensor task
    // pinned to core 1 (APP_CPU_NUM). Wi-Fi/lwIP are confined to core 0 by
    // sdkconfig (CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0 + CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0)
    // so the IMU never gets preempted by network ISRs.
    imu->setFilterMode(MPU9250::MAHONY);

    MPU9250::Config imuCfg;
    imuCfg.intPin       = GPIO_NUM_19;     // DATA_READY -> ISR -> 1 kHz wakeup
    imuCfg.taskCoreId   = APP_CPU_NUM;     // core 1 (real-time stack)
    imuCfg.taskPriority = 5;               // PID task below at prio 4
    err = imu->init(i2cBus, imuCfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MAIN, "Failed to initialize MPU9250 (err=%d)", err);
        // In HIL mode the absence of physical sensor is expected and not fatal,
        // we still want the rest of the firmware (USB bridge, ESP-NOW) running.
    }

    // ---- Runtime IMU tuning ------------------------------------------------
    // Adjust these for THIS airframe. Defaults below are sane starting points
    // for a typical 5" quad with the MPU9250 mounted right-way-up, X axis
    // pointing forward.
    //
    // Mahony gains:
    //   Kp ~ 1.0 = responsive correction (good for drones). 0.5 = smoother
    //              but slower convergence. 2.0+ = nervous, more noise.
    //   Ki = 0.0 default — non-zero Ki causes integral wind-up during
    //              aggressive maneuvers (flips, fast yaws). Only enable
    //              (typ. < 0.005) if observed gyro bias drift in flight
    //              justifies it.
    imu->setMahonyGains(1.0f, 0.0f);

    // Axis orientation knobs — adjust ONLY if your physical mounting differs
    // from the "X forward / Y right / Z down" reference:
    //
    //   setSwitchRollPitch(true) -> swap roll/pitch in the Euler output
    //     (use if a pure pitch motion in flight reads as roll on the
    //     ground-station HUD, or vice-versa).
    //
    //   setInvertAxis(true, _, _) -> negate accel/gyro/mag X axis
    //     (use if a roll-right input results in roll-left actual motion).
    //   setInvertAxis(_, true, _) -> negate Y axis (same logic for pitch).
    //   setInvertAxis(_, _, true) -> negate Z axis (same logic for yaw).
    //
    // Determine by hand-tilting the disarmed drone and watching the
    // telemetry quaternion / euler: tilt one axis at a time and confirm
    // the sign matches your expectation. Adjust here once, commit, done.
    imu->setSwitchRollPitch(false);
    imu->setInvertAxis(false, false, false);

    // ---- Mahony fast-init boost (imu-lib v1.2.0+) -------------------------
    // AUTOMATIQUE — pas besoin d'appeler quoi que ce soit ici. La lib boost
    // Kp à 10 pendant 3 s au démarrage du sensor task ET après une
    // calibrateGyroAccel, ce qui amène le quaternion Mahony de l'identité
    // (boot) vers l'attitude réelle en < 1 s au lieu de 5-10 s.
    //
    // Pour reconfigurer (rare — par défaut OK pour un drone) :
    //   imu->setMahonyBoost(15.0f, 2000);  // boostKp=15, 2 s au lieu de 3
    //
    // Pour forcer un boost manuel (ex: après atterrissage brutal détecté) :
    //   imu->triggerMahonyBoost();
    //
    // Pour désactiver totalement le boost (déconseillé) :
    //   imu->setMahonyBoost(0.0f, 0);

    // ---- Home / mount-offset (imu-lib v1.2.0+) ----------------------------
    // setHome() capture la position courante comme "level chassis", uniquement
    // sur roll/pitch (yaw absolu préservé pour le mag heading). Persisté en
    // NVS — une fois fait, c'est valide pour tous les boots suivants.
    //
    // À déclencher UNE FOIS, drone posé en position "level" (typiquement à
    // l'assemblage), via l'une des options :
    //
    // OPTION A — Trigger codé en dur (le plus simple pour démarrer) :
    //   Décommente le bloc ci-dessous, flash, place le drone level sur surface
    //   plane, attends que le boost Mahony se termine (~3 s + temps de boot),
    //   puis le setHome se fait automatiquement et est persisté. Recommente
    //   ensuite pour éviter qu'il se redéclenche à chaque boot.
    //
    /*
    if(!motorManager->modeHIL){
        vTaskDelay(pdMS_TO_TICKS(5000));  // attente convergence Mahony
        esp_err_t homeErr = imu->setHome();
        if (homeErr == ESP_OK)
            ESP_LOGI(TAG_MAIN, "Home set & persisted to NVS. RE-COMMENT THIS BLOCK.");
        else
            ESP_LOGE(TAG_MAIN, "setHome failed: %d", homeErr);
    }
    */
    //
    // OPTION B — Trigger par long-press sur bouton (à wirer plus tard) :
    //   Adapter la logique de PIN_BUTTON_ASSOCIATION dans EspNowHandler pour
    //   distinguer long-press court (3 s) = setHome vs très long (5 s) =
    //   reset pairing.
    //
    // OPTION C — Trigger par commande ESP-NOW dédiée :
    //   Ajouter un nouveau type de packet "SET_HOME" dans esp-lib, envoyé
    //   depuis le controller via un bouton de configuration.
    //
    // Pour effacer un home déjà set : `imu->clearHome();` (persisté NVS aussi)

    if(!motorManager->modeHIL){
        // imu->init() auto-loads NVS calibration. Only trigger a fresh
        // gyro/accel calibration if nothing was found on disk — otherwise
        // we'd burn 10 s of mandatory immobility on every boot.
        if (imu->getCalibrationStatus() != MPU9250::CALIBRATED) {
            err = imu->calibrate();
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG_MAIN, "Failed to start calibration (err=%d)", err);
                //return;
            }
            ESP_LOGI(TAG_MAIN, "Gyro/Accel calibration started (~10 s, keep still)");
        } else {
            ESP_LOGI(TAG_MAIN, "Calibration loaded from NVS, skipping cold calibration");
        }

        err = imu->startSensorTask();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_MAIN, "Failed to start sensor task");
            return;
        }
    }
    

    


    // Initialize ESP-NOW handlers
    if (!motorManager->init(imu, usbController))
    {
        ESP_LOGE("MAIN", "MotorManager init failed!");
        return;
    }

    // ---- 5. Application tasks (pinned for deterministic timing) ------------
    //
    // Priority hierarchy:
    //   IMU sensor task (created by imu->startSensorTask)  prio 5, core 1
    //   MotorManager (PID consumer)                         prio 4, core 1
    //   EspNowHandler (radio + telemetry)                   prio 3, core 0
    //
    // The PID task at prio 4 = (IMU prio - 1) ensures:
    //   - On DATA_READY ISR, the IMU task always wins first  ->  publishes
    //     a fresh snapshot.
    //   - As soon as it sleeps on its INT wait, the PID consumes the
    //     snapshot via waitForNewSample()  ->  PID locked to IMU rate.
    //
    // EspNowHandler stays on core 0 with Wi-Fi/lwIP so radio ISRs never
    // jitter the real-time stack on core 1.
    xTaskCreatePinnedToCore([](void *)
                { motorManager->Task(); },
                "MotorManagerTask",
                4096,
                &motorManager,
                4,                      // prio: IMU sensor (5) - 1
                nullptr,
                APP_CPU_NUM);           // core 1 (real-time)

    xTaskCreatePinnedToCore([](void *)
                { espNowHandler->Task(); },
                "EspNowHandlerTask",
                4096,
                &espNowHandler,
                3,                      // prio: below PID
                nullptr,
                PRO_CPU_NUM);           // core 0 (with Wi-Fi/lwIP)
}
