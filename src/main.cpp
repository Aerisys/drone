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
    // Default Config for now: intPin=GPIO_NUM_NC (polling 100 Hz fallback),
    // taskCoreId=tskNO_AFFINITY. Task B2 will wire INT to GPIO 19 and pin
    // the sensor task to APP_CPU_NUM for 1 kHz operation.
    imu->setFilterMode(MPU9250::MAHONY);

    MPU9250::Config imuCfg;
    // (default values — see imu-lib README; intPin/taskCoreId tuned in B2)
    err = imu->init(i2cBus, imuCfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MAIN, "Failed to initialize MPU9250 (err=%d)", err);
        // In HIL mode the absence of physical sensor is expected and not fatal,
        // we still want the rest of the firmware (USB bridge, ESP-NOW) running.
    }

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

    // Initialize tasks
    xTaskCreate([](void *)
                { motorManager->Task(); },
                "MotorManagerTask", 4096, &motorManager, 5, nullptr);
    xTaskCreate([](void *)
                { espNowHandler->Task(); },
                "EspNowHandlerTask", 4096, &espNowHandler, 5, nullptr);
}
