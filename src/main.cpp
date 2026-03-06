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

EspNowHandler *espNowHandler;
MotorManager *motorManager;
MPU9250 *imu;

static const char *TAG_MAIN = "MAIN";

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize ESP-NOW
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

    // Initialize MPU9250
    imu->setFilterMode(MPU9250::MAHONY);

    esp_err_t err = imu->init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22); // Set appropriate SDA/SCL pins
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_MAIN, "Failed to initialize MPU9250");
        //return;
    }

    if(!motorManager->modeHIL){
        err = imu->calibrate();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_MAIN, "Failed to start calibration");
            //return;
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
