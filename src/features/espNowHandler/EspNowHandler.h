#ifndef ESP_NOW_HANDLER_H
#define ESP_NOW_HANDLER_H

#include "features/motorManager/MotorManager.h"

#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <string.h>
#include <nvs_flash.h>

#define TAG_ESP_NOW "ESP_NOW"

#define ESP_DRONE_MAC {0xf8, 0xb3, 0xb7, 0x20, 0x38, 0xac}

class EspNowHandler
{
public:
    EspNowHandler();
    ~EspNowHandler();

    bool init();
    void send_data(const ControllerRequestData &requestData);
    void send_ping();
    void Task();

    static bool pingLost;
    static int64_t lastPingTimeUs;

private:
    ControllerRequestDTO lastControllerRequestDTO;
    static uint8_t peer_mac[6];

    
};

#endif // ESP_NOW_HANDLER_H
