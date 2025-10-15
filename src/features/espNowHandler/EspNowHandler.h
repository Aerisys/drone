#ifndef ESP_NOW_HANDLER_H
#define ESP_NOW_HANDLER_H

#include "features/motorManager/MotorManager.h"

#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <string.h>
#include <nvs_flash.h>

#define TAG_ESP_NOW "ESP_NOW"

#define ESP_DRONE_MAC {0x20, 0x43, 0xA8, 0x66, 0x43, 0xC8}

class EspNowHandler
{
public:
    EspNowHandler();
    ~EspNowHandler();

    bool init();
    void send_data(const ControllerRequestData &requestData);

private:
    ControllerRequestDTO lastControllerRequestDTO;
    static uint8_t peer_mac[6];
};

#endif // ESP_NOW_HANDLER_H
