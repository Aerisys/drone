#ifndef ESP_NOW_HANDLER_H
#define ESP_NOW_HANDLER_H

#include "features/motorManager/MotorManager.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <string.h>
#include <nvs_flash.h>
#include <mpuDTO.h>

#define TAG_ESP_NOW "ESP_NOW"

#define PIN_LED_ASSOCIATION GPIO_NUM_2

#define PIN_BUTTON_ASSOCIATION GPIO_NUM_16

class EspNowHandler
{
public:
    EspNowHandler();
    ~EspNowHandler();

    bool init(MPU9250 *imu);
    void send_data(const ControllerRequestData &requestData);
    void send_data(const mpuDTO &mpuData);
    void send_ping();
    void Task();

    static bool pingLost;
    static int64_t lastPingTimeUs;
    static int64_t lastToggleTimeUs;
    
    // Pointeur statique pour accéder à l'instance depuis le callback C
    static EspNowHandler* instance; 

private:
    MPU9250 *imu = nullptr;
    static void IRAM_ATTR button_isr_handler_pairing(void *arg);
    volatile bool buttonPressedPairing = false;

    ControllerRequestDTO lastControllerRequestDTO;
    
    // Correction : déclaration simple sans nom de classe
    uint8_t peer_mac[6] = {0}; 

    bool _associationMode = false;
    int64_t lastAssociationBroadcast = 0; // Correction syntaxe
    int64_t lastSendData= 0; 

    bool loadPeerMacFromNvs();
    bool savePeerMacToNvs();
    void broadcastAssociationRequest();
    void updateAssociationLed();
    void resetAssociation();

    bool currentLedState = false;

    // Méthode statique pour le callback de réception
    static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
    static void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status);
};

#endif // ESP_NOW_HANDLER_H