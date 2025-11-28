#include "features/espNowHandler/EspNowHandler.h"
#include <PingRequestDTO.h>
#include <esp_timer.h>
#include <esp_mac.h>
#include <PairingPacket.h> // Assurez-vous que ce fichier existe
#include "EspNowHandler.h"
#include <mpuDTO.h>

// Initialisation des membres statiques
int64_t EspNowHandler::lastPingTimeUs = 0;
int64_t EspNowHandler::lastToggleTimeUs = 0;
bool EspNowHandler::pingLost = false;
EspNowHandler* EspNowHandler::instance = nullptr;

EspNowHandler::EspNowHandler() {
    instance = this; // Enregistre l'instance actuelle pour le callback
}

EspNowHandler::~EspNowHandler() {
    if (instance == this) instance = nullptr;
}

bool EspNowHandler::init(MPU9250 *imuTmp)
{
    this->imu = imuTmp;
    // Correction : une seule initialisation NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG_ESP_NOW, "Erreur d'init ESP-NOW");
        return false;
    }

    // Enregistrement des callbacks
    esp_now_register_recv_cb(EspNowHandler::onDataRecv);
    esp_now_register_send_cb(EspNowHandler::onDataSent);

    // Chargement du MAC
    loadPeerMacFromNvs(); 

    // Correction de la logique de vérification du MAC
    bool macKnown = false;
    for (int i = 0; i < 6; i++) {
        if (peer_mac[i] != 0) { // Si au moins un octet n'est pas 0, on suppose que c'est valide
            macKnown = true;
            break;
        }
    }

    _associationMode = !macKnown;

    if (macKnown) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, peer_mac, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            ESP_LOGE(TAG_ESP_NOW, "Erreur d'ajout du pair (Init)");
            _associationMode = true; // Si échec, on repasse en mode association
        } else {
            ESP_LOGI(TAG_ESP_NOW, "Pair connu ajouté : %02x:%02x:%02x:%02x:%02x:%02x", 
                     peer_mac[0], peer_mac[1], peer_mac[2], peer_mac[3], peer_mac[4], peer_mac[5]);
        }
    } else {
        ESP_LOGW(TAG_ESP_NOW, "Aucun pair connu, mode association activé");
    }

    ESP_LOGI(TAG_ESP_NOW, "ESP-NOW Initialisé");

    gpio_reset_pin(PIN_LED_ASSOCIATION); // Réinitialise l'état de la broche
    gpio_set_direction(PIN_LED_ASSOCIATION, GPIO_MODE_OUTPUT); // Définit la broche en mode Sortie
    gpio_set_level(PIN_LED_ASSOCIATION, 0);

    gpio_reset_pin(PIN_BUTTON_ASSOCIATION);
    gpio_set_direction(PIN_BUTTON_ASSOCIATION, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BUTTON_ASSOCIATION, GPIO_PULLUP_ONLY);


    return true;
}

// Fonction Callback de réception (Statique)
void EspNowHandler::onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    // Vérification que l'instance existe
    ESP_LOGI(TAG_ESP_NOW, "Données reçues, longueur: %d", len);
    if (instance == nullptr) return;

    if (len == sizeof(ControllerRequestData)) {  
        ControllerRequestData receivedData;
        memcpy(&receivedData, data, sizeof(receivedData));  

        ControllerRequestDTO controllerRequestDTO = ControllerRequestDTO::fromStruct(receivedData);

        if(controllerRequestDTO.flightController || controllerRequestDTO.buttonMotorArming || controllerRequestDTO.buttonMotorState){
            if (xSemaphoreTake(MotorManager::xControllerRequestMutex, portMAX_DELAY)) {
                MotorManager::currentControllerRequestDTO.addInControllerRequestDTO(controllerRequestDTO);
                xSemaphoreGive(MotorManager::xControllerRequestMutex);
            }
        }
    } 
    else if (len == sizeof(PingRequestDTO)) {
        PingRequestDTO ping;
        memcpy(&ping, data, sizeof(PingRequestDTO));
        ESP_LOGD(TAG_ESP_NOW, "Ping reçu"); // Changé en DEBUG pour éviter de spammer

        lastPingTimeUs = esp_timer_get_time();
        pingLost = false;
    }
    else if (len == sizeof(PairingPacket)) {
        // Logique d'appairage
        if (instance->_associationMode) {
            PairingPacket resp;
            memcpy(&resp, data, sizeof(resp));

            // NOUVELLE VÉRIFICATION : Comparer le champ 'magic'
            if (strncmp(resp.magic, RESP_MAGIC, sizeof(resp.magic)) != 0) {
                ESP_LOGW(TAG_ESP_NOW, "Paquet d'association rejeté : Magic Key incorrecte.");
                return; // Sortir immédiatement si la clé ne correspond pas
            }
            
            ESP_LOGI(TAG_ESP_NOW, "Paquet d'association reçu MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                    info->src_addr[0], info->src_addr[1], info->src_addr[2], info->src_addr[3], info->src_addr[4], info->src_addr[5]);

            memcpy(instance->peer_mac, info->src_addr, 6);
            instance->savePeerMacToNvs();

            // Vérifier si le peer existe déjà avant d'ajouter
            if (esp_now_is_peer_exist(instance->peer_mac)) {
                esp_now_del_peer(instance->peer_mac);
            }

            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, instance->peer_mac, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            
            if (esp_now_add_peer(&peerInfo) == ESP_OK) {
                instance->_associationMode = false;
                ESP_LOGI(TAG_ESP_NOW, "Association terminée avec succès !");
            } else {
                ESP_LOGE(TAG_ESP_NOW, "Echec ajout peer lors de l'association");
            }
        }
    }
    else {
        ESP_LOGE(TAG_ESP_NOW, "Taille incorrecte des données reçues ! Len: %d", len);
    } 
}

void EspNowHandler::onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
    // Optionnel : ne logger que les erreurs pour éviter de saturer la console
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG_ESP_NOW, "Échec envoi vers %02x:%02x:%02x:%02x:%02x:%02x (Statut : %d)", 
                 macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5], status);
    }
}

void EspNowHandler::send_data(const ControllerRequestData &requestData)
{
    if (_associationMode) return; // Ne pas envoyer si on n'est pas associé

    esp_now_send(peer_mac, (uint8_t *)&requestData, sizeof(requestData));
}

void EspNowHandler::send_data(const mpuDTO &mpuData)
{
    if (_associationMode) return; // Ne pas envoyer si on n'est pas associé

    esp_now_send(peer_mac, (uint8_t *)&mpuData, sizeof(mpuData));
}

void EspNowHandler::send_ping()
{
    if (_associationMode) return;

    PingRequestDTO ping = {true};
    esp_now_send(peer_mac, (uint8_t *)&ping, sizeof(ping));
}

void EspNowHandler::Task()
{
    while (true){
    
        updateAssociationLed();
        handleButtonPressLogic();
        // Broadcast Association
        if (_associationMode) {
            int64_t now = esp_timer_get_time();
            // Diffusion toutes les 1 seconde (1 000 000 us)
            if (now - lastAssociationBroadcast > 1000000LL) { 
                lastAssociationBroadcast = now;
                ESP_LOGI(TAG_ESP_NOW, "Broadcast Association Request...");
                broadcastAssociationRequest();
            }
        }

        // Gestion du timeout Ping
        int64_t now = esp_timer_get_time();
        if (lastPingTimeUs > 0 && !_associationMode) {
            float dtSec = (now - lastPingTimeUs) / 1000000.0f;
            if (dtSec > 2.0f && !pingLost) {
                ESP_LOGW(TAG_ESP_NOW, "Ping perdu ! (%.2fs)", dtSec);
                pingLost = true;
                // Optionnel : repasser en mode association si ping perdu trop longtemps ?
                // _associationMode = true; 
            }
        }

        if(!_associationMode){
            MPU9250::CalibrationStatus calibration = imu->getCalibrationStatus();
            if(calibration == MPU9250::CalibrationStatus::CALIBRATED){
                // Envoi périodique des données
                if (now - lastSendData > 20000LL) { // Toutes les 20 ms
                    lastSendData = now;

                    MPU9250::Vector3 accel = imu->getAccel();
                    MPU9250::Vector3 gyro = imu->getGyro();
                    MPU9250::Vector3 mag = imu->getMag();
                    MPU9250::Orientation orientation = imu->getOrientation();

                    mpuDTO dto;
                    dto.accel = accel;
                    dto.gyro = gyro;
                    dto.mag = mag;  
                    dto.orientation = orientation; 

                    send_data(dto);
                }
            }
            
        }
        
        // Pas de vTaskDelay ici si cette fonction est appelée dans une boucle externe qui gère déjà le timing.
        // Si c'est une FreeRTOS task dédiée, gardez le vTaskDelay.
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

bool EspNowHandler::loadPeerMacFromNvs() {
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READONLY, &handle) != ESP_OK) return false;
    size_t size = sizeof(peer_mac);
    esp_err_t err = nvs_get_blob(handle, "controller_mac", peer_mac, &size);
    nvs_close(handle);
    return err == ESP_OK;
}

bool EspNowHandler::savePeerMacToNvs() {
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READWRITE, &handle) != ESP_OK) return false;
    ESP_ERROR_CHECK(nvs_set_blob(handle, "controller_mac", peer_mac, sizeof(peer_mac)));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
    return true;
}

void EspNowHandler::broadcastAssociationRequest() {
    PairingPacket dto;
    memset(&dto, 0, sizeof(dto)); // Bonnes pratiques : on met tout à zéro d'abord

    // 1. Remplissage du Magic (doit correspondre au récepteur !)
    // On copie "MY_PAIRING" (max 12 octets)
    strncpy(dto.magic, REQ_MAGIC, sizeof(dto.magic)); 
    
    // 3. Configuration du Broadcast
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0; // 0 = utiliser le canal actuel du Wi-Fi
    peerInfo.encrypt = false;
    
    // On ajoute le peer broadcast s'il n'existe pas déjà
    if (!esp_now_is_peer_exist(broadcastAddress)) {
        esp_now_add_peer(&peerInfo);
    }

    // 4. Envoi
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&dto, sizeof(dto));
    
    if (result == ESP_OK) {
        ESP_LOGI(TAG_ESP_NOW, "Demande d'association envoyée (Broadcast)");
    } else {
        ESP_LOGE(TAG_ESP_NOW, "Erreur envoi association : %s", esp_err_to_name(result));
    }
}

// Fichier : EspNowHandler.cpp

void EspNowHandler::updateAssociationLed() {
    int64_t now = esp_timer_get_time();
    
    // --- Étape 1 : Vérification du Mode Association ---
    if (_associationMode) {
        int64_t timeElapsed = now - EspNowHandler::lastToggleTimeUs;
        if (timeElapsed > 200000LL) {             
            gpio_set_level(PIN_LED_ASSOCIATION, !currentLedState);
            
            currentLedState = !currentLedState;
            EspNowHandler::lastToggleTimeUs = now;
            
            ESP_LOGD(TAG_ESP_NOW, "lastToggleTimeUs mis à jour à %lld.", EspNowHandler::lastToggleTimeUs);

        } else {
            ESP_LOGD(TAG_ESP_NOW, "LED Association : Attente pour le prochain basculement.");
        }
        
    } else if(buttonPressed && !buttonLogPressedSucess){
        gpio_set_level(PIN_LED_ASSOCIATION, true);
        currentLedState = true;
    } else {
        currentLedState = false;
        gpio_set_level(PIN_LED_ASSOCIATION, currentLedState); 
        ESP_LOGD(TAG_ESP_NOW, "LED Association : Mode Inactif (Éteinte)."); 
    }
}

// Fichier : EspNowHandler.cpp

void EspNowHandler::resetAssociation() {
    // 1. Suppression de la MAC de la NVS
    // Note: Pour une suppression complète, il faut effacer la clé dans NVS
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_erase_key(handle, "controller_mac");
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG_ESP_NOW, "Ancienne MAC du pair effacée de la NVS.");
    }

    // 2. Suppression de tous les pairs ESP-NOW
    esp_now_del_peer(peer_mac); // Supprime l'ancien pair connu (si existant)

    // Optionnel : Effacer le reste de la liste des pairs (y compris l'adresse de broadcast si elle n'est plus nécessaire)
    esp_now_peer_info_t peerInfo = {};
    while (esp_now_fetch_peer(true, &peerInfo) == ESP_OK) {
        esp_now_del_peer(peerInfo.peer_addr);
    }
    ESP_LOGI(TAG_ESP_NOW, "Tous les pairs ESP-NOW existants ont été supprimés.");

    // 3. Réinitialiser la MAC locale (pour l'affichage futur)
    memset(peer_mac, 0, 6); 

    // 4. Activation du mode association
    _associationMode = true;
    lastAssociationBroadcast = 0; // Force le broadcast immédiat
}


void EspNowHandler::handleButtonPressLogic() {
    // 1. Lire l'état du GPIO pour voir si le bouton est pressé
    bool isPressedNow = gpio_get_level(PIN_BUTTON_ASSOCIATION) == 0; // Assumer Pull-up (0 = pressé)

    // 2. Détection d'appui initial
    if (isPressedNow && !buttonPressed && !_associationMode && !buttonLogPressedSucess) {
        buttonPressed = true;
        buttonLogPressedSucess = false;
        pressStartTime = esp_timer_get_time() / 1000; // Enregistrement en ms
        ESP_LOGI("BUTTON", "Button Pressed. Starting timer.");
    }
    // 3. Détection de maintien de l'appui
    else if (isPressedNow && buttonPressed && !buttonLogPressedSucess) {
        int64_t holdTime = (esp_timer_get_time() / 1000) - pressStartTime;
        
        if (holdTime >= LONG_PRESS_MS && !_associationMode) {
            buttonLogPressedSucess = true;
            ESP_LOGW("BUTTON", "Long Press Detected (%lld ms)! Entering Pairing Mode.", holdTime);
            resetAssociation();
        }
    }
    // 4. Détection de relâchement
    else if (!isPressedNow && buttonPressed) {
        // Réinitialisation de l'état
        buttonLogPressedSucess = false;
        buttonPressed = false;
        pressStartTime = 0;
    }
}