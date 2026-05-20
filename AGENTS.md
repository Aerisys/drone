# drone — Firmware drone ESP32 (Aerisys)

Firmware du **quadricoptère** : reçoit les commandes manette via
ESP-NOW, lit l'IMU MPU9250, exécute une **double boucle PID en cascade
(angle → rate)**, applique le **mixer X**, et pilote 4 ESC en PWM.
Possède un **mode HIL** dans lequel l'IMU est simulée par Unity via USB.

## Stack

- **Plateforme** : ESP32 DevKit (`esp32dev`, 240 MHz, flash 40 MHz)
- **Framework** : ESP-IDF via PlatformIO (`espressif32 @ 6.10.0`)
- **Langage** : C++ (FreeRTOS, MCPWM driver, I2C driver)
- **Dépendances** (cf. [platformio.ini](platformio.ini)) :
  - `github.com/Aerisys/esp-lib` (DTOs partagés ESP-NOW)
  - `github.com/Aerisys/imu-lib` (driver MPU9250)

## Commandes utiles

```powershell
pio run                                # build
pio run -t upload                      # flash USB
pio device monitor -b 115200           # logs
pio run -t clean                       # clean

# Test HIL sans Unity :
python test\controller_usb_simulator.py --port COM5 -v
```

## Architecture

```
app_main (main.cpp)
  ├── nvs / netif / esp_event init
  ├── EspNowHandler (gère pairing + RX/TX manette ESP-NOW)
  ├── MPU9250 imu  (init I2C, calibration, sensor task — sauté en HIL)
  ├── MotorManager (modeHIL boolean — sélectionne IMU réelle ou USB)
  └── ControllerUSB (instancié seulement si modeHIL)

Tâches FreeRTOS principales (toutes prio 5 sauf espNow prio 1) :
  - "MotorManagerTask"  : boucle PID + mixer + écriture MCPWM à ~100 Hz
  - "EspNowHandlerTask" : pairing, ping, traitement des paquets reçus
  - "sensor_task"       : lecture I2C IMU + filtre (Mahony par défaut)
```

Le **flag HIL** est dans le constructeur `MotorManager(bool modeHIL)`
appelé dans [src/main.cpp](src/main.cpp). Quand `modeHIL = true` :

1. L'IMU physique **n'est pas calibrée ni démarrée**.
2. Un `ControllerUSB` est créé et l'orientation provient d'UART0.
3. Les motor speeds sont **envoyés sur UART0** au lieu de piloter MCPWM ?
   → en réalité MCPWM est tout de même configuré ; vérifier les ESC
   physiquement débranchés en HIL.

## Boucle de contrôle (MotorManager)

Voir [src/features/motorManager/MotorManager.h](src/features/motorManager/MotorManager.h) — résumé :

```
stick (pitch/roll/yaw/throttle, normalisés ±1)
  │
  ▼
[OUTER LOOP — Angle PID]  (kp=4.5, ki=0.08, kd=0.05 pitch & roll)
  consigne_angle → rate_target
  │
  ▼
[INNER LOOP — Rate PID]   (kp=0.15, ki=0.05, kd=0.002 pitch & roll)
  rate_target − gyro → motor_correction
  │
  ▼
[Mixer X]
  M0 = base + pitch + roll − yaw   (Front-Left,  CCW)
  M1 = base + pitch − roll + yaw   (Front-Right, CW)
  M2 = base − pitch − roll − yaw   (Rear-Right,  CCW)
  M3 = base − pitch + roll + yaw   (Rear-Left,   CW)
  │
  ▼
PWM ESC 50 Hz, pulse 1000–2000 µs via MCPWM
```

Constantes à respecter :
- `CONTROL_LOOP_HZ = 100`
- `MAX_ROLL_ANGLE_DEG = MAX_PITCH_ANGLE_DEG = 45°`
- `MAX_YAW_RATE_DEG_S = 180`
- `THRUST_HEADROOM_RATIO = 0.15` (réserve 15 % pour corrections d'attitude)
- `THROTTLE_DEADZONE = 0.02` (désarme sous 2 %)
- `DT_MIN = 5 ms`, `DT_MAX = 20 ms` (sécurité dérivée)

## Pins matérielles

| Fonction         | GPIO        | Notes                          |
| ---------------- | ----------- | ------------------------------ |
| ESC M0 Avant-Gauche | GPIO26   | MCPWM, 50 Hz, 1-2 ms           |
| ESC M1 Avant-Droit  | GPIO25   |                                |
| ESC M2 Arrière-Droit| GPIO33   |                                |
| ESC M3 Arrière-Gauche| GPIO32  |                                |
| I2C SDA (MPU9250)| GPIO21      | I2C_NUM_0                      |
| I2C SCL (MPU9250)| GPIO22      |                                |
| LED pairing      | GPIO2       |                                |
| Bouton pairing   | GPIO16      | appui long 5 s = reset pairing |
| UART0 (HIL)      | TX0/RX0     | partagé avec logs ESP-IDF !    |

## Protocole HIL (USB)

Documenté en détail dans [src/features/controllerUSB/README.md](src/features/controllerUSB/README.md).
Résumé :

| Sens         | Format                                                         |
| ------------ | -------------------------------------------------------------- |
| Hôte → drone | `O: <pitch> <roll> <yaw>\n` (degrés, floats, séparés par espace) |
| Hôte → drone | `STOP\n` (latché), `ARM\n` (clear le stop)                     |
| Drone → hôte | `M<i>:<speed>\n` (i=0..3, speed = ticks µs offset depuis 1000) |
| Drone → hôte | `T:sp=.. sr=.. sy=.. st=.. tp=.. tr=.. ty=.. mcp=.. mcr=.. mcy=.. or=.. op=.. oy=..\n` (telemetry) |

> Attention : UART0 est aussi le canal des `ESP_LOG`. Toute trace lib
> brouille le parsing côté hôte. Mettre `esp_log_level_set("*", ESP_LOG_NONE)`
> ou compiler avec `CONFIG_LOG_DEFAULT_LEVEL_NONE` quand on flashe une
> build HIL pour Unity en prod.

## ESP-NOW (RX)

- Même protocole que le `controller`. La manette envoie un
  `ControllerRequestData` (cf. `esp-lib/include/ControllerRequestDTO.h`).
- Pairing : mêmes magic strings que la manette
  (`AERISYS_DRONE_PAIR` / `PAIR_CONFIRM`). MAC peer en NVS.
- `EspNowHandler::pingLost` + `lastPingTimeUs` permettent de détecter
  une perte de signal manette → idéal pour cut motors / mode failsafe
  (à brancher si pas encore fait).

## Conventions / pièges

- **Toujours** désarmer avant changement de mode HIL/normal.
- **Ne pas** initialiser l'IMU en HIL (la sensor task spammerait des
  reads I2C inutiles et le scheduler partagerait du temps avec UART0).
- Le **mapping moteur** Front-Left/Front-Right/Rear-Right/Rear-Left
  est en miroir entre firmware et Unity — vérifier
  [Unity/Assets/Scripts/DroneController.cs](../Unity/Assets/Scripts/DroneController.cs)
  `CalculateMotorPoints()` avant d'inverser quoi que ce soit ici.
- PID : pour retuner, modifier les `static constexpr float` dans
  [src/features/motorManager/MotorManager.h](src/features/motorManager/MotorManager.h)
  (les valeurs en commentaire sont des points de départ pour vol
  intérieur, châssis Skylab).
- Reset PID : `PidManager::reset()` doit être appelé à chaque réarme
  pour éviter l'**integral windup**.
- Buffer telemetry CSV côté Unity dans `Application.persistentDataPath`.

## Fichiers clés

- [src/main.cpp](src/main.cpp) — démarrage tâches et init mode HIL
- [src/features/motorManager/](src/features/motorManager/) — PID cascade + mixer + MCPWM
- [src/features/PidManager/](src/features/PidManager/) — implémentation PID générique
- [src/features/espNowHandler/](src/features/espNowHandler/) — RX manette + pairing
- [src/features/controllerUSB/](src/features/controllerUSB/) — bridge HIL UART
- [test/controller_usb_simulator.py](test/controller_usb_simulator.py) — simulateur Unity en Python (test HIL sans moteur Unity)

## Tests

Pas de tests unitaires C++. Le seul outil de test est le script Python
HIL : ouvre un port série, envoie des orientations aléatoires lissées
et logue les `M<i>:<speed>` reçus.
