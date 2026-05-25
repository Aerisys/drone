# Skylab Drone Firmware

> Firmware ESP32 (ESP-IDF / PlatformIO) d'un drone quad X — contrôle moteurs MCPWM, double-boucle PID (angle + rate), IMU MPU9250 à 1 kHz INT-driven, liaison radio ESP-NOW avec failsafes, mode HIL via USB pour simulation Unity.

## Stack

- **MCU** : ESP32 (DevKit / NodeMCU)
- **Framework** : ESP-IDF via PlatformIO
- **Langage** : C++17
- **Dépendances libs** :
  - [`imu-lib`](https://github.com/Aerisys/imu-lib) v1.1+ — driver MPU9250 + AK8963, Mahony 9-DOF, INT-driven, seqlock atomique
  - [`esp-lib`](https://github.com/Aerisys/esp-lib) v1.1+ — DTOs ESP-NOW (POD, zero heap) partagés contrôleur ↔ drone

## Fonctionnalités

- **Acquisition IMU 1 kHz INT-driven** (MPU9250 + AK8963), Mahony 9-DOF avec quaternion exposé
- **Boucle de contrôle 1 kHz lockstep** : PID en cascade *angle → rate → motor mixer X*
- **4 × ESC PWM 50 Hz** via MCPWM ESP-IDF (synchronisation 2 groupes timer)
- **Liaison radio ESP-NOW** :
  - Pairing dynamique avec MAC persisté NVS, bouton long-press pour reset
  - LED visuelle d'état d'association
  - Ping keepalive 0.5 Hz
- **Failsafes en cascade** :
  - Radio failsafe : ping perdu > 2 s → auto-disarm latché
  - IMU failsafe : `isSensorHealthy() == false` → auto-disarm latché
  - Calibration gate : `armMotors()` refuse si IMU non calibré
- **Mode HIL** : bridge UART0 vers Unity, orientation simulée + télémétrie texte
- **Pinning multi-core** : real-time stack sur core 1, networking sur core 0

## Matériel requis

| Élément | Quantité | Notes |
|---|---|---|
| ESP32 DevKit (ou équivalent) | 1 | 240 MHz, dual-core requis |
| **MPU9250** (GY-9250 / GY-91) | 1 | Pull-ups I²C 4.7 kΩ onboard pour Fast Mode 400 kHz |
| ESC brushless (PWM 1-2 ms / 50 Hz) | 4 | Calibrés pour la plage 1000-2000 µs |
| Moteurs brushless + hélices | 4 | Config X quadrirotor |
| LiPo + alim 5 V/3.3 V | 1 | Selon BEC ESC |
| LED + résistance | 1 | Indicateur association ESP-NOW |
| Bouton poussoir | 1 | Reset pairing radio (long press) |

## Brochage ESP32 → périphériques

| Périphérique | GPIO | Fonction |
|---|---|---|
| MPU9250 SDA | `21` | I²C Data |
| MPU9250 SCL | `22` | I²C Clock |
| MPU9250 INT | `19` | DATA_READY interrupt (1 kHz) |
| ESC Front-Left (M0) | `26` | MCPWM group 0 |
| ESC Front-Right (M1) | `25` | MCPWM group 0 |
| ESC Rear-Right (M2) | `33` | MCPWM group 1 |
| ESC Rear-Left (M3) | `32` | MCPWM group 1 |
| LED association | `2` | Output, blink en pairing |
| Bouton reset pairing | `16` | Input pull-up, long press 5 s |

## Architecture logicielle

```
src/
├── main.cpp                          # bootstrap : NVS, I²C bus, IMU, tasks
└── features/
    ├── motorManager/                 # PID cascade + MCPWM + failsafes
    │   ├── MotorManager.h
    │   └── MotorManager.cpp
    ├── espNowHandler/                # ESP-NOW radio + pairing + ping
    │   ├── EspNowHandler.h
    │   └── EspNowHandler.cpp
    ├── controllerUSB/                # HIL bridge UART0 ↔ Unity
    │   ├── ControllerUSB.h
    │   └── ControllerUSB.cpp
    └── PidManager/                   # PID basique avec anti-windup
        ├── PidManager.h
        └── PidManager.cpp

include/
├── DroneConstants.h                  # NUM_MOTORS (source unique)
└── TelemetryDTO.h                    # DTO télémétrie unifié IMU + motors
```

### Tasks FreeRTOS (pinning)

| Task | Priorité | Core | Source |
|---|---|---|---|
| **`mpu9250_task`** (IMU sensor) | 5 | **APP_CPU_NUM (1)** | `imu->startSensorTask()` via `Config::taskCoreId` |
| **`MotorManagerTask`** (PID + ESC) | 4 | **APP_CPU_NUM (1)** | `xTaskCreatePinnedToCore` dans `main.cpp` |
| **`EspNowHandlerTask`** (radio) | 3 | **PRO_CPU_NUM (0)** | `xTaskCreatePinnedToCore` dans `main.cpp` |
| Wi-Fi / lwIP (système) | défaut | **PRO_CPU_NUM (0)** | via `sdkconfig` |

**Principe** : core 1 = temps réel deterministe (IMU + PID), core 0 = networking. Aucune préemption croisée. Le PID consomme l'IMU en lockstep via `imu->waitForNewSample()` qui le wake immédiatement à chaque publication snapshot — pas de phase drift.

### Boucle de contrôle (PID cascade)

```
[stick input]
     │
     ▼
[outer PID angle]         (angle_target - measured_angle) → rate_target
     │ pitch/roll uniquement (yaw skip — stick → rate direct)
     ▼
[inner PID rate]          (rate_target - measured_rate) → motor_correction
     │ utilise s.gyro DIRECT (filtré DLPF MPU + optional notch)
     ▼
[X-config mixer]          throttle ± pitch ± roll ± yaw → 4 ESCs
     │
     ▼
[MCPWM 50 Hz]
```

## sdkconfig clés (déjà appliqués)

| Setting | Valeur | Raison |
|---|---|---|
| `CONFIG_FREERTOS_HZ` | **1000** | Tick 1 ms — résolution `vTaskDelay(pdMS_TO_TICKS(N))` à la ms |
| `CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0` | y | Wi-Fi confiné core 0 |
| `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0` | y | lwIP confiné core 0 |

## Mode HIL (Hardware-in-the-Loop)

Le mode HIL court-circuite l'IMU réel : orientation envoyée par Unity via UART0 (`O: roll pitch yaw\n`), motor outputs renvoyés (`M<idx>:<speed>\n`).

Activation : `motorManager = new MotorManager(true)` dans `main.cpp` (paramètre `modeHIL=true`).

Protocole détaillé : voir [`src/features/controllerUSB/README.md`](src/features/controllerUSB/README.md) et [`test/controller_usb_simulator.py`](test/controller_usb_simulator.py).

En mode HIL :
- Pas de calibration IMU (capteur virtuel)
- Pas de failsafe radio (pas de radio active)
- Boucle PID throttlée à 100 Hz (Unity ne ship pas de gyro → pas de lockstep possible)

## Installation

```bash
git clone https://github.com/SkylabYnov/skylab-drone.git
cd skylab-drone

# PlatformIO télécharge esp-lib et imu-lib automatiquement
pio run

# Flash + monitor
pio run -t upload && pio device monitor
```

Pour itérer localement sur `esp-lib` ou `imu-lib` sans pousser sur GitHub :

```ini
; platformio.ini
lib_deps =
    file://../esp-lib
    file://../imu-lib
```

## Tuning IMU (à régler par airframe)

Dans `main.cpp`, après l'init IMU :

```cpp
imu->setMahonyGains(1.0f, 0.0f);          // Kp ~1.0 OK pour drone ; Ki=0 anti-windup
imu->setSwitchRollPitch(false);            // true si pitch/roll inversés vs HUD
imu->setInvertAxis(false, false, false);   // flip par axe selon montage physique
```

Procédure (drone désarmé, télémétrie ouverte) :
1. Incline le drone sur un axe à la fois → vérifie que l'axe Euler correspondant change dans le bon sens
2. Si roll/pitch mélangés → `setSwitchRollPitch(true)`
3. Si un axe inversé (correction PID dans le mauvais sens) → flip le bool correspondant

Voir [`imu-lib`](https://github.com/Aerisys/imu-lib) pour les détails (calibration NVS auto, notch filter optionnel, T° comp gyro).

## Failsafes — comportement attendu

| Évènement | Action immédiate | Comment recover |
|---|---|---|
| Ping radio perdu > 2 s | `disarmMotors()` + latch `failsafeEngaged` | Attendre ping retour, puis user bouton arm (latch cleared) |
| `isSensorHealthy() == false` | `disarmMotors()` + latch | Reset hardware / vérif câblage I²C |
| Arm tenté pendant calibration | Refus + log "IMU not calibrated" | Attendre fin de calibration (10 s) |
| Tilt physique > 45° en vol | Throttle + corrections forcés à 0 (cut moteurs) | Remettre à plat, re-armer |
| Throttle < deadzone (~2 %) | Reset intégrales PID (anti-windup) | Auto, à chaque cycle |

## Commands utiles

```powershell
pio run                      # build
pio run -t upload            # flash
pio device monitor           # 115200 baud
pio run -t clean
pio pkg update               # met à jour esp-lib + imu-lib depuis GitHub

# Profiler IMU (debug timing — désactivé en prod)
pio run -t upload --project-option="build_flags=-DMPU9250_PROFILER=1"
```

## Pièges connus / TODO

- **Reset counter contrôleur** : si le contrôleur reboot, son `counter` repart à 0 → drone refuse les paquets jusqu'à dépasser l'ancien max (bug esp-lib, à traiter côté drone via timeout).
- **`setInvertAxis` ≠ swap axes** : les flips inversent les signes, pas les axes. Si le swap roll/pitch s'applique au gyro brut (consommé par le rate PID), modifier le mapping dans `MotorManager::Task` (lignes ~600).
- **Pas de tests unitaires** : `test/` ne contient que le simulateur HIL Python. À étoffer (PID, mixer, slew rate).
- **calibrateMag()** : non câblé à l'UX. À déclencher manuellement (long press bouton dédié) dans une future itération.

## Contribution

1. Fork
2. Branche `feature/<nom>` depuis `develop`
3. Commits atomiques, build vert à chaque étape
4. PR vers `develop`

## Licence

MIT. Voir [LICENSE](LICENSE).

## Contact

[skylab@ynov.com](mailto:skylab@ynov.com)
