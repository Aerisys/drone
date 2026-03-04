# Test Utilities

This directory contains helper scripts used for testing the HIL protocol
between the ESP32 firmware and a host application (e.g. Unity).

## controller_usb_simulator.py

This Python script emulates the behaviour of the host side of the
`ControllerUSB` protocol, allowing you to exercise the firmware without
running a full Unity project.

### Features

* Sends random orientation packets over serial at a configurable interval.
* Receives `M:` motor speed messages from the device and prints them to the
  console.
* Simple and self-contained; uses only `pyserial`.

### Requirements

```bash
pip install pyserial
```

### Usage

```bash
python3 controller_usb_simulator.py --port /dev/ttyUSB0 [--baud 115200] [--interval 0.01]
```

Replace `/dev/ttyUSB0` with the appropriate serial port name on your host.

The `--interval` option controls how frequently orientation data is sent
(default 10 ms).

### Extension

You can modify the script to send deterministic orientation sequences,
log data to a file, or integrate with other test rigs. It’s primarily
intended as a lightweight substitute for the Unity side of the HIL demo.
