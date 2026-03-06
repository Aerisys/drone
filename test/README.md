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
python3 controller_usb_simulator.py --port /dev/ttyUSB0 [--baud 115200] [--interval 0.01] [-v] [--log logfile.txt]
```

Replace `/dev/ttyUSB0` with the appropriate serial port name on your host.

**Options:**

- `--interval` : Controls how frequently orientation data is sent (default 10 ms).
- `-v` / `--verbose` : Show all serial data being transmitted; useful for debugging the HIL protocol.
- `--log FILE` : Save all sent/received data to a file for later analysis.

**Example with logging:**

```bash
python3 controller_usb_simulator.py --port /dev/ttyUSB0 -v --log hil_session.log
```

### Extension

You can modify the script to send deterministic orientation sequences,
log data to a file, or integrate with other test rigs. It’s primarily
intended as a lightweight substitute for the Unity side of the HIL demo.
### Troubleshooting

If you see "Opened /dev/ttyUSB0 at 115200 baud" but no data appears:

1. **Verify the ESP32 is running in HIL mode:**
   - Check `src/main.cpp` - ensure `motorManager = new MotorManager(true);` is set.
   - Upload the latest firmware with `platformio run --target upload`.

2. **Try verbose mode:**
   ```bash
   python3 controller_usb_simulator.py --port /dev/ttyUSB0 -v
   ```
   This will print every byte sent and received.

3. **Check the USB connection:**
   - Verify the cable is working: `ls -la /dev/ttyUSB*`
   - Try `minicom -D /dev/ttyUSB0 -b 115200` to open a raw serial monitor.

4. **Check firmware logs:**
   - Run `platformio device monitor` to see ESP32 boot messages and logs.
   - You should see "ControllerUSB" messages if HIL mode initializes correctly.