# ControllerUSB Protocol and Unity Integration 📡

This document explains how the `ControllerUSB` class communicates with a
host application (e.g. Unity) while running the drone firmware in
**Hardware‑In‑The‑Loop (HIL)** mode. The protocol is intentionally minimal
and text‑based so it can be easily implemented on both ends.

---

## Overview

- The ESP32 runs the standard drone code but with IMU readings replaced by
  orientation data sent from the host over the USB serial line.
- In return, motor speed commands computed by the flight controller are
  brushed back to the host, allowing a physics simulator (such as Unity) to
  apply forces and update orientation.

The serial interface used is the **USB‑to‑UART bridge on UART0**, which is
exposed to the host as a virtual COM port.  On a PC, open the port at
115200 baud (default) to exchange messages.

---

## Packet Formats

### Orientation (host → ESP32)

The host must send a line terminated by `\n` with this structure:

```
O: <pitch> <roll> <yaw>
```

- `O:` is the packet prefix.
- Each value is a floating‑point number (e.g. `1.23` or `-45.6`).
- Pitch/roll/yaw are measured in degrees and follow the same conventions
  used by `MPU9250::Orientation`.

Example sent from Unity:

```csharp
serialPort.WriteLine("O: 0.0 0.0 90.0");
```

On receipt the firmware updates its orientation used by the PID controller.


### Motor Speeds (ESP32 → host)

The firmware sends individual motor speed updates to the host. Each packet
is a single line terminated by `\n` in the form:

```
M<index>:<speed>
```

- `M` prefix denotes a motor command.
- `<index>` is the zero-based motor number (0..3 in a quadcopter).
- `<speed>` is the unsigned integer value written by
  `ControllerUSB::setData()` (typically the pulse‑width offset from
  `MIN_PULSE_TICKS`).

This design lets the host receive the four motor values one at a time, which
matches the way the firmware updates each motor internally. The host can
store or convert them to forces/RPM as required.

Example parsing code in Unity:

```csharp
string line = serialPort.ReadLine();
if (line.Length > 2 && line[0] == 'M') {
    // format: M0:12345
    int colon = line.IndexOf(':');
    if (colon > 1) {
        int motorIndex = int.Parse(line.Substring(1, colon-1));
        float speed = float.Parse(line.Substring(colon+1));
        motorSpeeds[motorIndex] = speed;
    }
}
```


## Unity Integration Tips

1. **Opening the serial port:**
   - Use `System.IO.Ports.SerialPort` or a plugin if running in WebGL.
   - Configure `baudRate = 115200`, `parity = None`, `stopBits = One`.

2. **Timing:**
   - Read orientation packets in `Update()` or a dedicated thread.
   - Write motor speed packets whenever new values are available from the
     simulation (drone physics step).
   - The controller on the ESP runs at 100 Hz (`vTaskDelay(pdMS_TO_TICKS(10))`),
     so earlier/later updates will simply be ignored until the next iteration.

3. **Error handling:**
   - The firmware ignores malformed lines; however the host should ensure
     proper formatting to avoid desynchronisation.
   - If the host stops sending orientation, the last known orientation will
     be reused (no smoothing).

4. **Debugging:**
   - Use `screen`/`minicom` on the host to monitor raw data during early
     testing.
   - Add simple GUI sliders in Unity to manually send orientation values when
     debugging the physical model.


## Expanding the Protocol

The current design is easily extended.  For example, you may add:

- A timestamp field to each packet for latency measurement.
- A heartbeat message (`H:`) from either side to detect disconnects.
- Binary framing with length prefixes if performance becomes critical.

However, keep compatibility with the simple text format used by
`ControllerUSB` in the firmware.

---

**That's it!** With this setup you can run the flight controller on the
actual ESP32 hardware while simulating sensor input and motor behaviour in
Unity or another desktop application. Enjoy the HIL workflow! 🚁
