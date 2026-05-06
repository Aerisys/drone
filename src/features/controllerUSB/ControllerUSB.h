#pragma once

#include "mpu9250.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <cstring>
/**
 * @brief Simple USB controller used in Hardware-in-the-Loop (HIL) mode.
 *
 * The class communicates over the USB serial port (UART0) with a host
 * application (e.g. Unity or a custom PC tool).  The bridge is the same one
 * used by the ESP32's console output so HIL mode will disrupt normal logs, but
 * it simplifies wiring on typical development boards. The protocol is text-based:
 *
 *   - Orientation updates are received from the host as lines starting with
 *     "O:" followed by three floats (roll pitch yaw) separated by spaces.
 *     Example: "O: -4.56 1.23 78.9\n". Note: Unity sends roll before pitch.
 *   - Motor commands are sent to the host one motor at a time using a
 *     line of the form "M<index>:<speed>\n".  `<index>` is the motor number
 *     (0–3) and `<speed>` is an unsigned integer value.  The host can collect
 *     these for its simulation. */
class ControllerUSB {
public:
    ControllerUSB();
    ~ControllerUSB();

    /**
     * @brief Initialise the USB controller (usually called once in setup).
     *
     * @param baudRate Serial baud rate, default 115200.
     * @return true if initialisation looks ok (always true currently).
     */
    bool init(uint32_t baudRate = 115200);

    /**
     * @brief Return the most recently received orientation vector.
     *
     * Calling this method will also attempt to read any pending packet from
     * the host so the returned value is always up-to-date.
     */
    MPU9250::Orientation getOrientation();

    /**
     * @brief Send the current motor speeds to the host.
     *
     * The motorSpeeds array is expected to have at least `numMotors` entries.
     * The method simply writes a human-readable line to Serial; the host can
     * parse it on the other end.
     */
    void setData(int index, uint32_t motorSpeed);

    /**
     * @brief Send one-line control telemetry for host-side debugging/plotting.
     *
    * Format:
    * T:sp=<..> sr=<..> sy=<..> st=<..> tp=<..> tr=<..> ty=<..> mcp=<..> mcr=<..> mcy=<..> or=<..> op=<..> oy=<..>\n
     */
    void sendTelemetry(float stickPitch,
                       float stickRoll,
                       float stickYaw,
                       float stickThrottle,
                       float targetPitchAngle,
                       float targetRollAngle,
                       float targetYawRate,
                       float motorCorrectionPitch,
                       float motorCorrectionRoll,
                       float motorCorrectionYaw,
                       float orientationRoll,
                       float orientationPitch,
                       float orientationYaw);

private:
    // read a line from UART0 into the provided buffer, returns true if a
    // complete line (terminated by '\n') was received.
    bool readLine(char *buf, size_t maxLen);

    char _uartBuffer[256];         // accumulate incoming UART data
    size_t _uartBufferLen = 0;    // current position in buffer

private:
    void readOrientationPacket();

    MPU9250::Orientation _orientation;
};
