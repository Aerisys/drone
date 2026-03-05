#pragma once

#include "mpu9250.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <cstring>
/**
 * @brief Simple USB controller used in Hardware-in-the-Loop (HIL) mode.
 *
 * The class communicates over UART2 (GPIO16=RX, GPIO17=TX) with a host
 * application (e.g. Unity or a custom PC tool). This avoids interfering with
 * the system logs on UART0. The protocol is text-based:
 *
 *   - Orientation updates are received from the host as lines starting with
 *     "O:" followed by three floats (pitch roll yaw) separated by spaces.
 *     Example: "O: 1.23 -4.56 78.9\n".
 *   - Motor commands are sent to the host by writing a line beginning with
 *     "M:" and the four motor speed values as floats. The host can then
 *     feed this information back into a simulation.
 */
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
    void setData(const float motorSpeeds[], int numMotors);

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
