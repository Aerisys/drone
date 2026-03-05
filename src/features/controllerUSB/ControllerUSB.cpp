#include "features/controllerUSB/ControllerUSB.h"
#include "esp_log.h"

static const char *TAG = "ControllerUSB";
static constexpr uart_port_t UART_NUM = UART_NUM_0;

ControllerUSB::ControllerUSB()
    : _orientation{0.0f, 0.0f, 0.0f}
{
}

ControllerUSB::~ControllerUSB()
{
}

bool ControllerUSB::init(uint32_t baudRate)
{
    // configure UART0 for simple line-based communication.  On most ESP32
    // boards the USB-to-UART bridge is attached to UART0 by default, so this
    // gives us access to the serial monitor.
    uart_config_t uart_config = {};
    uart_config.baud_rate = static_cast<int>(baudRate);
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_driver_install(UART_NUM, 1024, 1024, 0, nullptr, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "USB controller initialized at %u baud", (unsigned)baudRate);
    return true;
}

MPU9250::Orientation ControllerUSB::getOrientation()
{
    readOrientationPacket();
    return _orientation;
}

bool ControllerUSB::readLine(char *buf, size_t maxLen)
{
    // Read available bytes from UART into the accumulation buffer
    size_t available = uart_read_bytes(UART_NUM, (uint8_t*)_uartBuffer + _uartBufferLen, 
                                        sizeof(_uartBuffer) - _uartBufferLen - 1, 
                                        pdMS_TO_TICKS(50));
    
    if (available > 0) {
        _uartBufferLen += available;
        _uartBuffer[_uartBufferLen] = '\0'; // null-terminate for debugging
    }

    // Look for a complete line (newline character)
    for (size_t i = 0; i < _uartBufferLen; i++) {
        if (_uartBuffer[i] == '\n') {
            // Found a newline - extract the line
            size_t lineLen = i + 1;  // include the newline
            if (lineLen < maxLen) {
                memcpy(buf, _uartBuffer, lineLen);
                buf[lineLen] = '\0';
                
                // Shift remaining data in buffer
                memmove(_uartBuffer, _uartBuffer + lineLen, _uartBufferLen - lineLen);
                _uartBufferLen -= lineLen;
                
                ESP_LOGV(TAG, "RX line: %s", buf);
                return true;
            }
            break;
        }
    }
    
    return false;
}

void ControllerUSB::readOrientationPacket()
{
    char line[128];
    if (readLine(line, sizeof(line))) {
        if (strncmp(line, "O:", 2) == 0) {
            float p, r, y;
            int matched = sscanf(line + 2, "%f %f %f", &p, &r, &y);
            if (matched == 3) {
                _orientation.pitch = p;
                _orientation.roll = r;
                _orientation.yaw = y;
                ESP_LOGD(TAG, "Orientation RX: pitch=%.2f roll=%.2f yaw=%.2f", p, r, y);
            } else {
                ESP_LOGW(TAG, "Failed to parse orientation from: %s", line);
            }
        } else {
            ESP_LOGW(TAG, "Unknown packet type: %s", line);
        }
    }
}

void ControllerUSB::setData(const float motorSpeeds[], int numMotors)
{
    // build a single string and write it so output is atomic
    char buf[128];
    int off = snprintf(buf, sizeof(buf), "M:");
    for (int i = 0; i < numMotors; ++i) {
        off += snprintf(buf + off, sizeof(buf) - off, " %.4f", motorSpeeds[i]);
    }
    off += snprintf(buf + off, sizeof(buf) - off, "\n");
    uart_write_bytes(UART_NUM, (const char*)buf, off);
    ESP_LOGD(TAG, "Motor TX: %s", buf);
}
