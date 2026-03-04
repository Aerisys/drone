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
    size_t len = 0;
    while (len + 1 < maxLen) {
        uint8_t ch;
        int r = uart_read_bytes(UART_NUM, &ch, 1, pdMS_TO_TICKS(10));
        if (r <= 0) {
            break; // no data available right now
        }
        buf[len++] = static_cast<char>(ch);
        if (ch == '\n') {
            buf[len] = '\0';
            return true;
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
            }
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
    uart_write_bytes(UART_NUM, buf, off);
}
