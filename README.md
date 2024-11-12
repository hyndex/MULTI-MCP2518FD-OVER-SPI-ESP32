# MCP2518FD Driver Library for ESP32

This is a production-ready driver library for the MCP2518FD CAN FD controller, designed for use with the ESP32 microcontroller and the ESP-IDF framework. It provides a high-level API for initializing the device, configuring CAN settings dynamically, and sending/receiving CAN messages with support for CAN FD features.

## **Features**

- **Dynamic Configuration**: Configure CAN parameters at runtime, including bit rates, operation modes, and filters.
- **High-Level API**: Simplifies message transmission and reception with easy-to-use functions.
- **CAN FD Support**: Supports both Classical CAN and CAN FD frames.
- **Interrupt-Driven Operation**: Efficient message handling using interrupts.
- **Edge Case Handling**: Comprehensive error checking and handling for robust operation.
- **Production Ready**: Suitable for time-sensitive applications like PLCs and power modules.

## **Table of Contents**

- [Installation](#installation)
- [Getting Started](#getting-started)
  - [Hardware Setup](#hardware-setup)
  - [Software Setup](#software-setup)
- [API Overview](#api-overview)
- [Usage Example](#usage-example)
- [License](#license)

## **Installation**

1. **Clone the Repository**

   ```bash
   git clone https://github.com/yourusername/mcp2518fd-esp32.git
   ```

2. **Copy the Library**

   Copy the `mcp2518fd.h` and `mcp2518fd.c` files into your ESP-IDF project's `components` directory or include them directly in your project.

3. **Update CMakeLists.txt**

   If you're using CMake, ensure that your project's `CMakeLists.txt` includes the library:

   ```cmake
   idf_component_register(SRCS "main.c" "mcp2518fd.c"
                          INCLUDE_DIRS ".")
   ```

## **Getting Started**

### **Hardware Setup**

- **ESP32 Pins**: Connect the MCP2518FD to the ESP32 via SPI. Here’s an example pin configuration:

  | MCP2518FD | ESP32       |
  |-----------|-------------|
  | SCK       | GPIO 18     |
  | SI (MOSI) | GPIO 23     |
  | SO (MISO) | GPIO 19     |
  | CS        | GPIO 5      |
  | INT       | GPIO 4      |
  | VDD       | 3.3V        |
  | VSS       | GND         |
  | Other Pins| As required |

- **CAN Bus Connection**: Connect the CAN High and CAN Low pins to your CAN bus network.

- **Termination Resistor**: Ensure that the CAN bus is properly terminated with 120-ohm resistors at both ends.

### **Software Setup**

- **ESP-IDF**: Ensure you have ESP-IDF installed and configured. This library is compatible with ESP-IDF v4.x and later.

- **SPI Bus Initialization**: Initialize the SPI bus in your application code before using the MCP2518FD driver.

## **API Overview**

### **Initialization and Configuration**

- `mcp2518fd_create()`: Initializes the MCP2518FD device with the specified configuration.
- `mcp2518fd_destroy()`: Cleans up and releases resources used by the device.
- `mcp2518fd_reset()`: Resets the MCP2518FD device.
- `mcp2518fd_set_mode()`: Sets the operation mode of the device.
- `mcp2518fd_set_baud_rates()`: Dynamically changes the nominal and data bit rates.

### **Message Transmission and Reception**

- `mcp2518fd_send_message()`: Sends a CAN message.
- `mcp2518fd_receive_message()`: Receives a CAN message from the RX queue.

### **Error and Status Handling**

- `mcp2518fd_get_status()`: Retrieves the device's status flags.
- `mcp2518fd_get_error_counts()`: Retrieves the TX and RX error counters.
- `mcp2518fd_strerror()`: Returns a string representation of an error code.

### **Filter Configuration**

- `mcp2518fd_configure_filter()`: Configures acceptance filters to control which messages are received.

## **Usage Example**

Below is an example demonstrating how to use the MCP2518FD driver library.

```c
#include "mcp2518fd.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "CAN_APP"

void app_main() {
    // Initialize SPI Bus
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_NUM_19,
        .mosi_io_num = GPIO_NUM_23,
        .sclk_io_num = GPIO_NUM_18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Configure MCP2518FD Device
    mcp2518fd_config_t can_config = {
        .spi_host = VSPI_HOST,
        .cs_io = GPIO_NUM_5,
        .int_io = GPIO_NUM_4,
        .spi_clock_speed_hz = 10000000,
        .osc_frequency = 40000000,
        .nominal_bitrate = 500000,
        .data_bitrate = 2000000,
        .op_mode = MCP2518FD_MODE_NORMAL,
        .tx_fifo_size = 8,
        .rx_fifo_size = 8,
        .rx_queue_size = 32, // Size of the RX queue
    };

    mcp2518fd_t can_device;
    mcp2518fd_error_t ret = mcp2518fd_create(&can_device, &can_config);
    if (ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCP2518FD: %s", mcp2518fd_strerror(ret));
        return;
    }

    // Send a CAN Message
    mcp2518fd_message_t tx_msg = {
        .id = 0x123,
        .dlc = 8,
        .extended = false,
        .fd_frame = true,
        .brs = true,
        .esi = false,
        .data = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88},
    };

    ret = mcp2518fd_send_message(&can_device, &tx_msg, pdMS_TO_TICKS(1000));
    if (ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to send message: %s", mcp2518fd_strerror(ret));
    } else {
        ESP_LOGI(TAG, "Message sent");
    }

    // Receive CAN Messages in a Loop
    mcp2518fd_message_t rx_msg;
    while (1) {
        ret = mcp2518fd_receive_message(&can_device, &rx_msg, portMAX_DELAY);
        if (ret == MCP2518FD_OK) {
            ESP_LOGI(TAG, "Received message ID: 0x%X", rx_msg.id);
            ESP_LOG_BUFFER_HEX(TAG, rx_msg.data, rx_msg.dlc);
            // Process the message as needed
        } else if (ret == MCP2518FD_NO_MESSAGE) {
            // No message received within the timeout
        } else {
            ESP_LOGE(TAG, "Failed to receive message: %s", mcp2518fd_strerror(ret));
        }
    }

    // Clean up (if needed)
    // mcp2518fd_destroy(&can_device);
    // spi_bus_free(VSPI_HOST);
}
