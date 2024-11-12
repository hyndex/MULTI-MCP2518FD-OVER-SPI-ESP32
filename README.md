# MCP2518FD Driver Library for ESP32

This is a production-ready driver library for the MCP2518FD CAN FD controller, designed for use with the ESP32 microcontroller and the ESP-IDF framework. It provides a high-level API for initializing the device, configuring CAN settings dynamically, and sending/receiving CAN messages with support for both Classical CAN (CAN 2.0B) and CAN FD protocols.

---

## **Table of Contents**

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Hardware Setup](#hardware-setup)
- [Getting Started](#getting-started)
- [API Overview](#api-overview)
- [Examples](#examples)
  - [Basic CAN FD Example](#basic-can-fd-example)
  - [Classical CAN (CAN 2.0B) Example](#classical-can-can-20b-example)
  - [Multi Baud Rate Example](#multi-baud-rate-example)
  - [CAN Bus Bridging Example](#can-bus-bridging-example)
- [License](#license)

---

## **Features**

- **Supports Classical CAN and CAN FD**: Communicate with devices using both CAN 2.0B and CAN FD protocols.
- **Dynamic Configuration**: Configure CAN parameters at runtime, including bit rates, operation modes, and filters.
- **High-Level API**: Simplifies message transmission and reception with easy-to-use functions.
- **CAN FD Features**: Supports extended data payloads up to 64 bytes and Bit Rate Switching (BRS).
- **Interrupt-Driven Operation**: Efficient message handling using interrupts.
- **Edge Case Handling**: Comprehensive error checking and handling for robust operation.
- **Production Ready**: Suitable for time-sensitive applications like PLCs and power modules.

---

## **Requirements**

- **ESP32 Development Board**
- **MCP2518FD CAN FD Controller**
- **ESP-IDF Framework (v4.x or later)**
- **CAN Transceiver** (e.g., MCP2562FD)

---

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

---

## **Hardware Setup**

### **Connections**

- **ESP32 Pins**: Connect the MCP2518FD to the ESP32 via SPI. Here’s an example pin configuration:

  | MCP2518FD Pin | ESP32 Pin   |
  |---------------|-------------|
  | SCK           | GPIO 18     |
  | SI (MOSI)     | GPIO 23     |
  | SO (MISO)     | GPIO 19     |
  | CS            | GPIO 5      |
  | INT           | GPIO 4      |
  | VDD           | 3.3V        |
  | VSS           | GND         |
  | Other Pins    | As required |

- **CAN Bus Connection**: Connect the CAN High and CAN Low pins to your CAN bus network using a CAN transceiver (e.g., MCP2562FD).

- **Termination Resistor**: Ensure that the CAN bus is properly terminated with 120-ohm resistors at both ends.

### **Notes**

- **Interrupt Pin**: The `INT` pin is optional but recommended for efficient interrupt-driven operation.
- **SPI Bus Selection**: You can use either `HSPI_HOST` or `VSPI_HOST` for the SPI bus. Adjust the pin configurations accordingly.
- **Power Supply**: Ensure that the MCP2518FD and CAN transceiver are powered correctly, matching the voltage levels (typically 3.3V).

---

## **Getting Started**

### **ESP-IDF Setup**

- Ensure you have the ESP-IDF framework installed and configured.
- Set up the required tools and environment variables as per the ESP-IDF documentation.

### **SPI Bus Initialization**

- Initialize the SPI bus in your application code before using the MCP2518FD driver.

```c
spi_bus_config_t buscfg = {
    .miso_io_num = GPIO_NUM_19,
    .mosi_io_num = GPIO_NUM_23,
    .sclk_io_num = GPIO_NUM_18,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 64,
};
ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
```

---

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

---

## **Examples**

Below are various examples demonstrating how to use the MCP2518FD driver library in different scenarios.

### **Basic CAN FD Example**

This example demonstrates how to initialize the MCP2518FD device for CAN FD communication, send a message, and receive messages.

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

    // Configure MCP2518FD Device for CAN FD
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
        .rx_queue_size = 32,
        .canfd_enabled = true, // Enable CAN FD
    };

    mcp2518fd_t can_device;
    mcp2518fd_error_t ret = mcp2518fd_create(&can_device, &can_config);
    if (ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCP2518FD: %s", mcp2518fd_strerror(ret));
        return;
    }

    // Send a CAN FD Message
    mcp2518fd_message_t tx_msg = {
        .id = 0x123,
        .dlc = 16, // 16 bytes of data
        .extended = false,
        .fd_frame = true,
        .brs = true,   // Enable Bit Rate Switching
        .esi = false,
        .data = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
                 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00},
    };

    ret = mcp2518fd_send_message(&can_device, &tx_msg, pdMS_TO_TICKS(1000));
    if (ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to send message: %s", mcp2518fd_strerror(ret));
    } else {
        ESP_LOGI(TAG, "CAN FD message sent");
    }

    // Receive CAN Messages in a Loop
    mcp2518fd_message_t rx_msg;
    while (1) {
        ret = mcp2518fd_receive_message(&can_device, &rx_msg, portMAX_DELAY);
        if (ret == MCP2518FD_OK) {
            ESP_LOGI(TAG, "Received message ID: 0x%X, DLC: %d", rx_msg.id, rx_msg.dlc);
            ESP_LOG_BUFFER_HEX(TAG, rx_msg.data, rx_msg.dlc);
            // Process the message as needed
        } else if (ret == MCP2518FD_NO_MESSAGE) {
            // No message received within the timeout
        } else {
            ESP_LOGE(TAG, "Failed to receive message: %s", mcp2518fd_strerror(ret));
        }
    }
}
```

---

### **Classical CAN (CAN 2.0B) Example**

This example demonstrates how to use the driver for Classical CAN communication.

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

    // Configure MCP2518FD Device for Classical CAN
    mcp2518fd_config_t can_config = {
        .spi_host = VSPI_HOST,
        .cs_io = GPIO_NUM_5,
        .int_io = GPIO_NUM_4,
        .spi_clock_speed_hz = 10000000,
        .osc_frequency = 40000000,
        .nominal_bitrate = 125000,    // Set the nominal bitrate (e.g., 125 kbps)
        .data_bitrate = 0,            // Not used in Classical CAN
        .op_mode = MCP2518FD_MODE_NORMAL,
        .tx_fifo_size = 8,
        .rx_fifo_size = 8,
        .rx_queue_size = 32,
        .canfd_enabled = false,       // Disable CAN FD
    };

    mcp2518fd_t can_device;
    mcp2518fd_error_t ret = mcp2518fd_create(&can_device, &can_config);
    if (ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCP2518FD: %s", mcp2518fd_strerror(ret));
        return;
    }

    // Send a Classical CAN Message
    mcp2518fd_message_t tx_msg = {
        .id = 0x456,
        .dlc = 8,
        .extended = false,
        .fd_frame = false, // Classical CAN frame
        .brs = false,
        .esi = false,
        .data = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11},
    };

    ret = mcp2518fd_send_message(&can_device, &tx_msg, pdMS_TO_TICKS(1000));
    if (ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to send message: %s", mcp2518fd_strerror(ret));
    } else {
        ESP_LOGI(TAG, "Classical CAN message sent");
    }

    // Receive CAN Messages in a Loop
    mcp2518fd_message_t rx_msg;
    while (1) {
        ret = mcp2518fd_receive_message(&can_device, &rx_msg, portMAX_DELAY);
        if (ret == MCP2518FD_OK) {
            ESP_LOGI(TAG, "Received message ID: 0x%X, DLC: %d", rx_msg.id, rx_msg.dlc);
            ESP_LOG_BUFFER_HEX(TAG, rx_msg.data, rx_msg.dlc);
            // Process the message as needed
        } else if (ret == MCP2518FD_NO_MESSAGE) {
            // No message received within the timeout
        } else {
            ESP_LOGE(TAG, "Failed to receive message: %s", mcp2518fd_strerror(ret));
        }
    }
}
```

---

### **Multi Baud Rate Example**

This example demonstrates how to dynamically change baud rates during runtime.

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

    // Configure MCP2518FD Device with Initial Baud Rates
    mcp2518fd_config_t can_config = {
        .spi_host = VSPI_HOST,
        .cs_io = GPIO_NUM_5,
        .int_io = GPIO_NUM_4,
        .spi_clock_speed_hz = 10000000,
        .osc_frequency = 40000000,
        .nominal_bitrate = 500000,  // Initial Nominal Bitrate
        .data_bitrate = 2000000,    // Initial Data Bitrate
        .op_mode = MCP2518FD_MODE_NORMAL,
        .tx_fifo_size = 8,
        .rx_fifo_size = 8,
        .rx_queue_size = 32,        // Size of the RX queue
        .canfd_enabled = true,      // Enable CAN FD
    };

    mcp2518fd_t can_device;
    mcp2518fd_error_t ret = mcp2518fd_create(&can_device, &can_config);
    if (ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCP2518FD: %s", mcp2518fd_strerror(ret));
        return;
    }

    // Send a CAN Message with Initial Baud Rates
    mcp2518fd_message_t tx_msg = {
        .id = 0x123,
        .dlc = 8,
        .extended = false,
        .fd_frame = true,
        .brs = true,   // Enable Bit Rate Switching
        .esi = false,
        .data = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88},
    };

    ret = mcp2518fd_send_message(&can_device, &tx_msg, pdMS_TO_TICKS(1000));
    if (ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to send message: %s", mcp2518fd_strerror(ret));
    } else {
        ESP_LOGI(TAG, "Message sent at initial baud rates");
    }

    // Wait for a moment before changing baud rates
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Change Baud Rates Dynamically
    uint32_t new_nominal_bitrate = 250000;  // New Nominal Bitrate
    uint32_t new_data_bitrate = 1000000;    // New Data Bitrate

    ret = mcp2518fd_set_baud_rates(&can_device, new_nominal_bitrate, new_data_bitrate);
    if (ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to set new baud rates: %s", mcp2518fd_strerror(ret));
    } else {
        ESP_LOGI(TAG, "Baud rates changed to Nominal: %d bps, Data: %d bps", new_nominal_bitrate, new_data_bitrate);
    }

    // Send a CAN Message with New Baud Rates
    tx_msg.id = 0x456;
    tx_msg.data[0] = 0xAA;
    tx_msg.data[1] = 0xBB;
    tx_msg.data[2] = 0xCC;
    tx_msg.data[3] = 0xDD;
    tx_msg.data[4] = 0xEE;
    tx_msg.data[5] = 0xFF;
    tx_msg.data[6] = 0x00;
    tx_msg.data[7] = 0x11;

    ret = mcp2518fd_send_message(&can_device, &tx_msg, pdMS_TO_TICKS(1000));
    if (ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to send message at new baud rates: %s", mcp2518fd_strerror(ret));
    } else {
        ESP_LOGI(TAG, "Message sent at new baud rates");
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
}
```

**Important Considerations:**

- **Synchronization**: When changing baud rates dynamically, ensure all nodes on the CAN bus switch baud rates simultaneously to maintain communication.
- **Error Handling**: Implement proper error handling to recover from communication errors during baud rate changes.

---

### **CAN Bus Bridging Example**

This example demonstrates how to implement a bridge device that connects two CAN buses operating at different baud rates or protocols (e.g., one using Classical CAN at 125 kbps and another using CAN FD at 500 kbps).

#### **Hardware Setup**

- **ESP32** connected to:
  - **MCP2518FD #1**:
    - Connected to Bus 1 (e.g., 125 kbps, Classical CAN).
    - SPI bus on ESP32 (e.g., VSPI_HOST).
    - CS pin (e.g., GPIO 5).
    - INT pin (e.g., GPIO 4).
  - **MCP2518FD #2**:
    - Connected to Bus 2 (e.g., 500 kbps, CAN FD).
    - SPI bus on ESP32 (e.g., HSPI_HOST).
    - CS pin (e.g., GPIO 15).
    - INT pin (e.g., GPIO 2).

#### **Software Implementation**

```c
#include "mcp2518fd.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "CAN_BRIDGE"

static void can_bridge_task(void *arg) {
    mcp2518fd_t *can1 = (mcp2518fd_t *)arg;
    mcp2518fd_t *can2 = can1 + 1; // Assuming can2 is next in the array

    mcp2518fd_message_t msg;

    while (1) {
        // Check for messages on Bus 1 (Classical CAN)
        if (mcp2518fd_receive_message(can1, &msg, pdMS_TO_TICKS(10)) == MCP2518FD_OK) {
            ESP_LOGI(TAG, "Received from Bus 1: ID=0x%X", msg.id);
            // Modify message if necessary
            // Forward message to Bus 2
            if (mcp2518fd_send_message(can2, &msg, pdMS_TO_TICKS(100)) != MCP2518FD_OK) {
                ESP_LOGE(TAG, "Failed to forward message to Bus 2");
            }
        }

        // Check for messages on Bus 2 (CAN FD)
        if (mcp2518fd_receive_message(can2, &msg, pdMS_TO_TICKS(10)) == MCP2518FD_OK) {
            ESP_LOGI(TAG, "Received from Bus 2: ID=0x%X", msg.id);
            // Modify message if necessary
            // Forward message to Bus 1
            if (mcp2518fd_send_message(can1, &msg, pdMS_TO_TICKS(100)) != MCP2518FD_OK) {
                ESP_LOGE(TAG, "Failed to forward message to Bus 1");
            }
        }

        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main() {
    // Initialize SPI buses for both MCP2518FD devices
    spi_bus_config_t buscfg1 = {
        .miso_io_num = GPIO_NUM_19,
        .mosi_io_num = GPIO_NUM_23,
        .sclk_io_num = GPIO_NUM_18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    spi_bus_config_t buscfg2 = {
        .miso_io_num = GPIO_NUM_12,
        .mosi_io_num = GPIO_NUM_13,
        .sclk_io_num = GPIO_NUM_14,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg1, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg2, SPI_DMA_CH_AUTO));

    // Configure MCP2518FD Device for Bus 1 (Classical CAN at 125 kbps)
    mcp2518fd_config_t can1_config = {
        .spi_host = VSPI_HOST,
        .cs_io = GPIO_NUM_5,
        .int_io = GPIO_NUM_4,
        .spi_clock_speed_hz = 10000000,
        .osc_frequency = 40000000,
        .nominal_bitrate = 125000,
        .data_bitrate = 0,
        .op_mode = MCP2518FD_MODE_NORMAL,
        .tx_fifo_size = 8,
        .rx_fifo_size = 8,
        .rx_queue_size = 32,
        .canfd_enabled = false, // Classical CAN
    };

    // Configure MCP2518FD Device for Bus 2 (CAN FD at 500 kbps)
    mcp2518fd_config_t can2_config = {
        .spi_host = HSPI_HOST,
        .cs_io = GPIO_NUM_15,
        .int_io = GPIO_NUM_2,
        .spi_clock_speed_hz = 10000000,
        .osc_frequency = 40000000,
        .nominal_bitrate = 500000,
        .data_bitrate = 2000000,
        .op_mode = MCP2518FD_MODE_NORMAL,
        .tx_fifo_size = 8,
        .rx_fifo_size = 8,
        .rx_queue_size = 32,
        .canfd_enabled = true, // CAN FD
    };

    // Create MCP2518FD Devices
    mcp2518fd_t can_devices[2];

    mcp2518fd_error_t ret = mcp2518fd_create(&can_devices[0], &can1_config);
    if (ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN1: %s", mcp2518fd_strerror(ret));
        return;
    }

    ret = mcp2518fd_create(&can_devices[1], &can2_config);
    if (ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN2: %s", mcp2518fd_strerror(ret));
        return;
    }

    // Create CAN bridge task
    xTaskCreate(can_bridge_task, "CAN_Bridge_Task", 4096, can_devices, 5, NULL);
}
```

**Explanation:**

- **CAN Devices Initialization**:
  - **CAN1**: Configured for Classical CAN at 125 kbps.
  - **CAN2**: Configured for CAN FD at 500 kbps nominal and 2 Mbps data bitrate.
- **Bridging Logic**:
  - The `can_bridge_task` function continuously checks for messages on both buses.
  - Messages received on one bus are forwarded to the other bus.
  - Modify messages if necessary to account for differences in protocols (e.g., DLC limitations).
- **Considerations**:
  - Ensure proper handling of message formats when bridging between Classical CAN and CAN FD.
  - Adjust message payloads to meet the constraints of each protocol.

---

## **License**

This library is open-source and available under the MIT License.

```
MIT License

Copyright (c)

Permission is hereby granted, free of charge, to any person obtaining a copy...
```

---

## **Additional Notes**

- **Filter Configuration**: If you need to receive only specific CAN IDs, configure the acceptance filters using `mcp2518fd_configure_filter()`.
- **Error Handling**: Always check the return values of the functions and handle errors appropriately.
- **CAN FD Frames**: To use CAN FD frames, set the `fd_frame` flag to `true` in your `mcp2518fd_message_t` structure and ensure `canfd_enabled` is `true` in the configuration.
- **Classical CAN Limitations**: When operating in Classical CAN mode, ensure that the `fd_frame` and `brs` flags are set to `false`, and the DLC does not exceed 8 bytes.

---

## **Support**

If you encounter any issues or have questions, feel free to open an issue on the repository or contact the maintainer.

---

**Disclaimer**: Please ensure to test and validate the code in your specific environment. Adjust the GPIO numbers and configurations according to your hardware setup.

---

**Happy Coding!**

If you have any further questions or need assistance with the MCP2518FD driver library, please feel free to reach out!

---