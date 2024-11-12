/*
 * test.c
 *
 * Test program for the MCP2518FD driver library.
 *
 * This program initializes the MCP2518FD device, sends a test message, and
 * receives messages in a loop, printing any received messages to the console.
 *
 * Adjust the GPIO pin assignments and configurations according to your hardware setup.
 */

#include "mcp2518fd.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"

#define TAG "MCP2518FD_TEST"

// Adjust these GPIO numbers to match your setup
#define SPI_MISO_GPIO GPIO_NUM_19
#define SPI_MOSI_GPIO GPIO_NUM_23
#define SPI_SCLK_GPIO GPIO_NUM_18
#define SPI_CS_GPIO   GPIO_NUM_5
#define INT_GPIO      GPIO_NUM_4

void app_main(void) {
    esp_err_t ret;

    // Initialize SPI Bus Configuration
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO_GPIO,
        .mosi_io_num = SPI_MOSI_GPIO,
        .sclk_io_num = SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
        .flags = 0,
        .intr_flags = 0,
    };

    // Initialize SPI bus
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }

    // Configure MCP2518FD Device
    mcp2518fd_config_t can_config = {
        .spi_host = VSPI_HOST,
        .cs_io = SPI_CS_GPIO,
        .int_io = INT_GPIO,
        .spi_clock_speed_hz = 10000000,    // Adjust as needed (up to 20 MHz)
        .osc_frequency = 40000000,         // Oscillator frequency (e.g., 40 MHz)
        .nominal_bitrate = 500000,         // Nominal bitrate (e.g., 500 kbps)
        .data_bitrate = 2000000,           // Data bitrate (e.g., 2 Mbps)
        .op_mode = MCP2518FD_MODE_NORMAL,  // Operation mode
        .tx_fifo_size = 8,                 // TX FIFO size
        .rx_fifo_size = 8,                 // RX FIFO size
        .rx_queue = NULL,                  // Use internal RX queue
        .rx_queue_size = 32,               // RX queue size
        .canfd_enabled = true,             // Enable CAN FD
    };

    mcp2518fd_t can_device;
    mcp2518fd_error_t mcp_ret;

    // Create MCP2518FD device
    mcp_ret = mcp2518fd_create(&can_device, &can_config);
    if (mcp_ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCP2518FD: %s", mcp2518fd_strerror(mcp_ret));
        return;
    }

    // Send a test CAN message
    mcp2518fd_message_t tx_msg = {
        .id = 0x100,
        .dlc = 8,
        .extended = false,
        .fd_frame = true,
        .brs = true,
        .esi = false,
        .data = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE},
    };

    mcp_ret = mcp2518fd_send_message(&can_device, &tx_msg, pdMS_TO_TICKS(1000));
    if (mcp_ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to send test message: %s", mcp2518fd_strerror(mcp_ret));
    } else {
        ESP_LOGI(TAG, "Test message sent");
    }

    // Receive messages in a loop
    mcp2518fd_message_t rx_msg;
    while (1) {
        mcp_ret = mcp2518fd_receive_message(&can_device, &rx_msg, portMAX_DELAY);
        if (mcp_ret == MCP2518FD_OK) {
            ESP_LOGI(TAG, "Received message:");
            ESP_LOGI(TAG, "ID: 0x%X", rx_msg.id);
            ESP_LOGI(TAG, "DLC: %d", rx_msg.dlc);
            ESP_LOGI(TAG, "Extended: %s", rx_msg.extended ? "Yes" : "No");
            ESP_LOGI(TAG, "FD Frame: %s", rx_msg.fd_frame ? "Yes" : "No");
            ESP_LOGI(TAG, "BRS: %s", rx_msg.brs ? "Yes" : "No");
            ESP_LOGI(TAG, "ESI: %s", rx_msg.esi ? "Yes" : "No");
            ESP_LOG_BUFFER_HEX(TAG, rx_msg.data, rx_msg.dlc);
        } else if (mcp_ret == MCP2518FD_NO_MESSAGE) {
            // No message received
            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            ESP_LOGE(TAG, "Failed to receive message: %s", mcp2518fd_strerror(mcp_ret));
        }
    }

    // Clean up resources (this part will not be reached in this infinite loop)
    mcp2518fd_destroy(&can_device);
    spi_bus_free(VSPI_HOST);
}
