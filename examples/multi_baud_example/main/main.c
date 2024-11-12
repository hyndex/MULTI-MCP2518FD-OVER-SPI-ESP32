#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mcp2518fd.h"
#include "driver/spi_common.h"
#include "esp_log.h"

static const char *TAG = "CAN_APP";

// RX Message Queue
#define RX_QUEUE_LENGTH 32

void app_main() {
    // Initialize SPI Bus
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_NUM_12,
        .mosi_io_num = GPIO_NUM_11,
        .sclk_io_num = GPIO_NUM_13,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Create RX Queue
    QueueHandle_t rx_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(mcp2518fd_message_t));

    // Configure MCP2518FD Device
    mcp2518fd_t can_dev = {
        .config = {
            .spi_host = SPI2_HOST,
            .cs_io = GPIO_NUM_10,
            .int_io = GPIO_NUM_9, // INT pin connected
            .spi_clock_speed_hz = 10000000,
            .osc_frequency = 40000000, // 40 MHz
            .nominal_bitrate = 500000, // 500 kbps
            .data_bitrate = 2000000,   // 2 Mbps
            .tx_fifo_size = 8,
            .rx_fifo_size = 8,
            .tx_queue_size = 8,
            .rx_queue = rx_queue,
        },
    };

    mcp2518fd_error_t ret = mcp2518fd_init(&can_dev);
    if (ret != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCP2518FD");
        return;
    }

    // Task to Handle Received Messages
    xTaskCreatePinnedToCore(rx_task, "RX Task", 4096, &can_dev, 5, NULL, APP_CPU_NUM);

    // Send Messages at Different Baud Rates
    uint32_t baud_rates[] = {250000, 500000, 1000000}; // Nominal bitrates
    for (int i = 0; i < sizeof(baud_rates) / sizeof(baud_rates[0]); i++) {
        ret = mcp2518fd_set_baud_rates(&can_dev, baud_rates[i], can_dev.config.data_bitrate);
        if (ret != MCP2518FD_OK) {
            ESP_LOGE(TAG, "Failed to set baud rates");
            continue;
        }
        ESP_LOGI(TAG, "Baud rate set to %u bps", baud_rates[i]);

        // Send Test Message
        mcp2518fd_message_t tx_message = {
            .id = 0x100 + i,
            .dlc = 8,
            .extended = false,
            .fd_frame = false,
            .brs = false,
            .data = {i, i+1, i+2, i+3, i+4, i+5, i+6, i+7},
        };
        ret = mcp2518fd_send_message(&can_dev, &tx_message);
        if (ret != MCP2518FD_OK) {
            ESP_LOGE(TAG, "Failed to send message");
        } else {
            ESP_LOGI(TAG, "Message sent at %u bps", baud_rates[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void rx_task(void *arg) {
    mcp2518fd_t *dev = (mcp2518fd_t *)arg;
    mcp2518fd_message_t message;
    while (1) {
        if (mcp2518fd_receive_message(dev, &message, portMAX_DELAY) == MCP2518FD_OK) {
            ESP_LOGI("RX_TASK", "Received message ID: 0x%X", message.id);
            // Process received data
        }
    }
}
