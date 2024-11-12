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
        // Check for messages on Bus 1 (125 kbps)
        if (mcp2518fd_receive_message(can1, &msg, pdMS_TO_TICKS(10)) == MCP2518FD_OK) {
            ESP_LOGI(TAG, "Received from Bus 1: ID=0x%X", msg.id);
            // Forward message to Bus 2
            if (mcp2518fd_send_message(can2, &msg, pdMS_TO_TICKS(100)) != MCP2518FD_OK) {
                ESP_LOGE(TAG, "Failed to forward message to Bus 2");
            }
        }

        // Check for messages on Bus 2 (500 kbps)
        if (mcp2518fd_receive_message(can2, &msg, pdMS_TO_TICKS(10)) == MCP2518FD_OK) {
            ESP_LOGI(TAG, "Received from Bus 2: ID=0x%X", msg.id);
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

    // Configure MCP2518FD Device for Bus 1 (125 kbps)
    mcp2518fd_config_t can1_config = {
        .spi_host = VSPI_HOST,
        .cs_io = GPIO_NUM_5,
        .int_io = GPIO_NUM_4,
        .spi_clock_speed_hz = 10000000,
        .osc_frequency = 40000000,
        .nominal_bitrate = 125000,
        .data_bitrate = 500000, // Adjust as needed
        .op_mode = MCP2518FD_MODE_NORMAL,
        .tx_fifo_size = 8,
        .rx_fifo_size = 8,
        .rx_queue_size = 32,
    };

    // Configure MCP2518FD Device for Bus 2 (500 kbps)
    mcp2518fd_config_t can2_config = {
        .spi_host = HSPI_HOST,
        .cs_io = GPIO_NUM_15,
        .int_io = GPIO_NUM_2,
        .spi_clock_speed_hz = 10000000,
        .osc_frequency = 40000000,
        .nominal_bitrate = 500000,
        .data_bitrate = 2000000, // Adjust as needed
        .op_mode = MCP2518FD_MODE_NORMAL,
        .tx_fifo_size = 8,
        .rx_fifo_size = 8,
        .rx_queue_size = 32,
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
