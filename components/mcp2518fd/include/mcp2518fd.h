#ifndef MCP2518FD_H
#define MCP2518FD_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/queue.h"

// Maximum Data Payload Size
#define MCP2518FD_MAX_DATA_BYTES 64

// Error Codes
typedef enum {
    MCP2518FD_OK = 0,
    MCP2518FD_FAIL,
    MCP2518FD_TIMEOUT,
    MCP2518FD_INVALID_ARGUMENT,
    MCP2518FD_SPI_ERROR,
    MCP2518FD_NO_MESSAGE,
    MCP2518FD_BUFFER_FULL,
    MCP2518FD_CONFIG_ERROR,
    MCP2518FD_INIT_ERROR,
    MCP2518FD_NOT_IMPLEMENTED,
} mcp2518fd_error_t;

// CAN Message Structure
typedef struct {
    uint32_t id;                                       // CAN Identifier
    uint8_t data[MCP2518FD_MAX_DATA_BYTES];            // Data Payload
    uint8_t dlc;                                       // Data Length Code
    bool extended;                                     // Extended ID Flag
    bool fd_frame;                                     // CAN FD Frame Flag
    bool brs;                                          // Bit Rate Switch Flag
    bool esi;                                          // Error State Indicator Flag
} mcp2518fd_message_t;

// CAN Operation Modes
typedef enum {
    MCP2518FD_MODE_NORMAL = 0,
    MCP2518FD_MODE_SLEEP,
    MCP2518FD_MODE_INTERNAL_LOOPBACK,
    MCP2518FD_MODE_LISTEN_ONLY,
    MCP2518FD_MODE_CONFIGURATION,
    MCP2518FD_MODE_EXTERNAL_LOOPBACK,
    MCP2518FD_MODE_CLASSIC,
    MCP2518FD_MODE_RESTRICTED,
} mcp2518fd_mode_t;

// MCP2518FD Configuration Structure
typedef struct {
    spi_host_device_t spi_host;                        // SPI Host (e.g., HSPI_HOST or VSPI_HOST)
    gpio_num_t cs_io;                                  // Chip Select GPIO Number
    gpio_num_t int_io;                                 // Interrupt GPIO Number (set to GPIO_NUM_NC if unused)
    int spi_clock_speed_hz;                            // SPI Clock Speed (up to 20 MHz)
    uint32_t osc_frequency;                            // Oscillator Frequency in Hz (e.g., 40 MHz)
    uint32_t nominal_bitrate;                          // Nominal Bitrate in bps (e.g., 500000 for 500 kbps)
    uint32_t data_bitrate;                             // Data Bitrate in bps (e.g., 2000000 for 2 Mbps)
    mcp2518fd_mode_t op_mode;                          // Operation Mode
    uint8_t tx_fifo_size;                              // TX FIFO Size (number of messages)
    uint8_t rx_fifo_size;                              // RX FIFO Size (number of messages)
    QueueHandle_t rx_queue;                            // FreeRTOS Queue Handle for RX Messages
    uint8_t rx_queue_size;                             // RX Queue Size
} mcp2518fd_config_t;

// MCP2518FD Device Handle
typedef struct {
    spi_device_handle_t spi_handle;
    mcp2518fd_config_t config;
    bool is_initialized;
} mcp2518fd_t;

// Initialization and Configuration
mcp2518fd_error_t mcp2518fd_create(mcp2518fd_t *dev, const mcp2518fd_config_t *config);
mcp2518fd_error_t mcp2518fd_destroy(mcp2518fd_t *dev);
mcp2518fd_error_t mcp2518fd_reset(mcp2518fd_t *dev);
mcp2518fd_error_t mcp2518fd_set_mode(mcp2518fd_t *dev, mcp2518fd_mode_t mode);
mcp2518fd_error_t mcp2518fd_set_baud_rates(mcp2518fd_t *dev, uint32_t nominal_bitrate, uint32_t data_bitrate);

// Message Transmission and Reception
mcp2518fd_error_t mcp2518fd_send_message(mcp2518fd_t *dev, const mcp2518fd_message_t *message, TickType_t ticks_to_wait);
mcp2518fd_error_t mcp2518fd_receive_message(mcp2518fd_t *dev, mcp2518fd_message_t *message, TickType_t ticks_to_wait);

// Error and Status Handling
mcp2518fd_error_t mcp2518fd_get_status(mcp2518fd_t *dev, uint32_t *status);
mcp2518fd_error_t mcp2518fd_get_error_counts(mcp2518fd_t *dev, uint8_t *tx_err_count, uint8_t *rx_err_count);

// Filter Configuration (Optional)
mcp2518fd_error_t mcp2518fd_configure_filter(mcp2518fd_t *dev, uint8_t filter_number, uint32_t id, uint32_t mask, bool extended);

// Utility Functions
const char* mcp2518fd_strerror(mcp2518fd_error_t error);

#endif // MCP2518FD_H
