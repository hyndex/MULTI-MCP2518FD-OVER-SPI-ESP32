#include "mcp2518fd.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MCP2518FD";

// MCP2518FD SPI Instructions
#define MCP2518FD_SPI_INSTRUCTION_RESET 0x00
#define MCP2518FD_SPI_INSTRUCTION_READ  0x03
#define MCP2518FD_SPI_INSTRUCTION_WRITE 0x02

// MCP2518FD Registers (Partial List)
#define REG_CiCON       0x000
#define REG_CiNBTCFG    0x004
#define REG_CiDBTCFG    0x008
#define REG_CiTDC       0x00C
#define REG_CiTREC      0x034
#define REG_CiBDIAG0    0x038
#define REG_CiBDIAG1    0x03C
#define REG_CiINT       0x01C
#define REG_CiINTFLAG   0x01C
#define REG_CiINTENABLE 0x01D
#define REG_CiFIFOBA    0x10
#define REG_CiTXQCON    0x050
#define REG_CiTXQSTA    0x054
#define REG_CiTXQUA     0x058
#define REG_CiFIFOCON   0x05C
#define REG_CiFIFOSTA   0x060
#define REG_CiFIFOUA    0x064
#define REG_CiFLTCON    0x1D0
#define REG_CiFLTOBJ    0x1F0
#define REG_CiMASK      0x1F4

// Maximum Values
#define MAX_BRP 256
#define MAX_TSEG1 256
#define MAX_TSEG2 128
#define MAX_SJW 128

// Helper Function Declarations
static mcp2518fd_error_t spi_transfer(mcp2518fd_t *dev, uint8_t *tx_data, uint8_t *rx_data, size_t length);
static mcp2518fd_error_t write_register(mcp2518fd_t *dev, uint16_t address, uint32_t value);
static mcp2518fd_error_t read_register(mcp2518fd_t *dev, uint16_t address, uint32_t *value);
static mcp2518fd_error_t configure_bit_timing(mcp2518fd_t *dev);
static bool calculate_bit_timing(uint32_t sysclk, uint32_t bitrate, uint32_t *brp, uint32_t *tseg1, uint32_t *tseg2, uint32_t *sjw);
static void IRAM_ATTR handle_interrupt(void *arg);
static mcp2518fd_error_t read_message_from_fifo(mcp2518fd_t *dev, mcp2518fd_message_t *message);
static mcp2518fd_error_t write_message_to_fifo(mcp2518fd_t *dev, const mcp2518fd_message_t *message);

// Utility Function Declarations
static mcp2518fd_error_t set_operation_mode(mcp2518fd_t *dev, mcp2518fd_mode_t mode);
static mcp2518fd_error_t configure_interrupts(mcp2518fd_t *dev);

// String representations of error codes
const char* mcp2518fd_strerror(mcp2518fd_error_t error) {
    switch (error) {
        case MCP2518FD_OK: return "No error";
        case MCP2518FD_FAIL: return "General failure";
        case MCP2518FD_TIMEOUT: return "Operation timed out";
        case MCP2518FD_INVALID_ARGUMENT: return "Invalid argument";
        case MCP2518FD_SPI_ERROR: return "SPI communication error";
        case MCP2518FD_NO_MESSAGE: return "No message available";
        case MCP2518FD_BUFFER_FULL: return "Buffer full";
        case MCP2518FD_CONFIG_ERROR: return "Configuration error";
        case MCP2518FD_INIT_ERROR: return "Initialization error";
        case MCP2518FD_NOT_IMPLEMENTED: return "Feature not implemented";
        default: return "Unknown error";
    }
}

// Create and Initialize MCP2518FD Device
mcp2518fd_error_t mcp2518fd_create(mcp2518fd_t *dev, const mcp2518fd_config_t *config) {
    if (dev == NULL || config == NULL) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    memset(dev, 0, sizeof(mcp2518fd_t));
    memcpy(&dev->config, config, sizeof(mcp2518fd_config_t));

    // SPI Device Configuration
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = dev->config.spi_clock_speed_hz,
        .mode = 0, // SPI mode 0
        .spics_io_num = dev->config.cs_io,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };

    // Add SPI Device
    esp_err_t ret = spi_bus_add_device(dev->config.spi_host, &devcfg, &dev->spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return MCP2518FD_SPI_ERROR;
    }

    // Reset MCP2518FD
    mcp2518fd_error_t err = mcp2518fd_reset(dev);
    if (err != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to reset MCP2518FD");
        return err;
    }

    // Configure Bit Timing
    err = configure_bit_timing(dev);
    if (err != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to configure bit timing");
        return err;
    }

    // Configure FIFOs
    // Implement FIFO configuration as needed
    // For example, configure TXQ and RX FIFO
    // Configure TXQ (Transmit Queue)
    uint32_t txqcon = 0;
    txqcon |= ((dev->config.tx_fifo_size - 1) & 0x1F) << 24; // TXQ Size
    txqcon |= (0x0F << 16); // Payload size (0x0F for 64 bytes)
    txqcon |= (0x00 << 8);  // TX Priority
    txqcon |= (1 << 31);    // Enable TXQ

    err = write_register(dev, REG_CiTXQCON, txqcon);
    if (err != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to configure TXQ");
        return err;
    }

    // Configure RX FIFO
    uint32_t rxfifocon = 0;
    uint8_t fifo_index = 0; // FIFO 0 for RX
    rxfifocon |= ((dev->config.rx_fifo_size - 1) & 0x1F) << 24; // FIFO Size
    rxfifocon |= (0x0F << 16); // Payload size (0x0F for 64 bytes)
    rxfifocon |= (0x00 << 8);  // RX Time Stamp Disabled
    rxfifocon |= (1 << 31);    // Enable FIFO

    err = write_register(dev, REG_CiFIFOCON + (fifo_index * 0x0C), rxfifocon);
    if (err != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to configure RX FIFO");
        return err;
    }

    // Configure Interrupts
    err = configure_interrupts(dev);
    if (err != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to configure interrupts");
        return err;
    }

    // Set Operation Mode
    err = set_operation_mode(dev, dev->config.op_mode);
    if (err != MCP2518FD_OK) {
        ESP_LOGE(TAG, "Failed to set operation mode");
        return err;
    }

    // Setup Interrupt Handling if INT pin is used
    if (dev->config.int_io != GPIO_NUM_NC) {
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_NEGEDGE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = 1ULL << dev->config.int_io,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        gpio_isr_handler_add(dev->config.int_io, handle_interrupt, (void *)dev);
    }

    // Create RX Queue if not provided
    if (dev->config.rx_queue == NULL && dev->config.rx_queue_size > 0) {
        dev->config.rx_queue = xQueueCreate(dev->config.rx_queue_size, sizeof(mcp2518fd_message_t));
        if (dev->config.rx_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create RX queue");
            return MCP2518FD_INIT_ERROR;
        }
    }

    dev->is_initialized = true;
    return MCP2518FD_OK;
}

// Destroy and Clean Up MCP2518FD Device
mcp2518fd_error_t mcp2518fd_destroy(mcp2518fd_t *dev) {
    if (dev == NULL || !dev->is_initialized) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    // Remove SPI Device
    spi_bus_remove_device(dev->spi_handle);

    // Delete RX Queue if created internally
    if (dev->config.rx_queue != NULL && dev->config.rx_queue_size > 0) {
        vQueueDelete(dev->config.rx_queue);
        dev->config.rx_queue = NULL;
    }

    // Remove ISR Handler
    if (dev->config.int_io != GPIO_NUM_NC) {
        gpio_isr_handler_remove(dev->config.int_io);
    }

    dev->is_initialized = false;
    return MCP2518FD_OK;
}

// Reset MCP2518FD
mcp2518fd_error_t mcp2518fd_reset(mcp2518fd_t *dev) {
    if (dev == NULL) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    uint8_t tx_buf[2] = {MCP2518FD_SPI_INSTRUCTION_RESET, 0x00};
    mcp2518fd_error_t ret = spi_transfer(dev, tx_buf, NULL, 2);
    if (ret != MCP2518FD_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for reset to complete
    return MCP2518FD_OK;
}

// Set Operation Mode
static mcp2518fd_error_t set_operation_mode(mcp2518fd_t *dev, mcp2518fd_mode_t mode) {
    if (dev == NULL) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    uint32_t con;
    mcp2518fd_error_t err = read_register(dev, REG_CiCON, &con);
    if (err != MCP2518FD_OK) {
        return err;
    }

    con &= ~(0x07 << 21);            // Clear Operation Mode bits
    con |= ((mode & 0x07) << 21);    // Set new mode
    err = write_register(dev, REG_CiCON, con);
    if (err != MCP2518FD_OK) {
        return err;
    }

    // Wait for mode change to complete
    uint32_t new_con;
    TickType_t start_tick = xTaskGetTickCount();
    const TickType_t timeout_tick = pdMS_TO_TICKS(100);
    do {
        err = read_register(dev, REG_CiCON, &new_con);
        if (err != MCP2518FD_OK) {
            return err;
        }
        if (((new_con >> 21) & 0x07) == mode) {
            return MCP2518FD_OK;
        }
    } while ((xTaskGetTickCount() - start_tick) < timeout_tick);

    return MCP2518FD_TIMEOUT;
}

// Configure Bit Timing
static mcp2518fd_error_t configure_bit_timing(mcp2518fd_t *dev) {
    if (dev == NULL) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    uint32_t sysclk = dev->config.osc_frequency;
    uint32_t nominal_bitrate = dev->config.nominal_bitrate;
    uint32_t data_bitrate = dev->config.data_bitrate;

    // Calculate Nominal Bit Timing
    uint32_t brp_n, tseg1_n, tseg2_n, sjw_n;
    bool result = calculate_bit_timing(sysclk, nominal_bitrate, &brp_n, &tseg1_n, &tseg2_n, &sjw_n);
    if (!result) {
        ESP_LOGE(TAG, "Failed to calculate nominal bit timing");
        return MCP2518FD_CONFIG_ERROR;
    }

    uint32_t nbtcfg = ((sjw_n & 0x7F) << 24) |
                      ((tseg2_n & 0x7F) << 16) |
                      ((tseg1_n & 0xFF) << 8) |
                      (brp_n & 0xFF);

    mcp2518fd_error_t err = write_register(dev, REG_CiNBTCFG, nbtcfg);
    if (err != MCP2518FD_OK) {
        return err;
    }

    // Calculate Data Bit Timing
    uint32_t brp_d, tseg1_d, tseg2_d, sjw_d;
    result = calculate_bit_timing(sysclk, data_bitrate, &brp_d, &tseg1_d, &tseg2_d, &sjw_d);
    if (!result) {
        ESP_LOGE(TAG, "Failed to calculate data bit timing");
        return MCP2518FD_CONFIG_ERROR;
    }

    uint32_t dbtcfg = ((sjw_d & 0x0F) << 24) |
                      ((tseg2_d & 0x0F) << 16) |
                      ((tseg1_d & 0x1F) << 8) |
                      (brp_d & 0x1F);

    err = write_register(dev, REG_CiDBTCFG, dbtcfg);
    if (err != MCP2518FD_OK) {
        return err;
    }

    // Configure Transmitter Delay Compensation (Optional)
    uint32_t tdc = 0x00000000; // Disable TDC
    err = write_register(dev, REG_CiTDC, tdc);
    if (err != MCP2518FD_OK) {
        return err;
    }

    return MCP2518FD_OK;
}

// Bit Timing Calculation Function
static bool calculate_bit_timing(uint32_t sysclk, uint32_t bitrate, uint32_t *brp, uint32_t *tseg1, uint32_t *tseg2, uint32_t *sjw) {
    if (sysclk == 0 || bitrate == 0 || brp == NULL || tseg1 == NULL || tseg2 == NULL || sjw == NULL) {
        return false;
    }

    uint32_t best_error = UINT32_MAX;
    uint32_t best_brp = 0, best_tseg1 = 0, best_tseg2 = 0, best_sjw = 0;

    for (uint32_t curr_brp = 1; curr_brp <= MAX_BRP; curr_brp++) {
        uint32_t tq = sysclk / (curr_brp * bitrate);
        if (tq < 8 || tq > (MAX_TSEG1 + MAX_TSEG2 + 1)) {
            continue;
        }
        for (uint32_t curr_tseg1 = 2; curr_tseg1 <= (tq - 2); curr_tseg1++) {
            uint32_t curr_tseg2 = tq - curr_tseg1 - 1;
            if (curr_tseg2 < 1 || curr_tseg2 > MAX_TSEG2) {
                continue;
            }
            uint32_t curr_bitrate = sysclk / (curr_brp * tq);
            int32_t error = ((int32_t)bitrate - (int32_t)curr_bitrate);
            if (abs(error) < best_error) {
                best_error = abs(error);
                best_brp = curr_brp - 1;
                best_tseg1 = curr_tseg1 - 1;
                best_tseg2 = curr_tseg2 - 1;
                best_sjw = (curr_tseg2 > 4) ? 3 : curr_tseg2 - 1;
                if (best_error == 0) {
                    goto done;
                }
            }
        }
    }

done:
    if (best_error == UINT32_MAX) {
        return false;
    }

    *brp = best_brp;
    *tseg1 = best_tseg1;
    *tseg2 = best_tseg2;
    *sjw = best_sjw;
    return true;
}

// Configure Interrupts
static mcp2518fd_error_t configure_interrupts(mcp2518fd_t *dev) {
    if (dev == NULL) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    // Enable RX FIFO Interrupts
    uint32_t int_enable = 0x00000001; // RX FIFO 0 Interrupt Enable
    mcp2518fd_error_t err = write_register(dev, REG_CiINTENABLE, int_enable);
    if (err != MCP2518FD_OK) {
        return err;
    }

    return MCP2518FD_OK;
}

// Send CAN Message
mcp2518fd_error_t mcp2518fd_send_message(mcp2518fd_t *dev, const mcp2518fd_message_t *message, TickType_t ticks_to_wait) {
    if (dev == NULL || message == NULL) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    // Wait if TX FIFO is full
    uint32_t txqsta;
    TickType_t start_tick = xTaskGetTickCount();
    mcp2518fd_error_t err;
    do {
        err = read_register(dev, REG_CiTXQSTA, &txqsta);
        if (err != MCP2518FD_OK) {
            return err;
        }
        if (!(txqsta & 0x8)) { // TXQ Not Full
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    } while ((xTaskGetTickCount() - start_tick) < ticks_to_wait);

    if (txqsta & 0x8) {
        return MCP2518FD_TIMEOUT; // TXQ is still full
    }

    // Write message to TXQ
    err = write_message_to_fifo(dev, message);
    if (err != MCP2518FD_OK) {
        return err;
    }

    // Set TXREQ to transmit the message
    uint32_t txqcon;
    err = read_register(dev, REG_CiTXQCON, &txqcon);
    if (err != MCP2518FD_OK) {
        return err;
    }

    txqcon |= 0x00000008; // Set TXREQ
    err = write_register(dev, REG_CiTXQCON, txqcon);
    if (err != MCP2518FD_OK) {
        return err;
    }

    return MCP2518FD_OK;
}

// Receive CAN Message
mcp2518fd_error_t mcp2518fd_receive_message(mcp2518fd_t *dev, mcp2518fd_message_t *message, TickType_t ticks_to_wait) {
    if (dev == NULL || message == NULL) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    if (xQueueReceive(dev->config.rx_queue, message, ticks_to_wait) == pdTRUE) {
        return MCP2518FD_OK;
    } else {
        return MCP2518FD_NO_MESSAGE;
    }
}

// Read Message from FIFO
static mcp2518fd_error_t read_message_from_fifo(mcp2518fd_t *dev, mcp2518fd_message_t *message) {
    if (dev == NULL || message == NULL) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    uint8_t fifo_index = 0; // FIFO 0 for RX

    // Check if FIFO is not empty
    uint32_t fifosta;
    mcp2518fd_error_t err = read_register(dev, REG_CiFIFOSTA + (fifo_index * 0x0C), &fifosta);
    if (err != MCP2518FD_OK) {
        return err;
    }

    if (!(fifosta & 0x00000001)) { // FIFO is empty
        return MCP2518FD_NO_MESSAGE;
    }

    // Get FIFO User Address
    uint32_t fifoua;
    err = read_register(dev, REG_CiFIFOUA + (fifo_index * 0x0C), &fifoua);
    if (err != MCP2518FD_OK) {
        return err;
    }

    uint16_t address = (uint16_t)(fifoua & 0xFFFF);

    // Read message from RAM
    uint8_t tx_buf[3];
    uint8_t rx_buf[72]; // Max message size
    tx_buf[0] = MCP2518FD_SPI_INSTRUCTION_READ;
    tx_buf[1] = (address >> 8) & 0xFF;
    tx_buf[2] = address & 0xFF;

    err = spi_transfer(dev, tx_buf, rx_buf, 3 + 72);
    if (err != MCP2518FD_OK) {
        return err;
    }

    uint8_t *ptr = rx_buf + 3;

    // Parse CAN ID
    uint32_t id = ((uint32_t)ptr[0] << 24) |
                  ((uint32_t)ptr[1] << 16) |
                  ((uint32_t)ptr[2] << 8) |
                  ((uint32_t)ptr[3]);

    message->extended = (id & 0x80000000) ? true : false;
    message->id = message->extended ? (id & 0x1FFFFFFF) : ((id >> 18) & 0x7FF);

    // Parse DLC
    uint8_t dlc = ptr[4] & 0x0F;
    static const uint8_t dlc_to_length[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    uint8_t length = dlc_to_length[dlc];
    message->dlc = length;

    // Parse Flags
    uint8_t flags = ptr[5];
    message->fd_frame = (flags & 0x80) ? true : false;
    message->brs = (flags & 0x40) ? true : false;
    message->esi = (flags & 0x20) ? true : false;

    // Copy Data
    memcpy(message->data, ptr + 8, length);

    // Update FIFO (Set UINC)
    uint32_t fifocon;
    err = read_register(dev, REG_CiFIFOCON + (fifo_index * 0x0C), &fifocon);
    if (err != MCP2518FD_OK) {
        return err;
    }

    fifocon |= 0x00010000; // Set UINC
    err = write_register(dev, REG_CiFIFOCON + (fifo_index * 0x0C), fifocon);
    if (err != MCP2518FD_OK) {
        return err;
    }

    return MCP2518FD_OK;
}

// Write Message to FIFO
static mcp2518fd_error_t write_message_to_fifo(mcp2518fd_t *dev, const mcp2518fd_message_t *message) {
    if (dev == NULL || message == NULL) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    // Get TXQ User Address
    uint32_t txqua;
    mcp2518fd_error_t err = read_register(dev, REG_CiTXQUA, &txqua);
    if (err != MCP2518FD_OK) {
        return err;
    }

    uint16_t address = (uint16_t)(txqua & 0xFFFF);

    uint8_t tx_buf[3 + 72]; // Command + max message size
    tx_buf[0] = MCP2518FD_SPI_INSTRUCTION_WRITE;
    tx_buf[1] = (address >> 8) & 0xFF;
    tx_buf[2] = address & 0xFF;

    uint8_t *ptr = tx_buf + 3;

    // Construct Message Object
    uint32_t id = 0;
    if (message->extended) {
        id = (message->id & 0x1FFFFFFF) | 0x80000000;
    } else {
        id = (message->id & 0x7FF) << 18;
    }

    ptr[0] = (id >> 24) & 0xFF;
    ptr[1] = (id >> 16) & 0xFF;
    ptr[2] = (id >> 8) & 0xFF;
    ptr[3] = id & 0xFF;

    // DLC and Flags
    uint8_t dlc = message->dlc;
    uint8_t length_code = 0;
    if (dlc <= 8) {
        length_code = dlc;
    } else if (dlc <= 12) {
        length_code = 9;
    } else if (dlc <= 16) {
        length_code = 10;
    } else if (dlc <= 20) {
        length_code = 11;
    } else if (dlc <= 24) {
        length_code = 12;
    } else if (dlc <= 32) {
        length_code = 13;
    } else if (dlc <= 48) {
        length_code = 14;
    } else {
        length_code = 15;
    }

    ptr[4] = length_code & 0x0F;
    ptr[5] = 0;
    if (message->fd_frame) {
        ptr[5] |= 0x80; // FDF
    }
    if (message->brs) {
        ptr[5] |= 0x40; // BRS
    }
    if (message->esi) {
        ptr[5] |= 0x20; // ESI
    }
    // Time Stamp (optional)
    ptr[6] = 0x00;
    ptr[7] = 0x00;

    // Copy Data
    memcpy(ptr + 8, message->data, message->dlc);

    // Total length
    size_t length = 3 + 8 + message->dlc;

    // Write to TXQ
    err = spi_transfer(dev, tx_buf, NULL, length);
    if (err != MCP2518FD_OK) {
        return err;
    }

    // Update TXQ (Set UINC)
    uint32_t txqcon;
    err = read_register(dev, REG_CiTXQCON, &txqcon);
    if (err != MCP2518FD_OK) {
        return err;
    }

    txqcon |= 0x00010000; // Set UINC
    err = write_register(dev, REG_CiTXQCON, txqcon);
    if (err != MCP2518FD_OK) {
        return err;
    }

    return MCP2518FD_OK;
}

// SPI Transfer Function
static mcp2518fd_error_t spi_transfer(mcp2518fd_t *dev, uint8_t *tx_data, uint8_t *rx_data, size_t length) {
    if (dev == NULL || tx_data == NULL || length == 0) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    spi_transaction_t t = {
        .length = length * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    esp_err_t ret = spi_device_transmit(dev->spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
        return MCP2518FD_SPI_ERROR;
    }

    return MCP2518FD_OK;
}

// Write Register
static mcp2518fd_error_t write_register(mcp2518fd_t *dev, uint16_t address, uint32_t value) {
    uint8_t tx_buf[7];
    tx_buf[0] = MCP2518FD_SPI_INSTRUCTION_WRITE;
    tx_buf[1] = (address >> 8) & 0xFF;
    tx_buf[2] = address & 0xFF;
    tx_buf[3] = (value >> 24) & 0xFF;
    tx_buf[4] = (value >> 16) & 0xFF;
    tx_buf[5] = (value >> 8) & 0xFF;
    tx_buf[6] = value & 0xFF;
    return spi_transfer(dev, tx_buf, NULL, sizeof(tx_buf));
}

// Read Register
static mcp2518fd_error_t read_register(mcp2518fd_t *dev, uint16_t address, uint32_t *value) {
    if (value == NULL) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    uint8_t tx_buf[7] = {0};
    uint8_t rx_buf[7] = {0};
    tx_buf[0] = MCP2518FD_SPI_INSTRUCTION_READ;
    tx_buf[1] = (address >> 8) & 0xFF;
    tx_buf[2] = address & 0xFF;

    mcp2518fd_error_t ret = spi_transfer(dev, tx_buf, rx_buf, sizeof(tx_buf));
    if (ret != MCP2518FD_OK) {
        return ret;
    }

    *value = ((uint32_t)rx_buf[3] << 24) |
             ((uint32_t)rx_buf[4] << 16) |
             ((uint32_t)rx_buf[5] << 8) |
             ((uint32_t)rx_buf[6]);

    return MCP2518FD_OK;
}

// Interrupt Handler
static void IRAM_ATTR handle_interrupt(void *arg) {
    mcp2518fd_t *dev = (mcp2518fd_t *)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;

    // Read Interrupt Flags
    uint32_t intf;
    if (read_register(dev, REG_CiINTFLAG, &intf) != MCP2518FD_OK) {
        ESP_EARLY_LOGE(TAG, "Failed to read interrupt flags");
        return;
    }

    // Handle RX FIFO Interrupt
    if (intf & 0x00000001) {
        // Read messages from RX FIFO
        mcp2518fd_message_t message;
        while (read_message_from_fifo(dev, &message) == MCP2518FD_OK) {
            xQueueSendFromISR(dev->config.rx_queue, &message, &higher_priority_task_woken);
        }
    }

    // Clear Interrupt Flags
    if (write_register(dev, REG_CiINTFLAG, intf) != MCP2518FD_OK) {
        ESP_EARLY_LOGE(TAG, "Failed to clear interrupt flags");
    }

    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

// Get Status
mcp2518fd_error_t mcp2518fd_get_status(mcp2518fd_t *dev, uint32_t *status) {
    if (dev == NULL || status == NULL) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    return read_register(dev, REG_CiINTFLAG, status);
}

// Get Error Counts
mcp2518fd_error_t mcp2518fd_get_error_counts(mcp2518fd_t *dev, uint8_t *tx_err_count, uint8_t *rx_err_count) {
    if (dev == NULL || tx_err_count == NULL || rx_err_count == NULL) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    uint32_t trec;
    mcp2518fd_error_t err = read_register(dev, REG_CiTREC, &trec);
    if (err != MCP2518FD_OK) {
        return err;
    }

    *tx_err_count = (trec >> 8) & 0xFF;
    *rx_err_count = trec & 0xFF;
    return MCP2518FD_OK;
}

// Configure Filter (Optional)
mcp2518fd_error_t mcp2518fd_configure_filter(mcp2518fd_t *dev, uint8_t filter_number, uint32_t id, uint32_t mask, bool extended) {
    if (dev == NULL || filter_number >= 32) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    // Configure Filter Object
    uint32_t fltobj = 0;
    if (extended) {
        fltobj = (id & 0x1FFFFFFF) | 0x80000000;
    } else {
        fltobj = (id & 0x7FF) << 21;
    }
    mcp2518fd_error_t err = write_register(dev, REG_CiFLTOBJ + (filter_number * 8), fltobj);
    if (err != MCP2518FD_OK) {
        return err;
    }

    // Configure Mask
    uint32_t maskreg = 0;
    if (extended) {
        maskreg = (mask & 0x1FFFFFFF) | 0x80000000;
    } else {
        maskreg = (mask & 0x7FF) << 21;
    }
    err = write_register(dev, REG_CiMASK + (filter_number * 8), maskreg);
    if (err != MCP2518FD_OK) {
        return err;
    }

    // Link Filter to FIFO
    uint32_t fltcon = (0x00000080) | (0x00000000); // Enable filter, link to FIFO 0
    err = write_register(dev, REG_CiFLTCON + (filter_number * 4), fltcon);
    if (err != MCP2518FD_OK) {
        return err;
    }

    return MCP2518FD_OK;
}

// Set Baud Rates
mcp2518fd_error_t mcp2518fd_set_baud_rates(mcp2518fd_t *dev, uint32_t nominal_bitrate, uint32_t data_bitrate) {
    if (dev == NULL || nominal_bitrate == 0 || data_bitrate == 0) {
        return MCP2518FD_INVALID_ARGUMENT;
    }

    dev->config.nominal_bitrate = nominal_bitrate;
    dev->config.data_bitrate = data_bitrate;

    // Enter Configuration Mode
    mcp2518fd_error_t err = set_operation_mode(dev, MCP2518FD_MODE_CONFIGURATION);
    if (err != MCP2518FD_OK) {
        return err;
    }

    // Reconfigure Bit Timing
    err = configure_bit_timing(dev);
    if (err != MCP2518FD_OK) {
        return err;
    }

    // Return to Previous Operation Mode
    err = set_operation_mode(dev, dev->config.op_mode);
    if (err != MCP2518FD_OK) {
        return err;
    }

    return MCP2518FD_OK;
}
