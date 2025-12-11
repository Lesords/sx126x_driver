#include "sx126x_hal.h"
#include "sx126x_hal_linux.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <time.h>
#include <errno.h>

#define SPI_SPEED_HZ 100000 // 100 kHz

// Helper to export and configure GPIO
static int gpio_export(int gpio) {
    char buffer[64];
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return -1;
    int len = snprintf(buffer, sizeof(buffer), "%d", gpio);
    write(fd, buffer, len);
    close(fd);
    return 0;
}

static int gpio_direction(int gpio, int dir) { // 0: in, 1: out
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "/sys/class/gpio/gpio%d/direction", gpio);
    int fd = open(buffer, O_WRONLY);
    if (fd < 0) return -1;
    if (dir) write(fd, "out", 3);
    else write(fd, "in", 2);
    close(fd);
    return 0;
}

int gpio_set_value(int gpio, int value) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "/sys/class/gpio/gpio%d/value", gpio);
    int fd = open(buffer, O_WRONLY);
    if (fd < 0) return -1;
    if (value) write(fd, "1", 1);
    else write(fd, "0", 1);
    close(fd);
    return 0;
}

int gpio_get_value(int gpio) {
    char buffer[64];
    char value_str[3];
    snprintf(buffer, sizeof(buffer), "/sys/class/gpio/gpio%d/value", gpio);
    int fd = open(buffer, O_RDONLY);
    if (fd < 0) return -1;
    read(fd, value_str, 3);
    close(fd);
    return atoi(value_str);
}

int sx126x_hal_linux_init(sx126x_hal_context_t *ctx) {
    // Open SPI
    ctx->spi_fd = open(ctx->spidev_path, O_RDWR);
    if (ctx->spi_fd < 0) {
        perror("Failed to open SPI device");
        return -1;
    }

    // Configure SPI
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED_HZ;

    // Force SPI Mode 0 (CPOL=0, CPHA=0)
    if (ioctl(ctx->spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("Failed to set SPI mode");
        return -1;
    }
    if (ioctl(ctx->spi_fd, SPI_IOC_RD_MODE, &mode) < 0) {
        perror("Failed to set SPI RD mode");
        return -1;
    }
    if (ioctl(ctx->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("Failed to set SPI bits");
        return -1;
    }
    if (ioctl(ctx->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("Failed to set SPI speed");
        return -1;
    }

    // Configure GPIOs
    gpio_export(ctx->reset_gpio);
    gpio_direction(ctx->reset_gpio, 1); // Output
    
    gpio_export(ctx->busy_gpio);
    gpio_direction(ctx->busy_gpio, 0); // Input

    if (ctx->dio1_gpio >= 0) {
        gpio_export(ctx->dio1_gpio);
        gpio_direction(ctx->dio1_gpio, 0); // Input
    }

    if (ctx->rf_sw_gpio >= 0) {
        gpio_export(ctx->rf_sw_gpio);
        gpio_direction(ctx->rf_sw_gpio, 1); // Output
        gpio_set_value(ctx->rf_sw_gpio, 1); // Default High (Enable?)
    }

    return 0;
}

void sx126x_hal_linux_cleanup(sx126x_hal_context_t *ctx) {
    if (ctx->spi_fd >= 0) close(ctx->spi_fd);
    // Unexport GPIOs if needed, but usually fine to leave them
}

static void sx126x_hal_wait_on_busy( const void* context ) {
    const sx126x_hal_context_t* ctx = ( const sx126x_hal_context_t* ) context;
    
    // Wait for BUSY to go low
    // Simple polling with timeout
    int timeout = 1000; // 1s timeout roughly
    int busy_val = gpio_get_value(ctx->busy_gpio);
    
    // DEBUG: Print BUSY state
    // printf("HAL: BUSY State = %d\n", busy_val);

    if (busy_val == 1) {
        printf("HAL: BUSY is High, waiting...\n");
        while (busy_val == 1 && timeout > 0) {
            usleep(1000); // 1ms
            timeout--;
            busy_val = gpio_get_value(ctx->busy_gpio);
        }
        if (timeout == 0) {
            printf("HAL: BUSY Timeout! GPIO %d stuck High?\n", ctx->busy_gpio);
        } else {
            printf("HAL: BUSY Released.\n");
        }
    }
}

sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length ) {
    const sx126x_hal_context_t* ctx = ( const sx126x_hal_context_t* ) context;
    
    sx126x_hal_wait_on_busy(context);

    // Combine command and data into a single buffer to ensure CS stays low
    uint8_t* tx_buffer = (uint8_t*)malloc(command_length + data_length);
    if (!tx_buffer) return SX126X_HAL_STATUS_ERROR;

    memcpy(tx_buffer, command, command_length);
    if (data_length > 0) {
        memcpy(tx_buffer + command_length, data, data_length);
    }

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));

    // We need a RX buffer to see the status byte even for writes
    uint8_t* rx_buffer = (uint8_t*)malloc(command_length + data_length);
    if (!rx_buffer) {
        free(tx_buffer);
        return SX126X_HAL_STATUS_ERROR;
    }
    memset(rx_buffer, 0, command_length + data_length);

    tr.tx_buf = (unsigned long)tx_buffer;
    tr.rx_buf = (unsigned long)rx_buffer;
    tr.len = command_length + data_length;
    tr.speed_hz = SPI_SPEED_HZ;
    tr.bits_per_word = 8;
    tr.delay_usecs = 10; // Add delay to ensure timing

    int ret = ioctl(ctx->spi_fd, SPI_IOC_MESSAGE(1), &tr);
    
    if (ret < 0) {
        perror("SPI write failed");
        free(tx_buffer);
        free(rx_buffer);
        return SX126X_HAL_STATUS_ERROR;
    }

    // DEBUG: Print Write Response (Status)
#ifdef DEBUG
    printf("SPI Write (Cmd=0x%02X, Len=%d): ", command[0], command_length + data_length);
    for(int i=0; i<command_length + data_length; i++) printf("%02X ", rx_buffer[i]);
    printf("\n");
#endif

    free(tx_buffer);
    free(rx_buffer);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length ) {
    const sx126x_hal_context_t* ctx = ( const sx126x_hal_context_t* ) context;

    sx126x_hal_wait_on_busy(context);

    // Combine command and data read into a single transaction
    // We send Command + NOPs (for data read)
    // We receive Status + Data
    
    uint16_t total_len = command_length + data_length;
    uint8_t* tx_buffer = (uint8_t*)malloc(total_len);
    uint8_t* rx_buffer = (uint8_t*)malloc(total_len);
    
    if (!tx_buffer || !rx_buffer) {
        if (tx_buffer) free(tx_buffer);
        if (rx_buffer) free(rx_buffer);
        return SX126X_HAL_STATUS_ERROR;
    }

    memset(tx_buffer, 0, total_len);
    memcpy(tx_buffer, command, command_length);
    // The rest of tx_buffer (data part) is 0 (NOP)

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));

    tr.tx_buf = (unsigned long)tx_buffer;
    tr.rx_buf = (unsigned long)rx_buffer;
    tr.len = total_len;
    tr.speed_hz = SPI_SPEED_HZ;
    tr.bits_per_word = 8;
    tr.cs_change = 0; // Keep CS active if needed, but usually 0 is fine for single transfer
    tr.delay_usecs = 10; // Add delay

    int ret = ioctl(ctx->spi_fd, SPI_IOC_MESSAGE(1), &tr);
    
    if (ret < 0) {
        perror("SPI read failed");
        free(tx_buffer);
        free(rx_buffer);
        return SX126X_HAL_STATUS_ERROR;
    }

    // DEBUG: Print RX Buffer
#ifdef DEBUG
    printf("SPI Read (CmdLen=%d, DataLen=%d): ", command_length, data_length);
    for(int i=0; i<total_len; i++) printf("%02X ", rx_buffer[i]);
    printf("\n");
#endif

    // Copy the data part to the user buffer
    if (data_length > 0) {
        memcpy(data, rx_buffer + command_length, data_length);
    }

    free(tx_buffer);
    free(rx_buffer);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset( const void* context ) {
    const sx126x_hal_context_t* ctx = ( const sx126x_hal_context_t* ) context;
    
    printf("HAL: Asserting Reset (GPIO %d)...\n", ctx->reset_gpio);
    gpio_set_value(ctx->reset_gpio, 0);
    usleep(20000); // 20ms
    gpio_set_value(ctx->reset_gpio, 1);
    printf("HAL: Released Reset. Waiting 100ms for boot...\n");
    usleep(100000); // 100ms wait to ensure chip is ready even if BUSY pin is wrong
    
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup( const void* context ) {
    const sx126x_hal_context_t* ctx = ( const sx126x_hal_context_t* ) context;
    
    // To wakeup, we can just perform a GetStatus command or simply toggle CS.
    // Here we send a GetStatus command (0xC0) with 0 length data.
    uint8_t cmd = 0xC0;
    sx126x_hal_write(context, &cmd, 1, NULL, 0);
    
    return SX126X_HAL_STATUS_OK;
}
