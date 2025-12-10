#ifndef SX126X_HAL_LINUX_H
#define SX126X_HAL_LINUX_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    const char *spidev_path;
    int spi_fd;
    int reset_gpio; // GPIO number (sysfs)
    int busy_gpio;  // GPIO number (sysfs)
    int dio1_gpio;  // GPIO number (sysfs) - Optional for basic test
    int rf_sw_gpio; // GPIO number (sysfs) - Optional for RF Switch
} sx126x_hal_context_t;

// Function to initialize the Linux HAL (open SPI, export GPIOs)
int sx126x_hal_linux_init(sx126x_hal_context_t *ctx);

// Function to cleanup
void sx126x_hal_linux_cleanup(sx126x_hal_context_t *ctx);

// Helper to control GPIO manually
int gpio_set_value(int gpio, int value);

#endif // SX126X_HAL_LINUX_H
