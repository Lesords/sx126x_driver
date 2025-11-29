#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>

// SX126x Commands
#define CMD_GET_STATUS 0xC0

static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len) {
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len,
        .speed_hz = 100000,
        .bits_per_word = 8,
    };

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1)
        perror("can't send spi message");
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s <device>\n", argv[0]);
        return 1;
    }

    const char *device = argv[1];
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("can't open device");
        return 1;
    }

    // Set SPI Mode 0
    uint8_t mode = SPI_MODE_0;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) {
        perror("can't set spi mode");
        return 1;
    }

    printf("Probing SX126x on %s (Mode 0)...\n", device);

    // Send GetStatus (0xC0) + NOP (0x00)
    uint8_t tx[] = { CMD_GET_STATUS, 0x00 };
    uint8_t rx[] = { 0, 0 };

    transfer(fd, tx, rx, sizeof(tx));

    printf("Sent: 0xC0 0x00\n");
    printf("Received: %02X %02X\n", rx[0], rx[1]);

    // Analyze Status (Byte 1 is the explicit status from GetStatus command)
    uint8_t status = rx[1]; 
    
    // DS_SX1261-2_V1.2.pdf Page 86:
    // Bit 7: Reserved
    // Bit 6:4: Chip Mode (0x2=STDBY_RC, 0x3=STDBY_XOSC, 0x4=FS, 0x5=RX, 0x6=TX)
    // Bit 3:1: Command Status
    // Bit 0: Reserved
    
    uint8_t chip_mode = (status >> 4) & 0x07;
    uint8_t cmd_status = (status >> 1) & 0x07;

    printf("Analysis:\n");
    printf("  Chip Mode: %d ", chip_mode);
    switch(chip_mode) {
        case 0x2: printf("(STDBY_RC)\n"); break;
        case 0x3: printf("(STDBY_XOSC)\n"); break;
        case 0x4: printf("(FS)\n"); break;
        case 0x5: printf("(RX)\n"); break;
        case 0x6: printf("(TX)\n"); break;
        default:  printf("(Unknown/Sleep)\n"); break;
    }

    printf("  Cmd Status: %d ", cmd_status);
    switch(cmd_status) {
        case 0x1: printf("(Success)\n"); break;
        case 0x2: printf("(Data Available)\n"); break;
        case 0x3: printf("(Cmd Timeout)\n"); break;
        case 0x4: printf("(Cmd Error)\n"); break;
        case 0x5: printf("(Failure)\n"); break;
        case 0x6: printf("(Tx Done)\n"); break;
        default:  printf("(Reserved)\n"); break;
    }

    if (status == 0x00 || status == 0xFF) {
        printf("\nFAILURE: Chip not responding (MISO stuck).\n");
    } else {
        printf("\nSUCCESS: Chip is alive!\n");
    }

    close(fd);
    return 0;
}
