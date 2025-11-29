#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "sx126x.h"
#include "sx126x_hal_linux.h"

// --- CONFIGURATION ---
// PLEASE UPDATE THESE VALUES FOR YOUR HARDWARE
#define SPI_DEV_PATH "/dev/spidev3.0"
#define GPIO_RESET   546  // Example GPIO number
#define GPIO_BUSY    600  // Example GPIO number
#define GPIO_DIO1    606   // Set to your DIO1 GPIO number (e.g., 547). Set -1 if not used (polling SPI).
// ---------------------

#define RF_FREQUENCY 470000000 // 470 MHz (Common in China)
//#define RF_FREQUENCY 868000000 // 868 MHz
#define TX_OUTPUT_POWER 14     // dBm

sx126x_hal_context_t hal_ctx = {
    .spidev_path = SPI_DEV_PATH,
    .reset_gpio = GPIO_RESET,
    .busy_gpio = GPIO_BUSY,
    .dio1_gpio = GPIO_DIO1,
    .spi_fd = -1
};

void check_status(sx126x_status_t status, const char* msg) {
    if (status != SX126X_STATUS_OK) {
        printf("Error: %s failed with status %d\n", msg, status);
        // exit(1); // Optional: exit on error
    }
}

int main() {
    printf("Starting SX126x LoRa Demo...\n");

    // 1. Initialize HAL
    if (sx126x_hal_linux_init(&hal_ctx) != 0) {
        fprintf(stderr, "Failed to initialize HAL\n");
        return 1;
    }
    printf("HAL Initialized.\n");

    // 2. Reset Radio
    printf("Resetting radio...\n");
    sx126x_reset(&hal_ctx);
    
    // --- DIAGNOSTICS: Read Chip Info ---
    sx126x_chip_status_t chip_status;
    sx126x_get_status(&hal_ctx, &chip_status);
    printf("Chip Status: CmdStatus=%d, ChipMode=%d\n", chip_status.cmd_status, chip_status.chip_mode);
    
    if (chip_status.chip_mode == 0 || chip_status.chip_mode == 0xFF) {
        printf("ERROR: SPI Read failed (read 0x00 or 0xFF). Check wiring (MISO)!\n");
        return 1;
    }

    // Set Standby first to allow reading registers safely
    check_status(sx126x_set_standby(&hal_ctx, SX126X_STANDBY_CFG_RC), "Set Standby");

    // --- 1. 优先配置 TCXO (解决起振失败问题) ---
    // Configure DIO3 as TCXO control (Voltage: 1.7V, Timeout: 5ms)
    check_status(sx126x_set_dio3_as_tcxo_ctrl(&hal_ctx, SX126X_TCXO_CTRL_1_7V, 320), "Set TCXO");

    // Configure DIO2 as RF Switch control
    check_status(sx126x_set_dio2_as_rf_sw_ctrl(&hal_ctx, true), "Set DIO2 as RF Switch");
    
    // --- 2. 清除启动时的瞬态错误 ---
    sx126x_clear_device_errors(&hal_ctx);
    // ----------------------------------

    // --- 3. 现在检查错误 (应该是 0x0000) ---
    sx126x_errors_mask_t device_errors;
    sx126x_get_device_errors(&hal_ctx, &device_errors);
    printf("Device Errors: 0x%04X\n", device_errors);
    if (device_errors & SX126X_ERRORS_RC64K_CALIBRATION) printf("  - RC64K Calibration Failed\n");
    if (device_errors & SX126X_ERRORS_RC13M_CALIBRATION) printf("  - RC13M Calibration Failed\n");
    if (device_errors & SX126X_ERRORS_PLL_CALIBRATION)   printf("  - PLL Calibration Failed\n");
    if (device_errors & SX126X_ERRORS_ADC_CALIBRATION)   printf("  - ADC Calibration Failed\n");
    if (device_errors & SX126X_ERRORS_IMG_CALIBRATION)   printf("  - Image Calibration Failed\n");
    if (device_errors & SX126X_ERRORS_XOSC_START)  printf("  - XOSC Failed to Start\n");
    if (device_errors & SX126X_ERRORS_PLL_LOCK)    printf("  - PLL Lock Failed\n");
    if (device_errors & SX126X_ERRORS_PA_RAMP)     printf("  - PA Ramp Failed\n");

    // 4. Set Packet Type
    check_status(sx126x_set_pkt_type(&hal_ctx, SX126X_PKT_TYPE_LORA), "Set Packet Type");

    // 5. Set RF Frequency
    check_status(sx126x_set_rf_freq(&hal_ctx, RF_FREQUENCY), "Set RF Frequency");

    // 6. Set PA Config
    sx126x_pa_cfg_params_t pa_params;
    pa_params.pa_duty_cycle = 0x04;
    pa_params.hp_max        = 0x07;
    pa_params.device_sel    = 0x00; // SX1262
    pa_params.pa_lut        = 0x01;
    check_status(sx126x_set_pa_cfg(&hal_ctx, &pa_params), "Set PA Config");

    // 7. Set TX Params (Keep it for reference, though we are doing RX)
    check_status(sx126x_set_tx_params(&hal_ctx, TX_OUTPUT_POWER, SX126X_RAMP_200_US), "Set TX Params");

    // 8. Set Buffer Base Address
    check_status(sx126x_set_buffer_base_address(&hal_ctx, 0x00, 0x00), "Set Buffer Base Address");

    // 9. Set Modulation Params (LoRa)
    sx126x_mod_params_lora_t mod_params;
    mod_params.sf = SX126X_LORA_SF7;
    mod_params.bw = SX126X_LORA_BW_125;
    mod_params.cr = SX126X_LORA_CR_4_5;
    mod_params.ldro = 0; 
    
    check_status(sx126x_set_lora_mod_params(&hal_ctx, &mod_params), "Set Modulation Params");

        // 10. Set Packet Params
    sx126x_pkt_params_lora_t pkt_params;
    pkt_params.preamble_len_in_symb = 8;
    pkt_params.header_type = SX126X_LORA_PKT_EXPLICIT;
    pkt_params.pld_len_in_bytes = 255; // Max length for RX
    pkt_params.crc_is_on = true;
    pkt_params.invert_iq_is_on = false;

    check_status(sx126x_set_lora_pkt_params(&hal_ctx, &pkt_params), "Set Packet Params");

    // 11. Set DIO IRQ Params (RX_DONE)

    // 11. Set DIO IRQ Params (RX_DONE)
    check_status(sx126x_set_dio_irq_params(&hal_ctx, SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT,
                                           SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT,
                                           SX126X_IRQ_NONE, SX126X_IRQ_NONE), "Set DIO IRQ");

    // 12. Start RX (Continuous)
    printf("Starting RX (Continuous)...\n");
    
    // Check for errors one last time before starting
    sx126x_get_device_errors(&hal_ctx, &device_errors);
    if (device_errors != 0) {
        printf("WARNING: Device Errors before RX: 0x%04X\n", device_errors);
        sx126x_clear_device_errors(&hal_ctx);
    }

    // SX126X_RX_CONTINUOUS is 0xFFFFFF
    // Note: sx126x_set_rx takes timeout in ms.
    // If we pass 0xFFFFFF, it's a huge timeout (hours).
    // If we pass 0, it's Single mode without timeout (wait forever).
    // To be safe for continuous RX, we should use a large value or 0 if we restart manually.
    // Let's try 0 (Single mode, wait forever) and restart in loop.
    check_status(sx126x_set_rx(&hal_ctx, 0), "Set RX"); 

    // 13. Wait for RxDone
    printf("Waiting for packets...\n");
    uint16_t irq_status = 0;
    sx126x_pkt_status_lora_t pkt_status;
    sx126x_chip_status_t loop_status;
    int loop_cnt = 0;

    while (1) {
        // Every 1 second, print chip status to ensure it's still in RX mode
        if (loop_cnt++ % 100 == 0) {
            sx126x_get_status(&hal_ctx, &loop_status);
            // Mode 5 = RX, Mode 2 = STDBY_RC, Mode 3 = STDBY_XOSC
            printf("[Status Monitor] Mode: %d (Should be 5/RX), CmdStatus: %d\n", 
                   loop_status.chip_mode, loop_status.cmd_status);
        }

        sx126x_get_irq_status(&hal_ctx, &irq_status);
        
        if (irq_status & SX126X_IRQ_RX_DONE) {
            printf("\nPacket Received!\n");
            
            // Get payload length and pointer
            sx126x_rx_buffer_status_t rx_status;
            sx126x_get_rx_buffer_status(&hal_ctx, &rx_status);
            
            printf("Length: %d, Start: %d\n", rx_status.pld_len_in_bytes, rx_status.buffer_start_pointer);

            // Read payload
            uint8_t buffer[256];
            memset(buffer, 0, sizeof(buffer));
            if (rx_status.pld_len_in_bytes > 0) {
                sx126x_read_buffer(&hal_ctx, rx_status.buffer_start_pointer, buffer, rx_status.pld_len_in_bytes);
                printf("Data (Hex): ");
                for(int i=0; i<rx_status.pld_len_in_bytes; i++) printf("%02X ", buffer[i]);
                printf("\nData (Str): %s\n", buffer);
            }

            // Get RSSI/SNR
            sx126x_get_lora_pkt_status(&hal_ctx, &pkt_status);
            printf("RSSI: %d dBm, SNR: %d dB\n", pkt_status.rssi_pkt_in_dbm, pkt_status.snr_pkt_in_db);

            // Clear IRQ
            sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_ALL);
            
            // Restart RX
            sx126x_set_rx(&hal_ctx, 0);
        }
        else if (irq_status & SX126X_IRQ_CRC_ERROR) {
            printf("CRC Error!\n");
            sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_ALL);
            sx126x_set_rx(&hal_ctx, 0);
        }
        else if (irq_status & SX126X_IRQ_TIMEOUT) {
            printf("RX Timeout (should not happen in continuous mode)\n");
            sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_ALL);
            sx126x_set_rx(&hal_ctx, 0);
        }

        usleep(10000); // 10ms
    }

    sx126x_hal_linux_cleanup(&hal_ctx);
    return 0;
}
