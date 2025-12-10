#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "sx126x.h"
#include "sx126x_regs.h"
#include "sx126x_hal_linux.h"

// --- CONFIGURATION ---
// PLEASE UPDATE THESE VALUES FOR YOUR HARDWARE
#define SPI_DEV_PATH "/dev/spidev3.0"
#define GPIO_BASE    519
#define GPIO_RESET   (GPIO_BASE + 34)
#define GPIO_BUSY    (GPIO_BASE + 88)
#define GPIO_DIO1    (GPIO_BASE + 94)
#define GPIO_RF_SW   (GPIO_BASE + 41)
// ---------------------

#define RF_FREQUENCY 915000000 // 915 MHz (For Wio-SX1262 High Band)
//#define RF_FREQUENCY 868000000 // 868 MHz
#define TX_OUTPUT_POWER 20     // dBm (Max Power for SX1262)

sx126x_hal_context_t hal_ctx = {
    .spidev_path = SPI_DEV_PATH,
    .reset_gpio = GPIO_RESET,
    .busy_gpio = GPIO_BUSY,
    .dio1_gpio = GPIO_DIO1,
    .rf_sw_gpio = GPIO_RF_SW,
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

    // --- NEW: Read Hardware Version ---
    // Register 0x0320 contains the hardware version (usually 0x12 for SX1261/2/8)
    uint8_t hw_version = 0;
    sx126x_read_register(&hal_ctx, 0x0320, &hw_version, 1);
    printf("Hardware Version: 0x%02X (Expected: 0x12)\n", hw_version);
    // ----------------------------------

    // Set Standby first to allow reading registers safely
    check_status(sx126x_set_standby(&hal_ctx, SX126X_STANDBY_CFG_RC), "Set Standby");

    // --- 1. 优先配置 TCXO (解决起振失败问题) ---
    // Configure DIO3 as TCXO control (Voltage: 1.7V, Timeout: 20ms)
    // 增加超时时间：320 -> 1280 (20ms)，确保 TCXO 有足够时间稳定
    check_status(sx126x_set_dio3_as_tcxo_ctrl(&hal_ctx, SX126X_TCXO_CTRL_1_7V, 1280), "Set TCXO");
    
    // 显式设置为 DCDC 模式 (Wio-SX1262 通常支持 DCDC)
    check_status(sx126x_set_reg_mode(&hal_ctx, SX126X_REG_MODE_DCDC), "Set Regulator DCDC");

    // Calibrate Image (Important for Frequency Setting)
    // For 915MHz (Band 902-928), we use freq 902 & 928
    check_status(sx126x_cal_img(&hal_ctx, 0xE1, 0xE9), "Calibrate Image (902-928MHz)");

    // Configure DIO2 as RF Switch control
    check_status(sx126x_set_dio2_as_rf_sw_ctrl(&hal_ctx, true), "Set DIO2 as RF Switch");
    
    // --- 2. 强制切换到 XOSC 模式以验证时钟 ---
    // 这会立即尝试启动 TCXO。如果失败，Device Errors 会置位。
    check_status(sx126x_set_standby(&hal_ctx, SX126X_STANDBY_CFG_XOSC), "Set Standby XOSC");
    
    // --- 3. 清除启动时的瞬态错误 ---
    sx126x_clear_device_errors(&hal_ctx);
    
    // --- Calibrate Image (Important for Frequency Setting) ---
    // For 915MHz (Band 902-928), we use freq 902 & 928
    check_status(sx126x_cal_img(&hal_ctx, 0xE1, 0xE9), "Calibrate Image (902-928MHz)");
    // ----------------------------------

    // --- 3. 现在检查错误 (应该是 0x0000) ---
    sx126x_errors_mask_t device_errors;
    sx126x_get_device_errors(&hal_ctx, &device_errors);
    printf("Device Errors: 0x%04X\n", device_errors);
    
    // Check Status again to see if it cleared
    sx126x_get_status(&hal_ctx, &chip_status);
    printf("Chip Status after Init: CmdStatus=%d, ChipMode=%d\n", chip_status.cmd_status, chip_status.chip_mode);
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
    // check_status(sx126x_set_rf_freq(&hal_ctx, RF_FREQUENCY), "Set RF Frequency");
    // MOVED TO LOOP FOR SCANNING

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

        // --- PHASE 1: TX (Send 100 packets rapidly for Spectrum Analyzer) ---
    printf("--- PHASE 1: Sending 100 Packets (Rapid Fire) ---\n");
    
    // TX Configuration
    sx126x_pkt_params_lora_t tx_pkt_params;
    tx_pkt_params.preamble_len_in_symb = 12; // Longer preamble
    tx_pkt_params.header_type = SX126X_LORA_PKT_EXPLICIT;
    tx_pkt_params.pld_len_in_bytes = 64; // Longer payload for visibility
    tx_pkt_params.crc_is_on = true;      
    tx_pkt_params.invert_iq_is_on = false; // Standard IQ for Uplink
    check_status(sx126x_set_lora_pkt_params(&hal_ctx, &tx_pkt_params), "Set TX Packet Params");

    // Set Sync Word (Public)
    check_status(sx126x_set_lora_sync_word(&hal_ctx, 0x34), "Set Sync Word (Public)");

    // --- DIAGNOSTIC: Read back Sync Word ---
    uint8_t sync_word_buff[2] = {0};
    sx126x_read_register(&hal_ctx, SX126X_REG_LR_SYNCWORD, sync_word_buff, 2);
    uint16_t sync_word_val = (sync_word_buff[0] << 8) | sync_word_buff[1];
    printf(">>> SYNC WORD CHECK: Wrote 0x3444 (Public), Read 0x%04X <<<\n", sync_word_val);
    // ---------------------------------------

    // Set Frequency (Ensure it's 915MHz)
    printf("Setting Frequency to 915MHz via Command...\n");
    check_status(sx126x_set_rf_freq(&hal_ctx, 915000000), "Set RF Frequency (915MHz)");
    
    // Check Status immediately
    sx126x_get_status(&hal_ctx, &chip_status);
    printf("Status after SetFreq: CmdStatus=%d\n", chip_status.cmd_status);

    // --- VERIFY FREQUENCY ---
    // 1. Calculate Register Value for 915MHz
    // Freq = Reg * 32MHz / 2^25
    // Reg = 915000000 * 2^25 / 32000000 = 959447040 = 0x39300000
    uint32_t target_freq_reg = 0x39300000; 
    uint8_t freq_buff[4];
    freq_buff[0] = (target_freq_reg >> 24) & 0xFF;
    freq_buff[1] = (target_freq_reg >> 16) & 0xFF;
    freq_buff[2] = (target_freq_reg >> 8) & 0xFF;
    freq_buff[3] = (target_freq_reg >> 0) & 0xFF;

    // 2. Force Write to Register 0x0860 (To ensure it's readable)
    sx126x_write_register(&hal_ctx, 0x0860, freq_buff, 4);

    // 3. Read Back
    uint8_t freq_reg[4] = {0};
    sx126x_read_register(&hal_ctx, 0x0860, freq_reg, 4);
    uint32_t freq_reg_val = (freq_reg[0] << 24) | (freq_reg[1] << 16) | (freq_reg[2] << 8) | freq_reg[3];
    double freq_hz = (double)freq_reg_val * 32000000.0 / 33554432.0;
    
    printf(">>> FREQUENCY CHECK: Register 0x0860 = 0x%08X, Calculated = %.2f Hz <<<\n", freq_reg_val, freq_hz);
    printf(">>> TARGET FREQUENCY: %d Hz <<<\n", 915000000);
    
    if (freq_reg_val == 0) {
         printf("WARNING: Register readback failed. Ignoring if TX DONE works.\n");
    }
    // ------------------------

    // Set IRQ for TX
    check_status(sx126x_set_dio_irq_params(&hal_ctx, SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                                           SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                                           SX126X_IRQ_NONE, SX126X_IRQ_NONE), "Set TX IRQ");

    printf("--- PHASE 1: Sending 30 Packets (Rapid Fire) ---\n");
    printf("Note: Toggling RF_SW (GPIO %d) every 10 packets to test switch logic.\n", hal_ctx.rf_sw_gpio);

    int rf_sw_state = 1; // Start with HIGH (Default)
    gpio_set_value(hal_ctx.rf_sw_gpio, rf_sw_state);
    for (int i = 1; i <= 30; i++) {
        // Toggle RF_SW every 20 packets for testing
        if (i % 10 == 0) {
            rf_sw_state = !rf_sw_state;

            if (rf_sw_state == 1) {
                printf(">>> RF_SW (GPIO %d) set to HIGH (Default) <<<\n", hal_ctx.rf_sw_gpio);
            } else {
                printf(">>> RF_SW (GPIO %d) set to LOW (Testing Inverted Logic) <<<\n", hal_ctx.rf_sw_gpio);
            }

            gpio_set_value(hal_ctx.rf_sw_gpio, rf_sw_state);
        }

        printf("Sending Packet %d/30... ", i);
        
        uint8_t tx_buffer[64];
        memset(tx_buffer, 0xAA, sizeof(tx_buffer)); // Fill with pattern
        snprintf((char*)tx_buffer, sizeof(tx_buffer), "Ping %d - Wio-SX1262 Test Packet for Spectrum Analyzer...", i);
        
        sx126x_write_buffer(&hal_ctx, 0x00, tx_buffer, sizeof(tx_buffer));
        sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_ALL);
        sx126x_set_tx(&hal_ctx, 1000); // 1s timeout

        // Wait for TX_DONE
        int wait_limit = 100; 
        uint16_t irq_status = 0;
        bool tx_done = false;
        while(wait_limit-- > 0) {
            sx126x_get_irq_status(&hal_ctx, &irq_status);
            if (irq_status & SX126X_IRQ_TX_DONE) {
                printf("TX DONE!\n");
                tx_done = true;
                break;
            }
            if (irq_status & SX126X_IRQ_TIMEOUT) {
                printf("TX TIMEOUT!\n");
                break;
            }
            usleep(5000); // 5ms check
        }
        if (!tx_done) printf("Wait timeout\n");
        
        // Short delay to keep duty cycle high for visibility
        usleep(50000); // 50ms delay
    }
    
    // --- PHASE 2: RX (Continuous) ---
    printf("\n--- PHASE 2: Entering RX Mode ---\n");

    // RX Configuration
    sx126x_pkt_params_lora_t rx_pkt_params;
    rx_pkt_params.preamble_len_in_symb = 8;
    rx_pkt_params.header_type = SX126X_LORA_PKT_EXPLICIT;
    rx_pkt_params.pld_len_in_bytes = 64; // Max length for RX
    rx_pkt_params.crc_is_on = false;      // Don't enforce CRC check for now
    rx_pkt_params.invert_iq_is_on = false; // Listen for Downlink (Inverted IQ)
    check_status(sx126x_set_lora_pkt_params(&hal_ctx, &rx_pkt_params), "Set RX Packet Params");

    // Set IRQ for RX
    check_status(sx126x_set_dio_irq_params(&hal_ctx, SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT,
                                           SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT,
                                           SX126X_IRQ_NONE, SX126X_IRQ_NONE), "Set RX IRQ");

    // --- NEW: Set Sync Word back to Public (0x34) ---
    // Most Gateways use Public Network.
    check_status(sx126x_set_lora_sync_word(&hal_ctx, 0x34), "Set Sync Word (Public)");

    // Check for errors one last time before starting
    sx126x_get_device_errors(&hal_ctx, &device_errors);
    if (device_errors != 0) {
        printf("WARNING: Device Errors before RX: 0x%04X\n", device_errors);
        sx126x_clear_device_errors(&hal_ctx);
    }

    // Set initial frequency
    // 902300000
    int freq = RF_FREQUENCY;
    sx126x_set_rf_freq(&hal_ctx, freq);
    sx126x_set_rx(&hal_ctx, 0);

    // Start RX
    // SX126X_RX_CONTINUOUS is 0xFFFFFF
    // Note: sx126x_set_rx takes timeout in ms.
    // If we pass 0xFFFFFF, it's a huge timeout (hours).
    // If we pass 0, it's Single mode without timeout (wait forever).
    // To be safe for continuous RX, we should use a large value or 0 if we restart manually.
    // Let's try 0 (Single mode, wait forever) and restart in loop.
    check_status(sx126x_set_rx(&hal_ctx, 0), "Set RX"); 

    // Stop Rx
    // sx126x_set_standby(&hal_ctx, SX126X_STANDBY_CFG_RC);

    // Monitor RSSI
    int16_t rssi_inst = 0;
    sx126x_get_rssi_inst(&hal_ctx, &rssi_inst);

    printf("[RX Monitor] RSSI INST: %d dBm\n", rssi_inst);

    int loop_cnt = 0;
    while (1) {
        uint16_t irq_status = 0;
        sx126x_get_irq_status(&hal_ctx, &irq_status);

        if (irq_status & SX126X_IRQ_RX_DONE) {
            printf("\nPacket Received!\n");

            // Get payload length and pointer
            sx126x_rx_buffer_status_t rx_status;
            sx126x_get_rx_buffer_status(&hal_ctx, &rx_status);
            printf("Length: %d, Start: %d\n", rx_status.pld_len_in_bytes, rx_status.buffer_start_pointer);

            // Get RSSI/SNR
            sx126x_pkt_status_lora_t pkt_status;
            sx126x_get_lora_pkt_status(&hal_ctx, &pkt_status);
            printf("[PKT_STATUS RSSI] RSSI: %d dBm, SNR: %d dB\n", pkt_status.rssi_pkt_in_dbm, pkt_status.snr_pkt_in_db);

            uint8_t buffer[256];
            memset(buffer, 0, sizeof(buffer));
            if (rx_status.pld_len_in_bytes > 0) {
                sx126x_read_buffer(&hal_ctx, rx_status.buffer_start_pointer, buffer, rx_status.pld_len_in_bytes);
                printf("Data: %s\n", buffer);
            } else {
                printf("Data: <Empty>\n");
            }
            
            sx126x_get_lora_pkt_status(&hal_ctx, &pkt_status);

            // Clear IRQ
            sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_ALL);

            // Restart Rx
            sx126x_set_rx(&hal_ctx, 0);
        }
        else if (irq_status & SX126X_IRQ_CRC_ERROR) {
            printf("CRC Error!\n");
            sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_ALL);
            sx126x_set_rx(&hal_ctx, 0);
        }
        else if (irq_status & SX126X_IRQ_TIMEOUT) {
            sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_ALL);
            sx126x_set_rx(&hal_ctx, 0);
        }

        loop_cnt++;
        if (loop_cnt % 1000 == 0) {
            loop_cnt = 0;
            printf("[irq_status=0x%04X]\n", irq_status);
        }

        usleep(100 * 1000);
    }

    sx126x_hal_linux_cleanup(&hal_ctx);
    return 0;
}
