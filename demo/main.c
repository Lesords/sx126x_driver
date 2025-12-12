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
// 外挂 RF_SW 由 SoC GPIO 控制，高=RX，低=TX
#define GPIO_RF_SW   (GPIO_BASE + 41)
// ---------------------

#define RF_FREQUENCY 915000000 // 915 MHz (For Wio-SX1262 High Band)
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

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s [tx|rx]\n", argv[0]);
        printf("  tx: Transmitter mode (Continuous)\n");
        printf("  rx: Receiver mode (Continuous)\n");
        return 1;
    }

    int mode = 0; // 1=TX, 2=RX
    int total_sum = 0xfffffff;
    if (strcmp(argv[1], "tx") == 0) mode = 1;
    else if (strcmp(argv[1], "rx") == 0) mode = 2;
    else {
        printf("Invalid mode. Use 'tx' or 'rx'.\n");
        return 1;
    }

    if (argc >= 3) {
        total_sum = atoi(argv[2]);
    }

    printf("Starting SX126x LoRa Demo in %s mode...\n", mode == 1 ? "TX" : "RX");

    // 1. Initialize HAL
    if (sx126x_hal_linux_init(&hal_ctx) != 0) {
        fprintf(stderr, "Failed to initialize HAL\n");
        return 1;
    }
    printf("HAL Initialized.\n");

    // 2. Reset Radio
    printf("Resetting radio...\n");
    sx126x_reset(&hal_ctx);

    // new addition
    sx126x_init_retention_list(&hal_ctx);
    sx126x_set_reg_mode(&hal_ctx, SX126X_REG_MODE_LDO);

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
    // 不能使用这个函数，会导致芯片休眠！！！
    // check_status(sx126x_set_standby(&hal_ctx, SX126X_STANDBY_CFG_RC), "Set Standby");

    // 先用 LDO 供电保证稳定，如需再评估切 DCDC
    check_status(sx126x_set_reg_mode(&hal_ctx, SX126X_REG_MODE_DCDC), "Set Regulator LDO");

    // 由芯片通过 DIO2 自动控制射频开关（同时保留外部 RF_SW GPIO）
    check_status(sx126x_set_dio2_as_rf_sw_ctrl(&hal_ctx, true), "Set DIO2 as RF Switch");

    // --- 1. 优先配置 TCXO & 供电模式 ---
    // Wio-SX1262 常用 1.8V TCXO，先保证时钟稳定，再切到 XOSC
    check_status(sx126x_set_dio3_as_tcxo_ctrl(&hal_ctx, SX126X_TCXO_CTRL_1_8V, 165), "Set TCXO 1.8V");
    // usleep(5000);

    sx126x_cal(&hal_ctx, SX126X_CAL_ALL); // 902-928MHz

    sx126x_cfg_rx_boosted(&hal_ctx, true); // RX Boosted for better sensitivity

    sx126x_stop_timer_on_preamble(&hal_ctx, false);
    sx126x_set_lora_symb_nb_timeout(&hal_ctx, 0); // No timeout

    // 4. Set Packet Type
    check_status(sx126x_set_pkt_type(&hal_ctx, SX126X_PKT_TYPE_LORA), "Set Packet Type");
    
    sx126x_set_rf_freq(&hal_ctx, RF_FREQUENCY);

    // // --- 2. 强制切换到 XOSC 模式以验证时钟 ---
    // // 这会立即尝试启动 TCXO。如果失败，Device Errors 会置位。
    // check_status(sx126x_set_standby(&hal_ctx, SX126X_STANDBY_CFG_XOSC), "Set Standby XOSC");

    // --- 3. 清除启动时的瞬态错误 ---
    // sx126x_clear_device_errors(&hal_ctx);

    // --- 3. 现在检查错误 (应该是 0x0000) ---
    // sx126x_errors_mask_t device_errors;
    // sx126x_get_device_errors(&hal_ctx, &device_errors);
    // printf("Device Errors: 0x%04X\n", device_errors);

    // // Check Status again to see if it cleared
    // sx126x_get_status(&hal_ctx, &chip_status);
    // printf("Chip Status after Init: CmdStatus=%d, ChipMode=%d\n", chip_status.cmd_status, chip_status.chip_mode);
    // if (device_errors & SX126X_ERRORS_RC64K_CALIBRATION) printf("  - RC64K Calibration Failed\n");
    // if (device_errors & SX126X_ERRORS_RC13M_CALIBRATION) printf("  - RC13M Calibration Failed\n");
    // if (device_errors & SX126X_ERRORS_PLL_CALIBRATION)   printf("  - PLL Calibration Failed\n");
    // if (device_errors & SX126X_ERRORS_ADC_CALIBRATION)   printf("  - ADC Calibration Failed\n");
    // if (device_errors & SX126X_ERRORS_IMG_CALIBRATION)   printf("  - Image Calibration Failed\n");
    // if (device_errors & SX126X_ERRORS_XOSC_START)  printf("  - XOSC Failed to Start\n");
    // if (device_errors & SX126X_ERRORS_PLL_LOCK)    printf("  - PLL Lock Failed\n");
    // if (device_errors & SX126X_ERRORS_PA_RAMP)     printf("  - PA Ramp Failed\n");

    sx126x_cfg_tx_clamp(&hal_ctx);

    // 6. Set PA Config
    sx126x_pa_cfg_params_t pa_params;
    pa_params.pa_duty_cycle = 0x04;
    pa_params.hp_max        = 0x07;
    pa_params.device_sel    = 0x00; // SX1262
    pa_params.pa_lut        = 0x01;
    check_status(sx126x_set_pa_cfg(&hal_ctx, &pa_params), "Set PA Config");


    // 7. Set TX Params (Keep it for reference, though we are doing RX)
    check_status(sx126x_set_tx_params(&hal_ctx, TX_OUTPUT_POWER, SX126X_RAMP_40_US), "Set TX Params");

    // 8. Set Buffer Base Address
    // check_status(sx126x_set_buffer_base_address(&hal_ctx, 0x00, 0x00), "Set Buffer Base Address");

    // 9. Set Modulation Params (LoRa)
    sx126x_mod_params_lora_t mod_params;
    mod_params.sf = SX126X_LORA_SF7;
    mod_params.bw = SX126X_LORA_BW_125;
    mod_params.cr = SX126X_LORA_CR_4_5;
    mod_params.ldro = 0; 
    check_status(sx126x_set_lora_mod_params(&hal_ctx, &mod_params), "Set Modulation Params");


    // TX Configuration 
    sx126x_pkt_params_lora_t tx_pkt_params;
    tx_pkt_params.preamble_len_in_symb = 8; 
    tx_pkt_params.header_type = SX126X_LORA_PKT_EXPLICIT;
    tx_pkt_params.pld_len_in_bytes = 0; 
    tx_pkt_params.crc_is_on = true;      
    tx_pkt_params.invert_iq_is_on = false; // Standard IQ for Uplink
    check_status(sx126x_set_lora_pkt_params(&hal_ctx, &tx_pkt_params), "Set TX Packet Params");


    // --- COMMON CONFIGURATION ---
    // Set Sync Word (Public)
    check_status(sx126x_set_lora_sync_word(&hal_ctx, 0x34), "Set Sync Word (Public)");

    // Set IRQ for TX
    check_status(sx126x_set_dio_irq_params(&hal_ctx, SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT | SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_HEADER_VALID,
                                            SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT | SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_HEADER_VALID,
                                            SX126X_IRQ_NONE, SX126X_IRQ_NONE), "Set TX IRQ");

    sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_ALL);

    // --- Calibrate Image (Important for Frequency Setting) ---
    // For 915MHz (Band 902-928), we use freq 902 & 928
    // check_status(sx126x_cal_img(&hal_ctx, 0xE1, 0xE9), "Calibrate Image (902-928MHz)");

    // Check Status immediately
    sx126x_get_status(&hal_ctx, &chip_status);
    printf("Status after SetFreq: CmdStatus=%d\n", chip_status.cmd_status);

    if (mode == 1) {
        printf("--- PHASE 1: TX Mode (Continuous) ---\n");

        // RF 开关：DIO2 自动 + 外部 RF_SW GPIO 双重控制
        printf("RF switch: DIO2 auto + GPIO %d LOW for TX.\n", hal_ctx.rf_sw_gpio);
        if (hal_ctx.rf_sw_gpio >= 0) {
            gpio_set_value(hal_ctx.rf_sw_gpio, 0); // 低电平 = 发射
        }

        // TX Configuration
        sx126x_pkt_params_lora_t tx_pkt_params;
        tx_pkt_params.preamble_len_in_symb = 8; 
        tx_pkt_params.header_type = SX126X_LORA_PKT_EXPLICIT;
        tx_pkt_params.pld_len_in_bytes = 64; 
        tx_pkt_params.crc_is_on = true;      
        tx_pkt_params.invert_iq_is_on = false; // Standard IQ for Uplink
        check_status(sx126x_set_lora_pkt_params(&hal_ctx, &tx_pkt_params), "Set TX Packet Params");

        // Set IRQ for TX
        check_status(sx126x_set_dio_irq_params(&hal_ctx, SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                                               SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                                               SX126X_IRQ_NONE, SX126X_IRQ_NONE), "Set TX IRQ");

        printf("\nStarting Continuous TX Loop...\n");
        if (total_sum != 0xfffffff) {
            printf("Will send total %d packets.\n", total_sum);
        } else {
            printf("Will send packets continuously.\n");
        }

        int i = 0;
        int sent_packets = 0;
        while (1) {
            i++;
            printf("\nSending Packet %d... ", i);

            uint8_t tx_buffer[64];
            memset(tx_buffer, 0xAA, sizeof(tx_buffer)); 
            snprintf((char*)tx_buffer, sizeof(tx_buffer), "Ping %d - Wio-SX1262 Test Packet...", i);

            sx126x_write_buffer(&hal_ctx, 0x00, tx_buffer, sizeof(tx_buffer));
            sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_ALL);
            sx126x_set_tx(&hal_ctx, 1000); // 1s timeout

            // Wait for TX_DONE
            int wait_limit = 200; 
            uint16_t irq_status = 0;
            bool tx_done = false;
            while(wait_limit-- > 0) {
                sx126x_get_irq_status(&hal_ctx, &irq_status);
                if (irq_status & SX126X_IRQ_TX_DONE) {
                    printf("TX DONE!\n");
                    tx_done = true;

                    if (++sent_packets >= total_sum) {
                        printf("\nSent total %d packets. Exiting.\n", sent_packets);
                        goto end;
                    }
                    break;
                }
                if (irq_status & SX126X_IRQ_TIMEOUT) {
                    printf("TX TIMEOUT!\n");
                    break;
                }
                usleep(5000); 
            }
            if (!tx_done) printf("Wait timeout\n");

            sleep(1); 
        }
    }

    if (mode == 2) {
        // --- PHASE 2: RX (Continuous) ---
        printf("\n--- PHASE 2: Entering RX Mode ---\n");

        // RF 开关：DIO2 自动 + 外部 RF_SW GPIO 双重控制
        printf("RF switch: DIO2 auto + GPIO %d HIGH for RX.\n", hal_ctx.rf_sw_gpio);
        if (hal_ctx.rf_sw_gpio >= 0) {
            gpio_set_value(hal_ctx.rf_sw_gpio, 1); // 高电平 = 接收
        }

        // RX Configuration(非必要，前面配置过了)
        sx126x_pkt_params_lora_t rx_pkt_params;
        rx_pkt_params.preamble_len_in_symb = 8;                         // Match TX
        rx_pkt_params.header_type          = SX126X_LORA_PKT_EXPLICIT;
        rx_pkt_params.pld_len_in_bytes     = 0;                         // Match TX
        rx_pkt_params.crc_is_on            = true;                      // Match TX
        rx_pkt_params.invert_iq_is_on      = false;                     // Match TX
        check_status(sx126x_set_lora_pkt_params(&hal_ctx, &rx_pkt_params), "Set RX Packet Params");

        // Set IRQ for RX (非必要，前面配置过了)
        check_status(sx126x_set_dio_irq_params(&hal_ctx, SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT | SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_HEADER_VALID,
                                               SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT | SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_HEADER_VALID,
                                               SX126X_IRQ_NONE, SX126X_IRQ_NONE), "Set RX IRQ");

        // 清掉可能残留的中断标志，进入连续接收
        sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_ALL);
        // 必须设为这个，否则无法接收到包数据！！！
        sx126x_set_rx_with_timeout_in_rtc_step(&hal_ctx, SX126X_RX_CONTINUOUS);

        // Start RX
        // SX126X_RX_CONTINUOUS is 0xFFFFFF
        // Note: sx126x_set_rx takes timeout in ms.
        // If we pass 0xFFFFFF, it's a huge timeout (hours).
        // If we pass 0, it's Single mode without timeout (wait forever).
        // To be safe for continuous RX, we should use a large value or 0 if we restart manually.
        // Let's try 0 (Single mode, wait forever) and restart in loop.

        // sx126x_set_rx(&hal_ctx, 0xFFFFFF); // 0 = wait forever
        if (total_sum != 0xfffffff) {
            printf("Will receive total %d packets.\n", total_sum);
        } else {
            printf("Will receive packets continuously.\n");
        }

        int loop_cnt = 0;
        int received_packets = 0;
        while (1) {
            // Check BUSY pin with timeout
            int busy_timeout = 1000; // 1 second
            while (busy_timeout > 0) {
                int busy_val = gpio_get_value(hal_ctx.busy_gpio);
                if (busy_val == 0) break;
                usleep(1000);
                busy_timeout--;
            }
            if (busy_timeout == 0) {
                printf("WARNING: BUSY pin stuck HIGH! Radio might be unresponsive.\n");
            }

            uint16_t irq_status = 0;
            sx126x_get_irq_status(&hal_ctx, &irq_status);

            // DEBUG: Print IRQ Status periodically
            // if (loop_cnt % 100 == 0) {
            //     printf("[RX Loop] IRQ: 0x%04X\n", irq_status);
            // }

            if (irq_status & SX126X_IRQ_RX_DONE) {
                printf("\nPacket %d Received!\n", received_packets + 1);

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

                // Restart Rx (continuous)
                sx126x_set_rx(&hal_ctx, 0xFFFFF);

                if (++received_packets >= total_sum) {
                    printf("\nReceived total %d packets. Exiting.\n", received_packets);
                    break;
                }
            }
            else if (irq_status & SX126X_IRQ_PREAMBLE_DETECTED) {
                printf("Preamble Detected!\n");
                sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_PREAMBLE_DETECTED);

                // Restart Rx (continuous)
                sx126x_set_rx(&hal_ctx, 0xFFFFFF);
            }
            else if (irq_status & SX126X_IRQ_HEADER_VALID) {
                printf("Header Valid!\n");
                sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_HEADER_VALID);
            }
            else if (irq_status & SX126X_IRQ_CRC_ERROR) {
                printf("CRC Error!\n");
                sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_ALL);
                sx126x_set_rx(&hal_ctx, 0xFFFFFF);
            }
            else if (irq_status & SX126X_IRQ_TIMEOUT) {
                printf("Time out!\n");
                sx126x_clear_irq_status(&hal_ctx, SX126X_IRQ_ALL);
                sx126x_set_rx(&hal_ctx, 0xFFFFFF);
            }

            loop_cnt++;
            if (loop_cnt % 1000 == 0) {
                loop_cnt = 0;
                printf("[irq_status=0x%04X]\n", irq_status);
            }

            usleep(1 * 1000);
        }
    }

end:
    sx126x_hal_linux_cleanup(&hal_ctx);
    return 0;
}
