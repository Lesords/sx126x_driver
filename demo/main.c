#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
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

// --- 定频默认参数组：FCC DTS（认证用，可被命令行覆盖）---
// FCC US915 500kHz 定频信道: 903.0 / 904.6 / 906.2 / 907.8 / 909.4 / 911.0 / 912.6 / 914.2 MHz
// CE EU868 参考: -f 876100000 -b 125 -p 16（LoRa）/ -f 868800000（FSK, 本固件未实现）
#define DEFAULT_FREQ_HZ     903000000UL
#define DEFAULT_POWER_DBM   22          // dBm, SX1262 +22dBm 档
#define DEFAULT_SF          7
#define DEFAULT_BW_KHZ      500
#define DEFAULT_CR_DENOM    5           // 编码率 4/5
#define DEFAULT_SYNC_WORD   0x34        // 对齐参考工程定频测试
#define DEFAULT_PLD_LEN     64
#define DEFAULT_INTERVAL_MS 1000
// --------------------------------------------------------

typedef struct
{
    uint32_t freq_hz;             // rf_freq_in_hz
    int16_t  power_dbm;           // output_pwr_in_dbm
    uint8_t  sf;                  // mod_params.sf
    uint32_t bw_khz;              // mod_params.bw（用 kHz 表示，避免枚举歧义）
    uint8_t  cr_denom;            // mod_params.cr: 4/(cr_denom), 5..8
    uint8_t  sync_word;           // sync_word
    uint16_t pld_len;             // pkt_params.pld_len_in_bytes
    uint32_t interval_ms;         // 发包间隔
    uint32_t pkt_cnt;             // 目标包数, 0 = 无限
    bool     nointer;             // 背靠背连发（认证用连续调制波）
    bool     infinite_preamble;   // 无限前导码发射
} radio_cfg_t;

sx126x_hal_context_t hal_ctx = {
    .spidev_path = SPI_DEV_PATH,
    .reset_gpio = GPIO_RESET,
    .busy_gpio = GPIO_BUSY,
    .dio1_gpio = GPIO_DIO1,
    .rf_sw_gpio = GPIO_RF_SW,
    .spi_fd = -1
};

static volatile sig_atomic_t g_exit = 0;

static void sig_handler( int sig )
{
    (void) sig;
    g_exit = 1;
}

void check_status( sx126x_status_t status, const char* msg )
{
    if( status != SX126X_STATUS_OK )
    {
        printf( "Error: %s failed with status %d\n", msg, status );
        // exit(1); // Optional: exit on error
    }
}

// 带宽 kHz -> 驱动枚举
static sx126x_lora_bw_t bw_khz_to_enum( uint32_t bw_khz, bool* ok )
{
    *ok = true;
    switch( bw_khz )
    {
    case 7:   return SX126X_LORA_BW_007;
    case 10:  return SX126X_LORA_BW_010;
    case 15:  return SX126X_LORA_BW_015;
    case 20:  return SX126X_LORA_BW_020;
    case 31:  return SX126X_LORA_BW_031;
    case 41:  return SX126X_LORA_BW_041;
    case 62:  return SX126X_LORA_BW_062;
    case 125: return SX126X_LORA_BW_125;
    case 250: return SX126X_LORA_BW_250;
    case 500: return SX126X_LORA_BW_500;
    default:  *ok = false; return SX126X_LORA_BW_500;
    }
}

// LDRO 按符号时长自动：符号时间 > 16ms 时开启（对齐 Meshtastic preset 规则）
static uint8_t ldro_auto( uint8_t sf, uint32_t bw_khz )
{
    return ( ( 1u << sf ) / bw_khz > 16 ) ? 1 : 0;
}

// 粗略估算 LoRa 空中时间(ms)，用于推导 TX 超时（公式来自 SX127x datasheet 附录 4）
static uint32_t lora_airtime_ms( uint8_t sf, uint32_t bw_khz, uint8_t cr_denom, uint16_t plen,
                                 uint16_t preamble_symb, bool crc_on, bool explicit_hdr, uint8_t ldro )
{
    const double tsym_ms = ( double ) ( 1u << sf ) / ( double ) bw_khz;
    const int    de      = ldro ? 2 : 0;
    const int    ih      = explicit_hdr ? 0 : 1;
    const int    crc     = crc_on ? 1 : 0;

    const int num = 8 * plen - 4 * sf + 28 + 16 * crc - 20 * ih;
    const int den = 4 * ( sf - 2 * de );
    const int ceil_num_den = ( num > 0 ) ? ( ( num + den - 1 ) / den ) : 0;
    const double n_payload = 8.0 + ( double ) ( ceil_num_den * ( cr_denom + 4 ) );

    return ( uint32_t ) ( ( ( double ) preamble_symb + 4.25 ) * tsym_ms + n_payload * tsym_ms + 0.5 );
}

// 按频段做镜像抑制校准（定频必做，否则 RX 灵敏度打折）
static void cal_image_for_freq( uint32_t freq_hz )
{
    static const struct
    {
        uint32_t f_min;
        uint32_t f_max;
        uint16_t lo_mhz;
        uint16_t hi_mhz;
    } bands[] = {
        { 902000000, 928000000, 902, 928 },  // US915
        { 863000000, 870000000, 863, 870 },  // EU868
        { 779000000, 787000000, 779, 787 },  // CN779
        { 470000000, 510000000, 470, 510 },  // CN470
        { 430000000, 445000000, 430, 445 },  // EU433
    };

    for( unsigned i = 0; i < sizeof( bands ) / sizeof( bands[0] ); i++ )
    {
        if( freq_hz >= bands[i].f_min && freq_hz <= bands[i].f_max )
        {
            check_status( sx126x_cal_img_in_mhz( &hal_ctx, bands[i].lo_mhz, bands[i].hi_mhz ),
                          "Calibrate Image" );
            printf( "Image calibration: %u-%u MHz\n", bands[i].lo_mhz, bands[i].hi_mhz );
            return;
        }
    }
    printf( "WARNING: freq %u Hz not in known band table, skip image calibration.\n", freq_hz );
}

// 射频开关：DIO2 自动控制 + 外部 RF_SW GPIO 双重控制
static void rf_sw_set( bool tx )
{
    if( hal_ctx.rf_sw_gpio >= 0 )
    {
        gpio_set_value( hal_ctx.rf_sw_gpio, tx ? 0 : 1 );  // 低=发射, 高=接收
    }
}

static void usage( const char* prog )
{
    printf( "Usage: %s <tx|rx|cw> [options]\n", prog );
    printf( "  tx : 定频发射（LoRa 调制波）\n" );
    printf( "  rx : 定频连续接收\n" );
    printf( "  cw : 单载波发射（认证测功率/频率稳定度）\n" );
    printf( "Options:\n" );
    printf( "  -f, --freq <Hz>      RF 频率, 默认 %lu\n", ( unsigned long ) DEFAULT_FREQ_HZ );
    printf( "  -p, --power <dBm>    发射功率, 默认 %d\n", DEFAULT_POWER_DBM );
    printf( "  -s, --sf <5-12>      扩频因子, 默认 %d\n", DEFAULT_SF );
    printf( "  -b, --bw <kHz>       带宽 {7,10,15,20,31,41,62,125,250,500}, 默认 %u\n", DEFAULT_BW_KHZ );
    printf( "  -r, --cr <5-8>       编码率 4/r, 默认 %d\n", DEFAULT_CR_DENOM );
    printf( "  -w, --sync <hex>     同步字, 默认 0x%02X\n", DEFAULT_SYNC_WORD );
    printf( "  -l, --len <bytes>    载荷长度 1-255, 默认 %d\n", DEFAULT_PLD_LEN );
    printf( "  -n, --cnt <n>        包数（0=无限）, 默认 0\n" );
    printf( "  -i, --interval <ms>  发包间隔, 默认 %u\n", DEFAULT_INTERVAL_MS );
    printf( "  -N, --nointer        背靠背连发（忽略间隔, 认证用连续调制波）\n" );
    printf( "  -P, --inp            无限前导码发射\n" );
    printf( "  -h, --help           帮助\n" );
    printf( "Examples:\n" );
    printf( "  %s tx -f 904600000 -p 22            # FCC DTS 信道 2\n", prog );
    printf( "  %s tx -f 876100000 -b 125 -p 16     # CE EU868\n", prog );
    printf( "  %s cw -f 903000000 -p 22            # 单载波\n", prog );
    printf( "  %s rx -f 903000000 -b 500 -n 100    # 收 100 包后退出\n", prog );
}

// 一次性下发全套定频参数（对应参考工程 ralf_setup_lora）
static int radio_apply( const radio_cfg_t* cfg )
{
    sx126x_chip_status_t chip_status;
    bool                 bw_ok = false;
    sx126x_lora_bw_t     bw    = bw_khz_to_enum( cfg->bw_khz, &bw_ok );

    // 1. 复位 + 命令保留列表
    sx126x_reset( &hal_ctx );
    sx126x_init_retention_list( &hal_ctx );

    // 2. 供电模式：板上未焊 DCDC 电感，固定 LDO
    check_status( sx126x_set_reg_mode( &hal_ctx, SX126X_REG_MODE_LDO ), "Set Reg Mode LDO" );

    // 3. 诊断：芯片状态 + 硬件版本
    sx126x_get_status( &hal_ctx, &chip_status );
    printf( "Chip Status: CmdStatus=%d, ChipMode=%d\n", chip_status.cmd_status, chip_status.chip_mode );
    if( chip_status.chip_mode == 0 || chip_status.chip_mode == 0xFF )
    {
        printf( "ERROR: SPI Read failed (read 0x00 or 0xFF). Check wiring (MISO)!\n" );
        return -1;
    }
    uint8_t hw_version = 0;
    sx126x_read_register( &hal_ctx, 0x0320, &hw_version, 1 );
    printf( "Hardware Version: 0x%02X (Expected: 0x12)\n", hw_version );

    // 4. DIO2 自动控制射频开关 + DIO3 TCXO 1.8V（先保证时钟稳定）
    check_status( sx126x_set_dio2_as_rf_sw_ctrl( &hal_ctx, true ), "Set DIO2 as RF Switch" );
    check_status( sx126x_set_dio3_as_tcxo_ctrl( &hal_ctx, SX126X_TCXO_CTRL_1_8V, 165 ), "Set TCXO 1.8V" );

    // 5. 包类型 + 频率
    check_status( sx126x_set_pkt_type( &hal_ctx, SX126X_PKT_TYPE_LORA ), "Set Packet Type" );
    check_status( sx126x_set_rf_freq( &hal_ctx, cfg->freq_hz ), "Set RF Freq" );

    // 6. 校准：全量校准 + 按频段镜像校准
    sx126x_cal( &hal_ctx, SX126X_CAL_ALL );
    cal_image_for_freq( cfg->freq_hz );

    // 7. PA 配置（SX1262 +22dBm 档）+ 发射功率
    sx126x_pa_cfg_params_t pa_params;
    pa_params.pa_duty_cycle = 0x04;
    pa_params.hp_max        = 0x07;
    pa_params.device_sel    = 0x00;  // SX1262
    pa_params.pa_lut        = 0x01;
    check_status( sx126x_set_pa_cfg( &hal_ctx, &pa_params ), "Set PA Config" );
    check_status( sx126x_set_tx_params( &hal_ctx, cfg->power_dbm, SX126X_RAMP_40_US ), "Set TX Params" );

    // 8. 调制参数（LDRO 按符号时长自动）
    uint8_t ldro = ldro_auto( cfg->sf, cfg->bw_khz );
    sx126x_mod_params_lora_t mod_params;
    mod_params.sf   = ( sx126x_lora_sf_t ) cfg->sf;
    mod_params.bw   = bw;
    mod_params.cr   = ( sx126x_lora_cr_t ) ( cfg->cr_denom - 4 );
    mod_params.ldro = ldro;
    check_status( sx126x_set_lora_mod_params( &hal_ctx, &mod_params ), "Set Modulation Params" );

    // 9. 包参数：preamble 8 / 显式头 / CRC on / IQ 不反转
    sx126x_pkt_params_lora_t pkt_params;
    pkt_params.preamble_len_in_symb = 8;
    pkt_params.header_type          = SX126X_LORA_PKT_EXPLICIT;
    pkt_params.pld_len_in_bytes     = cfg->pld_len;
    pkt_params.crc_is_on            = true;
    pkt_params.invert_iq_is_on      = false;
    check_status( sx126x_set_lora_pkt_params( &hal_ctx, &pkt_params ), "Set Packet Params" );

    // 10. 同步字 + 符号超时（0 = 不超时）
    check_status( sx126x_set_lora_sync_word( &hal_ctx, cfg->sync_word ), "Set Sync Word" );
    sx126x_set_lora_symb_nb_timeout( &hal_ctx, 0 );

    // 11. 其它
    sx126x_stop_timer_on_preamble( &hal_ctx, false );
    sx126x_cfg_rx_boosted( &hal_ctx, true );  // RX Boosted for better sensitivity
    sx126x_cfg_tx_clamp( &hal_ctx );
    sx126x_clear_irq_status( &hal_ctx, SX126X_IRQ_ALL );

    return 0;
}

int main( int argc, char* argv[] )
{
    if( argc < 2 )
    {
        usage( argv[0] );
        return 1;
    }

    int mode = 0;  // 1=TX, 2=RX, 3=CW
    if( strcmp( argv[1], "tx" ) == 0 ) mode = 1;
    else if( strcmp( argv[1], "rx" ) == 0 ) mode = 2;
    else if( strcmp( argv[1], "cw" ) == 0 ) mode = 3;
    else if( strcmp( argv[1], "-h" ) == 0 || strcmp( argv[1], "--help" ) == 0 )
    {
        usage( argv[0] );
        return 0;
    }
    else
    {
        printf( "Invalid mode '%s'. Use 'tx', 'rx' or 'cw'.\n", argv[1] );
        return 1;
    }

    // --- 默认定频参数组（FCC DTS），命令行可覆盖 ---
    radio_cfg_t cfg = {
        .freq_hz   = DEFAULT_FREQ_HZ,
        .power_dbm = DEFAULT_POWER_DBM,
        .sf        = DEFAULT_SF,
        .bw_khz    = DEFAULT_BW_KHZ,
        .cr_denom  = DEFAULT_CR_DENOM,
        .sync_word = DEFAULT_SYNC_WORD,
        .pld_len   = DEFAULT_PLD_LEN,
        .interval_ms = DEFAULT_INTERVAL_MS,
        .pkt_cnt   = 0,
        .nointer   = false,
        .infinite_preamble = false,
    };

    static const struct option long_opts[] = {
        { "freq",     required_argument, 0, 'f' },
        { "power",    required_argument, 0, 'p' },
        { "sf",       required_argument, 0, 's' },
        { "bw",       required_argument, 0, 'b' },
        { "cr",       required_argument, 0, 'r' },
        { "sync",     required_argument, 0, 'w' },
        { "len",      required_argument, 0, 'l' },
        { "cnt",      required_argument, 0, 'n' },
        { "interval", required_argument, 0, 'i' },
        { "nointer",  no_argument,       0, 'N' },
        { "inp",      no_argument,       0, 'P' },
        { "help",     no_argument,       0, 'h' },
        { 0, 0, 0, 0 }
    };

    optind = 2;  // 模式在 argv[1]，选项从 argv[2] 开始
    int opt;
    while( ( opt = getopt_long( argc, argv, "f:p:s:b:r:w:l:n:i:NPh", long_opts, NULL ) ) != -1 )
    {
        switch( opt )
        {
        case 'f': cfg.freq_hz     = strtoul( optarg, NULL, 0 ); break;
        case 'p': cfg.power_dbm   = ( int16_t ) atoi( optarg ); break;
        case 's': cfg.sf          = ( uint8_t ) atoi( optarg ); break;
        case 'b': cfg.bw_khz      = ( uint32_t ) atoi( optarg ); break;
        case 'r': cfg.cr_denom    = ( uint8_t ) atoi( optarg ); break;
        case 'w': cfg.sync_word   = ( uint8_t ) strtoul( optarg, NULL, 0 ); break;
        case 'l': cfg.pld_len     = ( uint16_t ) atoi( optarg ); break;
        case 'n': cfg.pkt_cnt     = ( uint32_t ) atoi( optarg ); break;
        case 'i': cfg.interval_ms = ( uint32_t ) atoi( optarg ); break;
        case 'N': cfg.nointer     = true; break;
        case 'P': cfg.infinite_preamble = true; break;
        case 'h': usage( argv[0] ); return 0;
        default:  usage( argv[0] ); return 1;
        }
    }

    // --- 参数校验 ---
    bool bw_ok = false;
    bw_khz_to_enum( cfg.bw_khz, &bw_ok );
    if( cfg.freq_hz < 150000000 || cfg.freq_hz > 960000000 )
    {
        printf( "ERROR: freq %u Hz out of SX1262 range (150M-960M).\n", cfg.freq_hz );
        return 1;
    }
    if( !bw_ok )
    {
        printf( "ERROR: unsupported bandwidth %u kHz.\n", cfg.bw_khz );
        return 1;
    }
    if( cfg.sf < 5 || cfg.sf > 12 )
    {
        printf( "ERROR: SF must be 5-12.\n" );
        return 1;
    }
    if( cfg.cr_denom < 5 || cfg.cr_denom > 8 )
    {
        printf( "ERROR: CR denominator must be 5-8.\n" );
        return 1;
    }
    if( cfg.pld_len < 1 || cfg.pld_len > 255 )
    {
        printf( "ERROR: payload length must be 1-255.\n" );
        return 1;
    }
    if( cfg.power_dbm > 22 )
    {
        printf( "WARNING: power %d dBm > 22, clamp to 22.\n", cfg.power_dbm );
        cfg.power_dbm = 22;
    }
    if( cfg.power_dbm < -9 )
    {
        printf( "WARNING: power %d dBm < -9, clamp to -9.\n", cfg.power_dbm );
        cfg.power_dbm = -9;
    }
    if( cfg.interval_ms < 10 )
    {
        cfg.interval_ms = 10;
    }

    const uint8_t ldro = ldro_auto( cfg.sf, cfg.bw_khz );

    printf( "Starting SX126x fixed-frequency demo: %s mode\n",
            mode == 1 ? "TX" : ( mode == 2 ? "RX" : "CW" ) );
    printf( "--- 定频参数 ---\n" );
    printf( "  freq=%u Hz, power=%d dBm\n", cfg.freq_hz, cfg.power_dbm );
    printf( "  SF%u / BW%uk / CR4/%u / LDRO=%u\n", cfg.sf, cfg.bw_khz, cfg.cr_denom, ldro );
    printf( "  sync=0x%02X, preamble=8, explicit header, CRC on, IQ normal\n", cfg.sync_word );
    if( mode == 1 )
    {
        printf( "  payload=%u B, interval=%u ms%s, count=%s\n", cfg.pld_len, cfg.interval_ms,
                cfg.nointer ? " (nointer: back-to-back)" : "",
                cfg.pkt_cnt ? "" : "infinite" );
    }

    // --- 初始化 HAL + 下发定频参数 ---
    if( sx126x_hal_linux_init( &hal_ctx ) != 0 )
    {
        fprintf( stderr, "Failed to initialize HAL\n" );
        return 1;
    }
    printf( "HAL Initialized.\n" );

    if( radio_apply( &cfg ) != 0 )
    {
        sx126x_hal_linux_cleanup( &hal_ctx );
        return 1;
    }

    signal( SIGINT, sig_handler );
    signal( SIGTERM, sig_handler );

    // TX 超时按空中时间推导（SF12 长包场景必须够长），驱动上限 262143ms
    const uint32_t airtime_ms = lora_airtime_ms( cfg.sf, cfg.bw_khz, cfg.cr_denom, cfg.pld_len, 8,
                                                 true, true, ldro );
    uint32_t tx_timeout_ms = airtime_ms * 2 + 200;
    if( tx_timeout_ms < 500 ) tx_timeout_ms = 500;
    if( tx_timeout_ms > 260000 ) tx_timeout_ms = 260000;

    if( mode == 1 )
    {
        // ================= TX =================
        printf( "\n--- TX Mode (airtime est. %u ms, tx timeout %u ms) ---\n", airtime_ms, tx_timeout_ms );
        rf_sw_set( true );

        if( cfg.infinite_preamble )
        {
            printf( "Transmitting infinite preamble. Press Ctrl+C to stop.\n" );
            sx126x_clear_irq_status( &hal_ctx, SX126X_IRQ_ALL );
            sx126x_set_tx_infinite_preamble( &hal_ctx );
            while( !g_exit )
            {
                sleep( 1 );
            }
            printf( "\nTX summary: infinite preamble stopped.\n" );
            goto end;
        }

        check_status( sx126x_set_dio_irq_params( &hal_ctx, SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                                                SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                                                SX126X_IRQ_NONE, SX126X_IRQ_NONE ), "Set TX IRQ" );

        printf( "\nStarting TX loop... Press Ctrl+C to stop.\n" );
        uint32_t seq = 0, sent = 0, timeouts = 0;
        while( !g_exit )
        {
            seq++;

            uint8_t tx_buf[255];
            memset( tx_buf, 0xAA, cfg.pld_len );
            snprintf( ( char* ) tx_buf, cfg.pld_len, "Ping %lu - SX1262 fixed-freq", ( unsigned long ) seq );

            sx126x_write_buffer( &hal_ctx, 0x00, tx_buf, cfg.pld_len );
            sx126x_clear_irq_status( &hal_ctx, SX126X_IRQ_ALL );
            check_status( sx126x_set_tx( &hal_ctx, tx_timeout_ms ), "Set TX" );

            // 等待 TX_DONE / TIMEOUT
            bool     done      = false;
            bool     is_tmo    = false;
            int      wait_left = ( int ) ( tx_timeout_ms / 5 ) + 200;
            uint16_t irq       = 0;
            while( wait_left-- > 0 && !g_exit )
            {
                sx126x_get_irq_status( &hal_ctx, &irq );
                if( irq & SX126X_IRQ_TX_DONE )
                {
                    done = true;
                    sent++;
                    break;
                }
                if( irq & SX126X_IRQ_TIMEOUT )
                {
                    is_tmo = true;
                    timeouts++;
                    break;
                }
                usleep( 5000 );
            }
            printf( "[%lu] %s\n", ( unsigned long ) seq,
                    done ? "TX DONE" : ( is_tmo ? "TX TIMEOUT" : "WAIT TIMEOUT" ) );

            if( cfg.pkt_cnt && sent >= cfg.pkt_cnt )
            {
                printf( "\nReached target %u packets.\n", cfg.pkt_cnt );
                break;
            }

            if( !cfg.nointer )
            {
                for( uint32_t ms = 0; ms < cfg.interval_ms && !g_exit; ms += 50 )
                {
                    usleep( 50000 );
                }
            }
        }
        printf( "\nTX summary: m_tx_cnt:%u, m_tx_timeout:%u\n", sent, timeouts );
    }

    if( mode == 2 )
    {
        // ================= RX =================
        printf( "\n--- RX Mode (continuous) ---\n" );
        rf_sw_set( false );

        // RX 侧包参数：显式头下 pld_len 由对端决定，设 255（对齐参考工程 SX1262 测试）
        sx126x_pkt_params_lora_t rx_pkt_params;
        rx_pkt_params.preamble_len_in_symb = 8;
        rx_pkt_params.header_type          = SX126X_LORA_PKT_EXPLICIT;
        rx_pkt_params.pld_len_in_bytes     = 255;
        rx_pkt_params.crc_is_on            = true;
        rx_pkt_params.invert_iq_is_on      = false;
        check_status( sx126x_set_lora_pkt_params( &hal_ctx, &rx_pkt_params ), "Set RX Packet Params" );

        check_status( sx126x_set_dio_irq_params( &hal_ctx,
                                                SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT |
                                                SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_HEADER_VALID,
                                                SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT |
                                                SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_HEADER_VALID,
                                                SX126X_IRQ_NONE, SX126X_IRQ_NONE ), "Set RX IRQ" );

        sx126x_clear_irq_status( &hal_ctx, SX126X_IRQ_ALL );
        // 必须用 RTC-step 接口设连续接收（sx126x_set_rx 的 ms 参数上限 262143，无法表达连续模式）
        sx126x_set_rx_with_timeout_in_rtc_step( &hal_ctx, SX126X_RX_CONTINUOUS );
        // 注：连续模式下芯片收完一包自动回到 RX，循环内只需清标志，不要重启 RX
        //     （重启会打断正在进行的接收）

        printf( "Waiting for packets... Press Ctrl+C to stop.\n" );
        if( cfg.pkt_cnt )
        {
            printf( "Will exit after %u good packets.\n", cfg.pkt_cnt );
        }

        uint32_t rx_ok = 0, rx_crc_err = 0;
        uint32_t idle_cnt = 0;
        uint16_t irq = 0;
        while( !g_exit )
        {
            sx126x_get_irq_status( &hal_ctx, &irq );

            if( irq & SX126X_IRQ_RX_DONE )
            {
                sx126x_rx_buffer_status_t rx_status;
                sx126x_pkt_status_lora_t  pkt_status;
                sx126x_get_rx_buffer_status( &hal_ctx, &rx_status );
                sx126x_get_lora_pkt_status( &hal_ctx, &pkt_status );

                uint8_t buffer[256];
                memset( buffer, 0, sizeof( buffer ) );
                if( rx_status.pld_len_in_bytes > 0 )
                {
                    sx126x_read_buffer( &hal_ctx, rx_status.buffer_start_pointer, buffer,
                                        rx_status.pld_len_in_bytes );
                }
                rx_ok++;
                printf( "RX Done: %u, len=%u, rssi=%d dBm, snr=%d dB, data: %s\n",
                        rx_ok, rx_status.pld_len_in_bytes,
                        pkt_status.rssi_pkt_in_dbm, pkt_status.snr_pkt_in_db, ( char* ) buffer );

                sx126x_clear_irq_status( &hal_ctx, SX126X_IRQ_ALL );

                if( cfg.pkt_cnt && rx_ok >= cfg.pkt_cnt )
                {
                    break;
                }
            }
            else if( irq & SX126X_IRQ_CRC_ERROR )
            {
                rx_crc_err++;
                printf( "RX CRC Error: %u\n", rx_crc_err );
                sx126x_clear_irq_status( &hal_ctx, SX126X_IRQ_ALL );
            }
            else if( irq & ( SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_HEADER_VALID | SX126X_IRQ_TIMEOUT ) )
            {
                // 过程标志：只清标志，不打断接收
                sx126x_clear_irq_status( &hal_ctx, SX126X_IRQ_PREAMBLE_DETECTED |
                                                SX126X_IRQ_HEADER_VALID | SX126X_IRQ_TIMEOUT );
            }
            else
            {
                usleep( 1000 );
                if( ++idle_cnt % 5000 == 0 )
                {
                    printf( "[waiting irq=0x%04X]\n", irq );
                }
            }
        }
        printf( "\nRX summary: m_rx_packet:%u, packet_ok: %u, packet_err:%u, packet_crc_err:%u\n",
                rx_ok + rx_crc_err, rx_ok, rx_crc_err, rx_crc_err );
    }

    if( mode == 3 )
    {
        // ================= CW =================
        printf( "\n--- CW Mode (continuous wave) ---\n" );
        rf_sw_set( true );

        check_status( sx126x_set_tx_cw( &hal_ctx ), "Set TX CW" );
        printf( "Transmitting CW carrier. Press Ctrl+C to stop.\n" );
        while( !g_exit )
        {
            sleep( 1 );
        }
        printf( "\nCW stopped.\n" );
    }

end:
    // 退出前回到 Standby（CW/无限前导码会一直发射，必须显式停止）
    sx126x_set_standby( &hal_ctx, SX126X_STANDBY_CFG_RC );
    rf_sw_set( false );  // 射频开关回接收态
    sx126x_hal_linux_cleanup( &hal_ctx );
    return 0;
}
