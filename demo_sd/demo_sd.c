/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/*******************************************************************************
 * @file     : demo_sd.c
 * @author   : Deepak Kumar
 * @email    : deepak@alifsemi.com
 * @version  : V0.0.1
 * @date     : 28-Nov-2022
 * @brief    : Baremeetal sd driver test Application.
 * @bug      : None.
 * @Note     : None
 ******************************************************************************/
/* System Includes */
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include "RTE_Device.h"
#include "se_services_port.h"

/* include for SD Driver */
#include "sd.h"
#include "pinconf.h"
#include "board_config.h"

#include "RTE_Components.h"
#include CMSIS_device_header
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#include "Driver_Common.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */
#include "Driver_IO.h"
#include "board_config.h"
#include "app_utils.h"

// Set to 0: Use application-defined SDC A revision pin configuration.
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

#define BAREMETAL_SD_TEST_RAW_SECTOR                                                               \
    0x2000  // start reading and writing raw data from partition sector
volatile unsigned char sdbuffer[512 * 4] __attribute__((section("sd_dma_buf")))
__attribute__((aligned(32)));

/* Backup of each sector's original content so the test is non-destructive. */
volatile unsigned char sdsavebuffer[512] __attribute__((section("sd_dma_buf")))
__attribute__((aligned(32)));

/* Large buffer for the read-only sequential throughput benchmark. */
#define SD_BENCH_CHUNK_SECTORS 128u  /* 64 KB per multi-block read */
#define SD_BENCH_TOTAL_SECTORS 8192u /* 4 MB total, read-only */
volatile unsigned char sdreadbuf[SD_BENCH_CHUNK_SECTORS * 512u]
    __attribute__((section("sd_dma_buf"))) __attribute__((aligned(32)));

const diskio_t   *p_SD_Driver  = &SD_Driver;
volatile uint32_t dma_done_irq = 0;

/**
  \fn           sd_cb(uint16_t cmd_status, uint16_t xfer_status)
  \brief        SD interrupt callback
  \param[in]    uint16_t cmd_status
  \param[in]    uint16_t xfer_status
  \return       none
*/
void sd_cb(uint16_t cmd_status, uint16_t xfer_status)
{

    ARG_UNUSED(cmd_status);

    if (xfer_status) {
        dma_done_irq = 1;
    }
}

#ifdef BOARD_SD_RESET_GPIO_PORT
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_SD_RESET_GPIO_PORT);

/**
  \fn           sd_pwr_cb(uint8_t power_on)
  \brief        SD power callback
  \param[in]    power_on: 0=off, 1=on
  \return       none
  */
void sd_pwr_cb(uint8_t power_on)
{
    int              status;
    ARM_DRIVER_GPIO *gpioSD_PWR = &ARM_Driver_GPIO_(BOARD_SD_RESET_GPIO_PORT);

    if (power_on) {
        status = gpioSD_PWR->SetValue(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
        if (status != ARM_DRIVER_OK) {
            SD_LOG_ERR("Failed to turn on SD power pin");
        }
    } else {
        status = gpioSD_PWR->SetValue(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
        if (status != ARM_DRIVER_OK) {
            SD_LOG_ERR("Failed to turn off SD power pin");
        }
        sys_busy_loop_us(SDMMC_RESET_DELAY_US);
    }

    return;
}
#endif

/* Issue a single-sector DMA read and block until the completion callback fires. */
static int sd_read_wait(uint32_t sector, volatile uint8_t *buf)
{
    dma_done_irq = 0;
    if (p_SD_Driver->disk_read(sector, 1, buf) != SD_DRV_STATUS_OK) {
        return -1;
    }
    while (!dma_done_irq) {
    }
    return 0;
}

/* Issue a single-sector DMA write and block until the completion callback fires. */
static int sd_write_wait(uint32_t sector, volatile uint8_t *buf)
{
    dma_done_irq = 0;
    if (p_SD_Driver->disk_write(sector, 1, buf) != SD_DRV_STATUS_OK) {
        return -1;
    }
    while (!dma_done_irq) {
    }
    return 0;
}

/* Issue a multi-block DMA read and block until the completion callback fires. */
static int sd_read_n_wait(uint32_t sector, uint16_t blocks, volatile uint8_t *buf)
{
    dma_done_irq = 0;
    if (p_SD_Driver->disk_read(sector, blocks, buf) != SD_DRV_STATUS_OK) {
        return -1;
    }
    while (!dma_done_irq) {
    }
    return 0;
}

/**
  \fn           BareMetalSDTest(uint32_t startSec, uint32_t EndSector)
  \brief        Baremetal SD driver Test Function
  \param[in]    starSecr - Test Read/Write start sector number
  \param[in]    EndSector - Test Read/Write End sector number
  \return       none
*/
void BareMetalSDTest(uint32_t startSec, uint32_t EndSector)
{

    int        j;
    uint32_t  *p = (uint32_t *) sdbuffer;
    /* Zero-init: 2.2.0 sd_param_t adds card_det_cb/vsel_cb the driver calls if non-NULL */
    sd_param_t sd_param = {0};

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    int32_t ret;
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }

#else
    /*
     * NOTE: The SDC A revision pins used in this test application are not configured
     * in the board support library. Therefore, pins are configured manually here.
     */
#ifdef BOARD_SD_RESET_GPIO_PORT

    uint32_t status;

    pinconf_set(PORT_(BOARD_SD_RESET_GPIO_PORT), BOARD_SD_RESET_GPIO_PIN, 0, 0);  // SD reset

    ARM_DRIVER_GPIO *sd_rst_gpio = &ARM_Driver_GPIO_(BOARD_SD_RESET_GPIO_PORT);

    status = sd_rst_gpio->Initialize(BOARD_SD_RESET_GPIO_PIN, NULL);
    if (status) {
        SD_LOG_ERR("Failed to initialize SD RST GPIO");
    }

    status = sd_rst_gpio->PowerControl(BOARD_SD_RESET_GPIO_PIN, ARM_POWER_FULL);
    if (status) {
        SD_LOG_ERR("Failed to power SD RST GPIO");
    }

    status = sd_rst_gpio->SetDirection(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (status) {
        SD_LOG_ERR("Failed to configure SD RST GPIO direction");
    }
    status = sd_rst_gpio->SetValue(BOARD_SD_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (status) {
        SD_LOG_ERR("Failed to set SD reset pin high");
    }

#endif
    pinconf_set(PORT_(BOARD_SD_CMD_A_GPIO_PORT),
                BOARD_SD_CMD_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_6,
                PADCTRL_READ_ENABLE);  // cmd
    pinconf_set(PORT_(BOARD_SD_CLK_A_GPIO_PORT),
                BOARD_SD_CLK_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_6,
                PADCTRL_READ_ENABLE);  // clk
    pinconf_set(PORT_(BOARD_SD_D0_A_GPIO_PORT),
                BOARD_SD_D0_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_7,
                PADCTRL_READ_ENABLE);  // d0

#if (RTE_SDC_BUS_WIDTH == SDMMC_4_BIT_MODE) || (RTE_SDC_BUS_WIDTH == SDMMC_8_BIT_MODE)
    pinconf_set(PORT_(BOARD_SD_D1_A_GPIO_PORT),
                BOARD_SD_D1_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_7,
                PADCTRL_READ_ENABLE);  // d1
    pinconf_set(PORT_(BOARD_SD_D2_A_GPIO_PORT),
                BOARD_SD_D2_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_7,
                PADCTRL_READ_ENABLE);  // d2
    pinconf_set(PORT_(BOARD_SD_D3_A_GPIO_PORT),
                BOARD_SD_D3_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_6,
                PADCTRL_READ_ENABLE);  // d3
#endif

#if RTE_SDC_BUS_WIDTH == SDMMC_8_BIT_MODE
    pinconf_set(PORT_(BOARD_SD_D4_A_GPIO_PORT),
                BOARD_SD_D4_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_6,
                PADCTRL_READ_ENABLE);  // d4
    pinconf_set(PORT_(BOARD_SD_D5_A_GPIO_PORT),
                BOARD_SD_D5_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_5,
                PADCTRL_READ_ENABLE);  // d5
    pinconf_set(PORT_(BOARD_SD_D6_A_GPIO_PORT),
                BOARD_SD_D6_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_5,
                PADCTRL_READ_ENABLE);  // d6
    pinconf_set(PORT_(BOARD_SD_D7_A_GPIO_PORT),
                BOARD_SD_D7_A_GPIO_PIN,
                PINMUX_ALTERNATE_FUNCTION_5,
                PADCTRL_READ_ENABLE);  // d7
#endif
#endif

    sd_param.dev_id       = SDMMC_DEV_ID;
    sd_param.clock_freq   = RTE_SDC_CLOCK_SELECT;
    sd_param.bus_width    = RTE_SDC_BUS_WIDTH;
    sd_param.dma_mode     = RTE_SDC_DMA_SELECT;
    sd_param.app_callback = sd_cb;

#ifdef BOARD_SD_RESET_GPIO_PORT
    sd_param.pwr_cb     = sd_pwr_cb;
#else
    sd_param.pwr_cb     = 0;
#endif

    if (p_SD_Driver->disk_initialize(&sd_param)) {
        printf("SD initialization failed...\n");
        return;
    }

    /* Write a known pattern to each sector, then read it back and verify.
     * A mismatch is the real proof of whether RTE_SDC_CLOCK_SELECT is reliable
     * end-to-end; the DWT cycle count lets you compare throughput vs 25MHz.
     * Each sector's original content is backed up and restored, so the test is
     * non-destructive. */
    uint32_t total = 0, passed = 0, failed = 0, first_bad = 0;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start_cycles = DWT->CYCCNT;

    while (startSec < EndSector) {

        total++;

        /* Back up the original sector so we can restore it afterwards. */
        if (sd_read_wait(startSec, (volatile uint8_t *) sdsavebuffer)) {
            printf("Backup read failed at sector %" PRIu32 "\n", startSec);
            if (!failed) first_bad = startSec;
            failed++;
            startSec++;
            continue;
        }

        /* Fill buffer with a sector-dependent known pattern and write it. */
        for (j = 0; j < 128; j++) {
            p[j] = (startSec << 8) ^ (0xA5A50000u + (uint32_t) j);
        }
        if (sd_write_wait(startSec, (volatile uint8_t *) sdbuffer)) {
            printf("Write failed at sector %" PRIu32 "\n", startSec);
            if (!failed) first_bad = startSec;
            failed++;
            startSec++;
            continue;
        }

        /* Overwrite buffer so the read-back must fetch from the card. */
        for (j = 0; j < 128; j++) {
            p[j] = 0xFFFFFFFFu;
        }
        if (sd_read_wait(startSec, (volatile uint8_t *) sdbuffer)) {
            printf("Read failed at sector %" PRIu32 "\n", startSec);
            if (!failed) first_bad = startSec;
            failed++;
            sd_write_wait(startSec, (volatile uint8_t *) sdsavebuffer); /* try to restore */
            startSec++;
            continue;
        }

        /* Verify read-back data against the written pattern. */
        bool sector_ok = true;
        for (j = 0; j < 128; j++) {
            uint32_t expect = (startSec << 8) ^ (0xA5A50000u + (uint32_t) j);
            if (p[j] != expect) {
                if (sector_ok) {
                    printf("MISMATCH sector %" PRIu32 " word %d: got %08" PRIx32
                           " exp %08" PRIx32 "\n",
                           startSec, j, p[j], expect);
                }
                sector_ok = false;
            }
        }
        if (sector_ok) {
            passed++;
        } else {
            if (!failed) first_bad = startSec;
            failed++;
        }

        /* Restore the original sector content. */
        if (sd_write_wait(startSec, (volatile uint8_t *) sdsavebuffer)) {
            printf("Restore write failed at sector %" PRIu32 "\n", startSec);
        }

        startSec++;
    }

    uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;

    printf("\n==== SD verify @ %d Hz: %" PRIu32 " sectors, %" PRIu32 " passed, %" PRIu32
           " failed ====\n",
           RTE_SDC_CLOCK_SELECT, total, passed, failed);
    if (failed) {
        printf("RESULT: FAIL (first bad sector = %" PRIu32 ")\n", first_bad);
    } else {
        printf("RESULT: PASS\n");
    }

    /* Throughput over all transfers (backup+write+verify+restore = 4 per sector),
     * including command/DMA overhead. */
    if (elapsed_cycles && SystemCoreClock) {
        uint64_t bytes = (uint64_t) total * 512u * 4u;
        uint32_t kbps  = (uint32_t) ((bytes * (uint64_t) SystemCoreClock) /
                                    ((uint64_t) elapsed_cycles * 1024ULL));
        printf("Throughput: %" PRIu32 " KB/s (%" PRIu32 " CPU cycles @ %" PRIu32 " Hz)\n",
               kbps, elapsed_cycles, (uint32_t) SystemCoreClock);
    }

    /* Read-only multi-block throughput benchmark: large sequential reads amortize
     * per-command overhead so the data phase (and thus the SD clock) dominates.
     * Reads have no card programming time, so this scales with RTE_SDC_CLOCK_SELECT. */
    {
        uint32_t sec       = BAREMETAL_SD_TEST_RAW_SECTOR;
        uint32_t remaining = SD_BENCH_TOTAL_SECTORS;
        uint32_t read_ok   = 0;

        DWT->CYCCNT        = 0;
        uint32_t bench_start = DWT->CYCCNT;

        while (remaining) {
            uint16_t n = (remaining < SD_BENCH_CHUNK_SECTORS) ? (uint16_t) remaining
                                                              : (uint16_t) SD_BENCH_CHUNK_SECTORS;
            if (sd_read_n_wait(sec, n, (volatile uint8_t *) sdreadbuf)) {
                printf("Benchmark read failed at sector %" PRIu32 "\n", sec);
                break;
            }
            sec       += n;
            remaining -= n;
            read_ok   += n;
        }

        uint32_t bench_cycles = DWT->CYCCNT - bench_start;
        if (bench_cycles && SystemCoreClock && read_ok) {
            uint64_t bytes = (uint64_t) read_ok * 512u;
            uint32_t kbps  = (uint32_t) ((bytes * (uint64_t) SystemCoreClock) /
                                        ((uint64_t) bench_cycles * 1024ULL));
            printf("\n==== SD sequential read benchmark @ %d Hz ====\n", RTE_SDC_CLOCK_SELECT);
            printf("Read %" PRIu32 " KB in %" PRIu32 " cycles (%u-sector blocks)\n",
                   (uint32_t) (bytes / 1024u), bench_cycles, (unsigned) SD_BENCH_CHUNK_SECTORS);
            printf("Read throughput: %" PRIu32 " KB/s (%" PRIu32 ".%02" PRIu32 " MB/s)\n",
                   kbps, kbps / 1024u, ((kbps % 1024u) * 100u) / 1024u);
        }
    }

    p_SD_Driver->disk_uninitialize(SDMMC_DEV_ID);

    return;
}

int main()
{
    uint32_t      service_error_code;
    uint32_t      error_code = SERVICES_REQ_SUCCESS;
    run_profile_t runp       = {0};
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    int32_t    ret;
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif

    /* Initialize the SE services */
    se_services_port_init();

    /* Enable SDMMC Clocks */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_100M,
                                              true,
                                              &service_error_code);
    if (error_code) {
        printf("SE: SDMMC 100MHz clock enable = %" PRIu32 "\n", error_code);
        return 0;
    }

    /* Enable HFOSC: provides the SDMMC controller reference clock on E8/Gen2 */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_HFOSC,
                                              true,
                                              &service_error_code);
    if (error_code) {
        printf("SE: SDMMC HFOSC clock enable = %" PRIu32 "\n", error_code);
        return 0;
    }

    /* A JLink/debugger load does not apply the boot TOC run profile, so the SD
     * block is left gated. Ungate the SDMMC IP clock and power the I/O LDO. */
    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("SE: get_run_cfg = %" PRIu32 "\n", error_code);
        return 0;
    }

    runp.ip_clock_gating |= SDC_MASK;
    runp.phy_pwr_gating  |= LDO_PHY_MASK;

    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        printf("SE: set_run_cfg = %" PRIu32 "\n", error_code);
        return 0;
    }

    /* Enter the Baremetal demo Application.  */
    BareMetalSDTest(BAREMETAL_SD_TEST_RAW_SECTOR, BAREMETAL_SD_TEST_RAW_SECTOR + 0x200);

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_100M,
                                              false,
                                              &service_error_code);
    if (error_code) {
        printf("SE: SDMMC 100MHz clock disable = %" PRIu32 "\n", error_code);
        return 0;
    }

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_HFOSC,
                                              false,
                                              &service_error_code);
    if (error_code) {
        printf("SE: SDMMC HFOSC clock disable = %" PRIu32 "\n", error_code);
        return 0;
    }

    return 0;
}
