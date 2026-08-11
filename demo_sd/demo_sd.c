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

/* Multi-block DMA buffer used by the reliability test (64 KB = 128 sectors). */
#define SD_CHUNK_SECTORS 128u
volatile unsigned char sdreadbuf[SD_CHUNK_SECTORS * 512u]
    __attribute__((section("sd_dma_buf"))) __attribute__((aligned(32)));

/* Reset-persistent reliability test: a one-sector "superblock" (magic header)
 * plus two alternating data regions (ping-pong). Destroys the card range
 * [base, base + 1 + 2*RT_REGION_SECTORS). base = BAREMETAL_SD_TEST_RAW_SECTOR. */
#define RT_MAGIC          0x53445254u /* 'SDRT' */
#define RT_VERSION        1u
#define RT_REGION_SECTORS 8192u       /* 4 MB per region */

/* disk_initialize can fail intermittently on a warm reset when the card was
 * left in 1.8V/UHS mode from the previous run; power-cycle the card via its
 * reset line and retry so a clean 3.3V power-on identification succeeds. */
#define SD_INIT_MAX_ATTEMPTS 10u

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

/* Issue a multi-block DMA write and block until the completion callback fires. */
static int sd_write_n_wait(uint32_t sector, uint32_t blocks, volatile uint8_t *buf)
{
    dma_done_irq = 0;
    if (p_SD_Driver->disk_write(sector, blocks, buf) != SD_DRV_STATUS_OK) {
        return -1;
    }
    while (!dma_done_irq) {
    }
    return 0;
}

/* Deterministic per-word pattern; mixing the absolute sector index makes a
 * misplaced or duplicated sector detectable. */
static uint32_t rt_word(uint32_t seed, uint32_t gen, uint32_t sector, uint32_t widx)
{
    uint32_t x = seed ^ (gen * 0x9E3779B1u) ^ (sector * 0x85EBCA77u) ^ (widx * 0xC2B2AE3Du);
    x ^= x >> 16;
    x *= 0x7FEB352Du;
    x ^= x >> 15;
    x *= 0x846CA68Bu;
    x ^= x >> 16;
    return x;
}

/* Simple checksum over the 7 header words, used to detect a valid superblock. */
static uint32_t rt_hdr_sum(const volatile uint32_t *h)
{
    uint32_t s = 0xA5A5A5A5u;
    for (int i = 0; i < 7; i++) {
        s = (s * 31u) + h[i];
    }
    return s;
}

/* Fill sdreadbuf in chunks and write [start, start+count) with the pattern. */
static int rt_write_region(uint32_t start, uint32_t count, uint32_t seed, uint32_t gen)
{
    uint32_t *w    = (uint32_t *) sdreadbuf;
    uint32_t  done = 0;

    while (done < count) {
        uint32_t nleft = count - done;
        uint16_t n     = (nleft < SD_CHUNK_SECTORS) ? (uint16_t) nleft : (uint16_t) SD_CHUNK_SECTORS;
        for (uint32_t s = 0; s < n; s++) {
            uint32_t sec = start + done + s;
            for (uint32_t k = 0; k < 128u; k++) {
                w[(s * 128u) + k] = rt_word(seed, gen, sec, k);
            }
        }
        if (sd_write_n_wait(start + done, n, (volatile uint8_t *) sdreadbuf)) {
            return -1;
        }
        done += n;
    }
    return 0;
}

/* Read [start, start+count) in chunks and count words that differ from the
 * expected pattern; records the first mismatch location. */
static uint32_t rt_verify_region(uint32_t start, uint32_t count, uint32_t seed, uint32_t gen,
                                 uint32_t *first_sec, uint32_t *first_widx)
{
    uint32_t *w    = (uint32_t *) sdreadbuf;
    uint32_t  done = 0;
    uint32_t  mism = 0;

    while (done < count) {
        uint32_t nleft = count - done;
        uint16_t n     = (nleft < SD_CHUNK_SECTORS) ? (uint16_t) nleft : (uint16_t) SD_CHUNK_SECTORS;
        if (sd_read_n_wait(start + done, n, (volatile uint8_t *) sdreadbuf)) {
            return count; /* read failure: treat whole chunk as failed */
        }
        for (uint32_t s = 0; s < n; s++) {
            uint32_t sec = start + done + s;
            for (uint32_t k = 0; k < 128u; k++) {
                if (w[(s * 128u) + k] != rt_word(seed, gen, sec, k)) {
                    if (mism == 0) {
                        *first_sec  = sec;
                        *first_widx = k;
                    }
                    mism++;
                }
            }
        }
        done += n;
    }
    return mism;
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

    /* Init can fail intermittently on a warm reset when the card was left in
     * 1.8V/UHS mode; power-cycle the card and retry (see SD_INIT_MAX_ATTEMPTS). */
    int      init_rc = -1;
    uint32_t attempt;
    for (attempt = 1; attempt <= SD_INIT_MAX_ATTEMPTS; attempt++) {
#ifdef BOARD_SD_RESET_GPIO_PORT
        sd_pwr_cb(0); /* drive card reset/power low (includes reset delay) */
        sd_pwr_cb(1); /* release */
        sys_busy_loop_us(SDMMC_RESET_DELAY_US);
#endif
        init_rc = p_SD_Driver->disk_initialize(&sd_param);
        if (init_rc == SD_DRV_STATUS_OK) {
            break;
        }
        printf("SD init attempt %" PRIu32 "/%u failed; power-cycling card and retrying...\n",
               attempt, (unsigned) SD_INIT_MAX_ATTEMPTS);
        p_SD_Driver->disk_uninitialize(SDMMC_DEV_ID);
        sys_busy_loop_us(20000u);
    }
    if (init_rc != SD_DRV_STATUS_OK) {
        printf("SD initialization failed after %u attempts...\n", (unsigned) SD_INIT_MAX_ATTEMPTS);
        return;
    }
    if (attempt > 1u) {
        printf("SD init succeeded on attempt %" PRIu32 ".\n", attempt);
    }

    /* ---- Reset-persistent reliability test ---------------------------------
     * Verify the generation written before the reset is actually on the card
     * (survived loss of cache/RAM), then write the next generation to the other
     * ping-pong region and commit the header to point at it. Repeat on every
     * reset to stress the bus at RTE_SDC_CLOCK_SELECT over time. */
    (void) EndSector;

    uint32_t  hdr_sector = startSec;
    uint32_t  region_a   = startSec + 1u;
    uint32_t  region_b   = region_a + RT_REGION_SECTORS;
    uint32_t *h          = (uint32_t *) sdbuffer;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    if (sd_read_wait(hdr_sector, (volatile uint8_t *) sdbuffer)) {
        printf("Reliability test: header read failed; aborting.\n");
        WAIT_FOREVER_LOOP
    }

    bool     valid = (h[0] == RT_MAGIC) && (h[1] == RT_VERSION) && (h[7] == rt_hdr_sum(h));
    uint32_t generation, seed, active;

    if (valid) {
        active                = h[2];
        generation            = h[3];
        seed                  = h[4];
        uint32_t region_count = h[5];
        uint32_t stored_clk   = h[6];
        uint32_t active_start = active ? region_b : region_a;

        printf("\n==== Reliability: found generation %" PRIu32 " (seed %08" PRIx32
               ", region %c, %" PRIu32 " sectors, written @ %" PRIu32 " Hz) ====\n",
               generation, seed, active ? 'B' : 'A', region_count, stored_clk);
        printf("VERIFY: reading back %" PRIu32 " sectors to confirm they survived the reset...\n",
               region_count);

        uint32_t first_sec = 0, first_widx = 0;
        DWT->CYCCNT   = 0;
        uint32_t mism = rt_verify_region(active_start, region_count, seed, generation,
                                         &first_sec, &first_widx);
        uint32_t vcyc = DWT->CYCCNT;

        if (mism == 0) {
            printf("VERIFY generation %" PRIu32 ": PASS (all %" PRIu32
                   " sectors matched after reset)\n",
                   generation, region_count);
        } else {
            printf("VERIFY generation %" PRIu32 ": FAIL (%" PRIu32 " mismatched words; first at "
                   "sector %" PRIu32 " word %" PRIu32 ")\n",
                   generation, mism, first_sec, first_widx);
        }
        if (vcyc && SystemCoreClock) {
            uint64_t bytes = (uint64_t) region_count * 512u;
            uint32_t kbps  = (uint32_t) ((bytes * (uint64_t) SystemCoreClock) /
                                        ((uint64_t) vcyc * 1024ULL));
            printf("VERIFY read: %" PRIu32 " KB/s (%" PRIu32 ".%02" PRIu32 " MB/s)\n",
                   kbps, kbps / 1024u, ((kbps % 1024u) * 100u) / 1024u);
        }

        /* Advance to the next generation in the other region. */
        generation += 1u;
        seed         = (seed * 1664525u) + 1013904223u + DWT->CYCCNT;
        active      ^= 1u;
    } else {
        printf("\n==== Reliability: no valid header found; initializing test ====\n");
        generation = 1u;
        seed       = 0xDEADBEEFu ^ DWT->CYCCNT;
        active     = 0u;
    }

    uint32_t target_start = active ? region_b : region_a;

    printf("WRITE: generation %" PRIu32 " (seed %08" PRIx32 ") to region %c (%" PRIu32
           " sectors)...\n",
           generation, seed, active ? 'B' : 'A', (uint32_t) RT_REGION_SECTORS);

    DWT->CYCCNT = 0;
    if (rt_write_region(target_start, RT_REGION_SECTORS, seed, generation)) {
        printf("WRITE failed; aborting.\n");
        WAIT_FOREVER_LOOP
    }
    uint32_t wcyc = DWT->CYCCNT;
    if (wcyc && SystemCoreClock) {
        uint64_t bytes = (uint64_t) RT_REGION_SECTORS * 512u;
        uint32_t kbps  = (uint32_t) ((bytes * (uint64_t) SystemCoreClock) /
                                    ((uint64_t) wcyc * 1024ULL));
        printf("WRITE: %" PRIu32 " KB/s (%" PRIu32 ".%02" PRIu32 " MB/s)\n",
               kbps, kbps / 1024u, ((kbps % 1024u) * 100u) / 1024u);
    }

    /* Commit the header last: it only becomes valid once the region is fully on
     * the card, so a reset during the write leaves the previous generation
     * intact and still pointed-to. */
    h[0] = RT_MAGIC;
    h[1] = RT_VERSION;
    h[2] = active;
    h[3] = generation;
    h[4] = seed;
    h[5] = RT_REGION_SECTORS;
    h[6] = (uint32_t) RTE_SDC_CLOCK_SELECT;
    h[7] = rt_hdr_sum(h);
    if (sd_write_wait(hdr_sector, (volatile uint8_t *) sdbuffer)) {
        printf("Header commit failed; aborting.\n");
        WAIT_FOREVER_LOOP
    }

    /* Read the header back from the card to confirm it is committed to media. */
    if (sd_read_wait(hdr_sector, (volatile uint8_t *) sdbuffer) || (h[0] != RT_MAGIC) ||
        (h[3] != generation) || (h[7] != rt_hdr_sum(h))) {
        printf("Header read-back mismatch; commit not confirmed!\n");
        WAIT_FOREVER_LOOP
    }

    printf("\nGeneration %" PRIu32 " committed and confirmed on card.\n", generation);

    /* Performance test: read-only sequential benchmark over the region just
     * written. Large multi-block reads with no per-word compare give the clean
     * data-phase throughput (scales with RTE_SDC_CLOCK_SELECT). */
    {
        uint32_t remaining = RT_REGION_SECTORS;
        uint32_t sec       = target_start;
        uint32_t read_ok   = 0;

        DWT->CYCCNT = 0;
        while (remaining) {
            uint16_t n = (remaining < SD_CHUNK_SECTORS) ? (uint16_t) remaining
                                                        : (uint16_t) SD_CHUNK_SECTORS;
            if (sd_read_n_wait(sec, n, (volatile uint8_t *) sdreadbuf)) {
                printf("Benchmark read failed at sector %" PRIu32 "\n", sec);
                break;
            }
            sec       += n;
            remaining -= n;
            read_ok   += n;
        }
        uint32_t bcyc = DWT->CYCCNT;
        if (bcyc && SystemCoreClock && read_ok) {
            uint64_t bytes = (uint64_t) read_ok * 512u;
            uint32_t kbps  = (uint32_t) ((bytes * (uint64_t) SystemCoreClock) /
                                        ((uint64_t) bcyc * 1024ULL));
            printf("\n==== Sequential read benchmark @ %d Hz ====\n", RTE_SDC_CLOCK_SELECT);
            printf("Read %" PRIu32 " KB in %" PRIu32 " cycles (%u-sector blocks)\n",
                   (uint32_t) (bytes / 1024u), bcyc, (unsigned) SD_CHUNK_SECTORS);
            printf("Read throughput: %" PRIu32 " KB/s (%" PRIu32 ".%02" PRIu32 " MB/s)\n",
                   kbps, kbps / 1024u, ((kbps % 1024u) * 100u) / 1024u);
        }
    }

    printf(">>> Please RESET the board now to verify it survives the reset. <<<\n");
    WAIT_FOREVER_LOOP
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
