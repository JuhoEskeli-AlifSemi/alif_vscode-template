/**
 ******************************************************************************
 * @file    SPI_support.c
 * @version V1.0.0
 * @date    11/25/2024
 * @brief   SPI wrapper functions
 *
 * * @rev
 * 	1.0.0
 * 		*11/25/2024 (GabrielDenk)- initial creation of the file.
 *      *11/26/2025 (lbourree)- port to the DANHUD platform.
 *      *12/22/2025 (lbourree)- added work around for the SPI SS pin control issue.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "Driver_SPI.h"
#include "spi.h"  // for SPI_RX_FIFO_DEPTH and SPI_TX_FIFO_DEPTH
#include "pinconf.h"
#include "SPI_support.h"
#include "spi.h"

#include "Driver_IO.h"
#include "pinconf.h"

#include <stdio.h>
#include <stdbool.h>

/* Private define ------------------------------------------------------------*/
#define LOG_TAG                     "SPI"
#define MHz                         *1000000
#define SPI_CLOCK                   (15 MHz)
#define SPI_HS_CLOCK                (30 MHz)

#define USE_SS_SW_CONTROL           0

/* Public variables ----------------------------------------------------------*/

//ADDED BY Juho
#define ISS_LOGE(tag, fmt, ...) \
    printf("[E][%s] " fmt "\n", tag, ##__VA_ARGS__)

#define MIN(a, b) ((a) < (b) ? (a) : (b))

volatile uint8_t spi1_cb_status = 0;
volatile uint8_t spi0_cb_status = 0;

/* Private variables ---------------------------------------------------------*/
uint8_t dummy_buffer[1600];  // used for read-only and write-only transactions

// External reference to the SPI drivers
extern ARM_DRIVER_SPI ARM_Driver_SPI_(SPI_0);
ARM_DRIVER_SPI *ptrSPI0 = &ARM_Driver_SPI_(SPI_0);
extern ARM_DRIVER_SPI ARM_Driver_SPI_(SPI_1);
ARM_DRIVER_SPI *ptrSPI1 = &ARM_Driver_SPI_(SPI_1);

bool s_spi_initialized[] = {false, false};

/* Private function prototypes -----------------------------------------------*/
static iss_err_status GPIO_LED_Init(uint8_t nLED);
static int32_t pinmux_config(int port);
static iss_err_status GPIO_IRQ_state(uint8_t nBTN, uint8_t state);

// ISR
static void SPI0_cb_func(uint32_t event);
static void SPI1_cb_func(uint32_t event);

/* Private typedef -----------------------------------------------------------*/


/* Public functions ----------------------------------------------------------*/
/**
 * @brief   Return the SPI initialized status
 * @param   port: The port whose initialization status is to be checked.
 * 
 * @retval  TRUE=initialized, FALSE=not initialized.
 */
bool SPI_support_IsInitialized(int port)
{
    return port >= 0 && port < sizeof(s_spi_initialized) ? s_spi_initialized[port] : false;
}

/**
 * @brief   Initialize the SPI port.
 * @param   port: The number of the SPI port to initialize.
 * @param   high_speed: TRUE for high-speed mode using SPI_HS_CLOCK, FALSE for normal mode using SPI_CLOCK.
 * 
 * @retval  status
 */
iss_err_status SPI_support_InitPort(int port, bool high_speed, uint8_t cpol, uint8_t cpha)
{
    int32_t ret = 0;
    ARM_DRIVER_SPI *ptrSPI;
    void (*callback)(uint32_t);

    switch (port)
    {
    case SPI_0: ptrSPI = ptrSPI0; callback = SPI0_cb_func; break;
    case SPI_1: ptrSPI = ptrSPI1; callback = SPI1_cb_func; break;
    default:
        ISS_LOGE(LOG_TAG, "Port %d not supported by driver", port);
        return ISS_ERR_FAIL;
    }

    if (!s_spi_initialized[port])
    {
        ret = pinmux_config(port);
        if (ret != ARM_DRIVER_OK)
        {
            ISS_LOGE(LOG_TAG, "Error in SPI%d pinmux configuration (code=%d)", port, ret);
            return ISS_ERR_FAIL;
        }

        /* Configuration as master */
        ret = ptrSPI->Initialize(callback);
        if (ret != ARM_DRIVER_OK)
        {
            ISS_LOGE(LOG_TAG, "Failed to initialize SPI%d (code=%d)", port, ret);
            return ISS_ERR_FAIL;
        }

        ret = ptrSPI->PowerControl(ARM_POWER_FULL);
        if (ret != ARM_DRIVER_OK)
        {
            ISS_LOGE(LOG_TAG, "Failed to power SPI%d (code=%d)", port, ret);
            return ISS_ERR_FAIL;
        }
    }

#if USE_SS_SW_CONTROL
    uint32_t spi_control = ARM_SPI_MODE_MASTER | ARM_SPI_SS_MASTER_SW | ARM_SPI_DATA_BITS(8);
#else
    uint32_t spi_control = ARM_SPI_MODE_MASTER | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_DATA_BITS(8);
#endif

    #if 0
    if (cpha & 0xFE || cpol & 0xFE)
    {
        ISS_LOGE(LOG_TAG, "Invalid cpol or cpha for SPI%d", port);
        return ISS_ERR_FAIL;
    }
    switch ((cpol << 1) + cpha)
    {
    case 0b00: spi_control |= ARM_SPI_CPOL0_CPHA0; break;
    case 0b01: spi_control |= ARM_SPI_CPOL0_CPHA1; break;
    case 0b10: spi_control |= ARM_SPI_CPOL1_CPHA0; break;
    case 0b11: spi_control |= ARM_SPI_CPOL1_CPHA1; break;
    }
    #else
    spi_control |= ARM_SPI_CPOL0_CPHA0;
    #endif

    ret = ptrSPI->Control(spi_control, high_speed ? SPI_HS_CLOCK : SPI_CLOCK);
    if (ret != ARM_DRIVER_OK)
    {
        ISS_LOGE(LOG_TAG, "Failed to configure SPI%d (code=%d)", port, ret);
        return ISS_ERR_FAIL;
    }

    s_spi_initialized[port] = true;

    return ISS_ERR_OK;
}

/**
 * @brief   Perform SPI data transfer.
 * @param   port: The SPI port on which to perform the transfer.
 * @param   data_out: Buffer to be transmitted (input to function, output from chip).
 * @param   data_in: Buffer to hold received data (input to chip, output from function).
 * @param   num: The number of bytes to transfer.
 * 
 * @retval  status
 * 
 * @note    If one of the in/out buffers is NULL, the transfer will be unidirectional,
 *          either transmit/write or receive/read.  If both buffers are valid pointers
 *          then a simultaneous transmit-and-receive operation will be performed.
 */
iss_err_status SPI_support_Transfer(int port, const void *data_out, void *data_in, uint32_t num)
{
    int32_t ret;
    ARM_DRIVER_SPI *ptrSPI;

    switch (port)
    {
    case SPI_0: ptrSPI = ptrSPI0; break;
    case SPI_1: ptrSPI = ptrSPI1; break;
    default:
        ISS_LOGE(LOG_TAG, "Port %d not supported by driver", port);
        return ISS_ERR_FAIL;
    }

    if (data_out == NULL && data_in == NULL) return ISS_ERR_OK;  // do nothing, nothing can go wrong

    ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);

    if (data_out == NULL)  // read only
    {
        int n_bytes;
        while (num)
        {
            n_bytes = MIN(num, SPI_RX_FIFO_DEPTH);
            ret = ptrSPI->Receive(data_in, n_bytes);
            if (ret != ARM_DRIVER_OK) break;
            num -= n_bytes;
            data_in = (void*)((uint32_t)data_in + n_bytes);
        }
    }
    else if (data_in == NULL)  // write only
    {
        int n_bytes;
        while (num)
        {
            n_bytes = MIN(num, SPI_TX_FIFO_DEPTH);
            ret = ptrSPI->Send(data_out, n_bytes);
            if (ret != ARM_DRIVER_OK) break;
            num -= n_bytes;
            data_out = (void*)((uint32_t)data_out + n_bytes);
            #if 0 // wait for operations to complete to comply with the API
            while( spi1_cb_status == 0);
            spi1_cb_status = 0;
            #endif
        }
    }
    else  // transact (read and write simultaneously)
    {
        int n_bytes;
        while (num)
        {
            n_bytes = MIN(num, MIN(SPI_TX_FIFO_DEPTH, SPI_RX_FIFO_DEPTH));
            ret = ptrSPI->Transfer(data_out, data_in, num);
            if (ret != ARM_DRIVER_OK) break;
            num -= n_bytes;
            data_out = (void*)((uint32_t)data_out + n_bytes);
            data_in = (void*)((uint32_t)data_in + n_bytes);
        }
    }

    ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

    if (ret == ARM_DRIVER_ERROR_BUSY)
    {
        return ISS_ERR_BUSY;
    }
    if (ret != ARM_DRIVER_OK)
    {
        return ISS_ERR_FAIL;
    }
    return ISS_ERR_OK;
}

/* Private functions ---------------------------------------------------------*/
/**
 * @brief   SPI0 & SPI1 pinmux configuration.
 * @param   port: SPI port number.
 * 
 * @retval  Execution status.
 */
static int32_t pinmux_config(int port)
{
    int32_t ret = ARM_DRIVER_OK;

    if (port == SPI_0)
    {
        /* pinmux configurations for SPI0 pins (using B version pins) */
        ret = pinconf_set(PORT_5, PIN_0, PINMUX_ALTERNATE_FUNCTION_4, PADCTRL_READ_ENABLE);
        if (ret)
        {
            printf("ERROR: Failed to configure PINMUX for SPI0_MISO_PIN\n");
            return ret;
        }
        ret = pinconf_set(PORT_5, PIN_1, PINMUX_ALTERNATE_FUNCTION_4, 0);
        if (ret)
        {
            printf("ERROR: Failed to configure PINMUX for SPI0_MOSI_PIN\n");
            return ret;
        }
        ret = pinconf_set(PORT_5, PIN_3, PINMUX_ALTERNATE_FUNCTION_3, 0);
        if (ret)
        {
            printf("ERROR: Failed to configure PINMUX for SPI0_CLK_PIN\n");
            return ret;
        }
#if !USE_SS_SW_CONTROL
        ret = pinconf_set(PORT_5, PIN_2, PINMUX_ALTERNATE_FUNCTION_4, 0);
        if (ret)
        {
            printf("ERROR: Failed to configure PINMUX for SPI0_SS_PIN\n");
            return ret;
        }
#endif
    }
    else if (port == SPI_1)
    {
        /* pinmux configurations for SPI1 pins (as master using C version pins) */
        ret = pinconf_set(PORT_(SPI1_MISO_PORT), SPI1_MISO_PIN, PINMUX_ALTERNATE_FUNCTION_2, PADCTRL_READ_ENABLE);
        if (ret)
        {
            ISS_LOGE(LOG_TAG, "Configure PINMUX for SPI1_MISO_PIN");
            return ret;
        }
        ret = pinconf_set(PORT_(SPI1_MOSI_PORT), SPI1_MOSI_PIN, PINMUX_ALTERNATE_FUNCTION_2, PADCTRL_SLEW_RATE_FAST | PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA);
        if (ret)
        {
            ISS_LOGE(LOG_TAG, "Configure PINMUX for SPI1_MOSI_PIN");
            return ret;
        }
        ret = pinconf_set(PORT_(SPI1_SCLK_PORT), SPI1_SCLK_PIN, PINMUX_ALTERNATE_FUNCTION_2, PADCTRL_SLEW_RATE_FAST | PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA);
        if (ret)
        {
            ISS_LOGE(LOG_TAG, "Configure PINMUX for SPI1_CLK_PIN");
            return ret;
        }
#if !USE_SS_SW_CONTROL
        ret = pinconf_set(PORT_(SPI1_SS0_PORT), SPI1_SS0_PIN, PINMUX_ALTERNATE_FUNCTION_2, PADCTRL_SLEW_RATE_FAST | PADCTRL_OUTPUT_DRIVE_STRENGTH_12MA);
        if (ret)
        {
            ISS_LOGE(LOG_TAG, "Configure PINMUX for SPI1_SS_PIN");
            return ret;
        }
#endif
    }
    else return ARM_DRIVER_ERROR_UNSUPPORTED;

    return ret;
}

/* ISR */
/**
 * @brief   SPI callback ???
 * @param   event: SPI Interrupt events
 */
static void SPI0_cb_func(uint32_t event)
{
    #if 0
    BaseType_t HigherPriorityTaskWoken = pdFALSE;
    portYIELD_FROM_ISR(HigherPriorityTaskWoken);
    #else
    spi0_cb_status = 0;
    #endif
}

/**
 * @brief   SPI callback ???
 * @param   event: SPI Interrupt events
 */
static void SPI1_cb_func(uint32_t event)
{
    #if 0
    BaseType_t HigherPriorityTaskWoken = pdFALSE;
    portYIELD_FROM_ISR(HigherPriorityTaskWoken);
    #else
    spi1_cb_status = 1;
    #endif
}


/* EOF */