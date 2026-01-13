/**
 ******************************************************************************
 * @file    SPI_support.h
 * @version V1.0.0
 * @date    11/25/2024
 * @brief   Header file
 *
 ******************************************************************************
 */
 
#ifndef SPI_SUPPORT_H_
#define SPI_SUPPORT_H_

/* Includes -----------------------------------------------------------------*/
//#include "common_iss.h"

/* Exported define ----------------------------------------------------------*/
#define  SPI_0                      0    /* SPI0 instance */
#define  SPI_1                      1    /* SPI1 instance */

#define SPI1_MISO_PORT              14
#define SPI1_MISO_PIN               4
#define SPI1_MOSI_PORT              14
#define SPI1_MOSI_PIN               5
#define SPI1_SCLK_PORT              14 
#define SPI1_SCLK_PIN               6
#define SPI1_SS0_PORT               14
#define SPI1_SS0_PIN                7

//#define BLE_WIFI_PORT SPI1
//#define BLE_WIFI_CPOL 0
//#define BLE_WIFI_CPHA 0

/* Typedef ------------------------------------------------------------------*/
typedef enum
{
    ISS_ERR_OK = 0,
    ISS_ERR_FAIL,
    ISS_ERR_BUSY
} iss_err_status;

/* Exported variables -------------------------------------------------------*/

/* Exported function prototypes ---------------------------------------------*/
bool 			SPI_support_IsInitialized(int port);
iss_err_status	SPI_support_InitPort(int port, bool high_speed, uint8_t cpol, uint8_t cpha);
iss_err_status	SPI_support_Transfer(int port, const void *data_out, void *data_in, uint32_t num);

#endif /* SPI_SUPPORT_H_ */