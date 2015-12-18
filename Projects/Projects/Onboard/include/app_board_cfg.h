#ifndef __APP_BOARD_CFG_
#define __APP_BOARD_CFG_


#if HAL_UART_DMA == 1 || HAL_UART_ISR == 1 
#define UART_PORT HAL_UART_PORT_0

#elif HAL_UART_DMA == 2 || HAL_UART_ISR == 2 
#define UART_PORT HAL_UART_PORT_1

#else
#define UART_PORT HAL_UART_PORT_MAX
#endif

#if  HAL_UART_SPI == 1 
#define SPI_PORT HAL_UART_PORT_0

#elif HAL_UART_SPI == 2
#define SPI_PORT HAL_UART_PORT_1

#else
#define SPI_PORT HAL_UART_PORT_MAX
#endif

#define USB_PORT  HAL_UART_PORT_MAX 

/********************************************************************
 * Board Configuration
 *
 */
#define SENSOR_CLUB         1
#define SENSOR_WRIST        2
#define SENSOR_BODY         3
#define SENSOR_UNKNOWN      4


#define SENSOR_BOARD        SENSOR_CLUB

#define HAL_UART_CUSTOM     

#define HAL_SPI_MASTER

/********************************************************************
 *
 *
 */

#define ENABLE_DEBUG        1


#if ENABLE_DEBUG
  #define dbprintf(format,...)            printf( format , ##__VA_ARGS__ )
#else
  #define dbprintf(format,...)            
#endif 

#endif