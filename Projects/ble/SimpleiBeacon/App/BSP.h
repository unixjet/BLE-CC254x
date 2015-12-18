#ifndef __BOARD_SUPPORTED_PACKAGE_H
#define __BOARD_SUPPORTED_PACKAGE_H



#if HAL_UART_DMA == 1 || HAL_UART_ISR == 1 || HAL_UART_SPI == 1 
#define UART_PORT HAL_UART_PORT_0

#elif HAL_UART_DMA == 2 || HAL_UART_ISR == 2 || HAL_UART_SPI == 2
#define UART_PORT HAL_UART_PORT_1

#else
#define UART_PORT HAL_UART_PORT_MAX
#endif


// start from 0x004
#define APP_SAMPLE_EVT                                  0x0004

#define APP_SAMPLE_PERIOD                               1 // 1 ms


void OnBoard_Init ( ) ;

#endif