#ifndef __HAL_UART_CUSTOM__
#define __HAL_UART_CUSTOM__

#pragma once 

//#if defined ( HAL_UART_CUSTOM ) 
  #if HAL_UART_DMA
  #include "_hal_uart_dma.c"
  #endif
  #if HAL_UART_ISR
  #include "_hal_uart_isr.c"
  #endif
  #if HAL_UART_SPI
  #include "_hal_uart_spi.c"
  #endif
  #if HAL_UART_USB
  #include "_hal_uart_usb.c"
  #endif
//#endif 

#endif