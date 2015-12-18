#include "bcomdef.h"

#include "OnBoard.h"
#include "hal_uart.h"

// app files 

#if (__CODE_MODEL__ == __CM_BANKED__)
__near_func int  putchar( int value )
#else
int  putchar( int value )
#endif
{
  uint8 p = ( uint8 ) value ;
  HalUARTWrite ( UART_PORT , (uint8*)&p , 1/*sizeof ( int ) / sizeof ( uint8 )*/ ) ;
  
  return p ;
}