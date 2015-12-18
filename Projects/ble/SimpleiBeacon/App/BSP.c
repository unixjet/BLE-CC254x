#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "hal_uart.h"

#include "hal_swi2c.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include "BSP.h"

static void UartInit() ;

static void HalUARTCBack( uint8 port, uint8 event ) ;

void HalUARTCBack( uint8 port, uint8 event )
{
}

void OnBoard_Init ( ) 
{
  UartInit () ;
}


void UartInit()
{
  halUARTCfg_t halUARTCfg = 
  {
    FALSE ,
    HAL_UART_BR_115200 ,
    HAL_UART_FLOW_OFF ,
    0 ,
    0 ,
    { 0 , 0 , 0 , 0 } ,
    { 0 , 0 , 0 , 0 } ,
    TRUE ,
    0 ,
    HalUARTCBack , 
  } ;
  HalUARTOpen ( UART_PORT , &halUARTCfg )  ;
}
