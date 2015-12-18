#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "hal_uart.h"

#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"

#include "OnBoardCenter.h"

#define HAL_BOARD_IO_SYSTEM_ON1_ENABLE_PORT   0   
#define HAL_BOARD_IO_SYSTEM_ON1_ENABLE_PIN    0

#define HAL_BOARD_IO_SYSTEM_ON2_ENABLE_PORT   1   
#define HAL_BOARD_IO_SYSTEM_ON2_ENABLE_PIN    2

#if !defined ( HAL_UART_SPI ) 
#define HAL_BOARD_IO_PORTA_ENABLE_PORT      1   
#define HAL_BOARD_IO_PORTA_ENABLE_PIN       6

#define HAL_BOARD_IO_PORTB_ENABLE_PORT      1   
#define HAL_BOARD_IO_PORTB_ENABLE_PIN       7
#endif

typedef struct _app_context
{
  uint8 appTaskId ;
  uint8 fwDataBuf[256] ; // in single thread (in this platform ), it is safe , 
  
} context_t ;

static context_t appContext = { 0 ,} ;
static context_t * pAppContext = & appContext ;

#if ( HAL_UART_ISR || HAL_UART_DMA ) 
static void UartInit ( void ) ;
static void HalUARTCBack( uint8 port, uint8 event ) ;
#endif

#if ( HAL_UART_USB ) 
static void USBInit( void ) ;
static void HalUSBCBack( uint8 port, uint8 event ) ;
#endif 

#if ( HAL_UART_SPI )
static void SPIInit( void ) ;
static void HalSPICBack( uint8 port , uint8 event ) ;
#endif

void OnBoardInit ( uint8 task_id )
{
  pAppContext->appTaskId = task_id ;
  
  MCU_IO_OUTPUT(HAL_BOARD_IO_SYSTEM_ON1_ENABLE_PORT, HAL_BOARD_IO_SYSTEM_ON1_ENABLE_PIN, 1) ;
  MCU_IO_OUTPUT(HAL_BOARD_IO_SYSTEM_ON2_ENABLE_PORT, HAL_BOARD_IO_SYSTEM_ON2_ENABLE_PIN, 1) ;
  
#if !defined ( HAL_UART_SPI ) 
//  MCU_IO_OUTPUT(HAL_BOARD_IO_PORTA_ENABLE_PORT, HAL_BOARD_IO_PORTA_ENABLE_PIN, 1) ;
//  MCU_IO_OUTPUT(HAL_BOARD_IO_PORTB_ENABLE_PORT, HAL_BOARD_IO_PORTB_ENABLE_PIN, 1) ;
#endif
  
#if ( HAL_UART_USB ) 
  USBInit( ) ;
#endif
  
  osal_start_timerEx( pAppContext->appTaskId , APP_START_DEVICE , APP_START_DEVICE_TIMEOUT ) ; 

}

void OnBoard_StartDevice()
{
#if ( HAL_UART_ISR || HAL_UART_DMA ) 
  UartInit ( ) ;
#endif

#if ( HAL_UART_SPI )
  SPIInit( ) ;
#endif
  
   osal_start_timerEx( pAppContext->appTaskId , APP_USBUART_EVT , APP_USBUART_TIMEOUT ) ;
}

void BoardTask( ) 
{
}


#if ( HAL_UART_ISR || HAL_UART_DMA ) 
void UartInit ( void )
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
  
  HalUARTOpen ( UART_PORT , & halUARTCfg )  ;
}

void HalUARTCBack( uint8 port, uint8 event )
{

}
#endif

static uint8 usb_tx_empty = 1 ; 
#if ( HAL_UART_USB ) 
void USBInit( void )
{
   halUARTCfg_t halUSBCfg = 
  {
    FALSE ,
    // when to open usb port , baudrate is not a matter !
    // yep ~ , anything is fuctional 
    HAL_UART_BR_115200 ,  
    HAL_UART_FLOW_OFF ,
    0 ,
    0 ,
    { 0 , 0 , 0 , 0 } ,
    { 0 , 0 , 0 , 0 } ,
    TRUE ,
    0 ,
    HalUSBCBack , 
  } ;
 
  HalUARTOpen ( USB_PORT , &halUSBCfg )  ;
}

void HalUSBCBack( uint8 port, uint8 event )
{
//  static uint8 buf[256] ;
  uint8 * buf = pAppContext->fwDataBuf ;
  uint16 bufLen = sizeof pAppContext->fwDataBuf ;
  uint16 len = 0 ;
 
  uint16 tryLen = 0 ; // bytes of tring to read 
  uint16 rdlen  = 0 ; // bytes read 
  
 // if ( event & HAL_UART_RX_ABOUT_FULL )
  {
    len = Hal_UART_RxBufLen ( USB_PORT ) ;
  
    while ( len > 0 ) 
    {
      tryLen = MIN ( bufLen , len ) ;
    
      // read from USB
      rdlen = HalUARTRead( USB_PORT , buf , tryLen ) ; 
       
      // write to SPI 
      HalUARTWrite( SPI_PORT , buf , rdlen ) ;
       
      len -= rdlen ;
    }
  }
  if ( event & HAL_UART_TX_EMPTY )
  {
    usb_tx_empty = 1 ;
  }
  
}
#endif

#if ( HAL_UART_SPI ) 

void SPIInit( void ) 
{
  halUARTCfg_t halSPICfg = 
  {
    FALSE ,
    // when to open usb port , baudrate is not a matter !
    // yep ~ , anything is fuctional 
    HAL_UART_BR_115200 ,  
    HAL_UART_FLOW_OFF ,
    0 ,
    0 ,
    { 0 , 0 , 0 , 0 } ,
    { 0 , 0 , 0 , 0 } ,
    TRUE ,
    0 ,
    HalSPICBack , 
  } ;
 
  HalUARTOpen ( SPI_PORT , &halSPICfg )  ;
}

void HalSPICBack( uint8 port , uint8 event ) 
{
  uint8 * buf = pAppContext->fwDataBuf ;
  uint16 bufLen = sizeof pAppContext->fwDataBuf ;
  uint16 len = 0 ;
 
  uint16 tryLen = 0 ; // bytes of tring to read 
  uint16 rdlen  = 0 ; // bytes read 
  
  len = Hal_UART_RxBufLen ( SPI_PORT ) ;
//HalUARTWrite( UART_PORT , "spi to uart\n" , 12 ) ; 
 
//  if ( len ) rxSPI = 1 ;
  
  while ( len > 0 
//         && usb_tx_empty 
           ) 
  {
    usb_tx_empty = 0 ; // spi data has been transfered to usb buffer 
    
    tryLen = MIN ( bufLen , len ) ;
    
    // read from SPI Port 
     rdlen = HalUARTRead( SPI_PORT , buf , tryLen ) ; 
       
    // write to USB 
     HalUARTWrite( USB_PORT , buf , rdlen ) ;
       
     len -= rdlen ;
  }
}

#endif
