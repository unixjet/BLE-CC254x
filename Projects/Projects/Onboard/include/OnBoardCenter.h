#ifndef __ON_BOARD_CENTER_
#define __ON_BOARD_CENTER_


#define APP_USBUART_EVT                    0x0004
#define APP_ENABLE_SENSOR_EVT              0x0008

#define APP_START_DISCOVERY_EVT            0x0010
#define APP_SVCFOUND_REQ_EVT               0x0020

#define APP_WRITECHAR_REQ_EVT              0x0080 // request
#define APP_WRITECHAR_RSP_EVT              0x0100 // respond

#define APP_START_DEVICE                   0x0200 

#define APP_USBUART_TIMEOUT                1000
#define APP_START_DISCOVERY_TIMEOUT        5000

#define APP_START_DEVICE_TIMEOUT           8000

void OnBoardInit ( uint8 task_id ) ;

void OnBoard_StartDevice() ;

void BoardTask( ) ;

#endif