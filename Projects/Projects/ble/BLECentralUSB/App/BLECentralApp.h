#ifndef __BLE_CENTRAL_APP_H_
#define __BLE_CENTRAL_APP_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */


// BLE Central Task Events
#define BCT_START_DEVICE_EVT                              0x0001
#define BCT_START_DISCOVERY_EVT                           0x0002
   
/*********************************************************************
 * MACROS
 */

// LCD macros
#if HAL_LCD == TRUE
#define LCD_WRITE_STRING(str, option)                       HalLcdWriteString( (str), (option))
#define LCD_WRITE_SCREEN(line1, line2)                      HalLcdWriteScreen( (line1), (line2) )
#define LCD_WRITE_STRING_VALUE(title, value, format, line)  HalLcdWriteStringValue( (title), (value), (format), (line) )
#else
#define LCD_WRITE_STRING(str, option)                     
#define LCD_WRITE_SCREEN(line1, line2)                    
#define LCD_WRITE_STRING_VALUE(title, value, format, line)
#endif

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void BLECentralApp_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 BLECentralApp_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif