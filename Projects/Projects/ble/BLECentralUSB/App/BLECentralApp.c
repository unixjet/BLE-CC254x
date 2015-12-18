
/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>

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

//#include "simpleGATTprofile.h"
#include "sensortag_util.h"
#include "gyroAccelService.h"

#include "BLECentralApp.h"

// App 
#include "OnBoardCenter.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      6

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      6

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           100

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           500

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE


// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

// Write action
enum
{
  BLE_WRITE_STANDBY ,
  BLE_WRITE_SENSOR_DATA ,
  BLE_WRITE_SENSOR_CONF ,
  BLE_WRITE_SENSOR_PERI ,
  BLE_WRITE_DONE      ,
} ;

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
  typedef struct _bleappcontext
  {
     uint8 taskID ;
     uint16 pkgIndex ;
     
     uint8 BLEScanRes;
     uint8 BLEScanIdx;

    // Scan result list
     gapDevRec_t BLEDevList[ DEFAULT_MAX_SCAN_RES ];
     
     uint8 desireAddrType;
     uint8 desirePeerAddr [ B_ADDR_LEN ] ;
     
     uint8 BLEScanning  ;

// RSSI polling state
     uint8 BLERssi;

// Connection handle of current connection 
     uint16 BLEConnHandle  ;

// Application state
     uint8 BLEState  ;

// Discovery state
     uint8 BLEDiscState  ;

// Discovered service start and end handle
     uint16 BLESvcStartHdl ;
     uint16 BLESvcEndHdl  ;

// Discovered characteristic handle
     uint16 BLESensorUUID ;
     uint16 * pBLESensorHdl ;
     
     uint16 BLESensorDataHdl ; 
     uint16 BLESensorConfHdl ;
     uint16 BLESensorPeriHdl ;
     
     uint8  BLESensorToSetValue ;
     
     bool   BLEBondPeerDevice ;

// case read / write 
     uint8 BLEWriteAction ;
     
// GATT read/write procedure state
     bool BLEProcedureInProgress ;
     
     bool BLEWriteSuccess ;

  } bleappcontext_t ;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
 
static bleappcontext_t   BLEAppContext = { 0 , }  ;
static bleappcontext_t * pBLEAppContext = & BLEAppContext ;

// Task ID for internal task/event processing
static uint8 BLETaskId;

// GAP GATT Attributes
static const uint8 BLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void BLECentralAppProcessGATTMsg( gattMsgEvent_t *pMsg );
static void BLECentralAppRssiCB( uint16 connHandle, int8  rssi );
static void BLECentralAppEventCB( gapCentralRoleEvent_t *pEvent );
static void BLECentralAppPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void BLECentralAppPairStateCB( uint16 connHandle, uint8 state, uint8 status );

#if defined( CC2540_MINIDK )
static void BLECentralApp_HandleKeys( uint8 shift, uint8 keys );
#endif

static void BLECentralApp_StartDiscovery ( void ) ;
static void BLECentralApp_StartConnect ( uint8 idx ) ;
//static bStatus_t BLECentralApp_ConnectDstDevice ( uint8 addrType , uint8 * peerAddr ) ;
static void BLECentralApp_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void BLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void BLECentralApp_StartServiceDiscovery( void );
static bool BLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void BLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
static char *bdAddr2Str ( uint8 *pAddr );

static void BLECentralAppContextInit ( void ) ;

static bStatus_t sensorDataNotify ( uint16 connHandle, uint16 attrHandle , uint8 set ) ;
static bStatus_t sensorConfSet( uint16 connHandle, uint16 attrHandle , uint8 set ) ;
static bStatus_t sensorPeriSet( uint16 connHandle, uint16 attrHandle , uint8 value ) ;

static bStatus_t BLECentralApp_GATTWriteCharValueReq ( uint8 Action ) ; // request 
static bStatus_t BLECentralApp_GATTWriteCharValueRsp ( uint8 Action ) ; // respond 


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t BLECentralAppRoleCB =
{
  BLECentralAppRssiCB,       // RSSI callback
  BLECentralAppEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t BLECentralAppBondCB =
{
  BLECentralAppPasscodeCB,
  BLECentralAppPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      BLECentralApp_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void BLECentralApp_Init( uint8 task_id )
{
   // Appication context initialize
  BLECentralAppContextInit( ) ;
  
  BLETaskId              = task_id ;
  pBLEAppContext->taskID = task_id ;
  

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) BLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( BLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

#if defined( CC2540_MINIDK )
  // Register for all key events - This app will handle all key events
  RegisterForKeys( BLETaskId );
  
#endif
  
  // makes sure LEDs are off
  HalLedSet( ( HAL_LED_1 | HAL_LED_2 ), HAL_LED_MODE_OFF );
  
  // App init
  OnBoardInit ( task_id ) ;
  
  // Setup a delayed profile startup
  osal_set_event( BLETaskId, BCT_START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 BLECentralApp_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( BLETaskId )) != NULL )
    {
      BLECentralApp_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return ( events ^ SYS_EVENT_MSG );
  }

  if ( events & BCT_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &BLECentralAppRoleCB );

    // Register with bond manager after starting device
    VOID GAPBondMgr_Register( (gapBondCBs_t *) & BLECentralAppBondCB );
    
#if !defined( CC2540_MINIDK )  
    osal_start_timerEx( BLETaskId , APP_START_DISCOVERY_EVT , APP_START_DISCOVERY_TIMEOUT ) ;
#endif

    return ( events ^ BCT_START_DEVICE_EVT );
  }
  
  if ( events & APP_START_DISCOVERY_EVT )
  {
    // start to Discovery
    BLECentralApp_StartDiscovery ( ) ;
    return ( events ^ APP_START_DISCOVERY_EVT ) ;
  }

  if ( events & BCT_START_DISCOVERY_EVT )
  {
    BLECentralApp_StartServiceDiscovery( );
    return ( events ^ BCT_START_DISCOVERY_EVT );
  }
  
  if ( events & APP_SVCFOUND_REQ_EVT )
  {
    dbprintf ( " Get SVC Handle dataHdl 0x%x , confHdl 0x%x , periHdl 0x%x \r\n" ,
                pBLEAppContext->BLESensorDataHdl ,
                pBLEAppContext->BLESensorConfHdl ,
                pBLEAppContext->BLESensorPeriHdl ) ;
      
    dbprintf ("Sensor Service Found Done \r\n" ) ;
    pBLEAppContext->BLEWriteAction = BLE_WRITE_SENSOR_DATA ;
    pBLEAppContext->BLESensorToSetValue = 1 ;
    osal_set_event ( BLETaskId , APP_WRITECHAR_REQ_EVT ) ;
    
    return ( events ^ APP_SVCFOUND_REQ_EVT ) ;
  }
  
  if ( events & APP_WRITECHAR_REQ_EVT )
  {
    VOID BLECentralApp_GATTWriteCharValueReq ( pBLEAppContext->BLEWriteAction ) ;
    return ( events ^ APP_WRITECHAR_REQ_EVT ) ;
  }
  if ( events & APP_WRITECHAR_RSP_EVT )
  {
    VOID BLECentralApp_GATTWriteCharValueRsp ( pBLEAppContext->BLEWriteAction )  ;
    if ( pBLEAppContext->BLEWriteAction == BLE_WRITE_SENSOR_DATA )
    {
      pBLEAppContext->BLEWriteAction      = BLE_WRITE_SENSOR_PERI ;
      pBLEAppContext->BLESensorToSetValue = 100 ;
      osal_set_event ( BLETaskId , APP_WRITECHAR_REQ_EVT ) ;
     
    }
    else if ( pBLEAppContext->BLEWriteAction == BLE_WRITE_SENSOR_PERI )
    {
      pBLEAppContext->BLEWriteAction      = BLE_WRITE_SENSOR_CONF ;
      pBLEAppContext->BLESensorToSetValue = 1 ;
      osal_set_event ( BLETaskId , APP_WRITECHAR_REQ_EVT ) ;
    }
    else 
    {
      pBLEAppContext->BLEWriteAction = BLE_WRITE_DONE ;
    }
    return ( events ^ APP_WRITECHAR_RSP_EVT ) ;
  }
  
  if ( events & APP_ENABLE_SENSOR_EVT )
  {
    sensorDataNotify ( pBLEAppContext->BLEConnHandle , pBLEAppContext->BLESensorDataHdl + 1 , pBLEAppContext->BLESensorToSetValue ) ;
    return ( events ^ APP_ENABLE_SENSOR_EVT ) ;
  }
  
  if ( events & APP_USBUART_EVT ) 
  {
    printf ( "I am here\r\n" ) ;
    osal_start_timerEx( BLETaskId , APP_USBUART_EVT , APP_USBUART_TIMEOUT ) ;
    return ( events ^ APP_USBUART_EVT ) ;
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void BLECentralApp_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
#if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      BLECentralApp_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
#endif

    case GATT_MSG_EVENT:
      BLECentralAppProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}

#if defined( CC2540_MINIDK )
/*********************************************************************
 * @fn      BLECentralApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static uint8 gStatus;
static void BLECentralApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_UP )
  {
    // Start or stop discovery
    if ( pBLEAppContext->BLEState != BLE_STATE_CONNECTED )
    {
      if ( !pBLEAppContext->BLEScanning )
      {
        pBLEAppContext->BLEScanning = TRUE;
        pBLEAppContext->BLEScanRes = 0;
        
        LCD_WRITE_STRING( "Discovering...", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
        
        GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                       DEFAULT_DISCOVERY_WHITE_LIST );      
      }
      else
      {
        GAPCentralRole_CancelDiscovery();
      }
    }
    else if ( pBLEAppContext->BLEState == BLE_STATE_CONNECTED &&
              pBLEAppContext->BLECharHdl != 0 &&
              pBLEAppContext->BLEProcedureInProgress == FALSE )
    {
      uint8 status;
#if 0 
      /*
      // Do a read or write as long as no other read or write is in progress
      if ( pBLEAppContext->BLEDoWrite )
      {
        // Do a write
        attWriteReq_t req;
        
        req.handle = pBLEAppContext->BLECharHdl;
        req.len = 1;
        req.value[0] = pBLEAppContext->BLECharVal;
        req.sig = 0;
        req.cmd = 0;
        status = GATT_WriteCharValue( pBLEAppContext->BLEConnHandle, &req, BLETaskId );         
      }
      else
      {
        // Do a read
        attReadReq_t req;
        
        req.handle = pBLEAppContext->BLECharHdl;
        status = GATT_ReadCharValue( pBLEAppContext->BLEConnHandle, &req, BLETaskId );
      }
      
      if ( status == SUCCESS )
       {
        pBLEAppContext->BLEProcedureInProgress = TRUE ;
        pBLEAppContext->BLEWriteAction = BLE_WRITE_UNKNOWN ;
        pBLEAppContext->BLEDoWrite = ! pBLEAppContext->BLEDoWrite;
       }
     */
#else
       pBLEAppContext->BLESensorNotiSet = 1 ;
       
       status = sensorDataNotify ( pBLEAppContext->BLEConnHandle , pBLEAppContext->BLESensorNotifyHdl , pBLEAppContext->BLESensorNotiSet ) ;
       
       if ( status == SUCCESS )
       {
          LCD_WRITE_STRING_VALUE( "Enable Sensor data", pBLEAppContext->BLESensorNotiSet , 10 , HAL_LCD_LINE_3 );
          pBLEAppContext->BLEDoWrite = TRUE ;//! pBLEAppContext->BLEDoWrite;
          
         if ( ! pBLEAppContext->BLESensorNotiSet )
         {
            pBLEAppContext->BLESensorNotiSet = 1 ;  
         }
         else
           pBLEAppContext->BLESensorNotiSet = 0 ;
       }
#endif
       
    }    
  }

  if ( keys & HAL_KEY_LEFT )
  {
    // Display discovery results
    if ( !pBLEAppContext->BLEScanning && pBLEAppContext->BLEScanRes > 0 )
    {
        // Increment index of current result (with wraparound)
        pBLEAppContext->BLEScanIdx++;
        if ( pBLEAppContext->BLEScanIdx >= pBLEAppContext->BLEScanRes )
        {
          pBLEAppContext->BLEScanIdx = 0;
        }
        
        LCD_WRITE_STRING_VALUE( "Device", pBLEAppContext->BLEScanIdx + 1 ,
                                10, HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( pBLEAppContext->BLEDevList[pBLEAppContext->BLEScanIdx].addr ),
                          HAL_LCD_LINE_2 );
    }
  }

  if ( keys & HAL_KEY_RIGHT )
  {
    // Connection update
    if ( pBLEAppContext->BLEState == BLE_STATE_CONNECTED )
    {
      GAPCentralRole_UpdateLink( pBLEAppContext->BLEConnHandle,
                                 DEFAULT_UPDATE_MIN_CONN_INTERVAL,
                                 DEFAULT_UPDATE_MAX_CONN_INTERVAL,
                                 DEFAULT_UPDATE_SLAVE_LATENCY,
                                 DEFAULT_UPDATE_CONN_TIMEOUT );
    }
  }
  
  if ( keys & HAL_KEY_CENTER )
  {
    uint8 addrType;
    uint8 *peerAddr;
    
    // Connect or disconnect
    if ( pBLEAppContext->BLEState == BLE_STATE_IDLE )
    {
      // if there is a scan result
      if ( pBLEAppContext->BLEScanRes > 0 )
      {
        // connect to current device in scan result
        peerAddr = pBLEAppContext->BLEDevList[pBLEAppContext->BLEScanIdx].addr;
        addrType = pBLEAppContext->BLEDevList[pBLEAppContext->BLEScanIdx].addrType;
      
        pBLEAppContext->BLEState = BLE_STATE_CONNECTING;
        
        GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                      DEFAULT_LINK_WHITE_LIST,
                                      addrType, peerAddr );
        
        pBLEAppContext->desireAddrType = addrType ;
        osal_memcpy ( pBLEAppContext->desirePeerAddr  , peerAddr , 
                     sizeof ( pBLEAppContext->BLEDevList[pBLEAppContext->BLEScanIdx].addr ) / sizeof ( pBLEAppContext->BLEDevList[pBLEAppContext->BLEScanIdx].addr[0] ) ) ;
  
        LCD_WRITE_STRING( "Connecting", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( peerAddr ), HAL_LCD_LINE_2 ); 
      }
    }
    else if ( pBLEAppContext->BLEState == BLE_STATE_CONNECTING ||
              pBLEAppContext->BLEState == BLE_STATE_CONNECTED )
    {
      // disconnect
      pBLEAppContext->BLEState = BLE_STATE_DISCONNECTING;

      gStatus = GAPCentralRole_TerminateLink( pBLEAppContext->BLEConnHandle );
      
      LCD_WRITE_STRING( "Disconnecting", HAL_LCD_LINE_1 ); 
    }
  }
  
  if ( keys & HAL_KEY_DOWN )
  {
    // Start or cancel RSSI polling
    if ( pBLEAppContext->BLEState == BLE_STATE_CONNECTED )
    {
      if ( !pBLEAppContext->BLERssi )
      {
        pBLEAppContext->BLERssi = TRUE;
        GAPCentralRole_StartRssi( pBLEAppContext->BLEConnHandle, DEFAULT_RSSI_PERIOD );
      }
      else
      {
        pBLEAppContext->BLERssi = FALSE;
        GAPCentralRole_CancelRssi( pBLEAppContext->BLEConnHandle );
        
        LCD_WRITE_STRING( "RSSI Cancelled", HAL_LCD_LINE_1 );
      }
    }
  }
}

#endif

/*********************************************************************
 * @fn      BLECentralApp_StartDiscovery
 *
 * @brief   start to Discovery 
 *
 * @return  none
 */
void BLECentralApp_StartDiscovery ( void )
{
  if ( pBLEAppContext->BLEState != BLE_STATE_CONNECTED )
  {
    if ( !pBLEAppContext->BLEScanning )
    {
      pBLEAppContext->BLEScanning = TRUE;
      pBLEAppContext->BLEScanRes = 0;
      
      LCD_WRITE_STRING( "Discovering...", HAL_LCD_LINE_1 );
      LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
      
      GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                     DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                     DEFAULT_DISCOVERY_WHITE_LIST );      
    }
  }
}

/*********************************************************************
 * @fn      BLECentralApp_StartConnect
 *
 * @brief   start to connect the device selected by index  
 *
 * @return  none
 */
void BLECentralApp_StartConnect ( uint8 idx )
{
  if ( !pBLEAppContext->BLEScanning && pBLEAppContext->BLEScanRes > 0 )
  {
    // wrap 
      if ( idx >= pBLEAppContext->BLEScanRes )
      {
        idx = 0;
      }
      
      LCD_WRITE_STRING_VALUE( "Device", pBLEAppContext->BLEScanIdx + 1 ,
                              10, HAL_LCD_LINE_1 );
      LCD_WRITE_STRING( bdAddr2Str( pBLEAppContext->BLEDevList[pBLEAppContext->BLEScanIdx].addr ),
                        HAL_LCD_LINE_2 );
  }
  
  if ( pBLEAppContext->BLEState == BLE_STATE_IDLE )
  {
    uint8 addrType ;
    uint8 *peerAddr ;
    
    // if there is a scan result
    if ( pBLEAppContext->BLEScanRes > 0 )
    {
      // connect to current device in scan result
      peerAddr = pBLEAppContext->BLEDevList[ idx ].addr;
      addrType = pBLEAppContext->BLEDevList[ idx ].addrType;
    
      pBLEAppContext->BLEState = BLE_STATE_CONNECTING;
      
      GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                    DEFAULT_LINK_WHITE_LIST,
                                    addrType, peerAddr );
      
      pBLEAppContext->desireAddrType = addrType ;
      osal_memcpy ( pBLEAppContext->desirePeerAddr  , peerAddr , 
                   sizeof ( pBLEAppContext->BLEDevList[ idx ].addr ) / sizeof ( pBLEAppContext->BLEDevList[ idx ].addr[0] ) ) ;

      pBLEAppContext->BLEScanIdx = idx ;
      
      LCD_WRITE_STRING( "Connecting", HAL_LCD_LINE_1 );
      LCD_WRITE_STRING( bdAddr2Str( peerAddr ), HAL_LCD_LINE_2 ); 
      dbprintf ( "Connecting\r\n" ) ;
    }
  }
}

/*********************************************************************
 * @fn      BLECentralApp_ConnectDstDevice
 *
 * @brief   if get a valid device , and address type , 
 *          we can use it to build the connection and skip the scan 
 *          
 *
 * @return  none
 */
/*
bStatus_t BLECentralApp_ConnectDstDevice ( uint8 addrType , uint8 * peerAddr )
{
  bStatus_t status = FAILURE ;
  
  if ( pBLEAppContext->BLEState == BLE_STATE_IDLE )
  {
    pBLEAppContext->BLEState = BLE_STATE_CONNECTING;
      
    status = GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                    DEFAULT_LINK_WHITE_LIST,
                                    addrType, peerAddr );
  }
  return status ;
}
*/
/*********************************************************************
 * @fn      BLECentralApp_GATTWriteCharValueRsq
 *
 * @brief   Process Central Wrist request
 *
 * @return  none
 */
bStatus_t BLECentralApp_GATTWriteCharValueReq ( uint8 Action ) 
{
  bStatus_t status = FAILURE ;
  pBLEAppContext->BLEWriteSuccess = FALSE ;
  
  if ( pBLEAppContext->BLEProcedureInProgress )
    return status ;
  
  switch ( Action )
  {
  case BLE_WRITE_SENSOR_DATA :
    status = sensorDataNotify ( pBLEAppContext-> BLEConnHandle , pBLEAppContext->BLESensorDataHdl + 1 , pBLEAppContext->BLESensorToSetValue ) ;
    break ;
  case BLE_WRITE_SENSOR_CONF :
    status = sensorConfSet ( pBLEAppContext-> BLEConnHandle , pBLEAppContext->BLESensorConfHdl , pBLEAppContext->BLESensorToSetValue ) ;
    break ;
  case BLE_WRITE_SENSOR_PERI :
    status = sensorPeriSet ( pBLEAppContext-> BLEConnHandle , pBLEAppContext->BLESensorPeriHdl , pBLEAppContext->BLESensorToSetValue ) ;
    break ;
    
  default :
    break ;
  }
  
  return status ;
}

/*********************************************************************
 * @fn      BLECentralApp_GATTWriteCharValueRsp
 *
 * @brief   Process Central Wrist Respond
 *
 * @return  none
 */
bStatus_t BLECentralApp_GATTWriteCharValueRsp ( uint8 Action )  // respond
{
  bStatus_t status = FAILURE ;
  
  switch ( Action ) 
  {
  case BLE_WRITE_SENSOR_DATA :
    if ( pBLEAppContext->BLEWriteSuccess ) 
    {
      status = SUCCESS ;
    }
    break ;
  case BLE_WRITE_SENSOR_CONF :
    if ( pBLEAppContext->BLEWriteSuccess ) 
    {
      status = SUCCESS ;
    }
    break ;
  case BLE_WRITE_SENSOR_PERI :
    if ( pBLEAppContext->BLEWriteSuccess ) 
    {
      status = SUCCESS ;
    }
    break ;
  default :
    break ;
  }
  
  return status ;
}

/*********************************************************************
 * @fn      BLECentralAppProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void BLECentralAppProcessGATTMsg( gattMsgEvent_t *pMsg )
{
 
  if ( pBLEAppContext->BLEState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    return;
  }
 
  if ( pMsg->method == ATT_HANDLE_VALUE_NOTI ) 
  {
    attHandleValueNoti_t notiVal ;
    uint16 index = 0 ;
    uint8  len = 0 ;
    
    osal_memcpy( & notiVal , &( pMsg->msg.handleValueNoti ), sizeof( attHandleValueNoti_t ) );
    if ( notiVal.handle == pBLEAppContext->BLESensorDataHdl ) 
    {
      len = notiVal.len ;
      index  = BUILD_UINT16 ( notiVal.value [ len - 1 ] ,  notiVal.value [ len - 2 ] ) ;
      LCD_WRITE_STRING_VALUE( "Sensor Notification :", index , 10, HAL_LCD_LINE_1 );
      dbprintf ( "s:%hu\r\n" , index ) ;
    }
  }
  else if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode ;
      LCD_WRITE_STRING_VALUE( "Read Error", status, 10, HAL_LCD_LINE_1 );
      dbprintf ( "Read Error %d\r\n" , status ) ;
    }
    else
    {
      // After a successful read, display the read value
      uint8 valueRead = pMsg->msg.readRsp.value[0] ;
      LCD_WRITE_STRING_VALUE( "Read rsp:", valueRead, 10, HAL_LCD_LINE_1 );
    }
    
    pBLEAppContext->BLEProcedureInProgress = FALSE;
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP  )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
      LCD_WRITE_STRING_VALUE( "Write Error", status, 10, HAL_LCD_LINE_1 );
      dbprintf ( "Write Error %d\r\n" , status ) ;
      pBLEAppContext->BLEWriteSuccess = FALSE ;
    }
    else
    {
      LCD_WRITE_STRING( "Write Success:" , HAL_LCD_LINE_1 );
      dbprintf ( "Write Success!\r\n" ) ;
      pBLEAppContext->BLEWriteSuccess = TRUE ;
    }
    osal_set_event ( BLETaskId , APP_WRITECHAR_RSP_EVT ) ;
    pBLEAppContext->BLEProcedureInProgress = FALSE;    

  }
  else if ( pBLEAppContext->BLEDiscState != BLE_DISC_STATE_IDLE )
  {
    BLEGATTDiscoveryEvent( pMsg );
  }
  
}

/*********************************************************************
 * @fn      BLECentralAppContextInit
 *
 * @brief   Call Once !!
 *
 * @return  none
 */
void BLECentralAppContextInit ( void )
{
  pBLEAppContext->pkgIndex = 1 ;
  
  pBLEAppContext->BLEScanRes = 0 ;
  pBLEAppContext->BLEScanIdx = 0 ;

// Scanning state
  pBLEAppContext->BLEScanning = FALSE ;

// RSSI polling state
  pBLEAppContext->BLERssi = FALSE;

// Connection handle of current connection 
  pBLEAppContext->BLEConnHandle = GAP_CONNHANDLE_INIT ;

// Application state
  pBLEAppContext->BLEState = BLE_STATE_IDLE ;

// Discovery state
  pBLEAppContext->BLEDiscState = BLE_DISC_STATE_IDLE ;

// Discovered service start and end handle
  pBLEAppContext->BLESvcStartHdl = 0 ;
  pBLEAppContext->BLESvcEndHdl = 0 ;

// Discovered characteristic handle
  
  pBLEAppContext->BLEBondPeerDevice = FALSE ;
  
  pBLEAppContext->BLESensorDataHdl = 0 ;
  pBLEAppContext->BLESensorConfHdl = 0 ;
  pBLEAppContext->BLESensorPeriHdl = 0 ;
  
  pBLEAppContext->pBLESensorHdl = 0 ;
  
  
  pBLEAppContext->BLEWriteAction = BLE_WRITE_STANDBY ;
 
// GATT read/write procedure state
  pBLEAppContext->BLEProcedureInProgress = FALSE ;
  
}

bStatus_t sensorDataNotify ( uint16 connHandle, uint16 attrHandle , uint8 set )
{
  attWriteReq_t req ;
  bStatus_t status = FAILURE ;
  uint8 notificationsOn[] = { set , 0x00 };

  req.handle = attrHandle ;

  req.len = 2;
  osal_memcpy( req.value, notificationsOn, 2 );

  req.sig = 0;
  req.cmd = 0;

  if ( pBLEAppContext->BLEProcedureInProgress == FALSE ) 
  {
    LCD_WRITE_STRING ( "sensor Notify" , HAL_LCD_LINE_4 ) ;
    dbprintf ( "sensor Notify\r\n" ) ;
    status = GATT_WriteCharValue( connHandle, &req, BLETaskId );
    
    if ( status == SUCCESS )
    {
      pBLEAppContext->BLEProcedureInProgress = TRUE ;
      pBLEAppContext->BLEWriteAction = BLE_WRITE_SENSOR_DATA ;
    }
  }
  
  return status ;
}

bStatus_t sensorConfSet( uint16 connHandle, uint16 attrHandle , uint8 set ) 
{
  attWriteReq_t req ;
  bStatus_t status = FAILURE ;
 
  req.handle = attrHandle ;

  req.len = 1 ;
  req.value[ 0 ] = set ;
  req.sig = 0;
  req.cmd = 0;

  if ( pBLEAppContext->BLEProcedureInProgress == FALSE ) 
  {
    LCD_WRITE_STRING ( "sensor Configure" , HAL_LCD_LINE_4 ) ;
    dbprintf ( "sensor Configure\r\n" ) ;
    status = GATT_WriteCharValue( connHandle, &req, BLETaskId );
    
    if ( status == SUCCESS )
    {
      pBLEAppContext->BLEProcedureInProgress = TRUE ;
      pBLEAppContext->BLEWriteAction = BLE_WRITE_SENSOR_CONF ;
    }
  }
  return status ;
}

bStatus_t sensorPeriSet( uint16 connHandle, uint16 attrHandle , uint8 value ) 
{
  attWriteReq_t req ;
  bStatus_t status = FAILURE ;
 
  req.handle = attrHandle ;

  req.len = 1 ;
  req.value[ 0 ] = value ;
  req.sig = 0;
  req.cmd = 0;

  if ( pBLEAppContext->BLEProcedureInProgress == FALSE ) 
  {
    LCD_WRITE_STRING ( "sensor Period" , HAL_LCD_LINE_4 ) ;
    dbprintf ( "sensor Period\r\n" ) ;
    status = GATT_WriteCharValue( connHandle, &req, BLETaskId );
    
    if ( status == SUCCESS )
    {
      pBLEAppContext->BLEProcedureInProgress = TRUE ;
      pBLEAppContext->BLEWriteAction = BLE_WRITE_SENSOR_PERI ;
    }
  }
  
  return status ;
}

/*********************************************************************
 * @fn      BLECentralAppRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void BLECentralAppRssiCB( uint16 connHandle, int8 rssi )
{
    LCD_WRITE_STRING_VALUE( "RSSI -dB:", (uint8) (-rssi), 10, HAL_LCD_LINE_1 );
}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void BLECentralAppEventCB( gapCentralRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        LCD_WRITE_STRING( "BLE Central", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( pEvent->initDone.devAddr ),  HAL_LCD_LINE_2 );
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
        {
          if ( BLEFindSvcUuid( SENSORTAG_SERVICE_UUID,
                                     pEvent->deviceInfo.pEvtData,
                                     pEvent->deviceInfo.dataLen ) )
          {
            BLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
          }
        }
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        pBLEAppContext->BLEScanning = FALSE;

        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          pBLEAppContext->BLEScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( pBLEAppContext->BLEDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }
        
        LCD_WRITE_STRING_VALUE( "Devices Found", pBLEAppContext->BLEScanRes,
                                10, HAL_LCD_LINE_1 );
        
        dbprintf ( "Device Found %d\r\n" , pBLEAppContext->BLEScanRes ) ;
        
        if ( pBLEAppContext->BLEScanRes > 0 )
        {
          LCD_WRITE_STRING( "<- To Select", HAL_LCD_LINE_2 );
#if !defined ( CC2540_MINIDK )
          BLECentralApp_StartConnect ( (uint8)0 ) ; // defualt select 0 
#endif
        }
#if !defined ( CC2540_MINIDK )
        else
        {
          osal_set_event ( BLETaskId , APP_START_DISCOVERY_EVT ) ;
        }
#endif

        // initialize scan index to last device
        pBLEAppContext->BLEScanIdx = pBLEAppContext->BLEScanRes;

      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if ( pEvent->gap.hdr.status == SUCCESS )
        {          
          pBLEAppContext->BLEState = BLE_STATE_CONNECTED;
          pBLEAppContext->BLEConnHandle = pEvent->linkCmpl.connectionHandle;
          pBLEAppContext->BLEProcedureInProgress = TRUE;    

          // If service discovery not performed initiate service discovery
          if ( pBLEAppContext->BLESensorDataHdl == 0 )
          {
            osal_start_timerEx( BLETaskId, BCT_START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
          }
                    
          LCD_WRITE_STRING_VALUE( "Connected H", pBLEAppContext->BLEConnHandle , 16 , HAL_LCD_LINE_6 );
          LCD_WRITE_STRING( bdAddr2Str( pEvent->linkCmpl.devAddr ), HAL_LCD_LINE_2 ); 
          dbprintf ( "connected\r\n" ) ;
        }
        else
        {
          pBLEAppContext->BLEState = BLE_STATE_IDLE;
          pBLEAppContext->BLEConnHandle = GAP_CONNHANDLE_INIT;
          pBLEAppContext->BLERssi = FALSE;
          pBLEAppContext->BLEDiscState = BLE_DISC_STATE_IDLE;
          
          LCD_WRITE_STRING( "Connect Failed", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING_VALUE( "Reason:", pEvent->gap.hdr.status, 10, HAL_LCD_LINE_2 );
          
          dbprintf ( "Connect Failed\r\n" ) ;
#if !defined( CC2540_MINIDK )
          osal_set_event ( BLETaskId , APP_START_DISCOVERY_EVT ) ;
#endif
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        pBLEAppContext->BLEState = BLE_STATE_IDLE;
        pBLEAppContext->BLEConnHandle = GAP_CONNHANDLE_INIT;
        pBLEAppContext->BLERssi = FALSE;
        pBLEAppContext->BLEDiscState = BLE_DISC_STATE_IDLE;
        
        if ( !pBLEAppContext->BLEBondPeerDevice )
        {
          pBLEAppContext->BLESensorDataHdl = 0 ;
          pBLEAppContext->BLESensorConfHdl = 0 ;
          pBLEAppContext->BLESensorPeriHdl = 0 ;
        }
        pBLEAppContext->BLEWriteAction = BLE_WRITE_STANDBY ;
        pBLEAppContext->BLEProcedureInProgress = FALSE;
          
        LCD_WRITE_STRING( "Disconnected", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING_VALUE( "Reason:", pEvent->linkTerminate.reason,
                                10, HAL_LCD_LINE_2 );
        dbprintf ( "Disconnected\r\n" ) ;
        
#if !defined( CC2540_MINIDK )
        osal_set_event ( BLETaskId , APP_START_DISCOVERY_EVT ) ;
#endif
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        LCD_WRITE_STRING( "Param Update", HAL_LCD_LINE_1 );
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void BLECentralAppPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Pairing success", HAL_LCD_LINE_1 );
    }
    else
    {
      LCD_WRITE_STRING_VALUE( "Pairing fail", status, 10, HAL_LCD_LINE_1 );
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
    }
  }
}

/*********************************************************************
 * @fn      BLECentralAppPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void BLECentralAppPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
#if (HAL_LCD == TRUE)

  uint32  passcode;
  uint8   str[7];

  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}

/*********************************************************************
 * @fn      BLECentralAppStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void BLECentralApp_StartServiceDiscovery( void )
{
  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16( SENSORTAG_SERVICE_UUID ),
                                   HI_UINT16( SENSORTAG_SERVICE_UUID ) };
  
  bStatus_t status = FAILURE ;
  
  // Initialize cached handles
  pBLEAppContext->BLESvcStartHdl = pBLEAppContext->BLESvcEndHdl = 0 ;
  
  pBLEAppContext->BLEDiscState = BLE_DISC_STATE_SVC;
  
  LCD_WRITE_STRING( "StartDiscovery...", HAL_LCD_LINE_3 ); 
  dbprintf ( "StartDiscovery...\r\n" ) ;
  
  // Discovery simple BLE service
  status = GATT_DiscPrimaryServiceByUUID( pBLEAppContext->BLEConnHandle,
                                 uuid,
                                 ATT_BT_UUID_SIZE,
                                 BLETaskId );
  
  if ( status != SUCCESS )
  {
    LCD_WRITE_STRING( "StartDiscovery fail ", HAL_LCD_LINE_3 ); 
  }
}

/*********************************************************************
 * @fn      BLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void BLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
 
  if ( pBLEAppContext->BLEDiscState == BLE_DISC_STATE_SVC )
  {
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
         pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
      pBLEAppContext->BLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
      pBLEAppContext->BLESvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      
      LCD_WRITE_STRING( "DiscoveryEvent Handle Found", HAL_LCD_LINE_3 ); 
      dbprintf ( "DiscoveryEvent Handle Found\r\n" ) ;
    }
    
    // If procedure complete
    if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
           pMsg->hdr.status == bleProcedureComplete ) ||
         ( pMsg->method == ATT_ERROR_RSP ) )
    {
      if ( pBLEAppContext->BLESvcStartHdl != 0 )
      {
        bStatus_t status ;
        // Discover characteristic
        pBLEAppContext->BLEDiscState = BLE_DISC_STATE_CHAR;
        
        LCD_WRITE_STRING( "DiscoveryEvent UUID ", HAL_LCD_LINE_3 ); 
        dbprintf ( "DiscoveryEvent UUID , start 0x%x , end 0x%x\r\n" , pBLEAppContext->BLESvcStartHdl , pBLEAppContext->BLESvcEndHdl ) ;

        // Find all in Once 
        status = GATT_DiscAllChars( pBLEAppContext->BLEConnHandle , pBLEAppContext->BLESvcStartHdl , pBLEAppContext->BLESvcEndHdl , BLETaskId ) ;
        if ( status != SUCCESS )
        {
          LCD_WRITE_STRING( "UUID Fail", HAL_LCD_LINE_4 ); 
          dbprintf ( "DiscoveryEvent UUID Fail\r\n" ) ;
        }
      }
    }
  }
  
  else if ( pBLEAppContext->BLEDiscState == BLE_DISC_STATE_CHAR )
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
         pMsg->msg.readByTypeRsp.numPairs > 0 )
    {
      uint8 i = 0 ;
      uint8 idx = 0 ;
      uint8 end = 0 ;
      uint16 uuid = 0 ;
      
      attReadByTypeRsp_t * prsp = &pMsg->msg.readByTypeRsp ;
      
      LCD_WRITE_STRING_VALUE( "ServiceFound H", * pBLEAppContext->pBLESensorHdl , 16 , HAL_LCD_LINE_3 );
      dbprintf("ServiceFound H 0x%x\r\n" , * pBLEAppContext->pBLESensorHdl ) ;
      
      pBLEAppContext->BLEProcedureInProgress = FALSE ;
      
      dbprintf ( "Characteristic pairs %d\r\n" , prsp->numPairs ) ;
      dbprintf ( "Characteristic found bytes %d\r\n" , prsp->len ) ;
      end = prsp->len * prsp->numPairs ;
      end -= 1 ;
      while ( idx < end )
      {
        for ( i = 0 ; i < prsp->len ; i ++ )
        {
          dbprintf ( "0x%x " , prsp->dataList[ i + idx ] ) ;
        }
        dbprintf ( "\r\n" ) ;
        uuid = BUILD_UINT16 ( prsp->dataList[ idx + 5 ] , prsp->dataList[ idx + 6] ) ;
        if ( uuid == GYROACCEL_DATA_UUID )
        {
          pBLEAppContext->pBLESensorHdl = & pBLEAppContext->BLESensorDataHdl ;
        }
        else if ( uuid == GYROACCEL_CONF_UUID )
        {
          pBLEAppContext->pBLESensorHdl = & pBLEAppContext->BLESensorConfHdl ;
        }
        else if ( uuid == GYROACCEL_PERI_UUID )
        {
          pBLEAppContext->pBLESensorHdl = & pBLEAppContext->BLESensorPeriHdl ;
        }
        else
        {
          pBLEAppContext->pBLESensorHdl = 0 ;
        }
        
        if ( pBLEAppContext->pBLESensorHdl )
          * ( pBLEAppContext->pBLESensorHdl ) = BUILD_UINT16 ( prsp->dataList[ idx + 3 ] , prsp->dataList[ idx + 4 ] ) ;
        idx += prsp->len ;
      }
      
    }
    else
    {
      LCD_WRITE_STRING( "No Sensor Service", HAL_LCD_LINE_3 );
      dbprintf ("No Sensor Service\r\n" ) ;
    }
    
    if ( pMsg->hdr.status == bleProcedureComplete )
    {
      pBLEAppContext->pBLESensorHdl = 0 ; // clear now 
      pBLEAppContext->BLEDiscState = BLE_DISC_STATE_IDLE ;
      osal_set_event ( BLETaskId , APP_SVCFOUND_REQ_EVT ) ; 
    }
    
  }    
}


/*********************************************************************
 * @fn      BLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool BLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData ;
      
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      BLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void BLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( pBLEAppContext->BLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < pBLEAppContext->BLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, pBLEAppContext->BLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( pBLEAppContext->BLEDevList[pBLEAppContext->BLEScanRes].addr, pAddr, B_ADDR_LEN );
    pBLEAppContext->BLEDevList[pBLEAppContext->BLEScanRes].addrType = addrType;
    
    // Increment scan result count
    pBLEAppContext->BLEScanRes++;
  }
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

/*********************************************************************
*********************************************************************/
