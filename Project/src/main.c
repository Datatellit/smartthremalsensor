#include "_global.h"
#include "delay.h"
#include "led.h"
#include "MyMessage.h"
#include "ProtocolParser.h"
#include "rf24l01.h"
#include "stm8l15x_rtc.h"
#include "timer4.h"
#ifndef RF24
#include "UsartDev.h"
#endif
#include "XlightComBus.h"

/*
Xlight Remoter Program
License: MIT

Auther: Baoshi Sun
Email: bs.sun@datatellit.com, bs.sun@uwaterloo.ca
Github: https://github.com/sunbaoshi1975
Please visit xlight.ca for product details

RF24L01 connector pinout:
GND    VCC
CE     CSN
SCK    MOSI
MISO   IRQ

Connections:
  PB4 -> CE
  PC6 -> CSN (11 buttons)
  PB5 -> SCK
  PB6 -> MOSI
  PB7 -> MISO
  PD5 -> IRQ

*/

/* Private define ------------------------------------------------------------*/
// Keep alive message interval, around 6 seconds
#define MAX_RF_FAILED_TIME              20      // Reset RF module when reach max failed times of sending
#define MAX_RF_RESET_TIME               3       // Reset Node when reach max times of RF module consecutive reset

#define SEND_MAX_INTERVAL_PIR           600     // about 6s (600 * 10ms)

/* Public variables ---------------------------------------------------------*/
// Public variables
Config_t gConfig;

MyMessage_t sndMsg;
MyMessage_t rcvMsg;
uint8_t *psndMsg = (uint8_t *)&sndMsg;
uint8_t *prcvMsg = (uint8_t *)&rcvMsg;
bool gNeedSaveBackup = FALSE;
bool gIsStatusChanged = FALSE;
bool gIsConfigChanged = FALSE;
bool gResetRF = FALSE;
bool gResetNode = FALSE;
bool gResendPresentation = FALSE;
uint8_t gSendDelayTick = 0;

/* Private variables ---------------------------------------------------------*/
uint8_t mSysStatus = SYS_ST_INIT;

//////////////////pir collect//////////////////
uint8_t pir_check_data = 0;
uint8_t pir_value = 0;
bool pir_ready = FALSE;
uint16_t pir_tick = 0;
//////////////////pir collect//////////////////

uint8_t _uniqueID[UNIQUE_ID_LEN];

// Keep Alive Timer
uint16_t mTimerKeepAlive = 0;
uint8_t m_cntRFSendFailed = 0;
uint8_t m_cntRFReset = 0;

uint16_t pirofftimeout = 0;
uint8_t mutex;

/* Private function prototypes -----------------------------------------------*/
void tmrProcess();

static void clock_init(void)
{
  CLK_DeInit();
  CLK_HSICmd(ENABLE);
  CLK_SYSCLKDivConfig(SYS_CLOCK_DIVIDER);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
  //CLK_ClockSecuritySystemEnable();
}

/*// Save config to Flash
void SaveConfig()
{
#ifndef ENABLE_SDTM
  if( gIsConfigChanged ) {
    Flash_WriteBuf(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
    gIsConfigChanged = FALSE;
  }
#endif  
}*/

void SetSysState(const uint8_t _st)
{
    uint8_t lv_st = _st;
    if( mSysStatus != lv_st ) {
        mSysStatus = lv_st;
        // Change LED Status Indicator accordingly
        //if power_low, led_red_on();
        // Notify the Gateway
        Msg_DevState(mSysStatus);
    }
}

uint8_t GetSysState()
{
    return mSysStatus;
}

// Save config to Flash
void SaveBackupConfig()
{
  if( gNeedSaveBackup ) {
    // Overwrite entire config bakup FLASH
    if(Flash_WriteDataBlock(BACKUP_CONFIG_BLOCK_NUM, (uint8_t *)&gConfig, sizeof(gConfig))) {
      gNeedSaveBackup = FALSE;
    }
  }
}

// Save status to Flash
void SaveStatusData()
{
    // Skip the first byte (version)
    uint8_t pData[50] = {0};
    uint16_t nLen = (uint16_t)(&(gConfig.nodeID)) - (uint16_t)(&gConfig);
    memcpy(pData, (uint8_t *)&gConfig, nLen);
    if(Flash_WriteDataBlock(STATUS_DATA_NUM, pData, nLen)) {
      gIsStatusChanged = FALSE;
    }
}

// Save config to Flash
void SaveConfig()
{
  if( gIsStatusChanged ) {
    // Overwrite only Static & status parameters (the first part of config FLASH)
    SaveStatusData();
    gIsConfigChanged = TRUE;
  } 
  if( gIsConfigChanged ) {
    // Overwrite entire config FLASH
    if(Flash_WriteDataBlock(0, (uint8_t *)&gConfig, sizeof(gConfig))) {
      gIsStatusChanged = FALSE;
      gIsConfigChanged = FALSE;
      gNeedSaveBackup = TRUE;
      return;
    }
  } 
}

bool IsConfigInvalid() {
  return( gConfig.version > XLA_VERSION || gConfig.version < XLA_MIN_VER_REQUIREMENT 
       || (gConfig.type != SEN_TYP_PIR && gConfig.type != SEN_TYP_ZENSOR) 
       || gConfig.nodeID == 0
       || gConfig.rfPowerLevel > RF24_PA_MAX || gConfig.rfChannel > 127 || gConfig.rfDataRate > RF24_250KBPS );
}

bool isNodeIdInvalid(uint8_t nodeid)
{
  return( !IS_SENSOR_NODEID(nodeid) );
}

void UpdateNodeAddress(uint8_t _tx) {
  memcpy(rx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  rx_addr[0] = gConfig.nodeID;
  memcpy(tx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  
  if( _tx == NODEID_RF_SCANNER ) {
    tx_addr[0] = NODEID_RF_SCANNER;
  } else {  
    tx_addr[0] = NODEID_GATEWAY;
  }
  RF24L01_setup(gConfig.rfChannel, gConfig.rfDataRate, gConfig.rfPowerLevel, BROADCAST_ADDRESS);     // With openning the boardcast pipe
}  

bool NeedUpdateRFAddress(uint8_t _dest) {
  bool rc = FALSE;
  if( sndMsg.header.destination == NODEID_RF_SCANNER && tx_addr[0] != NODEID_RF_SCANNER ) {
    UpdateNodeAddress(NODEID_RF_SCANNER);
    rc = TRUE;
  } else if( sndMsg.header.destination != NODEID_RF_SCANNER && tx_addr[0] != NODEID_GATEWAY ) {
    UpdateNodeAddress(NODEID_GATEWAY);
    rc = TRUE;
  }
  UpdateNodeAddress(NODEID_GATEWAY);
  return rc;
}


bool WaitMutex(uint32_t _timeout) {
  while(_timeout--) {
    if( mutex > 0 ) return TRUE;
    feed_wwdg();
  }
  return FALSE;
}

// reset rf
void ResetRFModule()
{
  if(gResetRF) {
    RF24L01_init();
    NRF2401_EnableIRQ();
    UpdateNodeAddress(NODEID_GATEWAY);
    gResetRF=FALSE;
    RF24L01_set_mode_RX();
  }
  if( gResendPresentation ) {
    // Send Presentation to confirm new settings are working
    Msg_Presentation();
    gResendPresentation = FALSE;
  }
}

uint16_t GetDelayTick(const uint8_t ds)
{
  uint16_t lv_delaytick = 0;
  if(ds == BROADCAST_ADDRESS) {
    // base = 210ms, delta = 100ms
    lv_delaytick = ((gConfig.nodeID - NODEID_MIN_SUPERSENSOR + 1) % 32 ) * 10 + 11;
  } else {
    lv_delaytick = 4;   // 40ms
  }
  return lv_delaytick;
}

// Send message and switch back to receive mode
bool SendMyMessage() {
#ifdef RF24  
  if( bMsgReady && delaySendTick == 0 ) {
    // Change tx destination if necessary
    NeedUpdateRFAddress(sndMsg.header.destination);
      
    uint8_t lv_tried = 0;
    uint16_t delay;
    while (lv_tried++ <= gConfig.rptTimes ) {
      
      mutex = 0;
      RF24L01_set_mode_TX();
      RF24L01_write_payload(psndMsg, PLOAD_WIDTH);
      WaitMutex(0x1FFFF);
      if (mutex == 1) {
        m_cntRFSendFailed = 0;
        m_cntRFReset = 0;
        break; // sent sccessfully
      } else {
        m_cntRFSendFailed++;
        if( m_cntRFSendFailed >= MAX_RF_FAILED_TIME ) {
          m_cntRFSendFailed = 0;
          m_cntRFReset++;
          if( m_cntRFReset >= MAX_RF_RESET_TIME ) {
            // Cold Reset
            WWDG->CR = 0x80;
            m_cntRFReset = 0;
            break;
          } else if( m_cntRFReset >= 2 ) {
            // Reset whole node
            mSysStatus = SYS_ST_RESET;
            break;
          }

          // Reset RF module
          //RF24L01_DeInit();
          delay = 0x1FFF;
          while(delay--)feed_wwdg();
          RF24L01_init();
          NRF2401_EnableIRQ();
          UpdateNodeAddress(NODEID_GATEWAY);
          continue;
        }
      }
      
      //The transmission failed, Notes: mutex == 2 doesn't mean failed
      //It happens when rx address defers from tx address
      //asm("nop"); //Place a breakpoint here to see memory
      // Repeat the message if necessary
      delay = 0xFFF;
      while(delay--)feed_wwdg();
    }
    
    // Switch back to receive mode
    bMsgReady = 0;
    RF24L01_set_mode_RX();
    
    // Reset Keep Alive Timer
    mTimerKeepAlive = 0;
  }
  return(mutex > 0);
#else
  return FALSE;
#endif  
}

// Init data and other GPIOs
void dataio_init()
{
    GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_In_PU_IT);
    EXTI_DeInit();
    EXTI_SetPinSensitivity(EXTI_Pin_4, EXTI_Trigger_Rising_Falling);
}

int main( void ) {

  // Init clock, timer and button
  clock_init();
  timer_init();
  dataio_init();
  
  // Init R&G-LED Indicator
  drv_led_init(LED_PIN_INIT_HIGH);
    
  // System enter setup state
  SetSysState(SYS_ST_SETUP);
  
#ifdef RF24
  // Go on only if NRF chip is presented
  RF24L01_init();
  while(!NRF24L01_Check());
#endif
  
  // Load config from Flash
  FLASH_DeInit();
  Read_UniqueID(_uniqueID, UNIQUE_ID_LEN);
  LoadConfig();  
  ////// not common config check/////////////
  if(gConfig.timeout == 0 || gConfig.timeout >= 60000) { // invalid timeout
    gConfig.timeout = 5;
  }  
  ////// not common config check/////////////
  
#ifdef RF24  
  // NRF_IRQ
  NRF2401_EnableIRQ();
#endif
  
  // Init Watchdog
  wwdg_init();
  
  TIM4_10ms_handler = tmrProcess;

  // Send Presentation Message
  UpdateNodeAddress(NODEID_GATEWAY);
  Msg_Presentation();
  SendMyMessage();
  
  // System enter running state
  SetSysState(SYS_ST_RUNNING);
  
  uint8_t pre_pir_value = 255;
  pir_value = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == RESET) ? 0 : 1;
  pir_ready = TRUE;
  while (1) {    
    // Feed the Watchdog
    feed_wwdg();   

    if( pre_pir_value != pir_value  || pir_tick > SEND_MAX_INTERVAL_PIR ) {
        // Reset send timer
        pir_tick = 0;
        pre_pir_value = pir_value;
        Msg_SendPIR(pre_pir_value);
        pir_ready = FALSE;
    }
    SendMyMessage();
    // Save Config if Changed
    SaveConfig();
    // Save config into backup area
    SaveBackupConfig();
  }
}

// Execute timer operations
void tmrProcess() {
  pir_tick++;
  if(pirofftimeout > 0) {
    pirofftimeout--;
  }
  if(pirofftimeout == 0 && pir_check_data == 0) {
    pir_value = 0;
    led_green_off();
  } 
}

void RF24L01_IRQ_Handler() {
  tmrIdleDuration = 0;
  if(RF24L01_is_data_available()) {
    //Packet was received
    RF24L01_clear_interrupts();
    RF24L01_read_payload(prcvMsg, PLOAD_WIDTH);
    bMsgReady = ParseProtocol();
    return;
  }
 
  uint8_t sent_info;
  if (sent_info = RF24L01_was_data_sent()) {
    //Packet was sent or max retries reached
    RF24L01_clear_interrupts();
    mutex = sent_info; 
    return;
  }

   RF24L01_clear_interrupts();
}

/**
  * @brief External IT PIN4 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI4_IRQHandler, 12)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  pir_ready = TRUE;
  if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4) == RESET) {
    //pir_value = 0;
    pir_check_data = 0;
    pirofftimeout = gConfig.timeout*100;
  } else {
    pir_check_data = 1;
    pir_value = 1;
    led_green_on();
  }
  EXTI_ClearITPendingBit(EXTI_IT_Pin4);    
}