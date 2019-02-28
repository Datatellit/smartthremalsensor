#include "_global.h"
#include "delay.h"
#include "rf24l01.h"
#include "timer4.h"
#include "MyMessage.h"
#include "ProtocolParser.h"
#include "sen_als.h"
#include "ADC1Dev.h"
#include "stm8l15x_rtc.h"
#include "UsartDev.h"
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


#define MAX_RF_FAILED_TIME              10      // Reset RF module when reach max failed times of sending



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
//////////////////pir collect//////////////////
uint8_t pir_value = 0;
bool pir_ready = FALSE;
uint16_t pir_tick = 0;
#define SEND_MAX_INTERVAL_PIR                    600    // about 6s (600 * 10ms)
//////////////////pir collect//////////////////

uint8_t _uniqueID[UNIQUE_ID_LEN];
uint8_t m_cntRFSendFailed = 0;


uint8_t mutex;
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

// Save config to Flash
void SaveBackupConfig()
{
  if( gNeedSaveBackup ) {
    // Overwrite entire config bakup FLASH
    if(Flash_WriteDataBlock(BACKUP_CONFIG_BLOCK_NUM, (uint8_t *)&gConfig, sizeof(gConfig)))
    {
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
    if(Flash_WriteDataBlock(STATUS_DATA_NUM, pData, nLen))
    {
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
    if(Flash_WriteDataBlock(0, (uint8_t *)&gConfig, sizeof(gConfig)))
    {
      gIsStatusChanged = FALSE;
      gIsConfigChanged = FALSE;
      gNeedSaveBackup = TRUE;
      return;
    }
  } 
}

bool IsConfigInvalid() {
  return( gConfig.version > XLA_VERSION || gConfig.version < XLA_MIN_VER_REQUIREMENT 
       || /*!IS_VALID_REMOTE(gConfig.type)  || */gConfig.nodeID == 0
       || gConfig.rfPowerLevel > RF24_PA_MAX || gConfig.rfChannel > 127 || gConfig.rfDataRate > RF24_250KBPS );
}

bool isNodeIdInvalid(uint8_t nodeid)
{
  return( !IS_SENSOR_NODEID(nodeid)  );
}

/*// Load config from Flash
void LoadConfig()
{
    // Load the most recent settings from FLASH
    Flash_ReadBuf(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
    if( IsConfigInvalid() ) {
      Flash_ReadBuf(BACKUP_CONFIG_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
      if( IsConfigInvalid() ) {
        memset(&gConfig, 0x00, sizeof(gConfig));
        gConfig.version = XLA_VERSION;
        gConfig.indDevice = 0;
        gConfig.present = 0;
        gConfig.inPresentation = 0;
        gConfig.enSDTM = 0;
        gConfig.rptTimes = 1;
        gConfig.nodeID = 130;
        gConfig.rfChannel = RF24_CHANNEL;
        gConfig.rfPowerLevel = RF24_PA_MAX;
        gConfig.rfDataRate = RF24_250KBPS;      
        memcpy(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
      }
      gIsConfigChanged = TRUE;
    }
    else {
      uint8_t bytVersion;
      Flash_ReadBuf(BACKUP_CONFIG_ADDRESS, (uint8_t *)&bytVersion, sizeof(bytVersion));
      if( bytVersion != gConfig.version ) gNeedSaveBackup = TRUE;
    }
    // Load the most recent status from FLASH
    uint8_t pData[50] = {0};
    uint16_t nLen = (uint16_t)(&(gConfig.nodeID)) - (uint16_t)(&gConfig);
    Flash_ReadBuf(STATUS_DATA_ADDRESS, pData, nLen);
    if(pData[0] >= XLA_MIN_VER_REQUIREMENT && pData[0] <= XLA_VERSION)
    {
      memcpy(&gConfig,pData,nLen);
    }
    gConfig.rfChannel = 87;
}*/

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
  if(gResetRF)
  {
    RF24L01_init();
    NRF2401_EnableIRQ();
    UpdateNodeAddress(NODEID_GATEWAY);
    gResetRF=FALSE;
  }
}


// Send message and switch back to receive mode
bool SendMyMessage() {
  if( bMsgReady ) {
    
    // Change tx destination if necessary
    NeedUpdateRFAddress(sndMsg.header.destination);
    
    uint8_t lv_tried = 0;
    uint16_t delay;
    while (lv_tried++ <= gConfig.rptTimes ) {
      feed_wwdg();
      mutex = 0;
      RF24L01_set_mode_TX();
      RF24L01_write_payload(psndMsg, PLOAD_WIDTH);
      WaitMutex(0x1FFFF);
      if (mutex == 1) {
        m_cntRFSendFailed = 0;
        break; // sent sccessfully
      } else if( m_cntRFSendFailed++ > MAX_RF_FAILED_TIME ) {
        // Reset RF module
        m_cntRFSendFailed = 0;
        //WWDG->CR = 0x80;
        // RF24 Chip in low power
        RF24L01_DeInit();
        delay = 0x1FFF;
        while(delay--)feed_wwdg();
        RF24L01_init();
        NRF2401_EnableIRQ();
        UpdateNodeAddress(NODEID_GATEWAY);
        continue;
      }
      
      //The transmission failed, Notes: mutex == 2 doesn't mean failed
      //It happens when rx address defers from tx address
      //asm("nop"); //Place a breakpoint here to see memory
      // Repeat the message if necessary
      uint16_t delay = 0xFFF;
      while(delay--)feed_wwdg();
    }
    
    // Switch back to receive mode
    bMsgReady = 0;
    RF24L01_set_mode_RX();
  }

  return(mutex > 0);
}

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
  // Go on only if NRF chip is presented
  RF24L01_init();
  while(!NRF24L01_Check());
  
  // Load config from Flash
  FLASH_DeInit();
  Read_UniqueID(_uniqueID, UNIQUE_ID_LEN);
  LoadConfig();
  // NRF_IRQ
  NRF2401_EnableIRQ();

  // Init Watchdog
  wwdg_init();
  
  TIM4_10ms_handler = tmrProcess;

  // Send Presentation Message
  Msg_Presentation();
  SendMyMessage();
  uint8_t pre_pir_value = 255;
  pir_value = (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4) == SET)?1:0;
  pir_ready = TRUE;
  while (1) {    
    // Feed the Watchdog
    feed_wwdg();   

    if( pir_ready  || pir_tick > SEND_MAX_INTERVAL_PIR ) {
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
  if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4) == SET)
  {
    pir_value = 1;
  }
  else
  {
    pir_value = 0;
  }
  EXTI_ClearITPendingBit(EXTI_IT_Pin4);    
}