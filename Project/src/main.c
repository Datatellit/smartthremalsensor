#include "_global.h"
#include "ADC1Dev.h"
#include "debugDefine.h"
#include "delay.h"
#include "led.h"
#include "MyMessage.h"
#include "ProtocolParser.h"
#include "rf24l01.h"
#include "timer4.h"
#include "UsartDev.h"
#include "wwdg.h"

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
// Use this to enable or disable Low Power Mode
//#define ENABLE_LOW_POWER_MODE

// Keep alive message interval, around 6 seconds
#define MAX_RF_FAILED_TIME              3       // Reset RF module when reach max failed times of sending
#define MAX_RF_RESET_TIME               3       // Reset Node when reach max times of RF module consecutive reset

#define SEND_MAX_INTERVAL_PIR           10      // about 10s

#define POWER_CHECK_INTERVAL            40      // about 400ms (40 * 10ms)
#define DC_FULLPOWER                    460
#define DC_LOWPOWER                     368

// Idle duration before enter low power mode
#define TIMEOUT_IDLE                    300     // The unit is 10 ms, so the duration is 3 s.

#define MAINLOOP_TIMEOUT                6000    // 60s for mainloop timeout

// Detect enable / disable
#define SET_SENSOR_ON                   GPIO_WriteBit(GPIOB, GPIO_Pin_0, SET)
#define SET_SENSOR_OFF                  GPIO_WriteBit(GPIOB, GPIO_Pin_0, RESET)

/* Public variables ---------------------------------------------------------*/
Config_t gConfig;

MyMessage_t sndMsg;
MyMessage_t rcvMsg;
uint8_t *psndMsg = (uint8_t *)&sndMsg;
uint8_t *prcvMsg = (uint8_t *)&rcvMsg;

/* Private variables ---------------------------------------------------------*/
// main-loop-dead-lock-check timer
#ifdef DEBUG_NO_WWDG
uint16_t m_MainloopTimeTick = 0;
#endif

uint8_t mSysStatus = SYS_ST_INIT;

//////////////////pir collect//////////////////
uint8_t pir_value = 0;
uint16_t pir_tick = 0;
//////////////////pir collect//////////////////

// Keep Alive Timer
uint16_t mTimerKeepAlive = 0;
uint8_t m_cntRFSendFailed = 0;
uint8_t m_cntRFReset = 0;
uint8_t m_nPowerCheckTick = POWER_CHECK_INTERVAL;

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

void SetSysState(const uint8_t _st)
{
    if( mSysStatus != _st ) {
        mSysStatus = _st;
        // Notify the Gateway
        //Msg_DevState(mSysStatus, 0);
    }
}

uint8_t GetSysState()
{
    return mSysStatus;
}

/* Low Power Mode Code -------------------------------------------------------*/
#ifdef ENABLE_LOW_POWER_MODE

/**
  * @brief  configure GPIOs before entering low power
	* @caller lowpower_config
  * @param None
  * @retval None
  */  
void GPIO_LowPower_Config(void)
{
    // Note: don't change PIR pin and green LED pin anyway
    GPIO_Init(GPIOA, GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOB, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOC, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOD, GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOE, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOF, GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
}

// Enter Low Power Mode, which can be woken up by external interupts
void lowpower_config(void) {
  // Enter Sleeping Mode
  // Don't use SetSysState(SYS_ST_SLEEP), because we don't want to send RF message
  mSysStatus = SYS_ST_SLEEP;
  
  // Set STM8 in low power
  PWR->CSR2 = 0x2;
  
  // Stop Timers
  //TIM1_DeInit();
  //TIM2_DeInit();
  //TIM3_DeInit();
  TIM4_DeInit();
  
  // Set GPIO in low power
  GPIO_LowPower_Config();
  
  // RF24 Chip in low power
  RF24L01_DeInit();
  
  // TODO how process???
  /*while ((CLK->ICKCR & 0x04) != 0x00) {
    feed_wwdg();
  }*/
  
  ADC_DeInit(ADC1);
  
  // Stop peripheral clocks
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM5, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_DAC, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_RTC, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_LCD, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_AES, DISABLE);
}

// Resume Normal Mode
void wakeup_config(void) {
  // System Woke Up
  mSysStatus = SYS_ST_ON_BATTERY;

  clock_init();
  timer_init();
  //dataio_init();
  ADC_Config();

  // Init R&G-LED Indicator
  //drv_led_init(LED_PIN_INIT_HIGH);
  led_red_off();    // Only turn off red, then it will be changed in timer handler according to battery state
  
  RF24L01_init();
  NRF2401_EnableIRQ();
  UpdateNodeAddress(NODEID_GATEWAY);
  
#ifdef DEBUG_LOG
  usart_config(9600);
#endif
}

bool WakeupMeFromSleep()
{
    if( mSysStatus == SYS_ST_SLEEP ) { 
        tmrIdleDuration = 0;
        // Wakeup
        wakeup_config();
        return TRUE;
    }
    return FALSE;
}

#endif // Low Power Mode Code

bool IsConfigInvalid() {
  return( gConfig.version > XLA_VERSION || gConfig.version < XLA_MIN_VER_REQUIREMENT 
       || (gConfig.type != SEN_TYP_PIR && gConfig.type != SEN_TYP_ZENSOR) 
       || gConfig.nodeID == 0
       || gConfig.rfPowerLevel > RF24_PA_MAX || gConfig.rfChannel > 127 || gConfig.rfDataRate > RF24_250KBPS );
}

bool isNodeIdInvalid(const uint8_t nodeid)
{
  return( !(IS_SENSOR_NODEID(nodeid) || IS_GROUP_NODEID(nodeid)) );
}

bool WaitMutex(uint32_t _timeout) {
  while(_timeout--) {
    if( mutex > 0 ) return TRUE;
    feed_wwdg();
  }
  return FALSE;
}

void RestartCheck()
{
#ifdef DEBUG_NO_WWDG
  if( m_MainloopTimeTick < MAINLOOP_TIMEOUT ) m_MainloopTimeTick++;
  if( m_MainloopTimeTick >= MAINLOOP_TIMEOUT ) {
    //printlog("need restart!");
    WWDG->CR = 0x80;
  }
#endif
}

// reset rf
void ResetRFModule()
{
  if(gResetNode) {
#ifdef ENABLE_LOW_POWER_MODE    
    // Wake Me up
    WakeupMeFromSleep();
#endif
    mSysStatus = SYS_ST_RESET;
    gResetNode = FALSE;
    // Execute Cold Reset if there is no 'Setup Loop' in the main()
    //WWDG->CR = 0x80;
    return;
  }
  if(gResetRF) {
    RF24L01_init();
    NRF2401_EnableIRQ();
    UpdateNodeAddress(NODEID_GATEWAY);
    gResetRF = FALSE;
    RF24L01_set_mode_RX();
  }
  if( gResendPresentation ) {
    // Send Presentation to confirm new settings are working
    Msg_Presentation(1);   // Require-Ack
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
// Notes: never call this function in timer or other IRQ fucntions
bool SendMyMessage() {
#ifdef RF24
  if( bMsgReady && delaySendTick == 0 ) {

    // 暂时关闭检测
    SET_SENSOR_OFF;
    
    // Change tx destination if necessary
    NeedUpdateRFAddress(sndMsg.header.destination);
      
    uint8_t lv_tried = 0;
    while (lv_tried++ <= gConfig.rptTimes ) {
      
      mutex = 0;
      RF24L01_set_mode_TX();
      RF24L01_write_payload(psndMsg, PLOAD_WIDTH);
      if( !WaitMutex(0xFFFF) ) { // FFFF = 25ms, 1A000 = 40ms, 1FFFF = 50ms
        // Timeout: no IRQ
        mutex = (RF24L01_get_whatHappened() & RF_RESULT_SENT);
      }
      // Switch back to receive mode as early as possible
      RF24L01_set_mode_RX();
      if( mutex == 1 ) {
        m_cntRFSendFailed = 0;
        m_cntRFReset = 0;
        // Reset Keep Alive Timer
        mTimerKeepAlive = 0;
        break; // sent sccessfully
      } else {
        if( ++m_cntRFSendFailed >= MAX_RF_FAILED_TIME ) {
          m_cntRFSendFailed = 0;
          if( ++m_cntRFReset > MAX_RF_RESET_TIME ) {
            m_cntRFReset = 0;
            mTimerKeepAlive = 0;
            // Reset whole node
            gResetNode = TRUE;
            break;
          }
          // Reset RF module
          gResetRF = TRUE;
          ResetRFModule();
        }
      }
      
      //The transmission failed, Notes: mutex == 2 doesn't mean failed
      //It happens when rx address defers from tx address
      //asm("nop"); //Place a breakpoint here to see memory
      // Delay for a while and retry sending
      delay_ms(10);
    }
    bMsgReady = 0;

    // 启用检测
    SET_SENSOR_ON;
    
  }
  return(mutex > 0);
#else
  return FALSE;
#endif  
}

// Init data and other GPIOs
void dataio_init()
{
    // 检测使能口：高电平有效，才能进行人感检测
    GPIO_Init(GPIOB, GPIO_Pin_0, GPIO_Mode_Out_PP_High_Slow);

    // 人感信号检测口
    GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_In_PU_IT);
    EXTI_DeInit();
    // 检测上升沿+下降沿才有效
    EXTI_SetPinSensitivity(EXTI_Pin_4, EXTI_Trigger_Rising_Falling);    

    // 暂时关闭检测
    SET_SENSOR_OFF;
}

uint16_t m_eqv;
void Check_eq()
{
  uint16_t eq1, eq2;
  eq_checkData(&eq1, &eq2);
  printnum(eq1);
  printlog("-");
  printnum(eq2);
  if(eq1 > eq2 + 250) {
    // eqv1-eqv2 > 0.2v (eqv1 = eq1*3.3/4096)
    // 电池进电: 红灯亮、绿灯灭
    printlog("charge...");
    led_red_on();
    //led_green_off();
    SetSysState(SYS_ST_CHARGING);
  } else { // 电池不进电（插充电器但电池已充满或者没插充电器）
    printlog("normal...");
    m_eqv = (uint32_t)eq2 * 330 / 2048;
    printnum(m_eqv);
    if(m_eqv >= DC_FULLPOWER) {
      // 插充电器，电池已充满：红灯灭、绿灯亮
      printlog("full...");
      led_red_off();
      //led_green_on();
      SetSysState(SYS_ST_RUNNING);
    } else {
      // 没插充电器（电池供电）：红灯闪或灭、绿灯灭
      printlog("not full...");
      //led_green_off();      
      if(m_eqv < DC_LOWPOWER) {
        // 电池电压低：红灯闪
        SetSysState(SYS_ST_LOW_BATTERY);
        led_red_flashing();
      } else {
        // 电池电压正常：红灯灭
        SetSysState(SYS_ST_ON_BATTERY);
        led_red_off();
      }
    }
  }
}

int main( void ) {

  // Init clock, timer and button
  clock_init();
  timer_init();
  dataio_init();
  
  // Init R&G-LED Indicator
  drv_led_init(LED_PIN_INIT_HIGH);

  // Load config from Flash
  FLASH_DeInit();
  Read_UniqueID(_uniqueID, UNIQUE_ID_LEN);
  LoadConfig();  
  ////// not common config check/////////////
  if(gConfig.timeout == 0 || gConfig.timeout > 60000) { // invalid timeout
    gConfig.timeout = 5;
    gIsConfigChanged = TRUE;
  }
  if( (gConfig.keepalive > 0 && gConfig.keepalive < 5) || gConfig.keepalive > 600) {
    gConfig.keepalive = SEND_MAX_INTERVAL_PIR;
    gIsConfigChanged = TRUE;
  }
  ////// not common config check/////////////

#ifdef DEBUG_LOG
  usart_config(9600);
#endif
  
  // Init Watchdog
  wwdg_init();
  
  TIM4_10ms_handler = tmrProcess;

  // Init ADC
  ADC_Config();

  enableInterrupts();
  
  // System enter setup state
  SetSysState(SYS_ST_SETUP);
  printlog("start...");
  uint8_t pre_pir_value = 255;
  pir_value = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == RESET) ? 0 : 1;
  
  while(1) { // Setup Loop
    
    // 暂时关闭检测
    SET_SENSOR_OFF;
    
#ifdef RF24
    // Go on only if NRF chip is presented
    RF24L01_init();
    u16 timeoutRFcheck = 0;
    while(!NRF24L01_Check()) {
      if( timeoutRFcheck > 50 ) {
        WWDG->CR = 0x80;
        break;
      }
      // Feed the Watchdog
      feed_wwdg();
      // Reset main-loop-dead-lock-check timer
#ifdef DEBUG_NO_WWDG
      m_MainloopTimeTick = 0;
#endif
      timeoutRFcheck++;
    }
    // NRF_IRQ
    NRF2401_EnableIRQ();
    UpdateNodeAddress(NODEID_GATEWAY);
#endif
  
    // Send Presentation Message
    Msg_Presentation(mSysStatus == SYS_ST_SETUP);
    SendMyMessage();
    gIsStatusChanged = TRUE;

    // System enter running state
    SetSysState(SYS_ST_RUNNING);
    
    // 启用检测
    SET_SENSOR_ON;
    
    while( mSysStatus > SYS_ST_INIT &&  mSysStatus < SYS_ST_RESET ) { // Working Loop
      // Feed the Watchdog
      feed_wwdg();
      // Reset main-loop-dead-lock-check timer
#ifdef DEBUG_NO_WWDG
      m_MainloopTimeTick = 0;
#endif
      ////////////rfscanner process///////////////////////////////
      ProcessOutputCfgMsg();
      
      // reset rf if required
      ResetRFModule();  
      
/* Low Power Mode Code -------------------------------------------------------*/
#ifdef ENABLE_LOW_POWER_MODE      
      // Enter Low Power Mode
      if( (mSysStatus == SYS_ST_ON_BATTERY || mSysStatus == SYS_ST_LOW_BATTERY) && !gConfig.inConfigMode ) {
        if( tmrIdleDuration > TIMEOUT_IDLE ) {
          printlog("enter low...");
          tmrIdleDuration = 0;
          lowpower_config();
          halt();
        }
      }
      // Wake Me up
      if( WakeupMeFromSleep() ) {
          // Make sure data can be sent immieiately
          pre_pir_value = 255;
      }
#endif // Low Power Mode Code
      
      if( pre_pir_value != pir_value || (gConfig.keepalive > 0 && pir_tick >= gConfig.keepalive * 100) ) {
          // Reset send timer
          pir_tick = 0;
          pre_pir_value = pir_value;
          Msg_SendPIR(pre_pir_value);
      }
      
      // Send message if ready
      SendMyMessage();
      
      // Save Config if Changed
      SaveConfig();
      
      // Save config into backup area
      SaveBackupConfig();
    }
  }
}

// 10ms timer handler
void tmrProcess() {
  // Ticks
  mTimerKeepAlive++;
  if( delaySendTick > 0) delaySendTick--;
  
  if( (++m_nPowerCheckTick) % POWER_CHECK_INTERVAL == 0 ) {
    // Check Power voltage
    Check_eq();
  }
    
  pir_tick++;
  if( pirofftimeout > 0 && pir_value == 1 ) {
    pirofftimeout--;
    if( pirofftimeout == 0 ) {
      pir_value = 0;
      led_green_off();
    }
  }
  
  // Restart Check
  RestartCheck();
}

void RF24L01_IRQ_Handler() {
  tmrIdleDuration = 0;
  uint8_t lv_rfst = RF24L01_get_whatHappened();
  RF24L01_clear_interrupts();
  if( lv_rfst & RF_RESULT_RECEIVED ) {
    // Packet was received (4)
    RF24L01_read_payload(prcvMsg, PLOAD_WIDTH);
    bMsgReady = ParseProtocol();
  } 
  if( lv_rfst & (RF_RESULT_SENT | RF_RESULT_MAX_RT) ) {
    // Packet was sent (1) or max retries reached (2)
    mutex = (lv_rfst & RF_RESULT_SENT ? 1 : 2); 
  }
}

void PIR_IRQ_Handler()
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  tmrIdleDuration = 0;
  if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == RESET) {
    //pir_value = 0;        delay operation
    pirofftimeout = gConfig.timeout * 100;
  } else {
    pirofftimeout = 0;
    pir_value = 1;
    led_green_on();
  }
}