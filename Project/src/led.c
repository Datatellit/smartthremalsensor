#include "led.h"

// 初始状态为高 既 低电平有效
uint8_t m_LED_PIN_InitHigh = 1;

/**
  * @brief :LED初始化
  * @param :无
  * @note  :无
  * @retval:无
  */ 
void drv_led_init(const uint8_t _reversed)
{
    // 初始化LED引脚 推挽输出 慢速 
    // 初始状态为高(_reversed == 1) 或 低(_reversed == 0)
    m_LED_PIN_InitHigh = _reversed;
    const GPIO_Mode_TypeDef lv_pinMode = (_reversed ? GPIO_Mode_Out_PP_High_Slow : GPIO_Mode_Out_PP_Low_Slow);
	GPIO_Init( LED_RED_GPIO_PORT, LED_RED_GPIO_PIN, lv_pinMode );
	GPIO_Init( LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PIN, lv_pinMode );
    led_red_off();
    led_green_off();
}

/**
  * @brief :LED亮
  * @param :
  *			@_ledPort:LED选择，红色或绿色
  *			@_ledState:LED状态，开、关或闪烁
  * @note  :无
  * @retval:无
  */
void drv_led_change(const LedPortType _ledPort, const LedPortState _ledState)
{
    GPIO_TypeDef *lv_port;
    uint8_t lv_pin;
    if( LED_RED == _ledPort ) { // LED_RED
        lv_port = LED_RED_GPIO_PORT;
        lv_pin = LED_RED_GPIO_PIN;
    } else {
        lv_port = LED_GREEN_GPIO_PORT;
        lv_pin = LED_GREEN_GPIO_PIN;
    }
    if( _ledState == LED_ST_FLASH ) {
        GPIO_ToggleBits(lv_port, lv_pin);
    } else {
        GPIO_WriteBit(lv_port, lv_pin, (BitAction)(m_LED_PIN_InitHigh ^ _ledState));
    }    
}
