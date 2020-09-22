#include "led.h"

/**
  * @brief :LED初始化
  * @param :无
  * @note  :无
  * @retval:无
  */ 
void drv_led_init( void )
{
	//初始化LED引脚 推挽输出 慢速 初始状态为高
	GPIO_Init( LED_RED_GPIO_PORT, LED_RED_GPIO_PIN, GPIO_Mode_Out_PP_High_Slow );
	GPIO_Init( LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PIN, GPIO_Mode_Out_PP_High_Slow );
    drv_led_off(LED_RED);
    drv_led_off(LED_GREEN);
}

/**
  * @brief :LED亮
  * @param :
  *			@LedPort:LED选择，红色或绿色
  * @note  :无
  * @retval:无
  */
void drv_led_on( LedPortType LedPort )
{
	if( LED_RED == LedPort ) { // LED_RED
		GPIO_SetBits( LED_RED_GPIO_PORT, LED_RED_GPIO_PIN );
	} else { // LED_GREEN
		GPIO_SetBits( LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PIN );
	}	
}

/**
  * @brief :LED灭
  * @param :
  *			@LedPort:LED选择，红色或绿色
  * @note  :无
  * @retval:无
  */
void drv_led_off( LedPortType LedPort )
{
	if( LED_RED == LedPort ) { // LED_RED
		GPIO_ResetBits( LED_RED_GPIO_PORT, LED_RED_GPIO_PIN );	
	} else { // LED_GREEN
		GPIO_ResetBits( LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PIN );
	}	
}

/**
  * @brief :LED闪烁
  * @param :
  *			@LedPort:LED选择，红色或绿色
  * @note  :无
  * @retval:无
  */
void drv_led_flashing( LedPortType LedPort )
{
	
	if( LED_RED == LedPort ) {
		GPIO_ToggleBits( LED_RED_GPIO_PORT, LED_RED_GPIO_PIN );
	} else {
		GPIO_ToggleBits( LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PIN );
	}
}
