#ifndef	__DRV_LED_H__
#define __DRV_LED_H__

#include "stm8l15x_gpio.h"

// LED硬件定义
#define LED_RED_GPIO_PORT			GPIOD								
#define LED_RED_GPIO_PIN			GPIO_Pin_7

#define LED_GREEN_GPIO_PORT			GPIOD							
#define LED_GREEN_GPIO_PIN			GPIO_Pin_6

/** LED定义 */
typedef enum LedPort
{
	LED_RED = 0,		        // 红色LED
	LED_GREEN			        // 绿色LED
} LedPortType;

typedef enum LedPortState
{
	LED_ST_OFF = 0,
	LED_ST_ON,
    LED_ST_FLASH
} LedPortState;

void drv_led_init(const uint8_t _reversed);
void drv_led_change(const LedPortType _ledPort, const LedPortState _ledState);

// 红色LED操作函数
#define led_red_on( )				drv_led_change( LED_RED, LED_ST_ON )
#define led_red_off( )				drv_led_change( LED_RED, LED_ST_OFF )
#define led_red_flashing( )			drv_led_change( LED_RED, LED_ST_FLASH )

// 绿色LED操作函数
#define led_green_on( )				drv_led_change( LED_GREEN, LED_ST_ON )
#define led_green_off( )			drv_led_change( LED_GREEN, LED_ST_OFF )
#define led_green_flashing( )       drv_led_change( LED_GREEN, LED_ST_FLASH )

#endif

