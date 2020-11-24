#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "common.h"
#include "stm8l15x_conf.h"
#include "XlightComBus.h"

// Xlight Application Identification
#define XLA_ORGANIZATION          "xlight.ca"               // Default value. Read from EEPROM
#define XLA_PRODUCT_NAME          "XSensor"                 // Default value. Read from EEPROM

// Use the biggest possible NodeID of this type as default to avoid conflict with existing devices.
// After the device started, we can change it's NodeID to what ever we want.
#define XLA_PRODUCT_NODEID        NODEID_MAX_SUPERSENSOR

// Note: please modify this to reflect the correct device type!!!
/// This setting will be presented in devType ('tp') field of message
#define XLA_PRODUCT_Type          SEN_TYP_PIR               // Replace ZEN_TARGET_PIRSENSOR

// Version Format: [ver].[release], where [ver] & [release] are both 4 bits with a value between 0 to 15
#define XLA_VERSION               0x31                      // 3.1 
#define XLA_MIN_VER_REQUIREMENT   0x31

#define LED_PIN_INIT_HIGH         0         // Button LED Pin High or Low on initialization

typedef struct
{
  // Static & status parameters
  UC version                  :8;           // Data version, other than 0xFF
  UC present                  :1;           // 0 - not present; 1 - present
  UC inPresentation           :1;           // whether in presentation
  UC inConfigMode             :1;           // whether in config mode
  UC reserved0                :5;
  
  // Configurable parameters
  UC nodeID;                                // Node ID for Remote on specific controller
  UC subID;                                 // SubID
  UC NetworkID[6];
  UC rfChannel;                             // RF Channel: [0..127]
  UC rfPowerLevel             :2;           // RF Power Level 0..3
  UC rfDataRate               :2;           // RF Data Rate [0..2], 0 for 1Mbps, or 1 for 2Mbps, 2 for 250kbs
  UC rptTimes                 :2;           // Sending message max repeat times [0..3]
  UC enSDTM                   :1;           // Simple Direct Test Mode Flag
  UC reserved1                :1;
  UC type;                                  // Type of Remote
  US token;                                 // Current token
  UC indDevice                :3;           // Current Device Index: [0..3]
  UC reserved2                :5;
  US timeout                  :16;          // pir timeout
  US keepalive                :16;          // keepalive interval
  UL senMap                   :32;          // Sensor Map
} Config_t;

extern Config_t gConfig;
extern uint8_t mSysStatus;

void RF24L01_IRQ_Handler();
bool SendMyMessage();
bool IsConfigInvalid();
bool isNodeIdInvalid(const uint8_t nodeid);
uint16_t GetDelayTick(const uint8_t ds);

//#define TEST
#ifdef TEST
#define     PC1_Low                GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define     PC3_Low                GPIO_ResetBits(GPIOC, GPIO_Pin_3)
#define     PC5_Low                GPIO_ResetBits(GPIOC, GPIO_Pin_5)
#define     PC1_High               GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define     PC3_High               GPIO_SetBits(GPIOC, GPIO_Pin_3)
#define     PC5_High               GPIO_SetBits(GPIOC, GPIO_Pin_5)
#endif

#endif /* __GLOBAL_H */