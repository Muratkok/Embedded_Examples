#ifndef _APP
#define _APP

#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include <stdbool.h>

#define SET true
#define RESET false
#define SYNCHRO_TIMEOUT 3


#define IWDG_KEY_RELOAD                 ((uint32_t)0x0000AAAA)  /*!< IWDG Reload Counter Enable   */
#define IWDG_KEY_ENABLE                 ((uint32_t)0x0000CCCC)  /*!< IWDG Peripheral Enable       */
#define IWDG_KEY_WRITE_ACCESS_ENABLE    ((uint32_t)0x00005555)  /*!< IWDG KR Write Access Enable  */
#define IWDG_KEY_WRITE_ACCESS_DISABLE   ((uint32_t)0x00000000)  /*!< IWDG KR Write Access Disable */

void init_GPIO(GPIO_TypeDef* GPIOx,uint32_t GPIO_Pos);
void pin_Write(GPIO_TypeDef* GPIOx , uint16_t GPIO_Pin,bool GPIO_PinState);
void delayms(int ms);
void SysClockConfig (void);
void initRTC(void);
void lockRTC(void);
void unlockRTC(void);
void RTC_Config(void);

void RTC_Config_trial(void);
void RTC_Sync(void);
void rtc_wait_for_synchro(void);

uint8_t isIwdg_Ready(void);
void watchDogInit(void);
void check_reset_source(void);
void IWDG_Refresh(void);
uint8_t _rtc_dec_to_bcd(uint8_t dec);

#endif
