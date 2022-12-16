#include "app.h"
#define _IWDG
#include "stdio.h"
#include "string.h"
char buffer_hello[1] = {'a'};
char *ptr = buffer_hello;
char buffer_counter[10];
void HardFault_Handler(){

__NOP();
}
int main(){
	
	// Loop Variables
	SysClockConfig ();
	SysTick_Config(SystemCoreClock / 1000); 
	//RTC_Config();
	//RTC_Sync();
	//initRTC();
//	RTC_Sync();
	//systickinit();
	//init_GPIO();
	#ifdef _IWDG
	check_reset_source();
	watchDogInit();
	#endif
	//RTC_Config_trial();
	init_GPIO(GPIOD,GPIO_MODER_MODER12_0);
	init_GPIO(GPIOD,GPIO_MODER_MODER13_0);
	init_GPIO(GPIOD,GPIO_MODER_MODER14_0);
	init_GPIO(GPIOD,GPIO_MODER_MODER15_0);
	

	pin_Write(GPIOD,12,RESET);
	pin_Write(GPIOD,13,RESET);
	pin_Write(GPIOD,14,RESET);
	pin_Write(GPIOD,15,SET);	
	delayms(1000);// Loop repeats 2,000,000 implementing a delay
	while(1){
		// Turn on LEDs
		
		pin_Write(GPIOD,12,SET);
		pin_Write(GPIOD,13,SET);
		pin_Write(GPIOD,14,SET);
		pin_Write(GPIOD,15,SET);
	//	IWDG->KR  = IWDG_KEY_RELOAD;
		delayms(1000);// Loop repeats 2,000,000 implementing a delay
		
		pin_Write(GPIOD,12,RESET);
		pin_Write(GPIOD,13,RESET);
		pin_Write(GPIOD,14,RESET);
		pin_Write(GPIOD,15,RESET);	
		
		// Delay
		delayms(1000);// Loop repeats 2,000,000 implementing a delay
		
		IWDG_Refresh();
	}
}


