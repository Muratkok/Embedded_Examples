#include "app.h"
volatile uint32_t millis = 0;

void SysTick_Handler(void) // kesme olustugunda girilecek fonksiyon
{
	millis++;      // millis degerini 1 arttir
	if(IWDG->KR<10)
	IWDG->KR = IWDG_KEY_RELOAD;
//	check_reset_source();
}

void delayms(int ms)  // milisaniye cinsinden delay fonksiyonu
{
	int delay = millis+ms;
	while(millis<delay);
}

void systickinit(void)
{

SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Sayiciyi aktif et
SysTick->LOAD = 0x3E80-1;                     // Reload degerini ata
SysTick->VAL=0;                        // sayiciyi sifirla
SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;  // kesmeyi aktif et
SysTick->CTRL &= ~(1<<2);                      //Systick clock source = AHB/8
NVIC_EnableIRQ(SysTick_IRQn);     // NVIC tarafinda kesmeyi aç
NVIC_SetPriority (SysTick_IRQn,0);  //kesme önceligini ayarla
 
}

void SysClockConfig (void)
{
		/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
	
	1. ENABLE HSE and wait for the HSE to become Ready
	2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
	3. Configure the FLASH PREFETCH and the LATENCY Related Settings
	4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
	5. Configure the MAIN PLL
	6. Enable the PLL and wait for it to become ready
	7. Select the Clock Source and wait for it to be set
	
	********************************************************/
	
	
	#define PLL_M 	4
	#define PLL_N 	168
	#define PLL_P 	0  // PLLP = 2

	// 1. ENABLE HSE and wait for the HSE to become Ready
	RCC->CR |= RCC_CR_HSEON;  // RCC->CR |= 1<<16; 
	while (!(RCC->CR & RCC_CR_HSERDY));  // while (!(RCC->CR & (1<<17)));
	
	// 2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;  // RCC->APB1ENR |= 1<<28;
	PWR->CR |= PWR_CR_VOS;  // PWR->CR |= 3<<14; 
	
	
	// 3. Configure the FLASH PREFETCH and the LATENCY Related Settings
	FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;  // FLASH->ACR = (1<<8) | (1<<9)| (1<<10)| (5<<0);
	
	// 4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
	// AHB PR
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // RCC->CFGR &= ~(1<<4);
	
	// APB1 PR
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;  // RCC->CFGR |= (5<<10);
	
	// APB2 PR
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;  // RCC->CFGR |= (4<<13);
	
	
	// 5. Configure the MAIN PLL
	RCC->PLLCFGR = (PLL_M <<0) | (PLL_N << 6) | (PLL_P <<16) | (RCC_PLLCFGR_PLLSRC_HSE);  // (1<<22);

	// 6. Enable the PLL and wait for it to become ready
	RCC->CR |= RCC_CR_PLLON;  // RCC->CR |= (1<<24);
	while (!(RCC->CR & RCC_CR_PLLRDY));  // while (!(RCC->CR & (1<<25)));
	
	// 7. Select the Clock Source and wait for it to be set
	RCC->CFGR |= RCC_CFGR_SW_PLL;  // RCC->CFGR |= (2<<0);
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // while (!(RCC->CFGR & (2<<2)));
}
uint8_t _rtc_dec_to_bcd(uint8_t dec)
{
	return ((dec / 10) << 4) | (dec % 10);
}
void lockRTC()
{
	RTC->WPR = 0xFF;//lock command
}
void unlockRTC(){
	RTC->WPR = 0xCA;//unlock commands
	RTC->WPR = 0x53;
}

void RTC_Config_trial(void)
{
// 1) Enable PWR clock by by enabling PWREN bit in RCC_APB1 
	
	   RCC->APB1ENR = (0x01<<28);   //PWREN: Power interface clock enable
	
// 2) Allow RTC backup domain acess through DPB bit in PWR_CR
	  
	  	PWR->CR |= (0x01<<8);       //enable DBP bit in PWR_CR
	
// 3) Select the clock source for RTC by RTCSEL=10 in RCC_BDCG

	  	RCC->BDCR|=(2UL<<8);        //RTC clock source selection 10-LSI
	
	
//// 4) wait till LSI is ready when LSIRDY is set
//	  	while(1)
//		{
//			  if((RCC->CSR & RCC_CSR_LSIRDY)==1)
//			  break;
//		}
//     			 
// 5) Enable LSI clock by LSION bit in RCC_CSR 
	
	  	RCC->CSR|=1UL;              //Enable LSI clock by enabling LSION bit in RCC_CSR


// 6) Enable the RTC clock through RTCEN bit in RCC_BDCR
	  	RCC->BDCR |=(1UL<<15);
				unlockRTC();	
		//rtc_wait_for_synchro();
		//unlockRTC();
			RTC->ISR |= RTC_ISR_INIT;//(0x1ul << 7)
			RTC_Sync();
}


void initRTC(){
	// 1) Enable PWR clock by by enabling PWREN bit in RCC_APB1
		RCC->APB1ENR = RCC_APB1LPENR_PWRLPEN;
	// 2) Allow RTC backup domain acess through DPB bit in PWR_CR
		PWR->CR |= PWR_CR_DBP;//(0x01<<8);  //enable DBP bit in PWR_CR
		// Turn on LSE and wait until it become stable
//		RTC_WaitForSynchro();
		//RTC->WPR |= 0xCA;//unlock commands
		//RTC->WPR |= 0x53;
		RCC->BDCR |=	RCC_LSE_ON;
	  RTC_Config_trial();
		//RCC->BDCR &= ~(RCC_BDCR_LSERDY);
		//while(!(RCC->BDCR & RCC_BDCR_LSERDY));
		//RCC_LSEConfig(RCC_LSE_ON);
		//while(!(RCC->CSR & RCC_CSR_LSIRDY));
		// Select LSE as RTC clock source
	  RCC->BDCR  |= RCC_BDCR_RTCSEL_0;
		//RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
		// Enable RTC clock
		//RCC_RTCCLKCmd(ENABLE);
		RCC->BDCR   |=  RCC_BDCR_RTCEN;
		unlockRTC();	
		//rtc_wait_for_synchro();
		//unlockRTC();
	  RTC->ISR |= RTC_ISR_INIT;//(0x1ul << 7)
		//RTC->TR |= RTC_TR_PM;
	 // RTC->TR |= RTC_TR_HT
		//RTC->PRER &= RTC_PRER_PREDIV_S;	
	  //RTC->ISR 
		// Wait the registers to be synchronised 
		//while(!(RTC->ISR & ~RTC_ISR_INITF));
	
	
	  //RTC->ISR |= (0x01<<6);
		//RTC->ISR |= RTC_ISR_INITF;			
    lockRTC();
		//RTC->TR = 0 << 21 |
}

uint8_t isIwdg_Ready(void)
{
	return((READ_BIT(IWDG->SR, (IWDG_SR_PVU | IWDG_SR_RVU)) == 0U) ? 1UL : 0UL);
}
void watchDogInit(void)
{
		//RCC->CSR = RCC_CSR_IWDGRSTF;

	  IWDG->KR  = IWDG_KEY_ENABLE;
		IWDG->KR  = IWDG_KEY_WRITE_ACCESS_ENABLE;
		IWDG->PR  = IWDG_PR_PR_2;//IWDG_PR_PR_0;//precaler is 4
		IWDG->RLR = (uint32_t)0x00009C3F;//IWDG_RLR_RL; //0x0FFF 4095
	//	while(!(IWDG->SR == 0x00));
		while(!(isIwdg_Ready() == 1)){}
			
		IWDG->KR  = (uint32_t)0x00009C3F;//IWDG_KEY_RELOAD;
		//IWDG->RLR |=IWDG_RLR_RL; //0x0FFF 4095
}
void IWDG_Refresh()
{

		IWDG->KR  = (uint32_t)0x00009C3F;//IWDG_KEY_RELOAD;

}
void RTC_Sync(void)//sorunlu
{
	PWR->CR |= (0x01<<8);  //enable DBP bit in PWR_CR
	RTC->WPR=0XCA;
	RTC->WPR=0X53;
	
	
	//clear RSF flag
	RTC->ISR |=(0x00<<5);
	
	// Wait the registers to be synchronised 
	while(1)
	{

	  if((RTC->ISR & RTC_ISR_RSF )==1)
		  break;
	}
	
	RTC->WPR=0XFF;	
	
}

void check_reset_source(void)
{
	if(((RCC->CSR & RCC_CSR_IWDGRSTF) == RCC_CSR_IWDGRSTF))
	{
				//Clear IWDG Reset Flag
				RCC->CSR = RCC_CSR_RMVF;
				
			//	NVIC_SystemReset();
	}

}
void rtc_wait_for_synchro(void)
{
        /* Unlock RTC registers */
        //RTC->WPR = 0xca;
        //RTC->WPR = 0x53;
				//unlockRTC();
        RTC->ISR |= (RTC_ISR_RSF);
 
        while (!(RTC->ISR & RTC_ISR_RSF));
 
        /* disable write protection again */
        //RTC->WPR = 0xff;
		   lockRTC();
}
void init_GPIO(GPIO_TypeDef* GPIOx,uint32_t GPIO_Pos){

	// Configue LEDs
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN; // Enable the clock of port D of the GPIO
	//GPIO_Pos = GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;
	GPIOx->MODER |= GPIO_Pos; // Green LED, set pin 12 as output
	GPIOx->OSPEEDR = GPIO_OSPEEDER_OSPEEDR2;
}

void pin_Write(GPIO_TypeDef* GPIOx , uint16_t GPIO_Pin,bool GPIO_PinState){

		if(GPIO_PinState == true)
		{
				GPIOx -> BSRR = 1 << GPIO_Pin; // Set the BSRR bit 12 to 1 to turn respective LED on
		}
		else GPIOx -> BSRR = 1 << (GPIO_Pin + 16); // Set the BSRR bit 12 to 1 to turn respective LED on
			
}
