#include "_HAL_GPIO3.h"
#include "stm32f4xx.h"


  #define rcc_base_add         0x40023800U  
	#define  rcc_ahb1_offset     0x30
	#define rcc_ahb1_enable    (rcc_base_add+rcc_ahb1_offset)
	

	
void delay(void);
void delay(void)
	{
		 
	   for(uint32_t i=0;i<5000;i++);
	
	}
	
	
	void EXTI9_5_IRQHandler(void)
	{
	
	  
	
	
	}
int main()
{
  
	GPIO_Handle_t  Gpioled;
	GPIO_Handle_t  GButton;
	GPIO_PeriClockControl(GPIOD,ENABLE);  //enable portd clock
	
	//uint32_t *k=(uint32_t*)(rcc_ahb1_enable);
	//     *k|=(1<<3);
	
	
	/* button   ======*/
	  GPIO_PeriClockControl(GPIOD,ENABLE);
    GButton.pGPIOx=GPIOD;
    GButton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
    GButton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
    GButton.GPIO_PinConfig.GPIO_pinPuPdControl=GPIO_PIN_PU;
	
		/* button  END  ======*/
	
	
	
	Gpioled.pGPIOx=GPIOD;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	Gpioled.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	Gpioled.GPIO_PinConfig.GPIO_PinOPType=GPOI_OP_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_pinPuPdControl=GPIO_NO_PUPD;

   
	
	 GPIO_Init(&Gpioled);
	
	GPIO_Init(&GButton);
	
	//IRQ CONFIGRATION
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);//   PRIORITY 15
GPPI_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);
	
	
	
	
	
	
	while(1)
	{
	    if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0)==(uint8_t)0X00000001)
			{
		    delay();
	       GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
				//GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,1);
				//GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,0);
					delay();
			}
	}
	
	
	
	
	
	
	
	
	
return 0;
      
}


	