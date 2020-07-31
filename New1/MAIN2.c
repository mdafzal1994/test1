
#include "stm32f407xx.h"




void EXTI9_5_IRQHandler(void)
	{
	
	  
	
	
	}
void delay(void);
void delay(void)
	{
		 
	   for(uint32_t i=0;i<5000;i++);
	
	}
int main()
{
    RCC->AHB1ENR|=(1<<3);
	 delay();
	
	return 0;



  }