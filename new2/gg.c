//#include "_HAL_GPIO3.h"
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
  

	
	
	
	
return 0;
      
}


	