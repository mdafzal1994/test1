#include "stm32f407xx_gpio_driver.h"

//#include "_HAL_GPIO3.h"


/*this function  enable and disable the function */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{

	if(EnorDi==ENABLE)
	{
	
	  if(pGPIOx==GPIOA)
		   GPIOA_PCLK_EN();
		else if(pGPIOx==GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx==GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx==GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx==GPIOE)
			GPIOE_PCLK_EN();
	
	}

   else if(EnorDi==DISABLE)
	 {
	 
	 
	   if(pGPIOx==GPIOA)
		  GPIOA_PCLK_DI();  //GPIOA_PCLK_DI()
		else if(pGPIOx==GPIOB)
			GPIOB_PCLK_DI();
		else if(pGPIOx==GPIOD)
			GPIOD_PCLK_DI();
		else if(pGPIOx==GPIOE)
			GPIOE_PCLK_DI();
		else if(pGPIOx==GPIOE)
			GPIOE_PCLK_DI();
	 
	 }


} 





 
 void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
    {
			uint32_t temp=0;
		   /* 1. CONF THE MODE OF GPIO */
			 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=GPIO_MODE_ANALOG)
			 {
			 
			  temp=(uint32_t)(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(uint32_t)(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
				 		//	  temp=(uint32_t)(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(uint32_t)(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        
         pGPIOHandle->pGPIOx->MODER &=~(uint32_t)(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  //clearng bit 00  =0x3 
				 pGPIOHandle->pGPIOx->MODER |=temp;         
				 temp=0;
			 
			 }
		
		  /* 2. conf the speed of gpio */
			 temp=0;
			 temp=(uint32_t)(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (uint32_t)(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
         pGPIOHandle->pGPIOx->OSPEEDR&=~(uint32_t)(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);      	
			 pGPIOHandle->pGPIOx->OSPEEDR |=temp;
     /* 3. GPIO port output type register pull up pull down */
      temp=0;			 
		  temp=(uint32_t)(pGPIOHandle->GPIO_PinConfig.GPIO_pinPuPdControl << (uint32_t)(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
       pGPIOHandle->pGPIOx->PUPDR&=~(uint32_t)(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);      	


			 pGPIOHandle->pGPIOx->PUPDR |=temp;
			 
			 /* 4. GPIO port output type register */
		   temp=0;			 
		  temp=(uint32_t)(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
      	 
			  pGPIOHandle->pGPIOx->OTYPER&=~(uint32_t)(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			 pGPIOHandle->pGPIOx->OTYPER |=temp;

    /* cnf gpio alternate functionality	*/

    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN)
		{
		
		    uint8_t temp1,temp2;
        temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
        temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
			  pGPIOHandle->pGPIOx->AFR[temp1]&=~(uint32_t)(0xF <<(4*temp2));  
        pGPIOHandle->pGPIOx->AFR[temp1] |=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2));			
		
		}		
  
  else
			{
				if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT)
				{
					// 1. CONGIGUR RTSR
					
					EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					EXTI->FTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}
				else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT)
				{
				   // 2. CONGIGUR FTSR
					EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					EXTI->RTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				  }
				else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT)
				{
				   // 2. CONGIGUR FTSR  AND RTSR BOTH
					EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				  }
			   
					// CON GPIO PORT SELECION IN SYSCFG_EXTIR
					uint8_t TEMP1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
					uint8_t TEMP2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
					uint8_t PortCode=GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
					SYSCFG_PCLK_EN();
					
					
					SYSCFG->EXTICR[TEMP1]=(uint8_t)(PortCode<<(uint8_t)(TEMP2*4));
					EXTI->EMR|=1<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					
					
					
	     		
			}		

		
		
		}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{


   if(pGPIOx==GPIOA)
		   GPIOA_REG_RESET();
		else if(pGPIOx==GPIOB)
			GPIOB_REG_RESET();
		else if(pGPIOx==GPIOD)
			GPIOC_REG_RESET();
		else if(pGPIOx==GPIOE)
			GPIOD_REG_RESET();
		else if(pGPIOx==GPIOE)
			GPIOE_REG_RESET();  
}
 
/* GPIO READ FROM  INPUT PIN  */


 uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
 {
 
    uint8_t value;
	  value=(uint8_t)((pGPIOx->IDR >>PinNumber)&0x00000001);
			
	 return value;
 } 
 
 /* GPIO READ FROM  INPUT PORT */
 
 uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
	{
	   
	uint16_t value;
	  value=(uint16_t)pGPIOx->IDR ;
			
	 return value;
	
	
	}
 
	/* GPIO write to  out put PIN  */
	
 void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value)
	{
	    if(Value==GPIO_PIN_SET)
			{
			 //write 1 to output data resister at bit field corresponding
			  pGPIOx->ODR|=(1<<PinNumber);
				
			}
	   else
			{
			   pGPIOx->ODR&=~(1<<PinNumber);
			
			
			}
	
	
	}
	
	/* GPIO write to output port  PIN  */
 void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value)
	{
	
	    pGPIOx->ODR=Value;
	
	}
	
	/* GPIO toggle   */
 
	
	
	
	void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)

   {
	 
	   pGPIOx->ODR^=(1<<PinNumber);
	 
	 }


	 void GPPI_IRQInterruptConfig(uint8_t IQRNumber,uint8_t EnorDi)
	{
		if(EnorDi==ENABLE)
		{
	  if(IQRNumber<=31)
		{
		  // program ISER0  FOR ENABLE
			*NVIC_ISER0|=(1<<IQRNumber);
		
		}
     else if (IQRNumber>31 && IQRNumber<64)
		 {
			 // program ISER1
			 *NVIC_ISER1|=(1<<(IQRNumber%32));
		 }
     else if(IQRNumber>=64 && IQRNumber<96)
		 {
			 // program ISER2 REG 
			 *NVIC_ISER3|=(1<<(IQRNumber%32));
		 
		 }			 
	}		
		else
			{
				if(IQRNumber<=31)
		{
			//PROGRAM ICER0 REGISTER FOR  DISABLE   IT IS M CORTES NVIC USER GUID
		  *NVIC_ICER0|=(1<<IQRNumber);
		
		}
     else if (IQRNumber>31 && IQRNumber<64)
		 {
			 //PROGRAM ICER0 REGISTER FOR  DISABLE   IT IS M CORTES NVIC USER GUID 
			 *NVIC_ICER1|=(1<<(IQRNumber%32));
			 
		 }
     else if(IQRNumber>=64 && IQRNumber<96)
		 {
		     *NVIC_ICER3|=(1<<(IQRNumber%32));
		 }		
			
			
			}
	
	
	}
	
	 
 void GPIO_IRQPriorityConfig(uint8_t IQRNumber,uint8_t IRQPriority)
 {
     // 1.FIERST LET FIND OUT IPR REGISTER NO
	  uint8_t iprx=IQRNumber/4;
	 uint8_t  iprx_section=IQRNumber%4;
	 uint8_t shift_amount =(8*iprx_section)*(8-NO_PR_BITS_IMPLEMENTED);
	 *(NIVC_PR_BASE_ADDR+iprx*4)|=(IRQPriority<<shift_amount);
 
 }
	
	
	
	
 void GPIO_IRQHandling(uint8_t PinNumber)
 {
   // CLEAR EXTI PR REGISTER CORRESPONDING TO PIN NUMBER 
	 if(EXTI->PR&(1<<PinNumber))
	 {
	 
	    EXTI->PR|=(1<<PinNumber); // CLEAR BIT BYE WRITING 1
	 
	 }
 
 }

		