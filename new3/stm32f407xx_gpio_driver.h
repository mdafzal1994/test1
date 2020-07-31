/* stm32f407xx_gpio_driver.h  */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "_HAL_GPIO3.h"





		



/*    this fconfigration structure  for a GPIO pin */
  
	typedef struct
   {
	   uint8_t GPIO_PinNumber;
		 uint8_t GPIO_PinMode;
		 uint8_t GPIO_PinSpeed;
		 uint8_t GPIO_pinPuPdControl;
		 uint8_t GPIO_PinOPType;
		 uint8_t GPIO_PinAltFunMode;
	 
	 
	 }GPIO_PinConfig_t;
	
	

/* THIS IS HANDLE STRUCTURE OF GPIO   */

 typedef struct
 {
    // pointer to hold base add of gpio pin 
    GPIO_RegDef_t *pGPIOx;  //this pin hold base add of GPIO port to which it belog //    
    GPIO_PinConfig_t GPIO_PinConfig; // this hold the gpio pin setting 

 }GPIO_Handle_t;


 /* gpio pin NUMBER */
 
 #define GPIO_PIN_NO_0       0
 #define GPIO_PIN_NO_1       1
 #define GPIO_PIN_NO_2       2
 #define GPIO_PIN_NO_3       3
 #define GPIO_PIN_NO_4       4
 #define GPIO_PIN_NO_5       5
 #define GPIO_PIN_NO_6       6
 #define GPIO_PIN_NO_7       7
 #define GPIO_PIN_NO_8       8
 #define GPIO_PIN_NO_9       9
 #define GPIO_PIN_NO_10      10
 #define GPIO_PIN_NO_11      11
 #define GPIO_PIN_NO_12      12
 #define GPIO_PIN_NO_13      13
 #define GPIO_PIN_NO_14      14
 #define GPIO_PIN_NO_15      15
 
 
 
 
 
 /* GPOI  MODE  PIN */
 
 #define  GPIO_MODE_IN      0
 #define  GPIO_MODE_OUT     1
 #define GPIO_MODE_ALTFN    2
 #define  GPIO_MODE_ANALOG  3
 #define  GPIO_MODE_IT_RT   4    /* addditional rising edge and fallinf age */
 #define  GPIO_MODE_IT_FT   5
 #define  GPIO_MODE_IT_RFT  6
 
 
 /* GPIO port output type register   push pupp or open darain */
  #define GPOI_OP_TYPE_PP    0
  #define   GPIO_OP_TYPE_OO   1

/* GPIO port output speed register  */

#define GPIO_SPEED_LOW         0
 #define GPIO_SPEED_MEDIUM     1
 #define GPIO_SPEED_FAST       2
 #define GPIO_SPEED_HIGH       3
 
 
 
 /* GPIO port pull-up/pull-down register */
 
   #define  GPIO_NO_PUPD         0
	 #define GPIO_PIN_PU           1
	 #define GPIO_PIN_PD           2
	 
 
 
 
 
 
 
 /*  ===============================*/
      /* APIs SUPPPORT BY THIS DRIVER */
 
    /* PERIPHERAL CLOCK SETUP */
 void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);
 
 /* INT AND DI INIT */
 
 void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
 void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
 
 /* RED INPUT PIN */
 uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
 
 
 /* read fro input port */
 uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
 
 void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
 void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value);
 
 void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
 
 // IQR CONFIGRATIO   
 
 void GPPI_IRQInterruptConfig(uint8_t IQRNumber,uint8_t EnorDi);
  void GPIO_IRQPriorityConfig(uint8_t IQRNumber,uint8_t IRQPriority);
 void GPIO_IRQHandling(uint8_t PinNumber);
 
 
 
 
 
 


#endif