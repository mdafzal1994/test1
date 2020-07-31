 
 //#ifndef  INC_STM32F407xx_H_
 //#define   INC_STM32F407xx_H_
 
 #ifndef    _HAL_GPIO1
 #define   _HAL_GPIO1
 
 
 #include<stdint.h>   //  uint32_t  DEFINE UNDER THIS HEADER 
 #define _vo  volatile
 
/* BASE ADD OF FLASH AND SRAM */

 #define FLASH_BASEADDR        0x08000000U
 #define SRAM1_BASEADDR        0x20000000U
 #define SRAM2_BASEADDR        0x2001C000U
 #define ROM_BASEADDR          0x1FFF0000U
 #define SRAM                  SRAM1_BASEADDR
 
 /*  BASE ADD OF BUS */
 
 #define  AHB1PERIPH_BASE     			   0x40016800U
 #define  AHB2PERIPH_BASE    				   0x50000000U
 #define  PERIPH_BASE    	             0x40000000U
 #define  APB1PERIPH_BASE    				   PERIPH_BASE
 #define  APB2PERIPH_BASE     				             0x40010000U

 /* BASE  ADD OF PERIPHERALS WHICH HANGING AHB1 */
 
   #define GPIOA_BASEADDR      0x40020000U
	 #define GPIOB_BASEADDR      0x40020400U
   #define GPIOC_BASEADDR      0x40020800U
   #define GPIOD_BASEADDR      0x40020C00U
   #define GPIOE_BASEADDR      0x40021000U
	 
	 
 /* BASE  ADD OF PERIPHERALS WHICH HANGING APB1 */
   
	 #define I2C1_BASEADDR          0x40005400U
	 #define I2C2_BASEADDR          0x40005800U
	 #define I2C3_BASEADDR         (APB1PERIPH_BASE +0X5C00)
		
		#define SPI2_BASEADDR         (APB1PERIPH_BASE +0X3800)
		#define SPI3_BASEADDR         (APB1PERIPH_BASE +0X3C00)
		
		#define USART2_BASEADDR         (APB1PERIPH_BASE +0X4400)
		#define USART3_BASEADDR         (APB1PERIPH_BASE +0X4800)
	 
	 #define UART4_BASEADDR         (APB1PERIPH_BASE +0X4C00)
	 #define UART5_BASEADDR         (APB1PERIPH_BASE +0X5000)
	 
	 #define RCC_BASEADDR        (AHB1PERIPH_BASE +0X3000)  
	 
	 /* BASE  ADD OF PERIPHERALS WHICH HANGING APB2 BUS */
	 
	   #define EXTI_BASEADDR         (APB2PERIPH_BASE +0X3C00)
		 #define SYSCFG_BASEADDR         (APB2PERIPH_BASE +0X3800)
	   #define SPI1_BASEADDR         (APB2PERIPH_BASE +0X3000)
		 #define USART1_BASEADDR         (APB2PERIPH_BASE +0X1000)
     #define USART6_BASEADDR         (APB2PERIPH_BASE +0X1400)

/*   ========== PERIPHERL RESIGSTER DEFINATION  STRUCTURE ==========*/
    /* peripharal register og GPIO  */
		
		typedef struct 
		{
		  _vo	 uint32_t MODER;      /* GPIO port mode register  ADDRESS OFFSET : 0X00 */
			_vo  uint32_t OTYPER;    /* ADDRESS OFFSET : 0X04 */
			_vo  uint32_t OSPEEDR;
			_vo  uint32_t PUPDR;
			_vo  uint32_t IDR;
			_vo  uint32_t ODR;
			_vo  uint32_t BSRR;
			_vo  uint32_t LCKR;
			_vo  uint32_t AFR[2];
			  
		}GPIO_RegDef_t;   // HOW TO USE THIS STRUCTURE 
		                  /* GPIO_RegDef_t  *pGPIOA=(GPIO_Reg_Def_t*)GPIOA_BASEADDR
	                           */	
		
		

		/*   ========== PERIPHERL RESIGSTER DEFINATION  STRUCTURE ==========*/
    /* peripharal register oF RCC   */
		
		typedef struct 
		{
		  _vo	 uint32_t CR;      /* RCC clock control register ADDRESS OFFSET : 0X00 */
			_vo  uint32_t PLLCFGR;    /* ADDRESS OFFSET : 0X04 */
			_vo  uint32_t CFGR;
			_vo  uint32_t CIR;
			_vo  uint32_t AHB1RSTR;
			_vo  uint32_t AHB2RSTR;
			_vo  uint32_t AHB3RSTR;
			uint32_t    RESERVED0;
			_vo  uint32_t ABP1RSTR;
					_vo  uint32_t ABP2RSTR;
			uint32_t    RESERVED1[2];
				_vo  uint32_t AHB1ENR;
				_vo  uint32_t AHB2ENR;
			_vo  uint32_t AHB3ENR;
			uint32_t    RESERVED2;
			_vo  uint32_t APB1ENR;
			_vo  uint32_t APB2ENR;
			 
			uint32_t    RESERVED3[2];
			_vo  uint32_t AHB1LPENR;
			_vo  uint32_t AHB2LPENR;
			_vo  uint32_t AHB3LPENR;
			uint32_t    RESERVED4;
			
			_vo  uint32_t  APB1LPENR;
			_vo  uint32_t  APB2LPENR;
			uint32_t    RESERVED5[2];
			
			_vo  uint32_t  BDCR;
			_vo  uint32_t  CSR;
			uint32_t    RESERVED6[2];
			_vo  uint32_t  SSCGR;
			_vo  uint32_t  PLLI2SCFGR;
			_vo  uint32_t  PLLSAICFGR;
			_vo  uint32_t  DCKCFGR;
			_vo  uint32_t  CKGATENR;
			_vo  uint32_t  DCKCFGR2;     
			
			}RCC_RegDef_t;	
		
		
		
	/*   PERIPHERIAL DEFINETION  (PERIPHERIAL BASE ADDRESSS TEPECAST TO XXX REGDEF_T)    */
 
		#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
		#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
		#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
		#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
		#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
		
		#define RCC	   ((RCC_RegDef_t*)RCC_BASEADDR)
		
		
		
	/* CLOCK ENABLE MACRO FOR GPIOx PERIPHERALS  */
			
			#define  GPIOA_PCLK_EN()    (RCC->AHB1ENR |=(1<<0))
		  #define  GPIOB_PCLK_EN()    (RCC->AHB1ENR |=(1<<1))
			#define  GPIOC_PCLK_EN()    (RCC->AHB1ENR |=(1<<2))
			#define  GPIOD_PCLK_EN()    (RCC->AHB1ENR |=(1<<3))
			#define  GPIOE_PCLK_EN()    (RCC->AHB1ENR |=(1<<4))
			
		/* CLOCK ENABLE MACRO FOR I2CXx PERIPHERALS  */	 
		
		  #define  I2C1_PCLK_EN()    (RCC->APB1ENR |=(1<<21))



    /* CLOCK ENABLE MACRO FOR SPIXx PERIPHERALS  */	 
		
		  #define  SPI1_PCLK_EN()    (RCC->APB2ENR |=(1<<12))


/* CLOCK DIABLE MACRO FOR  PERIPHERALS=====================  */	 
		
		  


/* CLOCK DISABLE MACRO FOR GPIOx PERIPHERALS  */
			
			#define  GPIOA_PCLK_DI()    (RCC->AHB1ENR &=~(1<<0))
		  #define  GPIOB_PCLK_DI()    (RCC->AHB1ENR &=~(1<<1))
			#define  GPIOC_PCLK_DI()    (RCC->AHB1ENR &=~(1<<2))
			#define  GPIOD_PCLK_DI()    (RCC->AHB1ENR &=~(1<<3))
			#define  GPIOE_PCLK_DI()    (RCC->AHB1ENR &=~(1<<4))
			
		/* CLOCK DISABLE MACRO FOR I2CXx PERIPHERALS  */	 
		
		  #define  I2C1_PCLK_DI()    (RCC->APB1ENR &=~(1<<21))



    /* CLOCK DISABLE MACRO FOR SPIXx PERIPHERALS  */	 
		
		  #define  SPI1_PCLK_DI()    (RCC->APB2ENR &=~(1<<12))







 


   
 	
	
 
 
 
 
 
 
 
 #endif