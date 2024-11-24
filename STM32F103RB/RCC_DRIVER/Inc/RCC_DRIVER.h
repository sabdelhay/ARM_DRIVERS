/*
 * RCC_DRIVER.h
 *
 *  Created on: Nov 18, 2024
 *      Author: Sherif Abdelhay
 */

#ifndef RCC_DRIVER_H
#define RCC_DRIVER_H

//RCC registers
typedef struct{
	uint32_t    RCC_CR;
	uint32_t    RCC_CFGR;
	uint32_t    RCC_CIR;
	uint32_t    RCC_APB2RSTR;
	uint32_t    RCC_APB1RSTR;
	uint32_t    RCC_AHBENR;
	uint32_t    RCC_APB2ENR;
	uint32_t    RCC_APB1ENR;
	uint32_t    RCC_BDCR;
	uint32_t    RCC_CSR;
}RCC_Reg_t;

//Bus types
typedef enum {
    RCC_AHB = 0,   // AHB bus
    RCC_APB1 = 1,  // APB1 bus
    RCC_APB2 = 2   // APB2 bus
}RCC_BusType;

//Peripherals
typedef enum Peripheral{
    //AHB peripherals
    RCC_DMA1,
    RCC_DMA2,
    RCC_SRAM,
    RCC_FLITF,
    RCC_CRC,

    //APB1 peripherals
    RCC_TIM2 = 32,
    RCC_TIM3,
    RCC_TIM4,
    RCC_TIM5,
    RCC_TIM6,
    RCC_TIM7,
    RCC_WWDG,

    //APB2 peripherals
    RCC_AFIO = 64,
    RCC_GPIOA,
    RCC_GPIOB,
    RCC_GPIOC,
    RCC_GPIOD,
    RCC_GPIOE,
    RCC_ADC1,
    RCC_ADC2,
    RCC_TIM1,
    RCC_SPI1,
    RCC_USART1,
    RCC_TIM8,
};

//RCC -> RCC_CR bits
enum{
  HSION,
  HSIRDY,
  HSEON=16,
  HSERDY,
  PLLON=24,
  PLLRDY
};

//RCC -> RCC_CFGR bits
enum{
  SW0,
  SW1,
  SWS0,
  SWS1,
  PLLSRC=16,
  PLLXTPRE,
  PLLMUL0,
  PLLMUL1,
  PLLMUL2,
  PLLMUL3,
  MCO0=24,
  MCO1,
  MC02
};

//HSE Source
typedef enum HSESrc{
  notDivided,
  dividedByTwo,
};

//PLL source
typedef enum PLLClkSrc{
  halfHSI,
  halfHSE,
  fullHSE
};

//Error status
typedef enum {
    ERROR_NONE = 0,                     // No error
    ERROR_TIMEOUT,                      // Timeout occurred
    ERROR_INVALID_CLOCK_TYPE,           // Invalid clock type
    ERROR_INVALID_PLLMUL_VALUE          // Invalid PLL multiplier value
}ErrorCode;

//Determine the clock status
#define RCC_CFGR_SWS0     READ_BIT(RCC -> RCC_CFGR, SWS0)
#define RCC_CFGR_SWS1     READ_BIT(RCC -> RCC_CFGR, SWS1)

//Clock type
enum clockType {HSI, HSE, PLL};

//Clock status
enum Status {on,off};

//Functions declarations
void Rcc_setClkSts(enum clockType clktype, enum Status status);
void RCC_setSysClk(enum clockType clkType);
void RCC_PLLConfig(enum PLLClkSrc pllClkSrc, int PLLMulVal);
void RCC_HSEConfig(enum HSESrc hseSrc);
void RCC_PeripheralClocks(enum Peripheral peripheral, uint8_t enable);
void errorsHandling(ErrorCode error);

#endif
