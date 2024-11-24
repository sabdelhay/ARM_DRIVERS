/*
 * RCC_DRIVER.C
 *
 *  Created on: Nov 18, 2024
 *      Author: Sherif Abdelhay
 */

#include <stdint.h>
#include "RCC_DRIVER.h"
#include "STD_MACROS.h"
#include <stdio.h>

RCC_Reg_t *RCC = (RCC_Reg_t *)(0x40021000);

volatile ErrorCode errorFlag = ERROR_NONE;
volatile int ERROR_STATE = 0;

//Enabling clocks
void Rcc_setClkSts(enum clockType clktype, enum Status status) {
    uint32_t counter = 0;
    const uint32_t timeout = 10000;
    switch (clktype) {
        case HSI:
            if (status == on) {
                SET_BIT(RCC->RCC_CR, HSION); // Enable HSI clock

                while (!(READ_BIT(RCC->RCC_CR, HSIRDY))) {
                    counter++;
                    if (counter >= timeout) {
                        ERROR_STATE = -1;
                        break;
                    }
                }
                }else {
                if (RCC_CFGR_SWS0 == 0 && RCC_CFGR_SWS1 == 0) {
                    SET_BIT(RCC->RCC_CFGR, SW0);
                    CLR_BIT(RCC->RCC_CFGR, SW1);

                    while (!(READ_BIT(RCC->RCC_CR, HSERDY))) {
                        counter++;
                        if (counter >= timeout) {
                            ERROR_STATE = -1;
                            break;
                        }
                    }
                }
                CLR_BIT(RCC->RCC_CR, HSION); // Disable HSI clock
            }
            break;

        case HSE:
            if (status == on) {
                SET_BIT(RCC->RCC_CR, HSEON); // Enable HSE clock

                while (!READ_BIT(RCC->RCC_CR, HSERDY)) {
                    counter++;
                    if (counter >= timeout) {
                        ERROR_STATE = -1;
                        break;
                    }
                }
            }else {
                if(RCC_CFGR_SWS0 == 0 && RCC_CFGR_SWS1 == 1) {

                    CLR_BIT(RCC->RCC_CFGR, SW0);
                    CLR_BIT(RCC->RCC_CFGR, SW1);

                    while (!(READ_BIT(RCC->RCC_CR, HSIRDY))) {
                        counter++;
                        if (counter >= timeout){
                            ERROR_STATE = -1;
                            break;
                        }
                    }
                }
                CLR_BIT(RCC->RCC_CR, HSEON); // Disable HSE clock
            }
            break;

        case PLL:
            if (status == on) {
                SET_BIT(RCC->RCC_CR, PLLON); // Enable PLL clock

                while (!(READ_BIT(RCC->RCC_CR, PLLRDY))) {
                    counter++;
                    if (counter >= timeout) {
                        ERROR_STATE = -1;
                        break;
                    }
                }
            }else {
              if (RCC_CFGR_SWS0 == 1 && RCC_CFGR_SWS1 == 0) {

                    CLR_BIT(RCC->RCC_CFGR, SW0);
                    CLR_BIT(RCC->RCC_CFGR, SW1);

                    while (!(READ_BIT(RCC->RCC_CR, HSIRDY))) {
                        counter++;
                        if (counter >= timeout) {
                            ERROR_STATE = -1;
                            break;
                        }
                    }
                }
                CLR_BIT(RCC->RCC_CR, PLLON);// Disable PLL clock
            }
            break;

        default:
            ERROR_STATE = -2;
            break;
    }
}

//Setting system's clock
void RCC_setSysClk(enum clockType clkType){
  switch(clkType){
    case HSI:
      CLR_BIT(RCC->RCC_CFGR, SW0);
      CLR_BIT(RCC->RCC_CFGR, SW1);
    break;

    case HSE:
      SET_BIT(RCC->RCC_CFGR, SW0);
      CLR_BIT(RCC->RCC_CFGR, SW1);
    break;

    case PLL:
      CLR_BIT(RCC->RCC_CFGR, SW0);
      SET_BIT(RCC->RCC_CFGR, SW1);
    break;

  default:
      ERROR_STATE = -2;
     break;
  }
  errorFlag = ERROR_NONE;
}

//HSE source
void RCC_HSEConfig(enum HSESrc hseSrc){
  switch(hseSrc){
    case notDivided:
       CLR_BIT(RCC->RCC_CFGR, PLLXTPRE);

    break;

    case dividedByTwo:
        SET_BIT(RCC->RCC_CFGR, PLLXTPRE);
     break;

   default:
     ERROR_STATE = -2;
     break;
  }
  errorFlag = ERROR_NONE;
}

void RCC_PLLConfig(enum PLLClkSrc pllClkSrc, int PLLMulVal) {
    // Configure the PLL source clock
    switch(pllClkSrc){
    case halfHSI:
      CLR_BIT(RCC->RCC_CFGR, PLLSRC);
      break;

    case halfHSE:
      SET_BIT(RCC->RCC_CFGR, PLLXTPRE);
      SET_BIT(RCC->RCC_CFGR, PLLSRC);
      break;

    case fullHSE:
      CLR_BIT(RCC->RCC_CFGR, PLLXTPRE);
      SET_BIT(RCC->RCC_CFGR, PLLSRC);
      break;

    default:
      ERROR_STATE = -2;
  }
    //Clearing the PLLMUL
    RCC->RCC_CFGR &= ~((1 << PLLMUL0) | (1 << PLLMUL1) | (1 << PLLMUL2) | (1 << PLLMUL3));

    //Setting the PLL multiplier value
    if (PLLMulVal >= 2 && PLLMulVal <= 16) {
        RCC->RCC_CFGR |= ((PLLMulVal - 2) << PLLMUL0);
    } else {
        ERROR_STATE = -3;
    }
    errorFlag = ERROR_NONE;
}

void RCC_PeripheralClocks(enum Peripheral peripheral, uint8_t enable) {
    uint32_t bus, pos;

    //Determine the bus and bit position
    if (peripheral < 32) {
        bus = RCC_AHB;
        pos = peripheral;
    } else if (peripheral < 64) {
        bus = RCC_APB1;
        pos = peripheral - 32;
    } else {
        bus = RCC_APB2;
        pos = peripheral - 64;
    }

    //Enable or disable the peripheral
    switch (bus) {
        case RCC_AHB:
            if(enable){
                RCC->RCC_AHBENR |= (1 << pos); // Enable
            } else {
                RCC->RCC_AHBENR &= ~(1 << pos); // Disable
            }
            break;

        case RCC_APB1:
            if (enable){
                RCC->RCC_APB1ENR |= (1 << pos); // Enable
            } else {
                RCC->RCC_APB1ENR &= ~(1 << pos); // Disable
            }
            break;

        case RCC_APB2:
            if (enable){
                RCC->RCC_APB2ENR |= (1 << pos); // Enable
            } else {
                RCC->RCC_APB2ENR &= ~(1 << pos); // Disable
            }
            break;

        default:
            ERROR_STATE = -2; // Invalid bus type
            return;
    }

    ERROR_STATE = 0; // No error
}

//Errors handling
void errorsHandling(){
  if(ERROR_STATE == -1){
      errorFlag = ERROR_TIMEOUT;
  }else if(ERROR_STATE == -2){
      errorFlag = ERROR_INVALID_CLOCK_TYPE;
  }else if(ERROR_STATE == -3){
      errorFlag = ERROR_INVALID_PLLMUL_VALUE;
  }
  else{
      errorFlag = ERROR_NONE;
  }
}


