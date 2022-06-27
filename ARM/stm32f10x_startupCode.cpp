/**
 * @file    stm32f10x_startupcode.cpp
 * @author  Ahmed Elnaqeeb (ahmed.elnaqeeb@rampush.com)
 * @brief   This file contains startup code of STM32f10x MCUs (Cortex-m3)
 * @version 1.0
 * @date    2022-06-27
 * 
 * ©2022 RAM Push copyright
 * 
 */

/************************************************************************************************************************************/
/*                                                             Includes                                                             */
/************************************************************************************************************************************/

#include "LSTD_TYPES.h"

/************************************************************************************************************************************/
/*                                                         Important macros                                                         */
/************************************************************************************************************************************/

#define SRAM_SIZE_BYTES             (96U * 1024U)
#define SRAM_START_ADDRESS          (0x20000000U)
#define SRAM_END_ADDRESS            ((SRAM_START_ADDRESS) + (SRAM_SIZE_BYTES))
#define SRAM_STACK_START_ADDRESS    (SRAM_END_ADDRESS)

/************************************************************************************************************************************/
/*                                                         Cortex-Mx handlers                                                       */
/************************************************************************************************************************************/

void Reset_Handler(void);
void NMI_Handler(void)                      __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)                __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)                __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)                 __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void)               __attribute__((weak, alias("Default_Handler")));
void SVCall_Handler(void)                   __attribute__((weak, alias("Default_Handler")));
void Debug_Monitor_Handler(void)            __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)                   __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)                  __attribute__((weak, alias("Default_Handler")));

/************************************************************************************************************************************/
/*                                                         STM32F10x handlers                                                       */
/************************************************************************************************************************************/

void WWDG_Handler(void)                     __attribute__((weak, alias("Default_Handler")));
void PVD_Handler(void)                      __attribute__((weak, alias("Default_Handler")));
void TAMPER_Handler(void)                   __attribute__((weak, alias("Default_Handler")));
void RTC_Handler(void)                      __attribute__((weak, alias("Default_Handler")));
void FLASH_Handler(void)                    __attribute__((weak, alias("Default_Handler")));
void RCC_Handler(void)                      __attribute__((weak, alias("Default_Handler")));
void EXTI0_Handler(void)                    __attribute__((weak, alias("Default_Handler")));
void EXTI1_Handler(void)                    __attribute__((weak, alias("Default_Handler")));
void EXTI2_Handler(void)                    __attribute__((weak, alias("Default_Handler")));
void EXTI3_Handler(void)                    __attribute__((weak, alias("Default_Handler")));
void EXTI4_Handler(void)                    __attribute__((weak, alias("Default_Handler")));

/**
 * @brief This global variable is used to act as system vector table
 * 
 */
u32_t gu32Arr_systemsISRs[] =
{
    (u32_t)SRAM_STACK_START_ADDRESS,
    (u32_t)Reset_Handler
};

void Reset_Handler(void)
{
    /*Return from this function*/
    return;
}

void Default_Handler(void)
{
    /*Infinite loop*/
    while(1);

    /*Return from this function*/
    return;
}