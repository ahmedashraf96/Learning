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

#define SRAM_SIZE_BYTES             (20U * 1024U)
#define SRAM_START_ADDRESS          (0x20000000U)
#define SRAM_END_ADDRESS            ((SRAM_START_ADDRESS) + (SRAM_SIZE_BYTES))
#define SRAM_STACK_START_ADDRESS    (SRAM_END_ADDRESS)

/************************************************************************************************************************************/
/*                                                            LD Symbols                                                            */
/************************************************************************************************************************************/

extern u32_t _etext;
extern u32_t _sdata;
extern u32_t _edata;
extern u32_t _sbss;
extern u32_t _ebss;

/************************************************************************************************************************************/
/*                                                         Main function declaration                                                */
/************************************************************************************************************************************/

/**
 * @brief This function is used as the code main entry point
 * 
 * @return 0 -> safe exit, otherwise report error 
 */
extern "C"
int main(void);

/************************************************************************************************************************************/
/*                                                         Cortex-Mx handlers                                                       */
/************************************************************************************************************************************/

extern "C"
{
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
}

/************************************************************************************************************************************/
/*                                                         STM32F10x handlers                                                       */
/************************************************************************************************************************************/

extern "C"
{
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
    void DMA1_Channel1_Handler(void)            __attribute__((weak, alias("Default_Handler")));
    void DMA1_Channel2_Handler(void)            __attribute__((weak, alias("Default_Handler")));
    void DMA1_Channel3_Handler(void)            __attribute__((weak, alias("Default_Handler")));
    void DMA1_Channel4_Handler(void)            __attribute__((weak, alias("Default_Handler")));
    void DMA1_Channel5_Handler(void)            __attribute__((weak, alias("Default_Handler")));
    void DMA1_Channel6_Handler(void)            __attribute__((weak, alias("Default_Handler")));
    void DMA1_Channel7_Handler(void)            __attribute__((weak, alias("Default_Handler")));
    void ADC1_2_Handler(void)                   __attribute__((weak, alias("Default_Handler")));
    void USB_HP_CAN_TX_Handler(void)            __attribute__((weak, alias("Default_Handler")));
    void USB_LP_CAN_RX0_Handler(void)           __attribute__((weak, alias("Default_Handler")));
    void CAN_RX1_Handler(void)                  __attribute__((weak, alias("Default_Handler")));
    void CAN_SCE_Handler(void)                  __attribute__((weak, alias("Default_Handler")));
    void EXTI9_5_Handler(void)                  __attribute__((weak, alias("Default_Handler")));
    void TIM1_BRK_Handler(void)                 __attribute__((weak, alias("Default_Handler")));
    void TIM1_UP_Handler(void)                  __attribute__((weak, alias("Default_Handler")));
    void TIM1_TRG_COM_Handler(void)             __attribute__((weak, alias("Default_Handler")));
    void TIM1_CC_Handler(void)                  __attribute__((weak, alias("Default_Handler")));
    void TIM2_Handler(void)                     __attribute__((weak, alias("Default_Handler")));
    void TIM3_Handler(void)                     __attribute__((weak, alias("Default_Handler")));
    void TIM4_Handler(void)                     __attribute__((weak, alias("Default_Handler")));
    void I2C1_EV_Handler(void)                  __attribute__((weak, alias("Default_Handler")));
    void I2C1_ER_Handler(void)                  __attribute__((weak, alias("Default_Handler")));
    void I2C2_EV_Handler(void)                  __attribute__((weak, alias("Default_Handler")));
    void I2C2_ER_Handler(void)                  __attribute__((weak, alias("Default_Handler")));
    void SPI1_Handler(void)                     __attribute__((weak, alias("Default_Handler")));
    void SPI2_Handler(void)                     __attribute__((weak, alias("Default_Handler")));
    void USART1_Handler(void)                   __attribute__((weak, alias("Default_Handler")));
    void USART2_Handler(void)                   __attribute__((weak, alias("Default_Handler")));
    void USART3_Handler(void)                   __attribute__((weak, alias("Default_Handler")));
    void EXTI15_10_Handler(void)                __attribute__((weak, alias("Default_Handler")));
    void RTCAlarm_Handler(void)                 __attribute__((weak, alias("Default_Handler")));
    void USBWakeup_Handler(void)                __attribute__((weak, alias("Default_Handler")));
    void TIM8_BRK_Handler(void)                 __attribute__((weak, alias("Default_Handler")));
    void TIM8_UP_Handler(void)                  __attribute__((weak, alias("Default_Handler")));
    void TIM8_TRG_COM_Handler(void)             __attribute__((weak, alias("Default_Handler")));
    void TIM8_CC_Handler(void)                  __attribute__((weak, alias("Default_Handler")));
    void ADC3_Handler(void)                     __attribute__((weak, alias("Default_Handler")));
    void FSMC_Handler(void)                     __attribute__((weak, alias("Default_Handler")));
    void SDIO_Handler(void)                     __attribute__((weak, alias("Default_Handler")));
    void TIM5_Handler(void)                     __attribute__((weak, alias("Default_Handler")));
    void SPI3_Handler(void)                     __attribute__((weak, alias("Default_Handler")));
    void UART4_Handler(void)                    __attribute__((weak, alias("Default_Handler")));
    void UART5_Handler(void)                    __attribute__((weak, alias("Default_Handler")));
    void TIM6_Handler(void)                     __attribute__((weak, alias("Default_Handler")));
    void TIM7_Handler(void)                     __attribute__((weak, alias("Default_Handler")));
    void DMA2_Channel1_Handler(void)            __attribute__((weak, alias("Default_Handler")));
    void DMA2_Channel2_Handler(void)            __attribute__((weak, alias("Default_Handler")));
    void DMA2_Channel3_Handler(void)            __attribute__((weak, alias("Default_Handler")));
    void DMA2_Channel4_5_Handler(void)          __attribute__((weak, alias("Default_Handler")));
}

/**
 * @brief This global variable is used to act as system vector table
 * 
 */
u32_t gu32Arr_systemsISRs[] __attribute__((section(".isr_vectors"))) =
{
    (u32_t)SRAM_STACK_START_ADDRESS,
    (u32_t)Reset_Handler,
    (u32_t)NMI_Handler,
    (u32_t)HardFault_Handler,
    (u32_t)MemManage_Handler,
    (u32_t)BusFault_Handler,
    (u32_t)UsageFault_Handler,
    (u32_t)0,
    (u32_t)0,
    (u32_t)0,
    (u32_t)0,
    (u32_t)SVCall_Handler,
    (u32_t)Debug_Monitor_Handler,
    (u32_t)0,
    (u32_t)PendSV_Handler,
    (u32_t)SysTick_Handler,
    (u32_t)WWDG_Handler,
    (u32_t)PVD_Handler,    
    (u32_t)TAMPER_Handler,
    (u32_t)RTC_Handler,
    (u32_t)FLASH_Handler,
    (u32_t)RCC_Handler,
    (u32_t)EXTI0_Handler,
    (u32_t)EXTI1_Handler,
    (u32_t)EXTI2_Handler,
    (u32_t)EXTI3_Handler,
    (u32_t)EXTI4_Handler,
    (u32_t)DMA1_Channel1_Handler,
    (u32_t)DMA1_Channel2_Handler,
    (u32_t)DMA1_Channel3_Handler,
    (u32_t)DMA1_Channel4_Handler,
    (u32_t)DMA1_Channel5_Handler,
    (u32_t)DMA1_Channel6_Handler,
    (u32_t)DMA1_Channel7_Handler,
    (u32_t)ADC1_2_Handler,
    (u32_t)USB_HP_CAN_TX_Handler,
    (u32_t)USB_LP_CAN_RX0_Handler,
    (u32_t)CAN_RX1_Handler,
    (u32_t)CAN_SCE_Handler,
    (u32_t)EXTI9_5_Handler,
    (u32_t)TIM1_BRK_Handler,
    (u32_t)TIM1_UP_Handler,
    (u32_t)TIM1_TRG_COM_Handler,
    (u32_t)TIM1_CC_Handler,
    (u32_t)TIM2_Handler,
    (u32_t)TIM3_Handler,
    (u32_t)TIM4_Handler,
    (u32_t)I2C1_EV_Handler,
    (u32_t)I2C1_ER_Handler,
    (u32_t)I2C2_EV_Handler,
    (u32_t)I2C2_ER_Handler,
    (u32_t)SPI1_Handler,
    (u32_t)SPI2_Handler,
    (u32_t)USART1_Handler,
    (u32_t)USART2_Handler,
    (u32_t)USART3_Handler,
    (u32_t)EXTI15_10_Handler,
    (u32_t)RTCAlarm_Handler,
    (u32_t)USBWakeup_Handler,
    (u32_t)TIM8_BRK_Handler,
    (u32_t)TIM8_UP_Handler,
    (u32_t)TIM8_TRG_COM_Handler,
    (u32_t)TIM8_CC_Handler,
    (u32_t)ADC3_Handler,
    (u32_t)FSMC_Handler,
    (u32_t)SDIO_Handler,
    (u32_t)TIM5_Handler,
    (u32_t)SPI3_Handler,
    (u32_t)UART4_Handler,
    (u32_t)UART5_Handler,
    (u32_t)TIM6_Handler,
    (u32_t)TIM7_Handler,
    (u32_t)DMA2_Channel1_Handler,
    (u32_t)DMA2_Channel2_Handler,
    (u32_t)DMA2_Channel3_Handler,
    (u32_t)DMA2_Channel4_5_Handler
};

void Reset_Handler(void)
{
    /*Local variable used to hold main function return value*/
    u8_t au8_mainReturn;

    /*Local pointer used to get the section source (FLASH) address*/
    u8_t* pu8_sectionSrc = (u8_t*)&_etext;

    /*Local pointer used to get the section destination (SRAM) address*/
    u8_t* pu8_sectionDes = (u8_t*)&_sdata;

    /*Local variable used to calculate section size*/
    u32_t au32_size = (u32_t)&_edata - (u32_t)&_sdata;

    /*Local varible used in looping operations*/
    u32_t au32_loopingVar = 0;

    /*Looping over .data section bytes*/
    for(au32_loopingVar = 0; au32_loopingVar < au32_size; au32_loopingVar++)
    {
        /*Getting .data section bytes from FLASH to SRAM*/
        *pu8_sectionDes++ = *pu8_sectionSrc++;
    }    

    /*Getting the start address of .bss section*/  
    pu8_sectionSrc = (u8_t*)&_sbss;

    /*Calculating .bss section size*/
    au32_size = (u32_t)&_ebss - (u32_t)&_sbss;

    /*Looping over .bss section bytes*/
    for(au32_loopingVar = 0; au32_loopingVar < au32_size; au32_loopingVar++)
    {
        /*Initialize .bss section bytes to zero*/
        *pu8_sectionSrc++ = 0;
    }

    /*Calling the main function*/
    au8_mainReturn = main();

    /*If the main function returns 0 report Safe exit otherwise report error*/
    if(!au8_mainReturn)
    {
        /*Report safe exit*/
    }
    else
    {
        /*Report error*/
    }

    /*Wait here forever*/
    while(1);
 
    /*Return from this function*/
    return;
}

extern "C" 
void Default_Handler(void)
{
    /*Infinite loop*/
    while(1);

    /*Return from this function*/
    return;
}
