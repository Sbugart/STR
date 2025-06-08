/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"

#include "miros.h"

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
//void PendSV_Handler(void)
//{
//  /* USER CODE BEGIN PendSV_IRQn 0 */
//
//  /* USER CODE END PendSV_IRQn 0 */
//  /* USER CODE BEGIN PendSV_IRQn 1 */
//
//  /* USER CODE END PendSV_IRQn 1 */
//}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
  rtos::OS_tick();
  __disable_irq();
  rtos::OS_sched();
  __enable_irq();
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
const uint32_t SRAM_START=0x20000000U;
const uint32_t SRAM_SIZE = (128U * 1024U); // 128KB
const uint32_t SRAM_END = ((SRAM_START) + (SRAM_SIZE));
const uint32_t STACK_START = SRAM_END;

extern "C" {
extern void Reset_Handler(void) __attribute__((weak));
void Default_Handler(void) __attribute__((weak));
}

void Default_Handler(void)
{
  while (1)
    ;
}

//redefindo outras interrupções para o tratamento padrão
void WWDG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PVD_PVM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_TAMP_LSECSS_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USB_HP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USB_LP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FDCAN1_IT0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FDCAN1_IT1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_TIM15_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_TIM16_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM17_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USBWakeUp_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM8_BRK_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM8_UP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM8_TRG_COM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM8_CC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FMC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPTIM1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM6_DAC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM7_DAC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UCPD1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void COMP1_2_3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void COMP4_5_6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void COMP7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HRTIM1_Master_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HRTIM1_TIMA_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HRTIM1_TIMB_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HRTIM1_TIMC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HRTIM1_TIMD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HRTIM1_TIME_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HRTIM1_FLT_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HRTIM1_TIMF_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CRS_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SAI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM20_BRK_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM20_UP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM20_TRG_COM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM20_CC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FPU_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C4_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C4_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FDCAN2_IT0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FDCAN2_IT1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FDCAN3_IT0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FDCAN3_IT1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RNG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPUART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMAMUX_OVR_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void QUADSPI_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel8_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Channel8_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CORDIC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FMAC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

/*tabela de interrupção*/
const uint32_t vectors[] __attribute__((section(".isr_vector"))) = {
    STACK_START,
    (uint32_t)Reset_Handler,
    (uint32_t)NMI_Handler,
    (uint32_t)HardFault_Handler,
    (uint32_t)MemManage_Handler,
    (uint32_t)BusFault_Handler,
    (uint32_t)UsageFault_Handler,
    0, // reserved
    0, // reserved
    0, // reserved
    0, // reserved
    (uint32_t)SVC_Handler,
    (uint32_t)DebugMon_Handler,
    0, // reserved
    (uint32_t)PendSV_Handler,
    (uint32_t)SysTick_Handler,
    (uint32_t)WWDG_IRQHandler,
    (uint32_t)PVD_PVM_IRQHandler,
    (uint32_t)RTC_TAMP_LSECSS_IRQHandler,
    (uint32_t)RTC_WKUP_IRQHandler,
	(uint32_t)FLASH_IRQHandler, // Flash global interrupt
    (uint32_t)RCC_IRQHandler,
    (uint32_t)EXTI0_IRQHandler,
    (uint32_t)EXTI1_IRQHandler,
    (uint32_t)EXTI2_IRQHandler,
    (uint32_t)EXTI3_IRQHandler,
    (uint32_t)EXTI4_IRQHandler,
    (uint32_t)DMA1_Channel1_IRQHandler,
    (uint32_t)DMA1_Channel2_IRQHandler,
    (uint32_t)DMA1_Channel3_IRQHandler,
    (uint32_t)DMA1_Channel4_IRQHandler,
    (uint32_t)DMA1_Channel5_IRQHandler,
    (uint32_t)DMA1_Channel6_IRQHandler,
    (uint32_t)DMA1_Channel7_IRQHandler,
    (uint32_t)ADC1_2_IRQHandler,
    (uint32_t)USB_HP_IRQHandler,
    (uint32_t)USB_LP_IRQHandler,
    (uint32_t)FDCAN1_IT0_IRQHandler,
    (uint32_t)FDCAN1_IT1_IRQHandler,
    (uint32_t)EXTI9_5_IRQHandler,
    (uint32_t)TIM1_BRK_TIM15_IRQHandler,
    (uint32_t)TIM1_UP_TIM16_IRQHandler,
    (uint32_t)TIM1_TRG_COM_TIM17_IRQHandler,
    (uint32_t)TIM1_CC_IRQHandler,
    (uint32_t)TIM2_IRQHandler,
    (uint32_t)TIM3_IRQHandler,
    (uint32_t)TIM4_IRQHandler,
    (uint32_t)I2C1_EV_IRQHandler,
    (uint32_t)I2C1_ER_IRQHandler,
    (uint32_t)I2C2_EV_IRQHandler,
    (uint32_t)I2C2_ER_IRQHandler,
    (uint32_t)SPI1_IRQHandler,
    (uint32_t)SPI2_IRQHandler,
    (uint32_t)USART1_IRQHandler,
    (uint32_t)USART2_IRQHandler,
    (uint32_t)USART3_IRQHandler,
    (uint32_t)EXTI15_10_IRQHandler,
    (uint32_t)RTC_Alarm_IRQHandler,
    (uint32_t)USBWakeUp_IRQHandler,
    (uint32_t)TIM8_BRK_IRQHandler,
    (uint32_t)TIM8_UP_IRQHandler,
    (uint32_t)TIM8_TRG_COM_IRQHandler,
    (uint32_t)TIM8_CC_IRQHandler,
    (uint32_t)ADC3_IRQHandler,
    (uint32_t)FMC_IRQHandler,
    (uint32_t)LPTIM1_IRQHandler,
    (uint32_t)TIM5_IRQHandler,
    (uint32_t)SPI3_IRQHandler,
    (uint32_t)UART4_IRQHandler,
    (uint32_t)UART5_IRQHandler,
    (uint32_t)TIM6_DAC_IRQHandler,
    (uint32_t)TIM7_DAC_IRQHandler,
    (uint32_t)DMA2_Channel1_IRQHandler,
    (uint32_t)DMA2_Channel2_IRQHandler,
    (uint32_t)DMA2_Channel3_IRQHandler,
    (uint32_t)DMA2_Channel4_IRQHandler,
	(uint32_t)DMA2_Channel5_IRQHandler,
	(uint32_t)ADC4_IRQHandler,
	(uint32_t)ADC5_IRQHandler,
	(uint32_t)UCPD1_IRQHandler,
	(uint32_t)COMP1_2_3_IRQHandler,
	(uint32_t)COMP4_5_6_IRQHandler,
	(uint32_t)COMP7_IRQHandler,
	(uint32_t)HRTIM1_Master_IRQHandler,
	(uint32_t)HRTIM1_TIMA_IRQHandler,
	(uint32_t)HRTIM1_TIMB_IRQHandler,
	(uint32_t)HRTIM1_TIMC_IRQHandler,
	(uint32_t)HRTIM1_TIMD_IRQHandler,
	(uint32_t)HRTIM1_TIME_IRQHandler,
	(uint32_t)HRTIM1_FLT_IRQHandler,
	(uint32_t)HRTIM1_TIMF_IRQHandler,
	(uint32_t)CRS_IRQHandler,
	(uint32_t)SAI1_IRQHandler,
	(uint32_t)TIM20_BRK_IRQHandler,
	(uint32_t)TIM20_UP_IRQHandler,
	(uint32_t)TIM20_TRG_COM_IRQHandler,
	(uint32_t)TIM20_CC_IRQHandler,
	(uint32_t)FPU_IRQHandler,
	(uint32_t)I2C4_EV_IRQHandler,
	(uint32_t)I2C4_ER_IRQHandler,
	(uint32_t)SPI4_IRQHandler,
	(uint32_t)0,
	(uint32_t)FDCAN2_IT0_IRQHandler,
	(uint32_t)FDCAN2_IT1_IRQHandler,
	(uint32_t)FDCAN3_IT0_IRQHandler,
	(uint32_t)FDCAN3_IT1_IRQHandler,
	(uint32_t)RNG_IRQHandler,
	(uint32_t)LPUART1_IRQHandler,
	(uint32_t)I2C3_EV_IRQHandler,
	(uint32_t)I2C3_ER_IRQHandler,
	(uint32_t)DMAMUX_OVR_IRQHandler,
	(uint32_t)QUADSPI_IRQHandler,
	(uint32_t)DMA1_Channel8_IRQHandler,
	(uint32_t)DMA2_Channel6_IRQHandler,
	(uint32_t)DMA2_Channel7_IRQHandler,
	(uint32_t)DMA2_Channel8_IRQHandler,
	(uint32_t)CORDIC_IRQHandler,
	(uint32_t)FMAC_IRQHandler,
};


/* USER CODE END 1 */
