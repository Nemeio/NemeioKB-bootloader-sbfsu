/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @author  MCD Application Team
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
#include "stm32l4xx_hal_pcd.h"
#include "sfu_leds.h"

/* Global variables ----------------------------------------------------------*/
/* RTC handler declaration */
RTC_HandleTypeDef RtcHandle;
TIM_HandleTypeDef hTimLedTimer;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;


/* Functions Definition ------------------------------------------------------*/
/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/* Exception handlers are implemented in the various source files of sbsfu application */

/* NMI exception handler is implemented in sfu_low_level_flash_int.c file */
/* Hard Fault exception handler is implemented in sfu_boot.c file */
/* Memory Manage exception handler is implemented in sfu_boot.c file */
/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                  */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l4xx.s).                                               */
/******************************************************************************/
#ifdef SFU_TAMPER_PROTECT_ENABLE
/**
  * @brief  This function handles Tamper interrupt request.
  * @param  None
  * @retval None
  */
void TAMP_STAMP_IRQHandler(void)
{
  HAL_RTCEx_TamperTimeStampIRQHandler(&RtcHandle);
}
#endif /* SFU_TAMPER_PROTECT_ENABLE */

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&hTimLedTimer);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim != NULL && TIM3 == htim->Instance) {
		SFU_LEDs_TIM_Callback();
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
