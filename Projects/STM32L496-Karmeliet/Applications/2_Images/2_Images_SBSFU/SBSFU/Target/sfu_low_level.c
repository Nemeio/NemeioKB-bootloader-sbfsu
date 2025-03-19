/**
  ******************************************************************************
  * @file    sfu_low_level.c
  * @author  MCD Application Team
  * @brief   SFU Low Level Interface module
  *          This file provides set of firmware functions to manage SFU low level
  *          interface.
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

#define SFU_LOW_LEVEL_C

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sfu_low_level.h"
#include "sfu_low_level_security.h"
#include "sfu_trace.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#ifndef SFU_TAMPER_PROTECT_ENABLE
extern RTC_HandleTypeDef RtcHandle;
#endif /* SFU_TAMPER_PROTECT_ENABLE */

/* Led parameters */
#define DISPLAY_DELAY_POWER_ON_MS	10
#define TIM_FREQ_HZ          	10000

extern TIM_HandleTypeDef hTimLedTimer;
extern USBD_DescriptorsTypeDef FS_Desc;

/* Private variables ---------------------------------------------------------*/
static CRC_HandleTypeDef    CrcHandle;
static SFU_BoolTypeDef      bUSBInited = SFU_FALSE;
static TIM_HandleTypeDef hTimGreenLed;
static TIM_HandleTypeDef hTimBlueLed;
USBD_HandleTypeDef hUsbDeviceFS;

/* Private function prototypes -----------------------------------------------*/
static void     SFU_LL_Error_Handler(void);

static void SFU_LL_Display_Init(void);
static SFU_ErrorStatus SFU_LL_PWM_Green_Blinking_Led_Init(uint32_t freq, uint8_t dutyCycle);
static SFU_ErrorStatus SFU_LL_PWM_Blue_Blinking_Led_Init(uint32_t freq, uint8_t dutyCycle);
static SFU_ErrorStatus SFU_LL_PWM_Green_Blinking_Led_DeInit();
static SFU_ErrorStatus SFU_LL_PWM_Blue_Blinking_Led_DeInit();
static SFU_ErrorStatus SFU_LL_PWM_Init(TIM_TypeDef* instance, TIM_HandleTypeDef* hTim, uint32_t channel, uint32_t freqHz, uint8_t dutyCycle);
static SFU_ErrorStatus SFU_LL_PWM_DeInit(TIM_HandleTypeDef* hTim);

/* Functions Definition ------------------------------------------------------*/
/**
  * @brief  Initialize SFU Interface.
  * @param  None
  * @retval SFU_ErrorStatus SFU_SUCCESS if successful, SFU_ERROR otherwise.
  */
SFU_ErrorStatus SFU_LL_Init(void)
{
  SFU_ErrorStatus e_ret_status = SFU_ERROR;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  SFU_LL_Display_Init();

  /* Initialize CRC */
  e_ret_status = SFU_LL_CRC_Init();

  return e_ret_status;
}

/**
  * @brief  DeInitialize SFU Interface.
  * @param  None
  * @retval SFU_ErrorStatus SFU_SUCCESS if successful, SFU_ERROR otherwise.
  */
SFU_ErrorStatus SFU_LL_DeInit(void)
{
  SFU_ErrorStatus e_ret_status = SFU_SUCCESS;

  SFU_LL_CRC_DeInit();

  // Do not deactivate all GPIO clocks as we need some for the screen
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();

  return e_ret_status;
}

/**
  * @brief  SFU IF CRC Init.
  * @param  None
  * @retval SFU_ErrorStatus SFU_SUCCESS if successful, SFU_ERROR otherwise.
  */
SFU_ErrorStatus SFU_LL_CRC_Init(void)
{
  SFU_ErrorStatus e_ret_status = SFU_ERROR;

  /* Configure the peripheral clock */
  __HAL_RCC_CRC_CLK_ENABLE();


  /* Configure CRC with default polynomial - standard configuration */
  e_ret_status = SFU_LL_CRC_Config(SFU_CRC_CONFIG_DEFAULT);

  return e_ret_status;
}

SFU_ErrorStatus SFU_LL_CRC_DeInit(void)
{
  HAL_CRC_DeInit(&CrcHandle);
  __HAL_RCC_CRC_CLK_DISABLE();

  return SFU_SUCCESS;
}

/**
  * @brief  SFU  IF CRC Configuration.
  * @param  eCRCConfg: SFU_CRC_ConfigTypeDef.
  *         This parameter can be a value of @ref SFU_CRC_ConfigTypeDef.
  * @retval SFU_ErrorStatus SFU_SUCCESS if successful, SFU_ERROR otherwise.
  */
SFU_ErrorStatus SFU_LL_CRC_Config(SFU_CRC_ConfigTypeDef eCRCConfg)
{
  SFU_ErrorStatus e_ret_status = SFU_SUCCESS;

  /* Check the parameters */
  assert_param(IS_SFU_CRC_CONF(eCRCConfg));

  /* Switch to the selected configuration */
  CrcHandle.Instance = CRC;

  /* The input data are not inverted */
  CrcHandle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;

  /* The output data are not inverted */
  CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;

  switch (eCRCConfg)
  {
    case SFU_CRC_CONFIG_DEFAULT:
      /* The Default polynomial is used */
      CrcHandle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
      /* The default init value is used */
      CrcHandle.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
      /* The input data are 32-bit long words */
      CrcHandle.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
      /* Valid parameter*/
      e_ret_status = SFU_SUCCESS;
      break;

    case SFU_CRC_CONFIG_16BIT:
      /* The CRC-16-CCIT polynomial is used */
      CrcHandle.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_DISABLE;
      CrcHandle.Init.GeneratingPolynomial    = 0x1021U;
      CrcHandle.Init.CRCLength               = CRC_POLYLENGTH_16B;
      /* The zero init value is used */
      CrcHandle.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_DISABLE;
      CrcHandle.Init.InitValue               = 0U;
      /* The input data are 8-bit long */
      CrcHandle.InputDataFormat              = CRC_INPUTDATA_FORMAT_BYTES;
      /* Valid parameter*/
      e_ret_status = SFU_SUCCESS;
      break;

    default:
      /* Invalid parameter */
      e_ret_status = SFU_ERROR;
      break;
  }

  /* Proceed to CRC Init (Correct Parameters) */
  if (e_ret_status == SFU_SUCCESS)
  {
    if (HAL_CRC_Init(&CrcHandle) != HAL_OK)
    {
      e_ret_status = SFU_ERROR;
    }
  }

  return e_ret_status;
}


/**
  * @brief  SFU IF CRC Calculate.
  * @param  pBuffer: pointer to data buffer.
  * @param  BufferLength: buffer length in bytes.
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
uint32_t SFU_LL_CRC_Calculate(uint32_t pBuffer[], uint32_t BufferLength)
{
  return HAL_CRC_Calculate(&CrcHandle, pBuffer, BufferLength);
}

/**
  * @brief SFU UART Init.
  * @param  None
  * @retval status of the Init operation
  *         SFU_ERROR : if the Init operation failed.
  *         SFU_SUCCESS : if the Init operation is successfully performed.
  */
SFU_ErrorStatus SFU_LL_UART_Init(void)
{
    SFU_ErrorStatus e_ret_status = SFU_SUCCESS;
    if (!bUSBInited)
    {

        RCC_OscInitTypeDef RCC_OscInitStruct;
        RCC_PeriphCLKInitTypeDef PeriphClkInit;
        RCC_CRSInitTypeDef RCC_CRSInitStruct;

        /**Initialize HSI48 for USB
        */
        memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
        RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        {
            /* Initialization Error */
            BSP_ErrorHandler();
        }

        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
        PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            /* Initialization Error */
            BSP_ErrorHandler();
        }

        /**Configure the main internal regulator output voltage
        */
        if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        {
            //* Initialization Error */
            BSP_ErrorHandler();
        }

        /**Configure the Systick
        */
        HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

        /**Enable the SYSCFG APB clock
        */
        __HAL_RCC_CRS_CLK_ENABLE();

        /**Configures CRS
        */
        RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
        RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
        RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
        RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
        RCC_CRSInitStruct.ErrorLimitValue = 34;
        RCC_CRSInitStruct.HSI48CalibrationValue = 32;

        HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        // PIN muxing
        HAL_GPIO_WritePin(USB_HUB_RESET_GPIO_Port, USB_HUB_RESET_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(VBUS_DET__RST_GPIO_Port, VBUS_DET__RST_Pin, GPIO_PIN_SET);

        GPIO_InitStruct.Pin = USB_HUB_RESET_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(USB_HUB_RESET_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = VBUS_DET__RST_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(VBUS_DET__RST_GPIO_Port, &GPIO_InitStruct);

        // Initialize USB
        /* Init Device Library, add supported class and start the library. */
        if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
        {
        BSP_ErrorHandler();
        }
        if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
        {
            BSP_ErrorHandler();
        }
        if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
        {
            BSP_ErrorHandler();
        }
        if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
        {
            BSP_ErrorHandler();
        }

        /* Refresh the IWDG */
        e_ret_status = SFU_LL_SECU_IWDG_Refresh();

        bUSBInited = SFU_TRUE;
    }
    return e_ret_status;
}

/**
  * @brief SFU IF UART DeInit.
  * @param  None
  * @note   The MPU prevents the access to the GPIO registers. This function may be called only after the MPU has been disabled.
  * @retval status of the Init operation
  *         SFU_ERROR : if the Init operation failed.
  *         SFU_SUCCESS : if the Init operation is successfully performed.
  */
SFU_ErrorStatus SFU_LL_UART_DeInit(void)
{
  SFU_ErrorStatus e_ret_status = SFU_ERROR;

  if (bUSBInited)
  {
      USBD_DeInit(&hUsbDeviceFS);

      HAL_GPIO_DeInit(USB_HUB_RESET_GPIO_Port, USB_HUB_RESET_Pin);
      HAL_GPIO_DeInit(VBUS_DET__RST_GPIO_Port, VBUS_DET__RST_Pin);

      bUSBInited = SFU_FALSE;
  }

  e_ret_status = SFU_SUCCESS;

  return e_ret_status;

}

/**
  * @brief  SFU IF Write data (send).
  * @param  pData: pointer to the 128bit data to write.
  * @param  DataLength: pointer to the 128bit data to write.
  * @param  pData: pointer to the 128bit data to write.
  * @param  Timeout: Timeout duration.
  * @retval status of the write operation
  *         SFU_ERROR : if the write operation is not performed
  *         SFU_SUCCESS : if the write operation is successfully performed
  */
SFU_ErrorStatus SFU_LL_UART_Transmit(uint8_t *pData, uint16_t DataLength, uint32_t Timeout)
{
  SFU_ErrorStatus e_ret_status = SFU_ERROR;
  uint32_t startTime = HAL_GetTick();
  uint8_t usbReturn = USBD_OK;

  /* Check the pointers allocation */
  if (pData == NULL)
  {
    return SFU_ERROR;
  }

  usbReturn = CDC_Transmit_FS(pData, DataLength);

  if (USBD_OK == usbReturn)
  {
      e_ret_status = SFU_SUCCESS;
      while (((USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData)->TxState != 0 && HAL_GetTick()  < startTime + Timeout)
      {
          SFU_LL_SECU_IWDG_Refresh();
      }
  }

  return e_ret_status;
}

/**
  * @brief  SFU IF Read data (receive).
  * @param  pData: pointer to the 128bit data where to store the received data.
  * @param  DataLength: the length of the data to be read in bytes.
  * @param Timeout: Timeout duration.
  * @retval status of the read operation
  *         SFU_ERROR : if the read operation is not performed
  *         SFU_SUCCESS : if the read operation is successfully performed
  */
SFU_ErrorStatus SFU_LL_UART_Receive(uint8_t *pData, uint16_t DataLength, uint32_t Timeout)
{
    SFU_ErrorStatus e_ret_status = SFU_ERROR;
    uint32_t startTime = HAL_GetTick();
    uint16_t lenRecvd = 0;


    /* Check the pointers allocation */
    if (pData == NULL )
    {
        return SFU_ERROR;
    }

    do
    {
        SFU_LL_SECU_IWDG_Refresh();
        lenRecvd = CDC_Receive_FS(pData, DataLength);
        DataLength -= lenRecvd;
        pData += lenRecvd;
    } while (DataLength > 0 && HAL_GetTick()  < startTime + Timeout);

    if (0 == DataLength)
    {
        e_ret_status = SFU_SUCCESS;
    }

    return e_ret_status;
}

/**
  * @brief  SFU HAL IF Flush.
  * @param  None.
  * @retval status of the operation.
  */
SFU_ErrorStatus SFU_LL_UART_Flush(void)
{
  return SFU_SUCCESS;
}

/**
  * @}
  */

/** @defgroup SFU_LOW_LEVEL_PWM_Functions PWM Functions
  * @{
  */

static SFU_ErrorStatus getTimerSourceClockFreq(TIM_TypeDef* instance, uint32_t* pFreq)
{
	if(NULL == instance || NULL == pFreq) {
		return SFU_ERROR;
	}

	if (TIM2 == instance ||
		TIM3 == instance ||
		TIM4 == instance ||
		TIM5 == instance ||
		TIM6 == instance ||
		TIM7 == instance) {
		*pFreq = HAL_RCC_GetPCLK1Freq();
	} else {
		*pFreq = HAL_RCC_GetPCLK2Freq();
	}

	return SFU_SUCCESS;
}

static void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
	if(timHandle->Instance==TIM1)
	{
		/**TIM1 GPIO Configuration
		PE9     ------> TIM1_CH1
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	}
	else if(timHandle->Instance==TIM4)
	{
		/**TIM4 GPIO Configuration
		PB8     ------> TIM4_CH3
		*/HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);
		GPIO_InitStruct.Pin = GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
}

static SFU_ErrorStatus SFU_LL_PWM_Green_Blinking_Led_Init(uint32_t freq, uint8_t dutyCycle)
{
	return SFU_LL_PWM_Init(TIM1, &hTimGreenLed, TIM_CHANNEL_1, freq, dutyCycle);
}

static SFU_ErrorStatus SFU_LL_PWM_Blue_Blinking_Led_Init(uint32_t freq, uint8_t dutyCycle)
{
	return SFU_LL_PWM_Init(TIM4, &hTimBlueLed, TIM_CHANNEL_3, freq, dutyCycle);
}

static SFU_ErrorStatus SFU_LL_PWM_Green_Blinking_Led_DeInit()
{
	return SFU_LL_PWM_DeInit(&hTimGreenLed);
}

static SFU_ErrorStatus SFU_LL_PWM_Blue_Blinking_Led_DeInit()
{
	return SFU_LL_PWM_DeInit(&hTimBlueLed);
}

static SFU_ErrorStatus SFU_LL_PWM_Init(TIM_TypeDef* instance, TIM_HandleTypeDef* hTim, uint32_t channel, uint32_t freqHz, uint8_t dutyCycle)
{
	uint32_t period = (TIM_FREQ_HZ / freqHz);

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	if(NULL == hTim) {
		return SFU_ERROR;
	}

	uint32_t apbFreq = 0;
	if(getTimerSourceClockFreq(instance, &apbFreq) != SFU_SUCCESS)
	{
		return SFU_ERROR;
	}
	uint32_t pulse = ((period * dutyCycle) / 100);

	hTim->Instance = instance;
	hTim->Init.Prescaler = (apbFreq / TIM_FREQ_HZ) - 1;
	hTim->Init.CounterMode = TIM_COUNTERMODE_UP;
	hTim->Init.Period = period - 1;
	hTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	hTim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(hTim) != HAL_OK) {
		SFU_LL_Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(hTim, &sClockSourceConfig) != HAL_OK) {
		SFU_LL_Error_Handler();
	}

	if (HAL_TIM_PWM_Init(hTim) != HAL_OK) {
		SFU_LL_Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(hTim, &sMasterConfig)
			!= HAL_OK) {
		SFU_LL_Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(hTim, &sConfigOC, channel) != HAL_OK) {
		SFU_LL_Error_Handler();
	}

	HAL_TIM_MspPostInit(hTim);

	return SFU_SUCCESS;
}

static SFU_ErrorStatus SFU_LL_PWM_DeInit(TIM_HandleTypeDef* hTim)
{
	SFU_ErrorStatus ret = SFU_ERROR;

	if(NULL != hTim) {
	  HAL_TIM_PWM_DeInit(hTim);
	  HAL_TIM_Base_DeInit(hTim);
	}

	return ret;
}

SFU_ErrorStatus SFU_LL_PWM_Green_Blinking_Led_Start()
{
	return HAL_OK == HAL_TIM_PWM_Start(&hTimGreenLed, TIM_CHANNEL_1) ? SFU_SUCCESS : SFU_ERROR;
}

SFU_ErrorStatus SFU_LL_PWM_Blue_Blinking_Led_Start()
{
	return HAL_OK == HAL_TIM_PWM_Start(&hTimBlueLed, TIM_CHANNEL_3) ? SFU_SUCCESS : SFU_ERROR;
}

SFU_ErrorStatus SFU_LL_PWM_Green_Blinking_Led_Stop()
{
	return HAL_OK == HAL_TIM_PWM_Stop(&hTimGreenLed, TIM_CHANNEL_1) ? SFU_SUCCESS : SFU_ERROR;
}

SFU_ErrorStatus SFU_LL_PWM_Blue_Blinking_Led_Stop()
{
	return HAL_OK == HAL_TIM_PWM_Stop(&hTimBlueLed, TIM_CHANNEL_3) ? SFU_SUCCESS : SFU_ERROR;
}

SFU_ErrorStatus SFU_LL_TIM_Led_Init(uint32_t periodMs)
{
	TIM_TypeDef* instance = TIM3;
	uint32_t period = TIM_FREQ_HZ * periodMs / 1000;
	uint32_t apbFreq = 0;
	if(getTimerSourceClockFreq(instance, &apbFreq) != SFU_SUCCESS) {
		return SFU_ERROR;
	}

	hTimLedTimer.Instance = instance;

	hTimLedTimer.Init.Period = period - 1;
	hTimLedTimer.Init.Prescaler = (apbFreq / TIM_FREQ_HZ) - 1;
	hTimLedTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	hTimLedTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
	hTimLedTimer.Init.RepetitionCounter = 0;
	hTimLedTimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	return HAL_OK == HAL_TIM_Base_Init(&hTimLedTimer) ? SFU_SUCCESS : SFU_ERROR;
}

SFU_ErrorStatus SFU_LL_TIM_Led_DeInit()
{
	return HAL_OK == HAL_TIM_Base_DeInit(&hTimLedTimer) ? SFU_SUCCESS : SFU_ERROR;
}

SFU_ErrorStatus SFU_LL_TIM_Led_Start()
{
	// UIF flag needs to be cleared before starting the timer
	// to avoid TIM_IT_UPDATE interrupt firing immediately
	__HAL_TIM_CLEAR_FLAG(&hTimLedTimer, TIM_SR_UIF);

	return HAL_OK == HAL_TIM_Base_Start_IT(&hTimLedTimer) ? SFU_SUCCESS : SFU_ERROR;
}

SFU_ErrorStatus SFU_LL_TIM_Led_Stop() {
	return HAL_OK == HAL_TIM_Base_Stop_IT(&hTimLedTimer) ? SFU_SUCCESS : SFU_ERROR;
}

SFU_ErrorStatus SFU_LL_Leds_Init(uint32_t blinkFreq, uint8_t blinkDutyCycle)
{
	SFU_ErrorStatus ret = SFU_LL_PWM_Green_Blinking_Led_Init(blinkFreq, blinkDutyCycle);

	if(SFU_SUCCESS == ret) {
		ret = SFU_LL_PWM_Blue_Blinking_Led_Init(blinkFreq, blinkDutyCycle);
	}

	return ret;
}

SFU_ErrorStatus SFU_LL_Leds_DeInit()
{
	HAL_TIM_PWM_Stop(&hTimGreenLed, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&hTimBlueLed, TIM_CHANNEL_3);
	SFU_LL_PWM_Blue_Blinking_Led_DeInit();
	SFU_LL_PWM_Green_Blinking_Led_DeInit();

	return SFU_SUCCESS;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM1)
  {
	__HAL_RCC_TIM1_CLK_ENABLE();
  }
  else if(htim_base->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
  else if(htim_base->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_9);
  }
  else if(htim_base->Instance==TIM3) {
    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  }
  else if(htim_base->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);
  }

}

/**
  * @}
  */

/** @defgroup SFU_LOW_LEVEL_SRAM_Functions SRAM Functions
  * @{
  */

/**
  * @brief  Erase Secure Boot area of SRAM
  * @param  None.
  * @retvat void
  */
void SFU_LL_SB_SRAM_Erase(void)
{
  uint32_t *pRam;

  for (pRam = (uint32_t *)SFU_SB_RAM_BASE; pRam < (uint32_t *)SFU_SB_RAM_END; pRam++)
  {
    *pRam = 0U;
  }
}

/**
  * @}
  */

/** @defgroup SFU_LOW_LEVEL_MSP_Functions MSP Functions
  * @{
  */

#ifndef SFU_TAMPER_PROTECT_ENABLE
/**
  * @brief SFU IF RTC Init.
  * @param  None
  * @retval status of the Init operation
  *         SFU_ERROR : if the Init operation failed.
  *         SFU_SUCCESS : if the Init operation is successfully performed.
  */
SFU_ErrorStatus SFU_LL_RTC_Init(void)
{
  SFU_ErrorStatus e_ret_status = SFU_ERROR;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follows:
  - Hour Format    = Format 24
  - Asynch Prediv  = Value according to source clock
  - Synch Prediv   = Value according to source clock
  - OutPut         = Output Disable
  - OutPutPolarity = High Polarity
  - OutPutType     = Open Drain */
  RtcHandle.Instance            = RTC;
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

  if (HAL_RTC_Init(&RtcHandle) == HAL_OK)
  {
    e_ret_status = SFU_SUCCESS;
  }

  return e_ret_status;
}

/**
  * @brief SFU IF RTC DeInit.
  * @param  None
  * @retval status of the Init operation
  *         SFU_ERROR : if the Init operation failed.
  *         SFU_SUCCESS : if the Init operation is successfully performed.
  */
SFU_ErrorStatus SFU_LL_RTC_DeInit(void)
{
  SFU_ErrorStatus e_ret_status = SFU_ERROR;

  /*
    * ADD SRC CODE HERE
    */

  e_ret_status = SFU_SUCCESS;

  return e_ret_status;

}

#endif /*SFU_TAMPER_PROTECT_ENABLE*/

/**
  * @brief SFU IF RTC MSP Initialization
  *        This function configures the hardware resources used in this example
  * @param hrtc: RTC handle pointer.
  * @note  Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select
  *        the RTC clock source; in this case the Backup domain will be reset in
  *        order to modify the RTC Clock source, as consequence RTC registers (including
  *        the backup registers) and RCC_BDCR register are set to their reset values.
  * @retval None
  */
void SFU_LL_RTC_MspInit(RTC_HandleTypeDef *hrtc)
{
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  UNUSED(hrtc);
  /*-1- Enables access to the backup domain */
  /* To enable access on RTC registers */
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_PWR_EnableBkUpAccess();

  /*-2- Configure LSE/LSI as RTC clock source */
#ifdef RTC_CLOCK_SOURCE_LSE

  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    SFU_LL_Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    SFU_LL_Error_Handler();
  }
#elif defined (RTC_CLOCK_SOURCE_LSI)
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    SFU_LL_Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    SFU_LL_Error_Handler();
  }
#else
#error Please select the RTC Clock source inside the main.h file
#endif /*RTC_CLOCK_SOURCE_LSE*/

  /*-3- Enable RTC peripheral Clocks */
  /* Enable RTC Clock */
  __HAL_RCC_RTC_ENABLE();
#ifdef SFU_TAMPER_PROTECT_ENABLE
  /*-4- Configure the NVIC for RTC Tamper */
  HAL_NVIC_SetPriority(TAMP_STAMP_IRQn, 0x0FU, 0U);
  HAL_NVIC_EnableIRQ(TAMP_STAMP_IRQn);
#endif /* SFU_TAMPER_PROTECT_ENABLE */
}

/**
  * @brief RTC MSP De-Initialization
  *        This function frees the hardware resources used in in SFU application:
  *          - Disable the Peripheral's clock
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void SFU_LL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc)
{
  UNUSED(hrtc);

  __HAL_RCC_RTC_DISABLE();
}

/**
  * @brief UART MSP Initialization
  *        This function configures the hardware resources used in SFU application.
  * @param huart: UART handle pointer
  * @retval None
  */
void SFU_LL_UART_MspInit(UART_HandleTypeDef *huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (huart->Instance == SFU_UART)
  {
    /* Peripheral Clock Enable */
    SFU_UART_CLK_ENABLE();

    /* GPIO Ports Clock Enable */
    SFU_UART_TX_GPIO_CLK_ENABLE();
    SFU_UART_RX_GPIO_CLK_ENABLE();

    /*Configure GPIO pins : SFU_UART_TX_Pin  */
    GPIO_InitStruct.Pin = SFU_UART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = SFU_UART_TX_AF;
    HAL_GPIO_Init(SFU_UART_TX_GPIO_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins : SFU_UART_RX_Pin  */
    GPIO_InitStruct.Pin = SFU_UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = SFU_UART_RX_AF;
    HAL_GPIO_Init(SFU_UART_RX_GPIO_PORT, &GPIO_InitStruct);

  }

}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in SFU application:
  *          - Disable the Peripheral's clock
  * @param huart: UART handle pointer
  * @retval None
  */
void SFU_LL_UART_MspDeInit(UART_HandleTypeDef *huart)
{

  if (huart->Instance == SFU_UART)
  {
    /* Peripheral clock disable */
    SFU_UART_CLK_DISABLE();

    /* GPIO DeInit*/
    HAL_GPIO_DeInit(SFU_UART_TX_GPIO_PORT, SFU_UART_TX_PIN);
    HAL_GPIO_DeInit(SFU_UART_RX_GPIO_PORT, SFU_UART_RX_PIN);

  }


}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void SFU_LL_Error_Handler(void)
{
  /*
    * ADD SRC CODE HERE
    */

  while (1 == 1)
  {
    ;
  }
}

static void SFU_LL_Display_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = EPD_CSN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPD_CSN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = EPD_Reset_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPD_Reset_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = EPD_ON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPD_ON_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(EPD_CSN_GPIO_Port, EPD_CSN_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(EPD_Reset_GPIO_Port, EPD_Reset_Pin, GPIO_PIN_RESET);
	HAL_Delay(DISPLAY_DELAY_POWER_ON_MS);
	HAL_GPIO_WritePin(EPD_ON_GPIO_Port, EPD_ON_Pin, GPIO_PIN_SET);
	HAL_Delay(DISPLAY_DELAY_POWER_ON_MS);
	HAL_GPIO_WritePin(EPD_Reset_GPIO_Port, EPD_Reset_Pin, GPIO_PIN_SET);
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
