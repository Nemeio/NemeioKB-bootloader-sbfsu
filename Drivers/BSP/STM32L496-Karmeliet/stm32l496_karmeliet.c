/* Includes ------------------------------------------------------------------*/
#include "stm32l496_karmeliet.h"
#include "sfu_low_level_security.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup STM32L496_KARMELIET STM32L496_KARMELIET
  * @{
  */

/** @defgroup STM32L496_KARMELIET_Common STM32L496_KARMELIET Common
  * @{
  */

/** @defgroup STM32L496_KARMELIET_Private_Defines Private Defines
  * @{
  */

/**
 * @brief STM32L496_KARMELIET BSP Driver version number
   */
#define __STM32L496_KARMELIET_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __STM32L496_KARMELIET_BSP_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version */
#define __STM32L496_KARMELIET_BSP_VERSION_SUB2   (0x01) /*!< [15:8]  sub2 version */
#define __STM32L496_KARMELIET_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32L496_KARMELIET_BSP_VERSION            ((__STM32L496_KARMELIET_BSP_VERSION_MAIN << 24)\
                                                        |(__STM32L496_KARMELIET_BSP_VERSION_SUB1 << 16)\
                                                        |(__STM32L496_KARMELIET_BSP_VERSION_SUB2 << 8 )\
                                                        |(__STM32L496_KARMELIET_BSP_VERSION_RC))
/**
  * @}
  */


/** @defgroup STM32L496_KARMELIET_Private_Macros Private Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup STM32L496_KARMELIET_Exported_Variables Exported Variables
  * @{
  */

const RCC_OscInitTypeDef RCC_OscInitStruct_Default =
{
    .OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI,
    .HSIState = RCC_HSI_ON,
    .HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT,
    .LSIState = RCC_LSI_ON,
    .PLL.PLLState = RCC_PLL_ON,
    .PLL.PLLSource = RCC_PLLSOURCE_HSI,
    .PLL.PLLM = 2,
    .PLL.PLLN = 40,
    .PLL.PLLP = RCC_PLLP_DIV2,
    .PLL.PLLQ = RCC_PLLQ_DIV2,
    .PLL.PLLR = RCC_PLLR_DIV4,
};

/**
  * @}
  */




__weak void BSP_ErrorHandler(void)
{
  while (1);
}

/** @defgroup STM32L496G_DISCOVERY_Exported_Functions Exported Functions
  * @{
  */

/**
  * @brief  This method returns the STM32L496 DISCOVERY BSP Driver revision
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
    return __STM32L496_KARMELIET_BSP_VERSION;
}



/**
  * @brief  Configures LED GPIOs.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED_GREEN
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
    // No led installed : stubbed
}


/**
  * @brief  Unconfigures LED GPIOs.
  * @param  Led: Specifies the Led to be unconfigured.
  *   This parameter can be one of following parameters:
  *     @arg LED_GREEN
  * @retval None
  */
void BSP_LED_DeInit(Led_TypeDef Led)
{
    // No led installed : stubbed
}


/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED_GREEN
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
    // No led installed : stubbed
}


/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg LED_GREEN
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
    // No led installed : stubbed
}


/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg LED_GREEN
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
    // No led installed : stubbed
}

/**
  * @brief  Initialize Hardware to detect a force installation sequence
  * @retval None
  */
void BSP_Init_ButtonForceInstall()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE(); /* TODO Why ? */
    __HAL_RCC_GPIOE_CLK_ENABLE();


    GPIO_InitStruct.Pin = Button_LEFT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Button_LEFT_GPIO_Port, &GPIO_InitStruct);
#ifdef MANUAL_MODE_TWO_BUTTONS
    GPIO_InitStruct.Pin = Button_RIGHT_Pin;
    HAL_GPIO_Init(Button_RIGHT_GPIO_Port, &GPIO_InitStruct);
#endif
}

void BSP_DeInit_ButtonForceInstall()
{
    HAL_GPIO_DeInit(Button_LEFT_GPIO_Port, Button_LEFT_Pin);
#ifdef MANUAL_MODE_TWO_BUTTONS
    HAL_GPIO_DeInit(Button_RIGHT_GPIO_Port, Button_RIGHT_Pin);
#endif
}

/**
  * @brief  Initialize Hardware to detect a force installation sequence
  * @retval None
  */
uint8_t BSP_IsButtonForceInstall()
{
    return (HAL_GPIO_ReadPin(Button_LEFT_GPIO_Port, Button_LEFT_Pin) == GPIO_PIN_RESET
#ifdef MANUAL_MODE_TWO_BUTTONS
            && HAL_GPIO_ReadPin(Button_RIGHT_GPIO_Port, Button_RIGHT_Pin) == GPIO_PIN_RESET
#endif
			);
}

/**
  * @brief  A function to check power and boot conditions.
  * @retval None
  */
void BSP_PreBootloader_PowerChecking(void)
{
    SFU_RESET_IdTypeDef e_wakeup_source_id;

    // Check boot reason,
    SFU_LL_SECU_GetResetSources(&e_wakeup_source_id);

    //if power on due to hardware ( battery plugin or USB if battery is empty)
    // Note: SBSFU will reboot in that case because memory is not correctly initialized
    // ==> We will be in SW_Reset in that case
    if (e_wakeup_source_id == SFU_RESET_HW_RESET)
    {
        // Check button is pressed
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        __HAL_RCC_GPIOE_CLK_ENABLE();
        __HAL_RCC_PWR_CLK_ENABLE();

        GPIO_InitStruct.Pin = Button_ON_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(Button_ON_GPIO_Port, &GPIO_InitStruct);

        // If button is not pressed, please power down the board
        if (HAL_GPIO_ReadPin(Button_ON_GPIO_Port, Button_ON_Pin) == GPIO_PIN_SET)
        {
            HAL_GPIO_DeInit(Button_ON_GPIO_Port, Button_ON_Pin);
            __disable_irq();
            __disable_fault_irq();

            HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN3_LOW);

            HAL_PWR_DisableWakeUpPin( PWR_WAKEUP_PIN1 );
            HAL_PWR_DisableWakeUpPin( PWR_WAKEUP_PIN2 );
            HAL_PWR_DisableWakeUpPin( PWR_WAKEUP_PIN4 );
            HAL_PWR_DisableWakeUpPin( PWR_WAKEUP_PIN5 );

            __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF1);
            __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
            __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF3);
            __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF4);
            __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF5);

            HAL_PWREx_EnterSHUTDOWNMode();

            // cannot power off ?
            // ==> If we reboot, we are not in HW Reset so we will boot
            // ==> If we stay stuck here ... cannot boot any more
            // Solution : reboot to clean up any configuration done ... but this is an issue without solution
            HAL_NVIC_SystemReset();
        }
    }
}

/**
  * @}
  */

/** @defgroup STM32L496G_DISCOVERY_BusOperations_Functions Bus Operations Functions
  * @{
  */






/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
