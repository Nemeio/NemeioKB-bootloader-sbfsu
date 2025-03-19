/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L496_KARMELIET_H
#define __STM32L496_KARMELIET_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Define for STM32L496_KARMELIET board
  */

#define EPD_CSN_Pin GPIO_PIN_4
#define EPD_CSN_GPIO_Port GPIOA
#define Button_LEFT_Pin GPIO_PIN_7
#define Button_LEFT_GPIO_Port GPIOE
#define Button_RIGHT_Pin GPIO_PIN_8
#define Button_RIGHT_GPIO_Port GPIOE
#define Button_ON_Pin GPIO_PIN_6
#define Button_ON_GPIO_Port GPIOE
#define EPD_Reset_Pin GPIO_PIN_9
#define EPD_Reset_GPIO_Port GPIOC
#define EPD_ON_Pin GPIO_PIN_8
#define EPD_ON_GPIO_Port GPIOC
#define USB_HUB_RESET_Pin GPIO_PIN_10
#define USB_HUB_RESET_GPIO_Port GPIOA
#define VBUS_DET__RST_Pin GPIO_PIN_5
#define VBUS_DET__RST_GPIO_Port GPIOB
#define LED_CAPSLOCK_Pin GPIO_PIN_9
#define LED_CAPSLOCK_GPIO_Port GPIOE
#define LED_BT_Pin GPIO_PIN_8
#define LED_BT_GPIO_Port GPIOB

/**
 * @brief LED Types Definition
 */
typedef enum
{
    LED_STUB  = 0
}
Led_TypeDef;


/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L496_KARMELIET
  * @{
  */

/** @addtogroup STM32L496_KARMELIET_Common
  * @{
  */

/** @defgroup STM32L496_KARMELIET_Exported_Types Exported Types
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32L496_KARMELIET_Exported_Constants Exported Constants
  * @{
  */
extern const RCC_OscInitTypeDef RCC_OscInitStruct_Default;

/**
  * @}
  */

/**
  * @}
  */


/** @defgroup STM32L496_KARMELIET_Exported_Functions Exported Functions
  * @{
  */
uint32_t                BSP_GetVersion(void);
void                    BSP_LED_Init(Led_TypeDef Led);
void                    BSP_LED_DeInit(Led_TypeDef Led);
void                    BSP_LED_On(Led_TypeDef Led);
void                    BSP_LED_Off(Led_TypeDef Led);
void                    BSP_LED_Toggle(Led_TypeDef Led);

/* These __weak functions can be surcharged by application code for specific application needs */
void                    BSP_ErrorHandler(void);


void BSP_Init_ButtonForceInstall();
void BSP_DeInit_ButtonForceInstall();
uint8_t BSP_IsButtonForceInstall();


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

#ifdef __cplusplus
}
#endif

#endif /* __STM32L496_KARMELIET_H */

/*s *****END OF FILE****/
