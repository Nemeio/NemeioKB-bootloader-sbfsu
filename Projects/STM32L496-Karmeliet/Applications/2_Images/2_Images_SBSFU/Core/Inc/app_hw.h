/**
  ******************************************************************************
  * @file    app_hw.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for Secure Firmware Update hardware
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_HW_H
#define APP_HW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l496_karmeliet.h"

#define  BUTTON_INIT()      BSP_Init_ButtonForceInstall()
#define  BUTTON_PUSHED()    BSP_IsButtonForceInstall()
#define  BUTTON_DEINIT()    BSP_DeInit_ButtonForceInstall()

#ifdef __cplusplus
}
#endif

#endif /* APP_HW_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

