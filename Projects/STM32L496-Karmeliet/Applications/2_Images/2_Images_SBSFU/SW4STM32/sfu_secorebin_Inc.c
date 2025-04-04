/**
  ******************************************************************************
  * @file    sfu_secorebin_Inc.c
  * @author  MCD Application Team
  * @brief   Include SECoreBin binary.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright(c) 2017 STMicroelectronics International N.V.
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
#include "string.h"

#if defined (__GNUC__)
asm(".section SE_CORE_Bin,\"a\";"
                             ".incbin \"../../../2_Images_SECoreBin/SW4STM32/STM32L496-Karmeliet_2_Images_SECoreBin/Debug/" SECORE_BIN_NAME "\";"
);
#endif 

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
