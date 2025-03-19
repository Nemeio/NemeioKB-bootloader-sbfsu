#include "sfu_backup_registers_write.h"

extern RTC_HandleTypeDef RtcHandle;

/**
  * @brief  Set a flag in backup register
  */
void SFU_BACKUP_REG_SetFlag(uint32_t mask)
{
  uint32_t value = HAL_RTCEx_BKUPRead(&RtcHandle, BKP_REG_SFU);

  HAL_RTCEx_BKUPWrite(&RtcHandle, BKP_REG_SFU, value | mask);
}

/**
  * @brief  Clear a flag in backup register
  */
void SFU_BACKUP_REG_ClearFlag(uint32_t mask)
{
  uint32_t value = HAL_RTCEx_BKUPRead(&RtcHandle, BKP_REG_SFU);

  HAL_RTCEx_BKUPWrite(&RtcHandle, BKP_REG_SFU, value & ~mask);
}
