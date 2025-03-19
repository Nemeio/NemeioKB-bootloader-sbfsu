#ifndef SFU_BACKUP_REGISTERS_H
#define SFU_BACKUP_REGISTERS_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32l4xx_hal.h"

#define BKP_REG_SFU  RTC_BKP_DR19 /* Backup register used bu SFU */
#define BKP_REG_SFU_UPDATE_FROM_LOADER_MASK (1U << 0U) /* Flag to inform the application that an update has been performed from the loader */

#ifdef __cplusplus
}
#endif

#endif /* SFU_BACKUP_REGISTERS_H */