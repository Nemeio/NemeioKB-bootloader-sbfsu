#ifndef SFU_BACKUP_REGISTERS_WRITE_H
#define SFU_BACKUP_REGISTERS_WRITE_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "sfu_backup_registers.h"

void SFU_BACKUP_REG_SetFlag(uint32_t mask);
void SFU_BACKUP_REG_ClearFlag(uint32_t mask);

#ifdef __cplusplus
}
#endif

#endif /* SFU_BACKUP_REGISTERS_WRITE_H */