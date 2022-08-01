#ifndef _FLASH_H_
#define _FLASH_H_

#include "main.h"

void Flash_erase(uint32_t page);
void Flash_Write(uint64_t value, uint32_t add);
int Flash_Read(uint32_t add);

#endif

