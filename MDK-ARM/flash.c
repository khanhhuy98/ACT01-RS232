#include "flash.h"
#include "stm32g0xx_hal_flash.h"
#include "stm32g0xx.h"
void Flash_erase(uint32_t page)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.Banks = 1;
	EraseInitStruct.NbPages = 1;
	EraseInitStruct.Page = page;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	uint32_t pageerror = 0;
	HAL_FLASHEx_Erase(&EraseInitStruct, &pageerror);
	HAL_FLASH_Lock();
}
void Flash_Write(uint64_t value, uint32_t add)
{
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, add, value);
	HAL_FLASH_Lock();
}
int Flash_Read(uint32_t add)
{
	return *(__IO uint32_t *)(add);
}

