#pragma once

#include <cstdint>

extern uint8_t TestFlash[128U * 1024U];
#define FLASH_BASE (reinterpret_cast<std::uintptr_t>(TestFlash))
#define FLASH_SIZE (128U * 1024U)
#define FLASH_PAGE_SIZE 2048U
#define FLASH_TYPEERASE_PAGES 2U
#define FLASH_BANK_1 1U
#define FLASH_TYPEPROGRAM_DOUBLEWORD 0U

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
struct FLASH_EraseInitTypeDef
{
    uint32_t TypeErase;
    uint32_t Banks;
    uint32_t Page;
    uint32_t NbPages;
};

HAL_StatusTypeDef HAL_FLASH_Unlock();
HAL_StatusTypeDef HAL_FLASH_Lock();
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef* erase, uint32_t* page_error);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t data);
