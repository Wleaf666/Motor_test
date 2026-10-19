// Host-only HAL substitute. Never include this directory in the firmware build.
#pragma once
#include <cstdint>
struct SPI_HandleTypeDef {};
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
void HAL_Delay(uint32_t);
uint32_t HAL_GetTick();
