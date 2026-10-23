#pragma once
#include <cstdint>

#define HAL_UART_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define USE_HAL_UART_REGISTER_CALLBACKS 1
constexpr uint32_t DMA_CIRCULAR = 1;
constexpr uint32_t DMA_NORMAL = 0;
constexpr uint32_t DMA_PERIPH_TO_MEMORY = 0;
constexpr uint32_t DMA_MEMORY_TO_PERIPH = 1;
constexpr uint32_t DMA_MDATAALIGN_BYTE = 0;
constexpr uint32_t DMA_PDATAALIGN_BYTE = 0;
constexpr uint32_t DMA_MINC_ENABLE = 1;
constexpr uint32_t DMA_PINC_DISABLE = 0;
constexpr uint32_t HAL_UART_ERROR_PE = 1;
constexpr uint32_t HAL_UART_ERROR_FE = 2;
constexpr uint32_t HAL_UART_ERROR_NE = 4;
constexpr uint32_t HAL_UART_ERROR_ORE = 8;
constexpr uint32_t HAL_UART_ERROR_DMA = 16;
constexpr uint32_t HAL_DMA_ERROR_NONE = 0;
constexpr uint32_t UART_FLAG_TC = 0x40U;
constexpr uint32_t RESET = 0U;
constexpr uint32_t SET = 1U;
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
struct DMA_InitTypeDef
{
    uint32_t Mode = DMA_NORMAL;
    uint32_t Direction = DMA_PERIPH_TO_MEMORY;
    uint32_t MemDataAlignment = DMA_MDATAALIGN_BYTE;
    uint32_t PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    uint32_t MemInc = DMA_MINC_ENABLE;
    uint32_t PeriphInc = DMA_PINC_DISABLE;
};
struct DMA_HandleTypeDef
{
    DMA_InitTypeDef Init{};
    uint32_t ErrorCode = 0;
    uint32_t Remaining = 0;
};
struct UART_HandleTypeDef;
using UartCallback = void (*)(UART_HandleTypeDef*);
enum HAL_UART_CallbackIDTypeDef { HAL_UART_RX_COMPLETE_CB_ID, HAL_UART_TX_COMPLETE_CB_ID, HAL_UART_ERROR_CB_ID };
struct UART_HandleTypeDef
{
    DMA_HandleTypeDef* hdmarx = nullptr;
    DMA_HandleTypeDef* hdmatx = nullptr;
    uint32_t ErrorCode = 0;
    UartCallback callbacks[3]{};
    uint32_t Flags = UART_FLAG_TC;
};
HAL_StatusTypeDef HAL_UART_RegisterCallback(UART_HandleTypeDef*, HAL_UART_CallbackIDTypeDef, UartCallback);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef*, const uint8_t*, uint16_t);
#define __HAL_UART_CLEAR_PEFLAG(uart) ((uart)->ErrorCode &= ~HAL_UART_ERROR_PE)
#define __HAL_UART_CLEAR_FEFLAG(uart) ((uart)->ErrorCode &= ~HAL_UART_ERROR_FE)
#define __HAL_UART_CLEAR_NEFLAG(uart) ((uart)->ErrorCode &= ~HAL_UART_ERROR_NE)
#define __HAL_UART_CLEAR_OREFLAG(uart) ((uart)->ErrorCode &= ~HAL_UART_ERROR_ORE)
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->Remaining)
#define __HAL_UART_GET_FLAG(uart, flag) (((uart)->Flags & (flag)) != 0U ? SET : RESET)
