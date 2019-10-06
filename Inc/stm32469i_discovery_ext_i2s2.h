/**
 ******************************************************************************
 * @file    stm32469i_discovery_ext_i2s2.h
 * @author  dragonman225@github.com
 * @brief   I2S2 driver for Extension connector CN12 on STM32F469I-DISCO
 ******************************************************************************
 */
#ifndef __STM32469I_DISCOVERY_EXT_I2S2_H
#define __STM32469I_DISCOVERY_EXT_I2S2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

// clang-format off
/* I2S2 peripheral */
#define EXT_I2S2                           SPI2
#define EXT_I2S2_CLK_ENABLE()              __HAL_RCC_SPI2_CLK_ENABLE()
#define EXT_I2S2_CLK_DISABLE()             __HAL_RCC_SPI2_CLK_DISABLE()
#define EXT_I2S2_MCK_CK_SD_WS_AF           GPIO_AF5_SPI2

#define EXT_I2S2_MCK_SD_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()
#define EXT_I2S2_MCK_SD_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()
#define EXT_I2S2_MCK_PIN                   GPIO_PIN_6
#define EXT_I2S2_SD_PIN                    GPIO_PIN_1
#define EXT_I2S2_MCK_SD_GPIO_PORT          GPIOC

#define EXT_I2S2_CK_WS_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define EXT_I2S2_CK_WS_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()
#define EXT_I2S2_CK_PIN                    GPIO_PIN_13
#define EXT_I2S2_WS_PIN                    GPIO_PIN_12
#define EXT_I2S2_CK_WS_GPIO_PORT           GPIOB

/* PCM1794 control pins */
#define PCM1794_RESET_ENABLE()             __HAL_RCC_GPIOC_CLK_ENABLE()
#define PCM1794_RESET_DISABLE()            __HAL_RCC_GPIOC_CLK_DISABLE()
#define PCM1794_RESET_PIN                  GPIO_PIN_13
#define PCM1794_RESET_GPIO_PORT            GPIOC

/* I2S DMA */
#define EXT_I2S2_DMAx_CLK_ENABLE()         __HAL_RCC_DMA1_CLK_ENABLE()
#define EXT_I2S2_DMAx_CLK_DISABLE()        __HAL_RCC_DMA1_CLK_DISABLE()
#define EXT_I2S2_DMAx_STREAM               DMA1_Stream4
#define EXT_I2S2_DMAx_CHANNEL              DMA_CHANNEL_0
#define EXT_I2S2_DMAx_IRQ                  DMA1_Stream4_IRQn
#define EXT_I2S2_DMAx_PERIPH_DATA_SIZE     DMA_PDATAALIGN_HALFWORD
#define EXT_I2S2_DMAx_MEM_DATA_SIZE        DMA_MDATAALIGN_WORD
#define DMA_MAX_SZE                        0xFFFF

#define EXT_I2S2_DMAx_IRQHandler           DMA1_Stream4_IRQHandler

/* Select the interrupt preemption priority for the DMA interrupt */
#define EXT_I2S2_IRQ_PREPRIO               4   /* Select the preemption priority level(0 is the highest) */

#define DMA_MAX(x)                         (((x) <= DMA_MAX_SZE)? (x):DMA_MAX_SZE)

/* Audio status definition */
#define AUDIO_OK                           ((uint8_t)0)
#define AUDIO_ERROR                        ((uint8_t)1)
#define AUDIO_TIMEOUT                      ((uint8_t)2)

uint8_t EXT_I2S2_Init(uint32_t AudioFreq);
uint8_t EXT_I2S2_Play(uint16_t* pBuffer, uint32_t Size);
uint8_t EXT_I2S2_Stop(void);
void    EXT_I2S2_SetFrequency(uint32_t AudioFreq);
uint8_t EXT_I2S2_SetMute(void);
void    EXT_I2S2_DeInit(void);

void    EXT_I2S2_MspInit(I2S_HandleTypeDef* hi2s);
void    EXT_I2S2_MspDeInit(I2S_HandleTypeDef* hi2s);
// clang-format on

#ifdef __cplusplus
}
#endif

#endif /* __STM32469I_DISCOVERY_EXT_I2S2_H */