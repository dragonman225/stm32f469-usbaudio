/**
 ******************************************************************************
 * @file    stm32469i_discovery_ext_i2s2.h
 * @author  dragonman225@github.com
 * @brief   I2S2 driver for Extension connector CN12 on STM32F469I-DISCO
 ******************************************************************************
 */

#include "stm32469i_discovery_audio.h"
#include "stm32f4xx_ll_dma.h"

#include "stm32469i_discovery_ext_i2s2.h"

I2S_HandleTypeDef haudio_out_i2s;

static uint8_t I2Sx_Init(uint32_t AudioFreq);
static void I2Sx_DeInit(void);

uint8_t EXT_I2S2_Init(uint32_t AudioFreq)
{
  uint8_t ret = AUDIO_OK;

  I2Sx_DeInit();

  BSP_AUDIO_OUT_ClockConfig(NULL, AudioFreq, NULL);

  haudio_out_i2s.Instance = EXT_I2S2;
  EXT_I2S2_MspInit(&haudio_out_i2s);
  I2Sx_Init(AudioFreq);

  return ret;
}

/**
 * @param  pBuffer
 * @param  Size  Number of 16-bit data length.
 */
uint8_t EXT_I2S2_Play(uint16_t* pBuffer, uint32_t Size)
{
  uint8_t ret = AUDIO_OK;

  if (HAL_I2S_Transmit_DMA(&haudio_out_i2s, pBuffer, DMA_MAX(Size)) != HAL_OK) {
    ret = AUDIO_ERROR;
  }

  return ret;
}

uint8_t EXT_I2S2_Stop()
{
  uint8_t ret = AUDIO_OK;

  if (HAL_I2S_DMAStop(&haudio_out_i2s) != HAL_OK) {
    ret = AUDIO_ERROR;
  }

  return ret;
}

void EXT_I2S2_SetFrequency(uint32_t AudioFreq)
{
  BSP_AUDIO_OUT_ClockConfig(NULL, AudioFreq, NULL);

  __HAL_I2S_DISABLE(&haudio_out_i2s);

  haudio_out_i2s.Init.AudioFreq = AudioFreq;
  HAL_I2S_Init(&haudio_out_i2s);

  __HAL_I2S_ENABLE(&haudio_out_i2s);
}

void EXT_I2S2_DeInit(void)
{
  I2Sx_DeInit();
}

__weak void EXT_I2S2_MspInit(I2S_HandleTypeDef* hi2s)
{
  static DMA_HandleTypeDef hdma_i2s_tx;
  GPIO_InitTypeDef gpio_init_structure;

  /* PCM1794 */
  PCM1794_RESET_ENABLE();
  gpio_init_structure.Pin = PCM1794_RESET_PIN;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(PCM1794_RESET_GPIO_PORT, &gpio_init_structure);
  HAL_GPIO_WritePin(PCM1794_RESET_GPIO_PORT, PCM1794_RESET_PIN, GPIO_PIN_SET);

  /* I2S */
  EXT_I2S2_CLK_ENABLE();

  __HAL_RCC_GPIOI_CLK_ENABLE();
  EXT_I2S2_MCK_SD_ENABLE();
  EXT_I2S2_CK_WS_ENABLE();

  gpio_init_structure.Pin = GPIO_PIN_2;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Alternate = GPIO_AF6_I2S2ext;
  HAL_GPIO_Init(GPIOI, &gpio_init_structure);

  gpio_init_structure.Pin = EXT_I2S2_MCK_PIN | EXT_I2S2_SD_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Alternate = EXT_I2S2_MCK_CK_SD_WS_AF;
  HAL_GPIO_Init(EXT_I2S2_MCK_SD_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = EXT_I2S2_CK_PIN | EXT_I2S2_WS_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Alternate = EXT_I2S2_MCK_CK_SD_WS_AF;
  HAL_GPIO_Init(EXT_I2S2_CK_WS_GPIO_PORT, &gpio_init_structure);

  /* DMA */
  EXT_I2S2_DMAx_CLK_ENABLE();

  if (hi2s->Instance == EXT_I2S2) {
    hdma_i2s_tx.Init.Channel = EXT_I2S2_DMAx_CHANNEL;
    hdma_i2s_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2s_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2s_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2s_tx.Init.PeriphDataAlignment = EXT_I2S2_DMAx_PERIPH_DATA_SIZE;
    hdma_i2s_tx.Init.MemDataAlignment = EXT_I2S2_DMAx_MEM_DATA_SIZE;
    hdma_i2s_tx.Init.Mode = DMA_CIRCULAR;
    hdma_i2s_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2s_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_i2s_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2s_tx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_i2s_tx.Init.PeriphBurst = DMA_MBURST_SINGLE;

    hdma_i2s_tx.Instance = EXT_I2S2_DMAx_STREAM;

    __HAL_LINKDMA(hi2s, hdmatx, hdma_i2s_tx);

    HAL_DMA_DeInit(&hdma_i2s_tx);

    HAL_DMA_Init(&hdma_i2s_tx);
  }

  HAL_NVIC_SetPriority(EXT_I2S2_DMAx_IRQ, EXT_I2S2_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(EXT_I2S2_DMAx_IRQ);
}

__weak void EXT_I2S2_MspDeInit(I2S_HandleTypeDef* hi2s)
{
  GPIO_InitTypeDef gpio_init_structure;

  HAL_NVIC_DisableIRQ(EXT_I2S2_DMAx_IRQ);

  if (hi2s->Instance == EXT_I2S2) {
    HAL_DMA_DeInit(hi2s->hdmatx);
  }

  __HAL_I2S_DISABLE(hi2s);

  HAL_GPIO_WritePin(PCM1794_RESET_GPIO_PORT, PCM1794_RESET_PIN, GPIO_PIN_RESET);

  gpio_init_structure.Pin = GPIO_PIN_2;
  HAL_GPIO_DeInit(GPIOI, gpio_init_structure.Pin);

  gpio_init_structure.Pin = EXT_I2S2_MCK_PIN | EXT_I2S2_SD_PIN;
  HAL_GPIO_DeInit(EXT_I2S2_MCK_SD_GPIO_PORT, gpio_init_structure.Pin);

  gpio_init_structure.Pin = EXT_I2S2_CK_PIN | EXT_I2S2_WS_PIN;
  HAL_GPIO_DeInit(EXT_I2S2_CK_WS_GPIO_PORT, gpio_init_structure.Pin);

  gpio_init_structure.Pin = PCM1794_RESET_PIN;
  HAL_GPIO_DeInit(PCM1794_RESET_GPIO_PORT, gpio_init_structure.Pin);

  EXT_I2S2_CLK_DISABLE();
}

static uint8_t I2Sx_Init(uint32_t AudioFreq)
{
  uint8_t ret = AUDIO_OK;

  haudio_out_i2s.Instance = EXT_I2S2;

  __HAL_I2S_DISABLE(&haudio_out_i2s);

  haudio_out_i2s.Init.AudioFreq = AudioFreq;
  haudio_out_i2s.Init.ClockSource = I2S_CLOCK_PLL;
  haudio_out_i2s.Init.CPOL = I2S_CPOL_LOW;
  haudio_out_i2s.Init.DataFormat = I2S_DATAFORMAT_24B;
  haudio_out_i2s.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  haudio_out_i2s.Init.Mode = I2S_MODE_MASTER_TX;
  haudio_out_i2s.Init.Standard = I2S_STANDARD_LSB;
  haudio_out_i2s.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;

  if (HAL_I2S_Init(&haudio_out_i2s) != HAL_OK) {
    ret = AUDIO_ERROR;
  }

  __HAL_I2S_ENABLE(&haudio_out_i2s);

  return ret;
}

static void I2Sx_DeInit(void)
{
  haudio_out_i2s.Instance = EXT_I2S2;

  __HAL_I2S_DISABLE(&haudio_out_i2s);

  HAL_I2S_DeInit(&haudio_out_i2s);
}