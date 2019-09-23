/**
 ******************************************************************************
 * @file    USB_Device/AUDIO_Standalone/Src/main.c
 * @author  MCD Application Team
 * @brief   USB device AUDIO demo main file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef USBD_Device;
uint32_t playing = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
  /* STM32F469xx HAL library initialization */
  HAL_Init();

  /* Configure the system clock to 180 MHz */
  SystemClock_Config();

  /* Configure LED1, LED2, LED3 and LED4 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);

  /* Init Device Library */
  USBD_Init(&USBD_Device, &AUDIO_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_AUDIO_CLASS);

  /* Add Interface callbacks for AUDIO Class */
  USBD_AUDIO_RegisterInterface(&USBD_Device, &USBD_AUDIO_fops);

  /* Start Device Process */
  USBD_Start(&USBD_Device);

  /* play_start is set to 1 when AUDIO_CMD_START */
  while (1) {
    if (playing) {
      BSP_LED_On(LED1);
    } else {
      BSP_LED_Off(LED1);
    }
  }
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 180000000
 *            HCLK(Hz)                       = 180000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 360
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            PLL_R                          = 2
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 *         The USB clock configuration from PLLSAI:
 *            PLLSAIM                        = 8
 *            PLLSAIN                        = 384
 *            PLLSAIP                        = 8
 * @param  None
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the
     device is clocked below the maximum system frequency, to update the
     voltage scaling value regarding system frequency refer to product
     datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#if defined(USE_STM32469I_DISCO_REVA)
  RCC_OscInitStruct.PLL.PLLM = 25;
#else
  RCC_OscInitStruct.PLL.PLLM = 8;
#endif /* USE_STM32469I_DISCO_REVA */
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Activate the OverDrive to reach the 180 MHz Frequency */
  HAL_PWREx_EnableOverDrive();

  /* Select PLLSAI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CK48CLKSOURCE_PLLSAIP;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 7;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

// void TMR2_Config(uint32_t freq, uint32_t SYSFREQ)
// {
//   GPIO_InitTypeDef GPIO_InitStructure;

//   // TIM2_CH1_ETR pin (PA.15) configuration
//   GPIO_InitStructure.Pin = GPIO_PIN_15;
//   GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
//   GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
//   HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

//   TIM_Base_InitTypeDef TIM_TimeBaseStructure;
//   TIM_IC_InitTypeDef TIM_ICInitStructure;
//   TIM_OC_InitTypeDef TIM_OCInitStructure;

//   TIM_HandleTypeDef htim2;

//   htim2.Instance = TIM2;
//   htim2.Init = TIM_TimeBaseStructure;
//   htim2.Channel = TIM_CHANNEL_1;
//   htim2.State = TIM_OUTPUTSTATE_DISABLE;

//   /* Enable the TIM2 clock */
//   __HAL_RCC_TIM2_CLK_ENABLE();

//   /* Time base configuration */
//   TIM_TimeBaseStructure.Period = 0xffffffff;
//   TIM_TimeBaseStructure.Prescaler = 0;
//   TIM_TimeBaseStructure.ClockDivision = 0;
//   TIM_TimeBaseStructure.CounterMode =  TIM_COUNTERMODE_UP;
//   TIM_TimeBaseStructure.RepetitionCounter = 0;

//   //clock TIM2 via ETR pin
//   TIM_ETR_SetConfig(TIM2, TIM_ETRPRESCALER_DIV1, TIM_ETRPOLARITY_NONINVERTED, 0);
  
//   /* Enable capture*/
//   TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_RISING;
//   TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_TRC;
//   TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
//   TIM_ICInitStructure.ICFilter = 0;

//   HAL_TIM_IC_Init(&htim2);
//   TIM_ICInit(TIM2, &TIM_ICInitStructure);

//   HAL_TIMEx_RemapConfig(TIM2, TIM_TIM2_USBFS_SOF);
  
//   /* TIM2 input trigger selection */
//   TIM_SelectInputTrigger(TIM2, TIM_TS_ITR1);
//   TIM_SelectSlaveMode(TIM2, TIM_SLAVEMODE_RESET);

//   TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
//   TIM_OCInitStructure.Pulse = 0x45A1CAC0;  // I/O update pulse length 3 periods
//   TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_LOW;
//   HAL_TIM_OC_Init(&htim2);
//   TIM_OC4Init(TIM2, &TIM_OCInitStructure);

//   TIM_Cmd(TIM2, ENABLE);
// }

/**
 * @brief This function provides accurate delay (in milliseconds) based
 *        on SysTick counter flag.
 * @note This function is declared as __weak to be overwritten in case of other
 *       implementations in user file.
 * @param Delay: specifies the delay time length, in milliseconds.
 * @retval None
 */

void HAL_Delay(__IO uint32_t Delay)
{
  while (Delay) {
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
      Delay--;
    }
  }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */

  /* Infinite loop */
  while (1) {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
