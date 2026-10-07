/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "adc.h"
#include "measure.h"        /* 通道/扫描顺序的唯一来源（MEAS_ADC_CH_xxx / MEAS_SCAN_COUNT） */

/* USER CODE BEGIN 0 */
/* ADC1 -> DMA1_Channel1 循环搬运（F1 上 ADC1 固定用 DMA1_CH1）
 * 说明：只用循环模式，不需要搬运完成中断——不使能 NVIC，
 *       所以也不需要在 stm32f1xx_it.c 里加 DMA1_Channel1_IRQHandler。
 *       若将来要用 XferCpltCallback，必须补上中断处理函数。 */
DMA_HandleTypeDef hdma_adc1;
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* 扫描表：下标 = DMA 缓冲下标 = measure.h 里的 meas_id_t 顺序。
 * 采样时间：外部通道 13.5 周期(1.1us@12MHz)；引脚上有 1nF，源阻抗 ~100Ω，
 * 够用。内部基准 VREFINT 按数据手册要求必须给足采样时间（≥5.1us），
 * 所以单独用 239.5 周期。 */
typedef struct {
  uint32_t channel;
  uint32_t sample_time;
} adc_scan_item_t;

static const adc_scan_item_t k_adc_scan[MEAS_SCAN_COUNT] = {
  { MEAS_ADC_CH_V24,   ADC_SAMPLETIME_13CYCLES_5 },  /* 0: 24V   */
  { MEAS_ADC_CH_V10,   ADC_SAMPLETIME_13CYCLES_5 },  /* 1: 10V   */
  { MEAS_ADC_CH_V5,    ADC_SAMPLETIME_13CYCLES_5 },  /* 2: 5V    */
  { MEAS_ADC_CH_V33,   ADC_SAMPLETIME_13CYCLES_5 },  /* 3: 3.3V  */
  { MEAS_ADC_CH_PH_A,  ADC_SAMPLETIME_13CYCLES_5 },  /* 4: A 相  */
  { MEAS_ADC_CH_PH_B,  ADC_SAMPLETIME_13CYCLES_5 },  /* 5: B 相  */
  { MEAS_ADC_CH_PH_C,  ADC_SAMPLETIME_13CYCLES_5 },  /* 6: C 相  */
  { MEAS_ADC_CH_CUR1,  ADC_SAMPLETIME_13CYCLES_5 },  /* 7: 电流1 */
  { MEAS_ADC_CH_CUR2,  ADC_SAMPLETIME_13CYCLES_5 },  /* 8: 电流2 */
  { ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_239CYCLES_5 } /* 9: 内部基准 */
};

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = MEAS_SCAN_COUNT;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channels (扫描顺序 = measure.h 的通道顺序)
  */
  /* USER CODE BEGIN ADC1_Init 2 */
  {
    uint8_t i;
    for (i = 0u; i < (uint8_t)MEAS_SCAN_COUNT; i++)
    {
      sConfig.Channel = k_adc_scan[i].channel;
      sConfig.Rank = (uint32_t)ADC_REGULAR_RANK_1 + (uint32_t)i;
      sConfig.SamplingTime = k_adc_scan[i].sample_time;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
    }
  }
  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    PB0     ------> ADC1_IN8
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC1_MspInit 1 */
    /* ADC1 -> DMA1_Channel1，循环模式、半字搬运。
     * 刻意**不**调 HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn)：
     * 循环模式下 DMA 硬件自己重装计数器，不需要软件参与，
     * 因此也不需要在 stm32f1xx_it.c 里放中断处理函数。 */
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_adc1.Instance                 = DMA1_Channel1;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);
    /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    PB0     ------> ADC1_IN8
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
