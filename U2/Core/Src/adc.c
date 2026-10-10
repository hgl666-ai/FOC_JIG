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

/* USER CODE BEGIN 0 */
#include "measure.h"        /* 信号序号 / MEAS_SCAN_COUNT 的唯一来源 */

/* ── 信号 → ADC 通道号（**U2 的接法**；与 U1 不同，依据 docs/06 §2 的逆推结果）
 *
 *    信号           k          U1 通道   U2 通道（本文件）
 *    24V 母线     47/371       IN5       IN5
 *    10V 驱动     47/371       IN4       IN6     ← 与 U1 不同
 *    5V 电源       3/5         IN0       IN1     ←
 *    3.3V 电源    10/11        IN1       IN0     ←
 *    A 相          1/8         IN8       IN4     ←
 *    B 相          1/8         IN7       IN3     ←
 *    C 相          1/8         IN6       IN2     ←
 *    电流 1        1.0         IN2       IN8     ←
 *    电流 2        1.0         IN3       IN7     ←
 *
 *    ⚠ 扫描顺序必须严格等于 measure.h 的信号序号（V24,V10,V5,V33,
 *      PHA,PHB,PHC,CUR1,CUR2），最后一项才是 VREFINT —— **两片顺序一致**，
 *      只有"信号→引脚通道"不同。顺序错了不会编译报错，但 9 路读数会整体错位。 */
#define ADCCH_V24    ADC_CHANNEL_5   /* PA5 */
#define ADCCH_V10    ADC_CHANNEL_6   /* PA6 */
#define ADCCH_V5     ADC_CHANNEL_1   /* PA1 */
#define ADCCH_V33    ADC_CHANNEL_0   /* PA0 */
#define ADCCH_PH_A   ADC_CHANNEL_4   /* PA4 */
#define ADCCH_PH_B   ADC_CHANNEL_3   /* PA3 */
#define ADCCH_PH_C   ADC_CHANNEL_2   /* PA2 */
#define ADCCH_CUR1   ADC_CHANNEL_8   /* PB0 */
#define ADCCH_CUR2   ADC_CHANNEL_7   /* PA7 */

/* 扫描表：下标 = DMA 缓冲下标 = measure.h 里的 meas_id_t 顺序。
 * 采样时间与 U1 保持一致：外部通道 13.5 周期，内部基准 239.5 周期
 * （VREFINT 按数据手册要求必须给足采样时间 ≥5.1us）。 */
typedef struct {
  uint32_t channel;
  uint32_t sample_time;
} adc_scan_item_t;

static const adc_scan_item_t k_adc_scan[MEAS_SCAN_COUNT] = {
  { ADCCH_V24,   ADC_SAMPLETIME_13CYCLES_5 },  /* 0: 24V   */
  { ADCCH_V10,   ADC_SAMPLETIME_13CYCLES_5 },  /* 1: 10V   */
  { ADCCH_V5,    ADC_SAMPLETIME_13CYCLES_5 },  /* 2: 5V    */
  { ADCCH_V33,   ADC_SAMPLETIME_13CYCLES_5 },  /* 3: 3.3V  */
  { ADCCH_PH_A,  ADC_SAMPLETIME_13CYCLES_5 },  /* 4: A 相  */
  { ADCCH_PH_B,  ADC_SAMPLETIME_13CYCLES_5 },  /* 5: B 相  */
  { ADCCH_PH_C,  ADC_SAMPLETIME_13CYCLES_5 },  /* 6: C 相  */
  { ADCCH_CUR1,  ADC_SAMPLETIME_13CYCLES_5 },  /* 7: 电流1 */
  { ADCCH_CUR2,  ADC_SAMPLETIME_13CYCLES_5 },  /* 8: 电流2 */
  { ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_239CYCLES_5 } /* 9: 内部基准 */
};
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;    /* ADC1 的 DMA 句柄（adc.h 里 extern 给别的文件用） */

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  *  与 U1 完全一致：10 通道扫描（9 路信号 + VREFINT）+ 连续转换 + DMA 循环
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
    /* ADC1 -> DMA1_Channel1，循环模式、半字搬运（与 U1 一致）。
     * 同样**不**使能 DMA1_Channel1_IRQn：循环模式下 DMA 硬件自己重装计数器，
     * 软件只按 CNDTR 回卷判断"有没有新的一轮"。 */
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
