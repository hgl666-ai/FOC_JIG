/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f1xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
#include "main.h"
#include "swd_config.h"   /* SWD_FREE_DEBUG_PINS：决定 PA13/PA14 当调试口还是普通 IO */
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* NOTE [2026-10-27 改] 新版治具板把 U1 的 PA13/PA14（U1-SWD / U1-SCLK）
   * **独立引出去烧 DUT** 了，所以这两颗脚要当普通 IO 用 → 必须关掉 U1 自己的
   * SW-DP（SWJ_CFG = 100）。副作用：烧 DUT 的固件不能再被 SWD 调试；
   * 想调 U1 就把 swd_config.h 的 SWD_FREE_DEBUG_PINS 置 0 重编（那时不能烧 DUT）。
   * 顺带好处：PA15/PB3/PB4 也变成普通 IO，PB4 = TIM3_CH1（B 相捕获）少一层重映射依赖。 */
#if (SWD_FREE_DEBUG_PINS != 0)
  __HAL_AFIO_REMAP_SWJ_DISABLE();
#else
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
#endif

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
