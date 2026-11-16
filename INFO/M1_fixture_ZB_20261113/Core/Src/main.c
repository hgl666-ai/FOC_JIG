/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash_mode.h"
#include "usart_m.h"
#include "cdc_transport.h"
#include "log.h"

#include "gd25q128.h"

#include "data_process.h"

#include "fx_adc.h"
#include "fx_bus.h"
#include "fx_input.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---------------- main loop subtask ---------------- */
void main_loop_1000ms(void)
{
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void main_loop_2ms(void)
{
}

void main_loop_1ms(void)
{
  uart_rx_handle();
  pc_app_poll();

  fx_adc_poll();
  fx_input_poll();
  fx_bus_poll();
}

/* ---------------- main loop model ---------------- */
#define MAIN_LOOP_NUMS    3

typedef void (*task_func_t)(void);

typedef struct {
    uint32_t last_tick;
    uint32_t period;
    const uint32_t *period_var;   // if not NULL, use *period_var as period
    task_func_t task;
} task_t;

static task_t tasks[MAIN_LOOP_NUMS] = {0};

void main_loop_init(void)
{
  task_func_t task_funcs[MAIN_LOOP_NUMS] = {
    main_loop_1000ms,
    main_loop_2ms,
    main_loop_1ms,
  };
  uint32_t periods[MAIN_LOOP_NUMS] = {
    1000,
    2,
    1,
  };
  const uint32_t *period_vars[MAIN_LOOP_NUMS] = {
    NULL,
    NULL,
    NULL,
  };
  for (int i = 0; i < MAIN_LOOP_NUMS; i++) {
    tasks[i].period = periods[i];
    tasks[i].period_var = period_vars[i];
    tasks[i].task = task_funcs[i];
  }
}

void main_loop(void)
{
  uint32_t tick_now = HAL_GetTick();

  for (int i = 0; i < MAIN_LOOP_NUMS; i++) {
    uint32_t period = tasks[i].period_var ? *tasks[i].period_var : tasks[i].period;
    if ((uint32_t)(tick_now - tasks[i].last_tick) >= period) {
      tasks[i].last_tick = tick_now;
      if (tasks[i].task) {
        tasks[i].task();
      }
    }
  }
}

/* ---------------- CDC datas test ---------------- */
#if 0
void CDC_FrameHandler(const uint8_t *data, uint16_t len)
{
  CDC_Transport_Send(data, len, true);
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
#if FLASH_MODE_UART5
  UART5_Init_UserCode();      /* UART5 = DUT bootloader 链路：开 RX 中断进环形缓�? */
#else
  /* SWD 模式：DUT SWD �? PD2/PC12（与 UART5 同脚）�?�这里主动释�? UART5�?
   * 避免 UART5 �? SWD 位翻转期间被噪声触发 RX 中断、干扰烧录时序�??
   * DeInit 会关 UART5 时钟�? NVIC；关中断做这几步，防�? ISR 正在访问
   * 已经关掉时钟�? UART5 寄存器�?? */
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    (void)HAL_UART_DeInit(&huart5);
    HAL_NVIC_ClearPendingIRQ(UART5_IRQn);
    if (primask == 0u) {
      __enable_irq();
    }
  }
#endif
  CDC_Transport_Init();
  GD25Q_Init();
  main_loop_init();
  LOGI("*********** Program Start ***********");
  if (GD25Q_Test()) {
    LOGI("GD25Q Test ok");
  } else {
    LOGI("GD25Q Test Failed");
  }
  LOGI("GD25Q ID = 0x%x", GD25Q_ReadID());
  LOGI("GD25Q Err Count = %d", GD25Q_ErrCount());
  pc_app_init();

  fx_bus_init();
  fx_input_init();
  fx_adc_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    main_loop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CDC_Transport_Poll();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
