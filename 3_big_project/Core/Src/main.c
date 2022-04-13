/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "queue.h"
#include "string.h"
#include "timers.h"

// OLED
#include "oled/oled.h"
#include "oled/gfx.h"
// Clock ecternal i2c RTC and OLED
#include "clock/i2c_scanner.h"
#include "clock/DS3231.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
volatile uint8_t user_data;						// For write data from UART

// Command
typedef struct
{
	uint8_t payload[10];				// Data bytes of the command
	uint8_t len;						// command length
}command_t;

// Application states
typedef enum
{
	sMainMenu = 0,
	sLedEffect,
	sRtcMenu,
	sRtcTimeConfig,
	sRtcDateConfig,
	sRtcReport,
}satte_t;
satte_t curr_state = sMainMenu;			// Set current state as sMainMenu

void process_command(command_t *cmd);
int extract_command(command_t *cmd);

const char *msg_inv = "/// Invalid option ///\n\r\n\r";

// LEDs redefinition
#define LED1 LD4_Pin
#define LED2 LD3_Pin
#define LED3 LD5_Pin
#define LED4 LD6_Pin

// Software timer handlers
TimerHandle_t handler_led_timer[4];

void led_effect_callback(TimerHandle_t xTimer);

// Encoder for clock ///////////////////////////
int32_t currCounter = 0;
int32_t prevCounter = 0;
int klick = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

I2S_HandleTypeDef hi2s3;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for menu_task */
osThreadId_t menu_taskHandle;
const osThreadAttr_t menu_task_attributes = {
  .name = "menu_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for led_task */
osThreadId_t led_taskHandle;
const osThreadAttr_t led_task_attributes = {
  .name = "led_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rtc_task */
osThreadId_t rtc_taskHandle;
const osThreadAttr_t rtc_task_attributes = {
  .name = "rtc_task",
  .stack_size = 800 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for print_task */
osThreadId_t print_taskHandle;
const osThreadAttr_t print_task_attributes = {
  .name = "print_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for cmd_handl_task */
osThreadId_t cmd_handl_taskHandle;
const osThreadAttr_t cmd_handl_task_attributes = {
  .name = "cmd_handl_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OLED_RTC */
osThreadId_t OLED_RTCHandle;
const osThreadAttr_t OLED_RTC_attributes = {
  .name = "OLED_RTC",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for input_Queue */
osMessageQueueId_t input_QueueHandle;
const osMessageQueueAttr_t input_Queue_attributes = {
  .name = "input_Queue"
};
/* Definitions for print_Queue */
osMessageQueueId_t print_QueueHandle;
const osMessageQueueAttr_t print_Queue_attributes = {
  .name = "print_Queue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void *argument);
void start_menu_task(void *argument);
void start_led_task(void *argument);
void start_rtc_task(void *argument);
void start_print(void *argument);
void start_cmd_handl(void *argument);
void StartOLED_RTC(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */

//  osTimerStart(test_timerHandle, 100);
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of input_Queue */
  input_QueueHandle = osMessageQueueNew (10, sizeof(char*), &input_Queue_attributes);

  /* creation of print_Queue */
  print_QueueHandle = osMessageQueueNew (10, sizeof(char*), &print_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of menu_task */
  menu_taskHandle = osThreadNew(start_menu_task, NULL, &menu_task_attributes);

  /* creation of led_task */
  led_taskHandle = osThreadNew(start_led_task, NULL, &led_task_attributes);

  /* creation of rtc_task */
  rtc_taskHandle = osThreadNew(start_rtc_task, NULL, &rtc_task_attributes);

  /* creation of print_task */
  print_taskHandle = osThreadNew(start_print, NULL, &print_task_attributes);

  /* creation of cmd_handl_task */
  cmd_handl_taskHandle = osThreadNew(start_cmd_handl, NULL, &cmd_handl_task_attributes);

  /* creation of OLED_RTC */
  OLED_RTCHandle = osThreadNew(StartOLED_RTC, NULL, &OLED_RTC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  // Create software timers (Created manually)  FOr LEDs
   handler_led_timer[0] = xTimerCreate("led_timer_1", pdMS_TO_TICKS(100), pdTRUE, (void*)1, led_effect_callback);
   handler_led_timer[1] = xTimerCreate("led_timer_2", pdMS_TO_TICKS(100), pdTRUE, (void*)2, led_effect_callback);
   handler_led_timer[2] = xTimerCreate("led_timer_3", pdMS_TO_TICKS(100), pdTRUE, (void*)3, led_effect_callback);
   handler_led_timer[3] = xTimerCreate("led_timer_4", pdMS_TO_TICKS(100), pdTRUE, (void*)4, led_effect_callback);

   // For write data from UART
   HAL_UART_Receive_IT(&huart2, &user_data , 1);			// Turn on (start) receive one char in interrupt mode



  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8400-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 100-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : encoder_button_Pin */
  GPIO_InitStruct.Pin = encoder_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(encoder_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/////////////////////////////////////////////////////////////////////////////
// Receive one char from UART. This function called by UART interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t dummy = 0;													// Buffer for extract '\n' sign
	int peek_buff = 0;

	if(!xQueueIsQueueFullFromISR(input_QueueHandle))
	{
		xQueueSendFromISR(input_QueueHandle, (void*)&user_data, NULL);		// Enqueue data byte
	}
	else 																	// Queue is full
	{
		if(user_data == '\r')												// Check, is user_data has '\n' sign?
		{
			xQueueReceiveFromISR(input_QueueHandle, (void*)&dummy, NULL );	// Delete '\n' sign from queue
			xQueueSendFromISR(input_QueueHandle, (void*)&user_data, NULL);  // Save user_data on the place of '\n'
		}
	}
	// Send notification to command task if user_data == '\n'
	if(user_data == '\r')
	{
		xTaskNotifyFromISR(cmd_handl_taskHandle, 0, eNoAction, NULL);		// Send notify to start_com_handl task
		//xTaskNotify(test_taskHandle, 0, eNoAction);

	}

	//HAL_UART_Receive_IT(&huart2, &user_data , 1);							// Enable receive data over UART again
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&user_data , 1);

	//HAL_GPIO_TogglePin(GPIOD, LED4);	// LED Blink for test  BLUE LED
}
/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//			LEDs
void led_effect_stop(void)
{
	for(int  i = 0; i < 4; i++)									// Stop all timer
	{
		xTimerStop(handler_led_timer[i], portMAX_DELAY);
	}
}
/////////////////////////////////////////////////////////////////////////////
void led_effect (uint8_t effect)
{
	led_effect_stop();											// Stop current led effect
	xTimerStart(handler_led_timer[effect-1], portMAX_DELAY);	// Start needed led timer
}
/////////////////////////////////////////////////////////////////////////////
void turn_off_all_leds(void)
{
	HAL_GPIO_WritePin(GPIOD, LED1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED4, GPIO_PIN_RESET);
}
/////////////////////////////////////////////////////////////////////////////
void turn_on_all_leds(void)
{
	HAL_GPIO_WritePin(GPIOD, LED1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LED2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LED3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LED4, GPIO_PIN_SET);
}
/////////////////////////////////////////////////////////////////////////////
void turn_on_even_leds(void)
{
	HAL_GPIO_WritePin(GPIOD, LED1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LED3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED4, GPIO_PIN_SET);
}
/////////////////////////////////////////////////////////////////////////////
void turn_on_odd_leds(void)
{
	HAL_GPIO_WritePin(GPIOD, LED1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LED2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, LED4, GPIO_PIN_RESET);
}
/////////////////////////////////////////////////////////////////////////////
void LED_control(int value)
{
	for(int i = 0; i < 4; i++)
	{
		HAL_GPIO_WritePin(GPIOD, (LED1 << i), ((value >> i)& 0x1));
	}
}
/////////////////////////////////////////////////////////////////////////////
void LED_effect1(void)
{
	static int flag = 1;
	(flag ^= 1) ? turn_off_all_leds() : turn_on_all_leds();			// Toggle LEDs
}
/////////////////////////////////////////////////////////////////////////////
void LED_effect2(void)
{
	static int flag = 1;
	(flag ^= 1) ? turn_on_even_leds() : turn_on_odd_leds();
}
/////////////////////////////////////////////////////////////////////////////
void LED_effect3(void)
{
	static int i = 0;
	LED_control (0x1 << (i++ % 4));  //  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<, ?????????????????????????????????????????????
}
/////////////////////////////////////////////////////////////////////////////
void LED_effect4(void)
{
	static int i = 0;
	LED_control(0x08 >> (i++ % 4));  //  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<, ?????????????????????????????????????????????
}
/////////////////////////////////////////////////////////////////////////////
void led_effect_callback(TimerHandle_t xTimer)
{
	int id;
	id = (uint32_t) pvTimerGetTimerID( xTimer );

	switch(id)
	{
		case 1:
			LED_effect1();
			break;
		case 2:
			LED_effect2();
			break;
		case 3:
			LED_effect3();
			break;
		case 4:
			LED_effect4();
	}
}
/////////////////////////////////////////////////////////////////////////////
// Function depends on selected menu chose(notify) needed task
void process_command(command_t *cmd)
{
	extract_command(cmd);

	switch(curr_state)
	{
		case sMainMenu:
			xTaskNotify(menu_taskHandle,(uint32_t*) cmd, eSetValueWithOverwrite);
			break;

		case sLedEffect:
	 		xTaskNotify(led_taskHandle, (uint32_t*) cmd, eSetValueWithOverwrite);
	 		break;

	 	case sRtcMenu:
	 	case sRtcTimeConfig:
	 	case sRtcDateConfig:
	 	case sRtcReport:
	 		xTaskNotify(rtc_taskHandle, (uint32_t*) cmd, eSetValueWithOverwrite);
	 		break;
	 }
}
/////////////////////////////////////////////////////////////////////////////
// Extract every char byte from input_QueueHandle into cmd struct
int extract_command(command_t *cmd)
{
	uint8_t item;
	BaseType_t status;

	status = uxQueueMessagesWaiting(input_QueueHandle);			// Waiting data on the queue
	if(!status)													// If no any messages on the queue (exit from where)
	{
		return -1;
	}

	uint8_t i = 0;
	do{
		status = xQueueReceive(input_QueueHandle, &item, 0);
		if(status == pdTRUE)
		{
			cmd -> payload[i++] = item;
		}
	}while(item != '\r');    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	cmd -> payload[i - 1] = '\0';		// add '\0' sign to the end
	cmd -> len = i - 1;					// Save length in struct

	return 0;
}
/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//			RTC
void show_time_date(void)
{
	static char showtime[40];
	static char showdate[40];

	RTC_DateTypeDef rtc_date;
	RTC_TimeTypeDef rtc_time;

	static char *time = showtime;
	static char *date = showdate;

	memset(&rtc_time,0, sizeof(rtc_time));
	memset(&rtc_date, 0, sizeof(rtc_date));

	// Get time
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	// Get date
	HAL_RTC_GetTime(&hrtc, &rtc_date, RTC_FORMAT_BIN);

	char *format;
	format = (rtc_time.TimeFormat == RTC_HOURFORMAT12_AM) ? "AM" : "PM";

	// Display time and data
	sprintf((char*)showtime, "%s: \t%02d:%02d:%02d [%s]", "\n Current Time&date", rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds, format);
	xQueueSend(print_QueueHandle, &time, portMAX_DELAY);		// Send to UART

	sprintf((char*)showdate, "\t%02d-%02d-%02d\n\r", rtc_date.Month, rtc_date.Date, 2000 + rtc_date.Year);
	xQueueSend(print_QueueHandle, &date, portMAX_DELAY);
}
/////////////////////////////////////////////////////////////////////////////
void rtc_configure_time(RTC_TimeTypeDef *time)
{
	time -> TimeFormat = RTC_HOURFORMAT12_AM;
//	time -> DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//	time -> StoreOperation = RTC_STOREOPERATION_RESET;

	HAL_RTC_SetTime(&hrtc, time, RTC_FORMAT_BIN);
}
/////////////////////////////////////////////////////////////////////////////
void rtc_configure_date(RTC_TimeTypeDef *date)
{
	HAL_RTC_SetDate(&hrtc, date, RTC_FORMAT_BIN);
}
/////////////////////////////////////////////////////////////////////////////
// Convert two char into two digits
uint8_t getnumber(uint8_t *p, int len)
{
	int value;
	if(len > 1)
	{
		value = (((p[0] - 48)*10) + (p[1] - 48));		// Convert two chars into digits
	}
	else
	{
		value = p[0] - 48;
	}

	return value;
}
/////////////////////////////////////////////////////////////////////////////
int validate_rtc_information(RTC_TimeTypeDef *time, RTC_DateTypeDef *date)
{
	// Validate hours, minutes and seconds
	if(((time -> Hours ) > 12) || ((time -> Hours < 0)))
	{
		return -1;
	}
	else if(((time -> Minutes > 59) || ((time -> Minutes  < 0))))
	{
		return -1;
	}
	else if(((time -> Seconds > 59) || ((time -> Seconds  < 0))))
	{
		return -1;
	}

	// Validate date, week day, year, month
	else if((date -> Date < 1 ) || (date -> Date > 31))
	{
		return -1;
	}
	else if((date -> WeekDay < 1 ) || (date -> WeekDay > 7))
	{
		return -1;
	}
	else if(date -> Year > 99 )
	{
		return -1;
	}
	else if((date -> Month < 1 ) || (date -> Month > 12))
	{
		return -1;
	}
}
/////////////////////////////////////////////////////////////////////////////

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

  for(;;)
  {
	  osDelay(100);
	  // Test LED blink
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//	  osDelay(100);
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//	  osDelay(900);


  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_menu_task */
/**
* @brief Function implementing the menu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_menu_task */
void start_menu_task(void *argument)
{
  /* USER CODE BEGIN start_menu_task */
  /* Infinite loop */

	uint32_t cmd_addr;
	command_t *cmd;			// Create object of command_t
	int option;
	const char* msg_manu = "=======================\n\r"
						   "|         MENU          |\n\r"
						   "========================\n\r"
						   "LED effect       ----> 0\n\r"
						   "Date and time    ----> 1\n\r"
						   "Exit             ----> 2\n\r"
						   "Enter your choice here: \n\r";

	while(1)
	{
		int status = 0;
		status = xQueueSend(print_QueueHandle, &msg_manu, portMAX_DELAY);		// Send data in print
		if(status != pdPASS)
		{
			// ERROR
			int ggg =999;
		}

		//xQueueSendToBack(print_QueueHandle, &msg_manu, portMAX_DELAY);
		xTaskNotifyWait(0,0,&cmd_addr,portMAX_DELAY);					// Waiting for selected menu (waiting the choise) (from 'process_command' function)
		cmd = (command_t*)cmd_addr;										// If number selected menu are selected, save it

		if(cmd->len == 1)												// Checking, must be one number, not more
		{
			option = cmd -> payload[0] - 48;							// Convert from char to number, and write it on struct

			switch (option)
			{
				case 0:													// If selected LED menu
					curr_state = sLedEffect;
					xTaskNotify(led_taskHandle, 0 ,eNoAction);
					break;

				case 1:
					curr_state = sRtcMenu;								// If selected RTC menu
					xTaskNotify(rtc_taskHandle, 0, eNoAction);
					break;

				case 2:		// Implement EXIT   (Return to main menu and print it)
					//////////////////////////////////////////////////
					curr_state = sMainMenu;
					xTaskNotify(menu_taskHandle, 0 ,eNoAction);
					/////////////////////////////////////////////////
					break;

				default:															// If input sign uncorrect
					xQueueSend(print_QueueHandle, &msg_inv, portMAX_DELAY);			// Print message: /// Invalid option ///
					continue;														// Return to while(1)
			}
		}
		else															// Invalid entry (entered more than one char)
		{
			xQueueSend(print_QueueHandle, &msg_inv, portMAX_DELAY);		// Print message: /// Invalid option ///
			continue;													// Return to while(1)
		}

		// Wait to run again when some other task notifies.
		// After notify return to "while(1)" main loop again
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

	}
  /* USER CODE END start_menu_task */
}

/* USER CODE BEGIN Header_start_led_task */
/**
* @brief Function implementing the led_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_led_task */
void start_led_task(void *argument)
{
  /* USER CODE BEGIN start_led_task */
  /* Infinite loop */


//	for(;;)
//	{
//		osDelay(100);
//	}
	uint32_t cmd_addr;
	command_t *cmd;
	const char* msg_led = "========================\n\r"
						  "|       LED effect      |\n\r"
						  "========================\n\r"
						  "(none, e1, e2, e3, e4, on, off)  \n\r"
						  "Enter your choice here : \n\r";

	while(1)
	{
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);					  	// Wait for notification (selected LEDs effect)
		xQueueSend(print_QueueHandle, &msg_led, portMAX_DELAY);			// Send data to print (Print LED menu)

		xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);				// Wait for LED command
		cmd = (command_t*) cmd_addr;

		if(cmd -> len <= 4)											    // Check input command (max input size must be less then 4)
		{
			// Select LED effect
			if( ! strcmp((char*)cmd->payload, "none" ))
			{
				led_effect_stop();
			}
			else if (! strcmp((char*)cmd -> payload, "e1"))
			{
				led_effect(1);
			}
			else if (! strcmp((char*)cmd -> payload, "e2"))
			{
				led_effect(2);
			}
			else if (! strcmp((char*)cmd -> payload, "e3"))
			{
				led_effect(3);
			}
			else if (! strcmp((char*)cmd -> payload, "e4"))
			{
				led_effect(4);
			}
			else if (! strcmp((char*)cmd -> payload, "on"))			// Work
			{
				turn_on_all_leds();
			}
			else if (! strcmp((char*)cmd -> payload, "off"))		// Work
			{
				turn_off_all_leds();
			}
			else
			{
				xQueueSend(print_QueueHandle, &msg_inv, portMAX_DELAY);				// Print invalid massage
			}
		}
		else
		{
			xQueueSend(print_QueueHandle, &msg_inv, portMAX_DELAY);					// Print invalid massage
		}

		curr_state = sMainMenu;								// Return to mai menu
		xTaskNotify(menu_taskHandle, 0, eNoAction);			// Notify menu task
	}
  /* USER CODE END start_led_task */
}

/* USER CODE BEGIN Header_start_rtc_task */
/**
* @brief Function implementing the rtc_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_rtc_task */
void start_rtc_task(void *argument)
{
  /* USER CODE BEGIN start_rtc_task */
  /* Infinite loop */

	const char* msg_rtc1 = "========================\n\r"
						   "|          RTC         |\n\r"
						   "========================\n\r"
						   "Configure Time   ----> 0\n\r"
						   "Configure Date   ----> 1\n\r"
						   "Enable reporting ----> 2\n\r"
			               "Exit             ----> 3\n\r"
			  	  	  	   "Enter your choice here : \n\r";

	const char *msg_rtc_hh = "Enter hour(1-12):";
	const char *msg_rtc_mm = "Enter minutes(0-59):";
	const char *msg_rtc_ss = "Enter seconds(0-59):";

	const char *msg_rtc_dd  = "Enter date(1-31):";
	const char *msg_rtc_mo  ="Enter month(1-12):";
	const char *msg_rtc_dow  = "Enter day(1-7 sun:1):";
	const char *msg_rtc_yr  = "Enter year(0-99):";

	const char *msg_conf = "Configuration successful\n";
	const char *msg_rtc_report = "Enable time&date reporting(y/n)?: ";

	uint32_t cmd_addr;
	command_t *cmd;

	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;

	//date.Date = 9;

	uint8_t menu_code; 					// For RTC menu
	static int rtc_state = 0;

	#define HH_CONFIG	0
	#define MM_CONFIG	1
	#define SS_CONFIG	2

	#define DATE_CONFIG 	0
	#define MONTH_CONFIG	1
	#define YEAR_CONFIG		2
	#define DAY_CONFIG		3

	while(1)
	{
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);								// Notify wait (wait till someone notifies)
		xQueueSend(print_QueueHandle, &msg_rtc1, portMAX_DELAY);			    // Print the menu
		show_time_date();														// Print the current date and time information
		//osDelay(100);


		//xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
		//xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);		 			// Wait for command notification (Notify wait)
		while(curr_state != sMainMenu)
		{
			///////////////////////////////////////////////
//			xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);					  	// Wait for notification (selected LEDs effect)
//			//xQueueSend(print_QueueHandle, &msg_led, portMAX_DELAY);			// Send data to print (Print LED menu)
//
//			xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);				// Wait for LED command
//			cmd = (command_t*) cmd_addr;
//
//			if(cmd -> len <= 4)											    // Check input command (max input size must be less then 4)
//					{

			/////////////////////////////////////////////
			int ret_status = xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);		 			// Waiting for command notification (Notify wait)
			// Return 0x410908 and  cmd_addr = 0x410908
			cmd = (command_t*)cmd_addr;


			//int test_var = cmd -> len;		 	// For test   HARD FOULT <<<<<<<<<<<<<<<<<<<<<<<<<

			switch(curr_state)
			{
				case sRtcMenu:{
					if((cmd -> len) == 1)			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
					{
						menu_code = cmd -> payload[0] - 48;
						switch(menu_code)
						{
						case 0:
							curr_state = sRtcTimeConfig;
							xQueueSend(print_QueueHandle, &msg_rtc_hh, portMAX_DELAY);
							break;

						case 1:
							curr_state = sRtcDateConfig;
							xQueueSend(print_QueueHandle, &msg_rtc_dd, portMAX_DELAY);
							break;

						case 2:
							curr_state = sRtcReport;
							xQueueSend(print_QueueHandle, &msg_rtc_report, portMAX_DELAY);
							break;

						case 3:
							curr_state = sMainMenu;
							break;
						default:
							curr_state = sMainMenu;
							xQueueSend(print_QueueHandle, &msg_inv, portMAX_DELAY);
						}
					}
					else
					{
						curr_state = sMainMenu;			// Go back to the main manu
						xQueueSend(print_QueueHandle, &msg_inv, portMAX_DELAY);		// Send invalid message
					}
					break;}

				case sRtcTimeConfig:{
					//  get hh, mm, ss infor and configure RTC
					//rtc_configure_time();
					// take care of invalid entries
					switch (rtc_state)
					{
						case HH_CONFIG:{
							uint8_t hour = getnumber(cmd -> payload, cmd -> len);
							time.Hours = hour;
							rtc_state = MM_CONFIG;
							xQueueSend(print_QueueHandle, &msg_rtc_mm, portMAX_DELAY);
							break;}				// back to: while(curr_state != sMainMenu)

						case MM_CONFIG:{
							uint8_t minute = getnumber(cmd -> payload, cmd -> len);
							time.Minutes = minute;
							rtc_state = SS_CONFIG;
							xQueueSend(print_QueueHandle, &msg_rtc_ss, portMAX_DELAY);
							break;}
						case SS_CONFIG:{
							uint8_t second = getnumber(cmd -> payload, cmd -> len);
							time.Seconds = second;

							if(!validate_rtc_information(&time, NULL))
							{
								// If input data is correct
								rtc_configure_time(&time);
								xQueueSend(print_QueueHandle, &msg_conf, portMAX_DELAY);
								show_time_date();
							}
							else
							{
								// If Input data isn't correct
								xQueueSend(print_QueueHandle, &msg_inv, portMAX_DELAY);		// Send invalid message
							}
							curr_state = sMainMenu;			// Back to main menu
							rtc_state = 0;					// Set first: case HH_CONFIG:{
							break;}
					}

					curr_state = sMainMenu;			// Go back to the main manu
					break;}

				case sRtcDateConfig:{
					switch (rtc_state)
					{
						case DATE_CONFIG:{
							uint8_t d = getnumber(cmd -> payload, cmd -> len);
							date.Date = d;
							rtc_state = DAY_CONFIG;
							xQueueSend(print_QueueHandle, &msg_rtc_mo, portMAX_DELAY);
							break;}

						case DAY_CONFIG:{
							uint8_t d = getnumber(cmd -> payload, cmd -> len);
							date.Year = d;
							rtc_state = MONTH_CONFIG;
							xQueueSend(print_QueueHandle, &msg_rtc_yr, portMAX_DELAY);
							break;}

						case MONTH_CONFIG:{
							uint8_t m = getnumber(cmd -> payload, cmd -> len);
							date.Month = m;
							rtc_state = YEAR_CONFIG;
							xQueueSend(print_QueueHandle, &msg_rtc_yr, portMAX_DELAY);
							break;}

						case YEAR_CONFIG:{
							uint8_t y = getnumber(cmd -> payload, cmd -> len);
							date.Year = y;

							if(!validate_rtc_information(NULL, &date))
							{
								rtc_configure_date(&date);
								xQueueSend(print_QueueHandle, &msg_conf, portMAX_DELAY);
								show_time_date();
							}
							else
							{
								xQueueSend(print_QueueHandle, &msg_inv, portMAX_DELAY);
							}


							break;}




					}
					/*TODO : get date, month, day , year info and configure RTC */
					//rtc_configure_date();
					/*TODO: take care of invalid entries */



					curr_state = sMainMenu;			// Go back to the main manu
					break;}

				case sRtcReport:{
					/*TODO: enable or disable RTC current time reporting over ITM printf */
					break;}

				}// switch end
//			}


		} //while end
		xTaskNotify(menu_taskHandle, 0, eNoAction);		// Notify menu task
	}
  /* USER CODE END start_rtc_task */
}

/* USER CODE BEGIN Header_start_print */
/**
* @brief Function implementing the print_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_print */
void start_print(void *argument)
{
  /* USER CODE BEGIN start_print */
  /* Infinite loop */
	uint32_t *msg;

	while(1)
	{
		int status = 0;
		status = xQueueReceive(print_QueueHandle, &msg, portMAX_DELAY);
		if(status != pdPASS)
		{
			// ERROR
			int ggg =999;
		}
	    HAL_UART_Transmit(&huart2,(uint8_t*)msg, strlen((char*)msg), HAL_MAX_DELAY);					// Doesen't work =(
	    int ggg = 888;

	//HAL_UART_Transmit(&huart2,(uint8_t*)msg, strlen((char*)msg), HAL_MAX_DELAY);  // Original <<<<<<<<<<<<<< Hard Foult
  }
  /* USER CODE END start_print */
}

/* USER CODE BEGIN Header_start_cmd_handl */
/**
* @brief Function implementing the cmd_handl_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_cmd_handl */
void start_cmd_handl(void *argument)
{
  /* USER CODE BEGIN start_cmd_handl */
  /* Infinite loop */

	BaseType_t ret;
	command_t cmd;			// Create command variable

	for(;;)
	{
		// Waiting on notify from HAL_UART_RxCpltCallback
		ret = xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
		if(ret == pdTRUE)
		{
			process_command(&cmd);
		}
	}
  /* USER CODE END start_cmd_handl */
}

/* USER CODE BEGIN Header_StartOLED_RTC */
/**
* @brief Function implementing the OLED_RTC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLED_RTC */
void StartOLED_RTC(void *argument)
{
  /* USER CODE BEGIN StartOLED_RTC */
  /* Infinite loop */

	osDelay(1000);

		// For resd time
		char time[20] = {0};
		char date[40] = {0};
		char time_buf[10] = {0};
		char time_buf_2[10] = {0};

		uint8_t seconds = 0;
		uint8_t minutes = 0;
		uint8_t hours = 0;
		uint8_t day = 0;
		uint8_t date_day = 0;
		uint8_t mounth = 0;
		uint8_t year = 0;
		uint8_t status = 9;
		//

		oled_init();
		oled_update();
		ds3231_I2C_init();

		// Encoder
		HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
		int32_t prevCounter = 0;
		//

		for(;;)
		{
			switch (klick)
			{
				case 0:					// Read time and data from DS3231

					strcat(time_buf, "TIME");
					graphics_text(40, 10, 3, time_buf);
					oled_update();
					osDelay(2000);
					clear();
					oled_update();

					while(klick == 0)
					{
						memset(time, 0, sizeof(time));
						memset(date, 0, sizeof(date));
						memset(time_buf, 0, sizeof(time_buf));
						memset(time_buf_2, 0, sizeof(time_buf_2));

						// 1. Read time from RTS
						// Red status (Detect DS3231)
						uint8_t buff= 0;
						status = HAL_I2C_Mem_Read(&hi2c3, (uint16_t)DS3231_I2C_ADDRESS<<1,(uint16_t)0, (uint16_t) 1, &buff, (uint16_t) 1,(uint32_t) 1000);
						if(status != HAL_OK )								// If DS3231 doesen'e detect
						{
							clear();
							while(status != HAL_OK)							// If ERROR
							{
								strcat(time_buf, "RTC ERROR");
								graphics_text(8, 5, 3, time_buf);
								oled_update();
								memset(time_buf, 0, sizeof(time_buf));
								osDelay(300);

								invert_rectangle(0, 0, 128, 32);
								oled_update();
								osDelay(300);

								clear();
								oled_update();

								status = HAL_I2C_Mem_Read(&hi2c3, (uint16_t)DS3231_I2C_ADDRESS<<1,(uint16_t)0, (uint16_t) 1, &buff, (uint16_t) 1,(uint32_t) 1000);
							}
							clear();
						}

						else	// If all DS3231 detected, read time and date
						{
							ds3231_read(DS3231_REGISTER_SECONDS_DEFAULT, &seconds);
							ds3231_read(DS3231_REGISTER_MINUTES_DEFAULT, &minutes);
							ds3231_read(DS3231_REGISTER_HOURS_DEFAULT, &hours);

							ds3231_read(DS3231_REGISTER_DAY_OF_WEEK_DEFAULT, &day);
							ds3231_read(DS3231_REGISTER_DATE_DEFAULT, &date_day);
							ds3231_read(DS3231_REGISTER_MONTH_DEFAULT, &mounth);
							ds3231_read(DS3231_REGISTER_YEAR_DEFAULT, &year);

							// Convert in string
							// Print minutes on OLED
							if(hours < 10)
							{
								memset(time_buf, 0, sizeof(time_buf));
								sprintf(time_buf, "%c", '0');
								sprintf(time_buf_2, "%d", hours);
								strcat(time_buf, time_buf_2);
								strcat(time, time_buf);
								strcat(time, ":");
							}
							else
							{
								sprintf(time_buf, "%d", hours);
								strcat(time, time_buf);
								strcat(time, ":");
								memset(time_buf, 0, sizeof(time_buf));
							}

							// Print minutes on OLED
							if(minutes < 10)
							{
								memset(time_buf, 0, sizeof(time_buf));
								sprintf(time_buf, "%c", '0');
								sprintf(time_buf_2, "%d", minutes);
								strcat(time_buf, time_buf_2);
								strcat(time, time_buf);
								strcat(time, ":");
							}
							else
							{
								sprintf(time_buf, "%d", minutes);
								strcat(time, time_buf);
								strcat(time, ":");
								memset(time_buf, 0, sizeof(time_buf));
							}

							// Print seconds on OLED
							if(seconds == 0)
							{
								clear();
								oled_update();
							}
							if(seconds < 10)
							{
								memset(time_buf, 0, sizeof(time_buf));
								sprintf(time_buf, "%c", '0');
								sprintf(time_buf_2, "%d", seconds);
								strcat(time_buf, time_buf_2);
								strcat(time, time_buf);
							}
							else
							{
								sprintf(time_buf, "%d", seconds);
								strcat(time, time_buf);
								memset(time_buf, 0, sizeof(time_buf));
							}

							uint8_t second_line = seconds*2;
							line_h(5, second_line, 19, 2, add);
	//						line_h(uint8_t x0, uint8_t x1, uint8_t y0, uint8_t width, uint8_t mode);
							//invert_rectangle(5, 15, second_line, 5);

							// Print date
							sprintf(time_buf, "%d", date_day);
							strcat(date, time_buf);
							strcat(date, ".");
							memset(time_buf, 0, sizeof(time_buf));

							sprintf(time_buf, "%d", mounth);
							strcat(date, time_buf);
							strcat(date, ".");
							memset(time_buf, 0, sizeof(time_buf));

							sprintf(time_buf, "%d", year);
							strcat(date, "20");
							strcat(date, time_buf);
							memset(time_buf, 0, sizeof(time_buf));

							// day
							switch (day)
							{
								case 1:
									strcat(date, " Monday");
									break;
								case 2:
									strcat(date, " Tuesday");
									break;
								case 3:
									strcat(date, " Wednesday");
									break;
								case 4:
									strcat(date, " Thursday");
									break;
								case 5:
									strcat(date, " Friday");
									break;
								case 6:
									strcat(date, " Saturday");
									break;
								case 7:
									strcat(date, " Sunday");
									break;
							}

							graphics_text(40, 0, 3, time);
							graphics_text(0, 24, 2, date);
							oled_update();

							osDelay(1000);
						}
					}
			  		break;


				case 1:
					// Set yer
					graphics_text(0, 0, 1, "   SET:");
					graphics_text(0, 8, 1, "YEAR   ");
					oled_update();

					__HAL_TIM_SET_COUNTER(&htim1, 0);								// Start count encoder from 0

					while(klick == 1)
					{
						currCounter = __HAL_TIM_GET_COUNTER(&htim1);
						currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

						if(currCounter != prevCounter)
						{
							prevCounter = currCounter;
							if(currCounter > 100)									// Encoder count from 0 to 100
							{
								__HAL_TIM_SET_COUNTER(&htim1, 0);
								currCounter = 0;
							}
							if(currCounter < 0)
							{
								__HAL_TIM_SET_COUNTER(&htim1, 0);
								currCounter = 0;
							}

							graphics_text(0, 16, 1, "           ");
							oled_update();

							sprintf(time_buf, "%d", currCounter);
							graphics_text(0, 16, 1, time_buf);
							oled_update();
							memset(time_buf, 0, sizeof(time_buf));
						}
					}
					if(klick == 2)
					{
						// write data
						ds3231_set(DS3231_REGISTER_YEAR_DEFAULT, &prevCounter);

						graphics_text(0, 16, 1, "                 ");
						oled_update();

						graphics_text(0, 16, 1, "installed");
						oled_update();

						osDelay(800);

						graphics_text(0, 16, 1, "                 ");
						oled_update();

						klick = 3;
					}
					break;

				case 3:
					// set month
					graphics_text(0, 8, 1, "MONTH");
					oled_update();

					__HAL_TIM_SET_COUNTER(&htim1, 1);								// Start count encoder from 1
					currCounter = 1;

					while(klick == 3)
					{
						currCounter = __HAL_TIM_GET_COUNTER(&htim1);
						currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

						if(currCounter != prevCounter)
						{
							prevCounter = currCounter;

							if(currCounter > 12)
							{
								__HAL_TIM_SET_COUNTER(&htim1, 1);
								currCounter = 1;
							}

							if(currCounter < 1)
							{
								__HAL_TIM_SET_COUNTER(&htim1, 1);
								currCounter = 1;
							}


							graphics_text(0, 16, 1, "           ");
							oled_update();

							sprintf(time_buf, "%d", currCounter);
							graphics_text(0, 16, 1, time_buf);
							oled_update();
							memset(time_buf, 0, sizeof(time_buf));

						}
					}
					if(klick == 4)
					{
						if((currCounter < 1) || (currCounter > 12))
						{
							__HAL_TIM_SET_COUNTER(&htim1, 1);
							prevCounter = 1;
						}

						// write data
						ds3231_set(DS3231_REGISTER_MONTH_DEFAULT, &prevCounter);

						graphics_text(0, 16, 1, "                 ");
						oled_update();

						graphics_text(0, 16, 1, "installed");
						oled_update();

						osDelay(800);

						graphics_text(0, 16, 1, "                 ");
						oled_update();

						klick = 5;
					}

					break;

				case 5:
					// Set date

					graphics_text(0, 8, 1, "              ");
					oled_update();
					graphics_text(0, 8, 1, "DATE");
					oled_update();

					__HAL_TIM_SET_COUNTER(&htim1, 1);								// Start count encoder from 1

					while(klick == 5)
					{
						currCounter = __HAL_TIM_GET_COUNTER(&htim1);
						currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

						if(currCounter != prevCounter)
						{
							prevCounter = currCounter;

							if(currCounter > 31)
							{
								__HAL_TIM_SET_COUNTER(&htim1, 1);					// Encoder count from 1 to 32
								currCounter = 1;
							}
							if(currCounter < 1)
							{
								__HAL_TIM_SET_COUNTER(&htim1, 1);					// Encoder count from 1 to 32
								currCounter = 1;
							}

							graphics_text(0, 16, 1, "           ");
							oled_update();

							sprintf(time_buf, "%d", currCounter);
							graphics_text(0, 16, 1, time_buf);
							oled_update();
							memset(time_buf, 0, sizeof(time_buf));

						}
					}
					if(klick == 6)
					{
						if((currCounter < 1) || (currCounter > 32))
						{
							__HAL_TIM_SET_COUNTER(&htim1, 1);
							prevCounter = 1;
						}

						// write data
						ds3231_set(DS3231_REGISTER_DATE_DEFAULT, &prevCounter);

						graphics_text(0, 16, 1, "                 ");
						oled_update();

						graphics_text(0, 16, 1, "installed");
						oled_update();

						osDelay(800);

						graphics_text(0, 16, 1, "                 ");
						oled_update();

						klick = 7;
					}

					break;

				case 7:
					// Set day of week

					graphics_text(0, 8, 1, "              ");
					oled_update();
					graphics_text(0, 8, 1, "DAY");
					oled_update();

					__HAL_TIM_SET_COUNTER(&htim1, 1);

					while(klick == 7)
					{
						currCounter = __HAL_TIM_GET_COUNTER(&htim1);
						currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

						if(currCounter != prevCounter)
						{
							prevCounter = currCounter;
							if(currCounter > 7)
							{
								__HAL_TIM_SET_COUNTER(&htim1, 1);
								currCounter = 1;
							}
							if(currCounter < 1)
							{
								__HAL_TIM_SET_COUNTER(&htim1, 1);
								currCounter = 1;
							}

							graphics_text(0, 16, 1, "           ");
							oled_update();

							sprintf(time_buf, "%d", currCounter);
							graphics_text(0, 16, 1, time_buf);
							oled_update();
							memset(time_buf, 0, sizeof(time_buf));

						}
					}
					if(klick == 8)
					{
						if((currCounter < 1) || (currCounter > 7))
						{
							__HAL_TIM_SET_COUNTER(&htim1, 1);
							prevCounter = 1;
						}

						// write data
						ds3231_set(DS3231_REGISTER_DAY_OF_WEEK_DEFAULT, &prevCounter);

						graphics_text(0, 16, 1, "                 ");
						oled_update();

						graphics_text(0, 16, 1, "installed");
						oled_update();

						osDelay(800);

						graphics_text(0, 16, 1, "                 ");
						oled_update();

						klick = 9;
					}
					break;

				case 9:
					// Set hour

					graphics_text(0, 8, 1, "              ");
					oled_update();
					graphics_text(0, 8, 1, "HOUR");
					oled_update();

					__HAL_TIM_SET_COUNTER(&htim1, 0);

					while(klick == 9)
					{
						currCounter = __HAL_TIM_GET_COUNTER(&htim1);
						currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

						if(currCounter != prevCounter)
						{
							prevCounter = currCounter;
							if(currCounter > 23)
							{
								__HAL_TIM_SET_COUNTER(&htim1, 0);
								currCounter = 0;
							}
							if(currCounter < 0)
							{
								__HAL_TIM_SET_COUNTER(&htim1, 0);
								currCounter = 0;
							}

							graphics_text(0, 16, 1, "           ");
							oled_update();

							sprintf(time_buf, "%d", currCounter);
							graphics_text(0, 16, 1, time_buf);
							oled_update();
							memset(time_buf, 0, sizeof(time_buf));
						}

						if(klick == 10)
						{
							// write data
							ds3231_set(DS3231_REGISTER_HOURS_DEFAULT, &prevCounter);

							graphics_text(0, 16, 1, "                 ");
							oled_update();

							graphics_text(0, 16, 1, "installed");
							oled_update();

							osDelay(800);

							graphics_text(0, 16, 1, "                 ");
							oled_update();

							klick = 11;
						}
					}
					break;

				case 11:
					// Set minutes

					graphics_text(0, 8, 1, "              ");
					oled_update();
					graphics_text(0, 8, 1, "MINUTES");
					oled_update();

					__HAL_TIM_SET_COUNTER(&htim1, 0);

					while(klick == 11)
					{
						currCounter = __HAL_TIM_GET_COUNTER(&htim1);
						currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

						if(currCounter != prevCounter)
						{
							prevCounter = currCounter;
							if(currCounter > 59)
							{
								__HAL_TIM_SET_COUNTER(&htim1, 0);
								prevCounter = 0;
							}
							if(currCounter < 0)
							{
								__HAL_TIM_SET_COUNTER(&htim1, 0);
								prevCounter = 0;
							}

							graphics_text(0, 16, 1, "           ");
							oled_update();

							sprintf(time_buf, "%d", currCounter);
							graphics_text(0, 16, 1, time_buf);
							oled_update();
							memset(time_buf, 0, sizeof(time_buf));
						}
					}
					if(klick == 12)
					{
						// write data
						ds3231_set(DS3231_REGISTER_MINUTES_DEFAULT, &prevCounter);

						graphics_text(0, 16, 1, "                 ");
						oled_update();

						graphics_text(0, 16, 1, "installed");
						oled_update();

						osDelay(800);

						graphics_text(0, 16, 1, "                 ");
						oled_update();

						klick = 13;
					}
					break;

				case 13:
					// Set minutes

					graphics_text(0, 8, 1, "              ");
					oled_update();
					graphics_text(0, 8, 1, "SECONDS");
					oled_update();

					__HAL_TIM_SET_COUNTER(&htim1, 0);

					while(klick == 13)
					{
						currCounter = __HAL_TIM_GET_COUNTER(&htim1);
						currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;

						if(currCounter != prevCounter)
						{
							prevCounter = currCounter;
							if(currCounter > 59)
							{
								__HAL_TIM_SET_COUNTER(&htim1, 0);
								currCounter = 0;
							}
							if(currCounter < 0)
							{
								__HAL_TIM_SET_COUNTER(&htim1, 0);
								currCounter = 0;
							}
							graphics_text(0, 16, 1, "           ");
							oled_update();

							sprintf(time_buf, "%d", currCounter);
							graphics_text(0, 16, 1, time_buf);
							oled_update();
							memset(time_buf, 0, sizeof(time_buf));
						}
					}
					if(klick == 14)
					{
						// write data
						ds3231_set(DS3231_REGISTER_SECONDS_DEFAULT, &prevCounter);

						graphics_text(0, 16, 1, "                 ");
						oled_update();

						graphics_text(0, 16, 1, "installed");
						oled_update();

						osDelay(800);

						graphics_text(0, 16, 1, "                 ");
						oled_update();

						klick = 15;
					}
					break;

				case 15:		// EXIT

					osDelay(500);
					clear();
					oled_update();

					for(uint8_t q = 0; q < 3; q ++)
					{
						graphics_text(20, 16, 2, "THE TIME IS SET");
						oled_update();
						osDelay(400);

						clear();
						oled_update();
						osDelay(200);
					}
					klick = 0;					// Return to show time

					break;
			}
		}
  /* USER CODE END StartOLED_RTC */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

