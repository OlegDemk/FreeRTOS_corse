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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// LEDs redefinition
#define LED1 LD4_Pin
#define LED2 LD3_Pin
#define LED3 LD5_Pin
#define LED4 LD6_Pin
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

void process_command(command_t *cmd);
int extract_command(command_t *cmd);
void led_effect_callback(TimerHandle_t xTimer);

volatile uint8_t user_data;						// For write data from UART

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
	sTestMenu,			// Test menu
	sTestPodMenu_1,
	sTestPodMenu_2,
	sTestPodMenu_3,
}satte_t;
satte_t curr_state = sMainMenu;			// Set current state as sMainMenu

// Software timer handlers
// TimerHandle_t handler_led_timer[4];

const char *msg_inv = "/// Invalid option ///\n\r\n\r";

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for print_task */
osThreadId_t print_taskHandle;
const osThreadAttr_t print_task_attributes = {
  .name = "print_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for cmd_handl */
osThreadId_t cmd_handlHandle;
const osThreadAttr_t cmd_handl_attributes = {
  .name = "cmd_handl",
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
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void start_menu_task(void *argument);
void start_led_task(void *argument);
void start_rtc_task(void *argument);
void start_print_task(void *argument);
void start_cmd_handl(void *argument);

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
  MX_USART2_UART_Init();
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
  print_taskHandle = osThreadNew(start_print_task, NULL, &print_task_attributes);

  /* creation of cmd_handl */
  cmd_handlHandle = osThreadNew(start_cmd_handl, NULL, &cmd_handl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

   int uart2_init_status = HAL_UART_Receive_IT(&huart2, &user_data , 1);			// Turn on (start) receive one char in interrupt mode
   if(uart2_init_status == HAL_ERROR)
   {
  	  int ggg =9;
   }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  huart2.Init.BaudRate = 115200;
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


	 	case sTestMenu:
	 	case sTestPodMenu_1:
	 	case sTestPodMenu_2:
	 	case sTestPodMenu_3:
			xTaskNotify(test_taskHandle, (uint32_t*) cmd, eSetValueWithOverwrite);
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
void rtc_configure_date(RTC_DateTypeDef *date)
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
	return 1;
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
	  HAL_GPIO_TogglePin(GPIOD, LED1);
	  osDelay(1000);
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
  for(;;)
  {
    osDelay(1);
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
  for(;;)
  {
    osDelay(1);
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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_rtc_task */
}

/* USER CODE BEGIN Header_start_print_task */
/**
* @brief Function implementing the print_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_print_task */
void start_print_task(void *argument)
{
  /* USER CODE BEGIN start_print_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_print_task */
}

/* USER CODE BEGIN Header_start_cmd_handl */
/**
* @brief Function implementing the cmd_handl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_cmd_handl */
void start_cmd_handl(void *argument)
{
  /* USER CODE BEGIN start_cmd_handl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_cmd_handl */
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

