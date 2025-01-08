/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "math.h"
#include "semphr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// HC-05 Message Struct
struct message{
	uint8_t rgb[3];
	int total_time;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// TCS34725

#define ADDRESS         0x29 // I2C Address
#define COMMAND_BIT     0x80

#define ENABLE          0x00 // Enable states and interrupts
#define ENABLE_PON      0x01 // Power On
#define ENABLE_AEN      0x02 // ADC Enable

#define ATIME           0x01 // Register for Integration Time
#define CONTROL         0x0F // Register for Gain

#define CDATAL 			0x14
#define RDATAL			0x16
#define GDATAL			0x18
#define BDATAL 			0x1A

#define ITIME 			0x00
#define GAIN 			0x00

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for dataReceive */
osThreadId_t dataReceiveHandle;
const osThreadAttr_t dataReceive_attributes = {
  .name = "dataReceive",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dataSend_BLT */
osThreadId_t dataSend_BLTHandle;
const osThreadAttr_t dataSend_BLT_attributes = {
  .name = "dataSend_BLT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for messageMutex */
osMutexId_t messageMutexHandle;
const osMutexAttr_t messageMutex_attributes = {
  .name = "messageMutex"
};
/* Definitions for binarySem */
osSemaphoreId_t binarySemHandle;
const osSemaphoreAttr_t binarySem_attributes = {
  .name = "binarySem"
};
/* Definitions for CountingSem */
osSemaphoreId_t CountingSemHandle;
const osSemaphoreAttr_t CountingSem_attributes = {
  .name = "CountingSem"
};
/* USER CODE BEGIN PV */

struct message msg;
uint8_t buffer[1];
bool isToggled = false;

int* itime = 0x00;
int* gain = 0x00;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void Task_dataReceive(void *argument);
void Task_dataSend_BLT(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// TCS34725 Colour Sensor

void tcs34725_i2c_write(uint8_t reg, uint32_t value) {
    uint8_t buffer[2];
    buffer[0] = COMMAND_BIT | reg;
    buffer[1] = value & 0xFF;
    HAL_I2C_Master_Transmit(&hi2c1, (ADDRESS << 1), buffer, 2, HAL_MAX_DELAY);
}

// Read a single register
uint8_t tcs34725_i2c_read8(uint8_t reg) {
	uint8_t buffer[2];

	buffer[0] = (COMMAND_BIT | reg);

	HAL_I2C_Master_Transmit(&hi2c1, (ADDRESS << 1), buffer, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, (ADDRESS << 1), buffer, 1, HAL_MAX_DELAY);

	return buffer[0];
}

uint16_t tcs34725_i2c_read16(uint8_t reg) {
	uint16_t time_delay = 1;
    uint16_t value;
    uint8_t reg_value[2];
    uint8_t cmd[2];
    cmd[0] = COMMAND_BIT | reg;

    HAL_I2C_Master_Transmit(&hi2c1, (ADDRESS << 1), cmd, 1, time_delay);
    HAL_I2C_Master_Receive(&hi2c1, (ADDRESS << 1), reg_value, 2, time_delay);

    value = reg_value[0];
    value <<= 8;
    value |= reg_value[1];

    return value;
}

void tcs34725_init_sensor(){
	tcs34725_i2c_write(ENABLE, ENABLE_PON);					// Power
	HAL_Delay(3);

	tcs34725_i2c_write(ATIME, ITIME);						// Integration time
	tcs34725_i2c_write(CONTROL, GAIN);						// Gain


	uint8_t reg_value = tcs34725_i2c_read8(ATIME);

	char reg_str[5];
	sprintf(reg_str, "%d", reg_value);

	char temp_msg[64];
	strcpy(temp_msg, "Sensor Integration Time: ");
	strcat(temp_msg, reg_str);
	strcat(temp_msg, "\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)temp_msg, strlen(temp_msg), HAL_MAX_DELAY);



	tcs34725_i2c_write(ENABLE, ENABLE_PON | ENABLE_AEN);    // ADC Enable

}

void tcs34725_integration_wait_time(int itime){
	int wait_time = 0;
	switch(itime){
	case 255:
		wait_time = 3;
		break;
	case 246:
		wait_time = 24;
		break;
	case 213:
		wait_time = 101;
		break;
	case 192:
		wait_time = 154;
		break;
	case 0:
		wait_time = 700;
		break;
	}
	HAL_Delay(wait_time);
}

void tcs34725_update_config_itime(int itime){
	tcs34725_i2c_write(ENABLE, ENABLE_PON);    // Pause Measuring
	tcs34725_i2c_write(ATIME, itime);
	tcs34725_i2c_write(ENABLE, ENABLE_PON | ENABLE_AEN);
}

void tcs34725_update_config_gain(int gain, int itime){
	tcs34725_i2c_write(ENABLE, ENABLE_PON ); // Pause Measuring
	tcs34725_i2c_write(CONTROL, gain);

	tcs34725_i2c_write(ENABLE, ENABLE_PON | ENABLE_AEN);    // ADC Enable
}

void read_colour_data(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c){
	// Read colour from all registers
	*r = tcs34725_i2c_read16(RDATAL);
	*g = tcs34725_i2c_read16(GDATAL);
	*b = tcs34725_i2c_read16(BDATAL);
	*c = tcs34725_i2c_read16(CDATAL);

	int itime = tcs34725_i2c_read8(ATIME);

	tcs34725_integration_wait_time(itime);
}

void normalize_colour_data(uint8_t *colours, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *clear){
	// Normalize to 0-255
	if(*clear == 0){	// clear 0 => black
		*(colours + 0) = 0;
		*(colours + 1) = 0;
		*(colours + 2) = 0;
		return;
	}

	*(colours + 0) = *r * 255 / *clear;
	*(colours + 1) = *g * 255 / *clear;
	*(colours + 2) = *b * 255 / *clear;

}

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
	  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable the use of DWT
  }
  DWT->CYCCNT = 0; // Reset the cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable the cycle counter

  tcs34725_init_sensor();
  char *message = "Waiting command...";
  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
  HAL_UART_Receive_IT(&huart1, buffer, 1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of messageMutex */
  messageMutexHandle = osMutexNew(&messageMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of binarySem */
  binarySemHandle = osSemaphoreNew(1, 1, &binarySem_attributes);

  /* creation of CountingSem */
  CountingSemHandle = osSemaphoreNew(3, 0, &CountingSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of dataReceive */
  dataReceiveHandle = osThreadNew(Task_dataReceive, NULL, &dataReceive_attributes);

  /* creation of dataSend_BLT */
  dataSend_BLTHandle = osThreadNew(Task_dataSend_BLT, NULL, &dataSend_BLT_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(&huart1, buffer, 1);

	char temp_msg[32];
	char str_itime[6];
	char str_gain[6];
	int itime = 0;
	int gain = 0;

	switch(buffer[0]){
	case '0':
		HAL_UART_Transmit(&huart1, (uint8_t *)"App OFF!\n", 10, HAL_MAX_DELAY);
		isToggled = false;
		break;
	case '1':
		HAL_UART_Transmit(&huart1, (uint8_t *)"App ON!\n", 9, HAL_MAX_DELAY);
		isToggled = true;
		break;
	case '2':
		itime = 0xFF;
		sprintf(str_itime, "0x%02X", itime);

		strcpy(temp_msg, "Set integration time to ");
		strcat(temp_msg, str_itime);
		strcat(temp_msg, "!\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) temp_msg, strlen(temp_msg), HAL_MAX_DELAY);
		tcs34725_update_config_itime(itime);
		break;
	case '3':
		itime = 0xF6;
		sprintf(str_itime, "0x%02X", itime);

		strcpy(temp_msg, "Set integration time to ");
		strcat(temp_msg, str_itime);
		strcat(temp_msg, "!\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) temp_msg, strlen(temp_msg), HAL_MAX_DELAY);
		tcs34725_update_config_itime(itime);
		break;
	case '4':
		itime = 0xD5;
		sprintf(str_itime, "0x%02X", itime);

		strcpy(temp_msg, "Set integration time to ");
		strcat(temp_msg, str_itime);
		strcat(temp_msg, "!\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) temp_msg, strlen(temp_msg), HAL_MAX_DELAY);
		tcs34725_update_config_itime(itime);
		break;
	case '5':
		itime = 0xC0;
		sprintf(str_itime, "0x%02X", itime);

		strcpy(temp_msg, "Set integration time to ");
		strcat(temp_msg, str_itime);
		strcat(temp_msg, "!\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) temp_msg, strlen(temp_msg), HAL_MAX_DELAY);
		tcs34725_update_config_itime(itime);
		break;
	case '6':
		itime = 0x00;
		sprintf(str_itime, "0x%02X", itime);

		strcpy(temp_msg, "Set integration time to ");
		strcat(temp_msg, str_itime);
		strcat(temp_msg, "!\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) temp_msg, strlen(temp_msg), HAL_MAX_DELAY);
		tcs34725_update_config_itime(itime);
		break;
	case '7':
		gain = 0x00;
		sprintf(str_gain, "0x%02X", gain);

		strcpy(temp_msg, "Set gain to ");
		strcat(temp_msg, str_gain);
		strcat(temp_msg, "!\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) temp_msg, strlen(temp_msg), HAL_MAX_DELAY);
		tcs34725_update_config_gain(gain, itime);
		break;
	case '8':
		gain = 0x01;
		sprintf(str_gain, "0x%02X", gain);

		strcpy(temp_msg, "Set gain to ");
		strcat(temp_msg, str_gain);
		strcat(temp_msg, "!\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) temp_msg, strlen(temp_msg), HAL_MAX_DELAY);
		tcs34725_update_config_gain(gain, itime);
		break;
	case '9':
		gain = 0x10;
		sprintf(str_gain, "0x%02X", gain);

		strcpy(temp_msg, "Set gain to ");
		strcat(temp_msg, str_gain);
		strcat(temp_msg, "!\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) temp_msg, strlen(temp_msg), HAL_MAX_DELAY);
		tcs34725_update_config_gain(gain, itime);
		break;
	case 'A':
		gain = 0x11;
		sprintf(str_gain, "0x%02X", gain);

		strcpy(temp_msg, "Set gain to ");
		strcat(temp_msg, str_gain);
		strcat(temp_msg, "!\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) temp_msg, strlen(temp_msg), HAL_MAX_DELAY);
		tcs34725_update_config_gain(gain, itime);
		break;

	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task_dataReceive */
/**
  * @brief  Function implementing the dataReceive thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task_dataReceive */
void Task_dataReceive(void *argument)
{
  /* USER CODE BEGIN 5 */

	uint16_t *r = (uint16_t*) malloc (sizeof(uint16_t));
	uint16_t *g = (uint16_t*) malloc (sizeof(uint16_t));
	uint16_t *b = (uint16_t*) malloc (sizeof(uint16_t));
	uint16_t *c = (uint16_t*) malloc (sizeof(uint16_t));
	uint8_t *colours = (uint8_t*) malloc (3 * sizeof(uint8_t));


  /* Infinite loop */
  for(;;)
  {
	  if (xSemaphoreTake(binarySemHandle, portMAX_DELAY)){ //semaphore
		  if(isToggled){
			  int start_cycle = DWT->CYCCNT;
			  read_colour_data(r, g, b, c);
			  normalize_colour_data(colours, r, g, b, c);
			  int end_cycle = DWT->CYCCNT;
			  int total_cycles = end_cycle - start_cycle;
			  int total_time = (total_cycles * 1000) / 180000000; // ms

			  if(xSemaphoreTake(messageMutexHandle, portMAX_DELAY)){
				  	  msg.rgb[0] = *(colours);
				  	  msg.rgb[1] = *(colours + 1);
				  	  msg.rgb[2] = *(colours + 2);
				  	  msg.total_time = total_time;
				  	  xSemaphoreGive(messageMutexHandle);
			  }
		  }
		  xSemaphoreGive(binarySemHandle);
	  }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task_dataSend_BLT */
/**
* @brief Function implementing the dataSend_BLT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_dataSend_BLT */
void Task_dataSend_BLT(void *argument)
{
  /* USER CODE BEGIN Task_dataSend_BLT */
  /* Infinite loop */
  for(;;)
  {
	  if(isToggled){
		  if (xSemaphoreTake(binarySemHandle, portMAX_DELAY)) {
			  char temp_msg[64] = "";

			  // Variables for the colour values detected
			  char r_str[4]; // Red value as char
			  char g_str[4]; // Green value as char
			  char b_str[4]; // Blue value as char

			  // Variables for the colour values detected
			  char r_hex_str[3]; // Red in HEX as char
			  char g_hex_str[3]; // Green in HEX as char
			  char b_hex_str[3]; // Blue in HEX as char

			  char total_time_str[16];

			  if(xSemaphoreTake(messageMutexHandle, portMAX_DELAY)){
				  // Convert int to char
				  sprintf(r_str, "%d", msg.rgb[0]);
				  sprintf(g_str, "%d", msg.rgb[1]);
				  sprintf(b_str, "%d", msg.rgb[2]);

				  // Convert values to hex and char
				  sprintf(r_hex_str, "%02X", msg.rgb[0]);
				  sprintf(g_hex_str, "%02X", msg.rgb[1]);
				  sprintf(b_hex_str, "%02X", msg.rgb[2]);

				  sprintf(total_time_str, "%d", msg.total_time);

				  xSemaphoreGive(messageMutexHandle);
			  }

			  strcpy(temp_msg, "R: ");
			  strcat(temp_msg, r_str);
			  strcat(temp_msg, " G: ");
			  strcat(temp_msg, g_str);
			  strcat(temp_msg, " B: ");
			  strcat(temp_msg, b_str);
			  strcat(temp_msg, " ");

			  strcat(temp_msg, "HEX: #");
			  strcat(temp_msg, r_hex_str);
			  strcat(temp_msg, g_hex_str);
			  strcat(temp_msg, b_hex_str);
			  strcat(temp_msg, " ");

			  strcat(temp_msg, "Time taken: ");
			  strcat(temp_msg, total_time_str);
			  strcat(temp_msg, "ms\n");
			  HAL_UART_Transmit(&huart1, (uint8_t*)temp_msg, strlen(temp_msg), HAL_MAX_DELAY);
		  }

		  xSemaphoreGive(binarySemHandle);
	  }
    osDelay(1);
  }
  /* USER CODE END Task_dataSend_BLT */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
