/* USER CODE BEGIN Header */
/**

  ******************************************************************************

  * @file           : main.c

  * @brief          : Main program body

  ******************************************************************************

  * @attention

  *

  * Copyright (c) 2025 STMicroelectronics.

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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
//#include "ssd1306.h"
#include "fonts.h"
#include <string.h>
#include <stdint.h>
#include "stdio.h"
//#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//flag enabled by the ISR to break the loop
volatile int flag_break = 0;
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Morse code lookup table for A-Z and 0-9
const char* MorseCodeMap[36] = {

  /* A-Z */

  ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---",

  "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-",

  "..-", "...-", ".--", "-..-", "-.--", "--..",

  /* 0-9 */

  "-----", ".----", "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----."
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t selector = 0;

ILI9341TypeDef display;

#define RX_BUFFER_SIZE 100
uint8_t rx_data; // single byte received
volatile char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;
volatile uint8_t message_received = 0;

char message[100];

volatile char mes_char;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

//static void drawAxis(ILI9341TypeDef *display);
//static void clearCursor(ILI9341TypeDef *display, uint16_t pos);
//static void clearTrigger(ILI9341TypeDef *display, uint16_t pos);
//static void drawCursor(ILI9341TypeDef *display, uint16_t pos, char *name, uint16_t color);
//static void drawTrigger(ILI9341TypeDef *display, uint16_t pos, char *name, uint16_t color);
//static void drawSignal(ILI9341TypeDef *display, uint32_t *adc_time, uint16_t *adc0, uint32_t adc_length, uint16_t pixel_dirty[280][2], uint16_t cursor, uint16_t color);
//static void clearSignal(ILI9341TypeDef *display, uint16_t pixel_dirty[280][2]);
//static void drawSignalParam(ILI9341TypeDef *display, char *string, size_t size, uint16_t adc_max, uint16_t adc_min, uint32_t adc_period);

void USB_DEVICE_MasterHardReset(void);

/**
 * @brief  gets the index for a character (A-Z, a-z, 0-9)
 * @param  c: character
 * @retval index
 */
int mapIndex(char c) {

    if (c >= 'A' && c <= 'Z') return c - 'A';       // 0-25

    if (c >= 'a' && c <= 'z') return c - 'a';         // 0-25 (case-insensitive)

    if (c >= '0' && c <= '9') return 26 + (c - '0');   // 26-35 for 0-9

    return -1; // invalid character

}

/**
 * @brief  Triggers the buzzer to reproduce Morse code.
 * @param  code: character
 * @retval none
 */
void blinkMorse(const char* code) {

    for (const char* p = code; *p != '\0'; p++) {

        if(flag_break == 1){
        	//breaks the for loop if flag_break is enabled in the interruption routine.
        	break;
        }

        // Changes Duty Cycle of the PWM output to 17% to trigger the buzzer.
        TIM2->CCR2 = 15;

        if (*p == '.') {

            HAL_Delay(200);       // Dot: 200ms on

        } else if (*p == '-') {

            HAL_Delay(600);       // Dash: 600ms on

        }

        // Changes Duty Cycle of the PWM output to 0% to turn off the buzzer.
        TIM2->CCR2 = 0;

        if(flag_break == 1){
        	//breaks the for loop if flag_break is enabled in the interruption routine.
        	break;
        }

        if (*(p+1) != '\0') {
        	// Intra-character gap: 200ms off if more symbols follow in the same letter
            HAL_Delay(200);
        }

    }

}

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
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Display init
  	display.spi             = &hspi2;
  	display.cs_gpio_port    = ILI9341_CS_GPIO_Port;
  	display.dc_gpio_port    = ILI9341_DC_GPIO_Port;
  	display.reset_gpio_port = ILI9341_RESET_GPIO_Port;
  	display.cs_pin          = ILI9341_CS_Pin;
  	display.dc_pin          = ILI9341_DC_Pin;
  	display.reset_pin       = ILI9341_RESET_Pin;
  	display.width           = 320;
  	display.height          = 240;
  	display.orientation     = ILI9341_ORIENTATION_ROTATE_RIGHT;

  	ILI9341_UNSELECT(&display);
  	ILI9341_Init(&display);
  	char string[255];

  	// Start Display background color
  	ILI9341_FillScreen(&display, ILI9341_BLACK);

  	// Print Title
  	snprintf(string, 255, "Morse Encoder");
  	ILI9341_WriteString(&display, 60, 18 * 0, string, Font_16x26, ILI9341_YELLOW, ILI9341_BLACK);

  	HAL_Delay(1000);

  	HAL_UART_Receive_IT(&huart1, &rx_data, 1);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	HAL_UART_Receive_IT(&huart1, &rx_data, 1);

	char blank_m[50];  //
	strcpy(blank_m, "                     ");  // blank string



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  //reset flag_break (Enable at interruption routine)
	  flag_break = 0;

		if (message_received==1) {
			message_received = 0;
			strcpy(message, rx_buffer);

		}

	  HAL_Delay(500);

	  // Iterate through each character of the message
	  for (uint32_t i = 0; i < strlen(message); i++){

			if(flag_break==1){
				//SSD1306_Clear();
				//SSD1306_GotoXY(0,0);
				break;
			}

			char c[2];
			c[0] = message[i];
			c[1] = '\0';
			ILI9341_WriteString(&display, 24+18*i, 18 * 3, c, Font_16x26, ILI9341_GREEN, ILI9341_BLACK);
			ILI9341_WriteString(&display, 170-18*2, 18 * 7, c, Font_16x26, ILI9341_RED, ILI9341_BLACK);

			// Handle word separation
			if (c[0] == ' ') {
				HAL_Delay(1400);
				 // Inter-word gap: 1400ms off
				continue;

			  }

			// Convert lowercase to uppercase (ensures case insensitivity)
			if (c[0] >= 'a' && c[0] <= 'z') {
			    c[0] -= 32;
			}


			// Get Morse code for valid alphanumeric characters
			int idx = mapIndex(c[0]);

			//Blinks Morse code for valid characters in the message
			if (idx != -1) {
				blinkMorse(MorseCodeMap[idx]);
			}

			if(flag_break==1){
				break;
			}

			// Inter-character gap: 600ms off after each character
			HAL_Delay(600);
	  }

	  ILI9341_WriteString(&display, 24, 18 * 3, blank_m, Font_16x26, ILI9341_BLACK, ILI9341_BLACK);
	  ILI9341_WriteString(&display, 24, 18 * 4, blank_m, Font_16x26, ILI9341_BLACK, ILI9341_BLACK);
	  // Adds a delay before repeating the message
	  HAL_Delay(600);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 95;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3750;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 95;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.Mode = UART_MODE_RX;
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
  HAL_GPIO_WritePin(ILI9341_DC_GPIO_Port, ILI9341_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ILI9341_CS_GPIO_Port, ILI9341_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ILI9341_LED_GPIO_Port, ILI9341_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ILI9341_RESET_GPIO_Port, ILI9341_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ILI9341_DC_Pin */
  GPIO_InitStruct.Pin = ILI9341_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ILI9341_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ILI9341_CS_Pin */
  GPIO_InitStruct.Pin = ILI9341_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ILI9341_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ILI9341_LED_Pin ILI9341_RESET_Pin */
  GPIO_InitStruct.Pin = ILI9341_LED_Pin|ILI9341_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */



  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  UART receive complete callback (interrupt mode).
  * @note   Triggered when a byte is received via USART1.
  * Assembles a string until '\n' is received.
  * @param  huart: Pointer to the UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART1) {
        if (rx_data != '\n' && rx_index < RX_BUFFER_SIZE - 1) {
            rx_buffer[rx_index++] = (char)rx_data;
        } else {
            rx_buffer[rx_index] = '\0';  // Null-terminate string
            rx_index = 0;
            message_received = 1;
            flag_break=1;
        }

        HAL_UART_Receive_IT(&huart1, &rx_data, 1); // Re-enable interrupt
    }
}

/*
void USB_DEVICE_MasterHardReset(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,0);
    HAL_Delay(1000);
}
*/




/**
 * @brief  External interrupt (EXTI) callback for GPIO_PIN_12.
 *         Triggered by a button press to set a control flag,
 *         clear the OLED display, and increment a selector variable.
 * @param  GPIO_Pin: The GPIO pin that triggered the interrupt.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_12) {
        if (flag_break != 1) {
            flag_break = 1;

            // Clear the OLED display
            //SSD1306_Clear();

            // Set the cursor to the top-left corner
            //SSD1306_GotoXY(0, 0);

            // Increment the selector (can be used to switch menu options)
            selector = selector + 1;

            // wrap selector within a certain range (e.g., 0 to 2)
            selector = selector % 3;
        }
    }
}


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
