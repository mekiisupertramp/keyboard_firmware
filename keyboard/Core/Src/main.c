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
#include "usb_device.h"
#include "usbd_hid.h"
#include "ws2812b.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
RTC_HandleTypeDef hrtc;
UART_HandleTypeDef huart3;

extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct
{
uint8_t MODIFIER;
uint8_t RESERVED;
uint8_t KEYCODE1;
uint8_t KEYCODE2;
uint8_t KEYCODE3;
uint8_t KEYCODE4;
uint8_t KEYCODE5;
uint8_t KEYCODE6;
} keyboardHID;

keyboardHID keyboardhid = {0, 0, 0, 0, 0, 0, 0, 0};
extern uint8_t keyboardpressed;
uint8_t hello_sequence[11] = {0x0B, 0x08, 0x0F, 0x0F, 0x12, 0x2c, 0x1A, 0x12, 0x15, 0x0F, 0x07}; // "Hello world"


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);

void sendKey(uint8_t hidCode);
void sendKeys(uint8_t* hidCodes, uint8_t modifier, uint8_t number);
void sendModifier(uint8_t hidCode);
void release();



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{


  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  initWs2812b();

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


  // enable the 1.5k pull up!!
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  if (keyboardpressed == 1){
		  for (int i = 0; i < sizeof(hello_sequence); i++){
			  sendKey(hello_sequence[i]);
		  }
		  keyboardpressed = 0;
	  }
	  else{
		  release();
	  }


	  // chase light
	  for (int i = 0; i < WS2812_LED_COUNT; i++)
	  {
		  setPWs2812b(i, 0, 255, 255);
		  refreshWs2812b();
		  HAL_Delay(100);

		  // turn off before next
		  setPWs2812b(i, 0, 0, 0);
	  }
  }
}
void sendKeys(uint8_t* hidCodes, uint8_t modifier, uint8_t number){
	switch(number){
		case 0: keyboardhid.MODIFIER=modifier; break;
		case 1: keyboardhid.MODIFIER=modifier; keyboardhid.KEYCODE1=hidCodes[0]; break;
		case 2: keyboardhid.MODIFIER=modifier; keyboardhid.KEYCODE1=hidCodes[0];
				keyboardhid.KEYCODE2=hidCodes[1]; break;
		case 3: keyboardhid.MODIFIER=modifier; keyboardhid.KEYCODE1=hidCodes[0];
				keyboardhid.KEYCODE2=hidCodes[1]; keyboardhid.KEYCODE3=hidCodes[2]; break;
		case 4: keyboardhid.MODIFIER=modifier; keyboardhid.KEYCODE1=hidCodes[0];
				keyboardhid.KEYCODE2=hidCodes[1]; keyboardhid.KEYCODE3=hidCodes[2];
				keyboardhid.KEYCODE4=hidCodes[3]; break;
		case 5: keyboardhid.MODIFIER=modifier; keyboardhid.KEYCODE1=hidCodes[0];
				keyboardhid.KEYCODE2=hidCodes[1]; keyboardhid.KEYCODE3=hidCodes[2];
				keyboardhid.KEYCODE4=hidCodes[3]; keyboardhid.KEYCODE5=hidCodes[4]; break;
		case 6: keyboardhid.MODIFIER=modifier; keyboardhid.KEYCODE1=hidCodes[0];
				keyboardhid.KEYCODE2=hidCodes[1]; keyboardhid.KEYCODE3=hidCodes[2];
				keyboardhid.KEYCODE4=hidCodes[3]; keyboardhid.KEYCODE5=hidCodes[4];
				keyboardhid.KEYCODE6=hidCodes[5]; break;
		default: return; break;
	}
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyboardhid, sizeof(keyboardhid));
	HAL_Delay(50);
}
void sendModifier(uint8_t hidCode){
	keyboardhid.MODIFIER=hidCode;
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyboardhid, sizeof(keyboardhid));
	HAL_Delay(50);
}
void sendKey(uint8_t hidCode){
	keyboardhid.MODIFIER = 0x00; // No modifier keys
	keyboardhid.KEYCODE1 = hidCode; // Press 'Page Down' button
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyboardhid, sizeof(keyboardhid));
	HAL_Delay(50);
}

void release(){
	keyboardhid.MODIFIER = 0x00; // No modifier keys
	keyboardhid.KEYCODE1 = 0x00; // Release key
	keyboardhid.KEYCODE2 = 0x00; // Release key
	keyboardhid.KEYCODE3 = 0x00; // Release key
	keyboardhid.KEYCODE4 = 0x00; // Release key
	keyboardhid.KEYCODE5 = 0x00; // Release key
	keyboardhid.KEYCODE6 = 0x00; // Release key
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyboardhid, sizeof(keyboardhid));
	HAL_Delay(50);
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART3
							  |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_TIM2;
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
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
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LDGreen_Pin|LDRed_Pin|LDBlue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LDGreen_Pin LDRed_Pin LDBlue_Pin */
  GPIO_InitStruct.Pin = LDGreen_Pin|LDRed_Pin|LDBlue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
