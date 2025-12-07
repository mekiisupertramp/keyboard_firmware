/*
 * ws2812b.c
 *
 *  Created on: Dec 6, 2025
 *      Author: mehmedblazevic
 */


#include "ws2812b.h"

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;


// GRB LED buffer
uint8_t ws2812_leds[WS2812_LED_COUNT][3];

// DMA waveform buffer: 24 bits per LED + 50 reset cycles
uint16_t ws2812_dmaBuffer[WS2812_LED_COUNT * 24 + 50];


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        // Stop DMA after sending all data
        HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4);
        asm("nop");
    }
}


void initWs2812b(void){
	MX_DMA_Init();
	MX_TIM2_Init();
}

void setPWs2812b(uint16_t index, uint8_t r, uint8_t g, uint8_t b){
    if (index >= WS2812_LED_COUNT) return;

    ws2812_leds[index][0] = g;
    ws2812_leds[index][1] = r;
    ws2812_leds[index][2] = b;
	/*HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 45); // 50% duty*/
}
void setWs2812b(void){
	uint32_t pos = 0;

	for (uint16_t led = 0; led < WS2812_LED_COUNT; led++)
	{
		uint8_t g = ws2812_leds[led][0];
		uint8_t r = ws2812_leds[led][1];
		uint8_t b = ws2812_leds[led][2];

		for (int bit = 7; bit >= 0; bit--)
			ws2812_dmaBuffer[pos++] = (g & (1 << bit)) ? WS2812_DUTY_1 : WS2812_DUTY_0;

		for (int bit = 7; bit >= 0; bit--)
			ws2812_dmaBuffer[pos++] = (r & (1 << bit)) ? WS2812_DUTY_1 : WS2812_DUTY_0;

		for (int bit = 7; bit >= 0; bit--)
			ws2812_dmaBuffer[pos++] = (b & (1 << bit)) ? WS2812_DUTY_1 : WS2812_DUTY_0;
	}

	// 50 reset cycles
	for (uint16_t i = 0; i < 50; i++){ws2812_dmaBuffer[pos++] = 0;}

}
void refreshWs2812b(void){
	setWs2812b(); // encode RGB in the send buffer

	//HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4); // Ensure previous DMA completed

	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, // Start DMA transfer
						  (uint32_t*)ws2812_dmaBuffer,
						  (WS2812_LED_COUNT * 24 + 50));
}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM2_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

   // __HAL_RCC_TIM2_CLK_ENABLE();

	// --- TIMER BASE ---
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;                     // 72 MHz timer clock
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 89;                       // 1.25 us period (800 kHz)
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
		Error_Handler();

	// --- PWM INIT ---
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
		Error_Handler();

	// --- MASTER CONFIG ---
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
		Error_Handler();

	// --- CHANNEL CONFIG ---
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;                           // Start at 0% duty
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
		Error_Handler();

	HAL_TIM_MspPostInit(&htim2);
}
