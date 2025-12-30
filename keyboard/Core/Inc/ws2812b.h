/*
 * ws2812b.h
 *
 *  Created on: Dec 6, 2025
 *      Author: mehmedblazevic
 */

#ifndef INC_WS2812B_H_
#define INC_WS2812B_H_

#include "main.h"


#define WS2812_LED_COUNT  3

// You already declared this:
extern uint8_t ws2812_leds[WS2812_LED_COUNT][3];

// Timer timing constants
#define WS2812_PERIOD    89
#define WS2812_DUTY_0    26   // ~0.350µs (logical 0) 26
#define WS2812_DUTY_1    65   // ~0.700µs (logical 1) 52

void MX_DMA_Init(void);
void MX_TIM2_Init(void);


void initWs2812b(void);
void setPWs2812b(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void setWs2812b(void);
void refreshWs2812b(void);



#endif /* INC_WS2812B_H_ */
