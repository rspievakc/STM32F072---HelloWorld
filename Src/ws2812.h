#ifndef INCLUDED_WS2812_H
#define INCLUDED_WS2812_H

#include "stdint.h"

#include "stm32f0xx.h"
#include "stm32f0xx_hal_tim.h"
#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_ll_dma.h"


typedef enum WS2812_state {
	WS2812_STOPPED,
	WS2812_UPDATING,
	WS2812_START_RESET,
	WS2812_END_RESET,
} WS2812_state;

typedef struct WS2812_config {
	uint16_t *output;
	uint8_t *leds; // Store the
	uint16_t count;
	uint16_t high_value;
	uint16_t low_value;

	TIM_TypeDef *timer;
	uint32_t timer_channel;
	DMA_TypeDef *dma;
	uint32_t dma_channel;

	uint16_t current_update_index;
	WS2812_state state;
} WS2812_config_t;

uint8_t WS2812_init(WS2812_config_t *config, uint8_t *leds, uint16_t *buffer, uint16_t low_value, uint16_t high_value, uint16_t count,
		TIM_TypeDef *timer, uint32_t timerChannel, DMA_TypeDef *dma, uint32_t dma_channel);

uint8_t WS2812_prepare_leds(WS2812_config_t *config, uint16_t position, uint16_t count);

uint8_t WS2812_putRGB_inline(WS2812_config_t *config, uint16_t position,
		uint8_t red, int8_t green, uint8_t blue);

uint8_t WS2812_putRGB32_inline(WS2812_config_t *config, uint16_t position,
		uint32_t rgb);

uint8_t WS2812_putRGB(WS2812_config_t *config, uint16_t position, uint8_t red,
		int8_t green, uint8_t blue);

uint8_t WS2812_putRGB32(WS2812_config_t *config, uint16_t position, uint32_t rgb);

uint8_t WS2812_clear(WS2812_config_t *config, uint16_t position, uint16_t count);

uint8_t WS2812_setRGB(WS2812_config_t *config, uint8_t red, uint8_t green,
		uint8_t blue, uint16_t position, uint16_t count);

uint8_t WS2812_setRGB32(WS2812_config_t *config, uint32_t rgb, uint16_t position,
		uint16_t count);

uint8_t WS2812_update(WS2812_config_t *config);

uint8_t WS2812_sendResetPulse(WS2812_config_t *config);

void WS2812_DMAHandler(WS2812_config_t *config);

#endif
