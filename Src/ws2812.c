#include "ws2812.h"

uint8_t WS2812_init(WS2812_config_t *config, uint8_t *leds, uint16_t *buffer, uint16_t low_value, uint16_t high_value, uint16_t count,
		TIM_TypeDef *timer, uint32_t timer_channel, DMA_TypeDef *dma, uint32_t dma_channel) {

	config->leds = leds;
	config->output = buffer;
	config->count = count;
	config->low_value = low_value;
	config->high_value = high_value;
	config->timer = timer;
	config->timer_channel = timer_channel;
	config->dma = dma;
	config->dma_channel = dma_channel;
	config->current_update_index = 0;
	config->state = WS2812_STOPPED;

	LL_TIM_EnableDMAReq_CC1(config->timer); // Enable period match DMA requisition
	LL_DMA_SetPeriphAddress(config->dma, config->dma_channel,  (uint32_t) &config->timer->CCR1);

	return 0;
}

uint8_t WS2812_prepare_leds(WS2812_config_t *config, uint16_t position, uint16_t count) {
	uint8_t i;
	while(count) {
		uint8_t * leds = (uint8_t *) config->leds + (3 * position);
		uint16_t * mem = (uint16_t *) config->output + (24 * (position % 2));
		i = 0;
		while (i < 8) {
			*(mem + i) = *(leds + 1) >> i & 0x01 ? config->high_value : config->low_value; // Green
			*(mem + 8 + i) = *(leds) >> i & 0x01 ? config->high_value : config->low_value; // Red
			*(mem + 16 + i) = *(leds + 2) >> i & 0x01 ? config->high_value : config->low_value; // Blue
			i++;
		}
		count--;
		position++;
	}
	return 0;
}

inline uint8_t WS2812_putRGB_inline(WS2812_config_t *config, uint16_t position,
		uint8_t red, int8_t green, uint8_t blue) {
	uint8_t * leds = (uint8_t *) (config->leds + 3 * position);
	*(leds) = red;
	*(leds+1) = green;
	*(leds+2) = blue;
	return 0;
}

inline uint8_t WS2812_putRGB32_inline(WS2812_config_t *config, uint16_t position,
		uint32_t rgb) {

	uint8_t red = rgb >> 16 & 0xFF;
	uint8_t green = rgb >> 8 & 0xFF;
	uint8_t blue = rgb & 0xFF;

	return WS2812_putRGB_inline(config, position, red, green, blue);
}

uint8_t WS2812_putRGB(WS2812_config_t *config, uint16_t position, uint8_t red,
		int8_t green, uint8_t blue) {
	return WS2812_putRGB_inline(config, position, red, green, blue);
}

uint8_t WS2812_putRGB32(WS2812_config_t *config, uint16_t position, uint32_t rgb) {
	return WS2812_putRGB32_inline(config, position, rgb);
}

uint8_t WS2812_clear(WS2812_config_t *config, uint16_t position, uint16_t count) {

	uint32_t i = count;
	while (i) {
		WS2812_putRGB(config, position++, 0, 0, 0);
		i--;
	}

	return 0;
}

uint8_t WS2812_setRGB(WS2812_config_t *config, uint8_t red, uint8_t green,
		uint8_t blue, uint16_t position, uint16_t count) {

	uint32_t i = count;
	while (i) {
		WS2812_putRGB_inline(config, position++, red, green, blue);
		i--;
	}
	return 0;
}

uint8_t WS2812_setRGB32(WS2812_config_t *config, uint32_t rgb, uint16_t position,
		uint16_t count) {

	uint32_t i = count;
	while (i) {
		WS2812_putRGB32_inline(config, position++, rgb);
		i--;
	}
	return 0;
}

uint8_t WS2812_sendResetPulse(WS2812_config_t *config) {

	if (config->state != WS2812_END_RESET)
		config->state = WS2812_START_RESET;

	//LL_TIM_CC_DisableChannel(config->timer, config->timer_channel);
	//LL_DMA_DisableChannel(config->dma, config->dma_channel);

	// Clear output -> 48 bits with 1.25us (800khz) each = 60us > required 50us reset time
	for (int i=0; i < 48; i++) {
		config->output[i] = 0;
	}

	LL_DMA_SetMode(config->dma, config->dma_channel , LL_DMA_MODE_NORMAL);
    LL_DMA_SetMemoryAddress(config->dma, config->dma_channel, (uint32_t) config->output);
    LL_DMA_SetDataLength(config->dma, config->dma_channel, 48);

    LL_DMA_ClearFlag_TC4(config->dma);
    LL_DMA_ClearFlag_HT4(config->dma);
	LL_DMA_DisableIT_HT(config->dma, config->dma_channel);  // Disabled the Half transfer interrupt
	LL_DMA_EnableIT_TC(config->dma, config->dma_channel); // Enabled the transfer completed interrupt

	LL_DMA_EnableChannel(config->dma, config->dma_channel);
	LL_TIM_CC_EnableChannel(config->timer, config->timer_channel);
	LL_TIM_EnableDMAReq_CC1(config->timer);
	LL_TIM_EnableCounter(config->timer);

	return 0;
}

uint8_t WS2812_update(WS2812_config_t *config) {

	if (config->state == WS2812_STOPPED) {
		return WS2812_sendResetPulse(config);
	} else {
		return 1;
	}

	return 0;
}

uint8_t WS2812_process_next(WS2812_config_t *config) {

	if (config->state == WS2812_START_RESET) {

		LL_TIM_CC_DisableChannel(config->timer, config->timer_channel);
		LL_DMA_DisableChannel(config->dma, config->dma_channel);

		config->state = WS2812_UPDATING;

		WS2812_prepare_leds(config, 0, 2); // Updates two leds to the output (0,1)
		config->current_update_index = 2;  // Position the next index on the 3rd led (Position 2)

		LL_DMA_SetMode(config->dma, config->dma_channel, DMA_CIRCULAR); // Re-enabled the DMA in circular mode

		LL_DMA_ClearFlag_TC4(config->dma);
		LL_DMA_ClearFlag_HT4(config->dma);
		LL_DMA_EnableIT_HT(config->dma, config->dma_channel); // Enabled the half transfer interrupt
		LL_DMA_EnableChannel(config->dma, config->dma_channel); // Enables the DMA channel
		LL_TIM_CC_EnableChannel(config->timer, config->timer_channel); // Enables the output on the channel
		LL_TIM_EnableDMAReq_CC1(config->timer);
		LL_TIM_EnableCounter(config->timer);

	} else if (config->state == WS2812_END_RESET) {

		LL_TIM_DisableCounter(config->timer);
		LL_TIM_CC_DisableChannel(config->timer, config->timer_channel);
		LL_DMA_DisableChannel(config->dma, config->dma_channel);

		config->state = WS2812_STOPPED;

	} else if (config->current_update_index == config->count - 1) {
		config->state = WS2812_END_RESET;
		WS2812_sendResetPulse(config);

	}  else {
		WS2812_prepare_leds(config, config->current_update_index, 1);
		config->current_update_index++;
	}

	return 0;
}

void WS2812_DMAHandler(WS2812_config_t *config) {

	HAL_GPIO_TogglePin(GPIO_TEST_GPIO_Port, GPIO_TEST_Pin);
	/* Half Transfer Complete */
	if (LL_DMA_IsActiveFlag_HT4(config->dma)) {
		LL_DMA_ClearFlag_HT4(DMA1);
		WS2812_process_next(config);

	}
	/* Transfer completed */
	if (LL_DMA_IsActiveFlag_TC4(config->dma)) {
		LL_DMA_ClearFlag_TC4(DMA1);
		WS2812_process_next(config);
	}

}

