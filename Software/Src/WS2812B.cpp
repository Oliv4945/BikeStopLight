/*
 * WS2812B.cpp
 *
 *  Created on: Aug 20, 2019
 *      Author: oliv
 */

#include "WS2812B.h"
#include <string.h>
#include "main.h"

extern TIM_HandleTypeDef htim2;

WS2812B::WS2812B(uint8_t led_number) :
		led_number(led_number) {
	pixel_array = new uint8_t[led_number * 3];
	led_data_dma = new uint8_t[led_number * 3 * 8 + 40 * 2];
	memset(pixel_array, 0, led_number * 3);
	memset(led_data_dma, 0, led_number * 3 * 8 + 40 * 2);
}

bool WS2812B::setColorAll(s_color color) {
	for (uint8_t i = 0; i < led_number; i++) {
		pixel_array[i * color_data_bytes + 0] = color.g;
		pixel_array[i * color_data_bytes + 1] = color.r;
		pixel_array[i * color_data_bytes + 2] = color.b;
	}

	return true;
}

bool WS2812B::setColorIndex(uint8_t index, s_color color) {
	if (led_number <= index) {
		return false;
	}
	pixel_array[index * color_data_bytes + 0] = color.g;
	pixel_array[index * color_data_bytes + 1] = color.r;
	pixel_array[index * color_data_bytes + 2] = color.b;
	return true;
}

bool WS2812B::setColorRange(uint8_t index, uint8_t range, s_color color) {
	if (led_number < index + range) {
		return false;
	}
	for (uint8_t i = index; i < index + range; i++) {
		pixel_array[i * color_data_bytes + 0] = color.g;
		pixel_array[i * color_data_bytes + 1] = color.r;
		pixel_array[i * color_data_bytes + 2] = color.b;
	}
	return true;
}

// TODO: Should not exists
bool WS2812B::setMemory(uint8_t *buffer, uint8_t size) {
	memcpy(pixel_array, buffer, size * color_data_bytes);
	return true;
}

void WS2812B::update() {
	for (uint32_t i = 0; i < led_number * 3; i++) {
		for (uint8_t j = 0; j < 8; j++) {
			uint8_t byte = (pixel_array[i] >> (7 - j)) & 0x01;
			led_data_dma[40+i * 8 + j] =
					byte ? ws2812_comp_high : ws2812_comp_low;
		}
	}
	if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3,
			(uint32_t*) led_data_dma, led_number * 3 * 8 + 40 * 2) != HAL_OK) {
		Error_Handler();
	}
}
