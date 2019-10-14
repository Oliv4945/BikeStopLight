#ifndef __WS2812B_H
#define __WS2812B_H

#include <stdint.h>
#include <inttypes.h>

typedef struct {
  uint8_t g;
  uint8_t r;
  uint8_t b;
} s_color;

class WS2812B {
public:
  WS2812B(uint8_t led_number);
  bool setColorAll(s_color color);
  bool setColorIndex(uint8_t index, s_color color);
  bool setColorRange(uint8_t index, uint8_t range, s_color color);
  bool setMemory(uint8_t *buffer, uint8_t size);
  void update();

private:
  const uint8_t color_data_bytes = 3; // 3 For RGB, 4 for RGBW (Not yet implemented)
  const uint8_t ws2812_comp_low = 6; // 0.4us / 8MHz
  const uint8_t ws2812_comp_high = 11;
  uint8_t *led_data_dma;
  uint8_t *pixel_array;
  uint8_t led_number;
};

#endif /* __WS2812B_H */
