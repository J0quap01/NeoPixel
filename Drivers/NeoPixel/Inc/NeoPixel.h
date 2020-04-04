/*
 * NeoPixel.h
 *
 *  Created on: 01.04.2020
 *      Author: Lukas
 */

#ifndef NEOPIXEL_H_
#define NEOPIXEL_H_

/* Public includes -----------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>

/* Public define -------------------------------------------------------------*/
#define NEOPIXEL_MAX_COLORS 4

/* Public typedef ------------------------------------------------------------*/
// LED type
typedef enum e_NeoPixel_Type_t{
	e_SK6812_RGB,
	e_SK6812_RGBW,
	e_SK6812_WWA,
	e_WS2812
}e_NeoPixel_Type_t;

// struct for every pixel
typedef struct s_NeoPixel_t{
	uint8_t u8_colors[NEOPIXEL_MAX_COLORS];	// color values

	e_NeoPixel_Type_t e_LED_Type;

	struct s_NeoPixel_t *sp_NeoPixel_next;
}s_NeoPixel_t;

// struct for output buffer
typedef struct s_NeoPixel_buffer_t{
	uint8_t *u8_Buffer;
	size_t size;
	size_t index;
}s_NeoPixel_buffer_t;

// status
typedef enum e_NeoPixel_status_t{
	e_NeoPixel_OK,
	e_NeoPixel_BUSY,
	e_NeoPixel_ERROR
}e_NeoPixel_status_t;

/* Public macro --------------------------------------------------------------*/
#define NEOPIXEL_BIT_PER_PIXEL(num_color, bit_per_color)		(num_color*bit_per_color)
#define NEOPIXEL_BYTE_PER_PIXEL(num_color, bit_per_color)		((num_color*bit_per_color)/8)
#define NEOPIXEL_BIT_PER_PIXEL_SPI(num_color, bit_per_color)	(num_color*bit_per_color*3)
#define NEOPIXEL_BYTE_PER_PIXEL_SPI(num_color, bit_per_color)	((num_color*bit_per_color*3)/8)

/* Public variables ----------------------------------------------------------*/
// enum for color order
enum {
	e_SK6812_RGB_green,
	e_SK6812_RGB_red,
	e_SK6812_RGB_blue
};

enum {
	e_SK6812_RGBW_green,
	e_SK6812_RGBW_red,
	e_SK6812_RGBW_blue,
	e_SK6812_RGBW_white
};

enum {
	e_SK6812_WWA_natural_white,
	e_SK6812_WWA_amber,
	e_SK6812_WWA_warm_white
};

s_NeoPixel_t *s_debug;

/* Public function prototypes ------------------------------------------------*/
e_NeoPixel_status_t e_NeoPixel_init(uint16_t u16_num, e_NeoPixel_Type_t e_type, s_NeoPixel_t **spp_head,s_NeoPixel_buffer_t *sp_buffer);
e_NeoPixel_status_t e_NeoPixel_write(s_NeoPixel_t *sp_head, s_NeoPixel_buffer_t *s_buffer);
e_NeoPixel_status_t e_NeoPixel_deinit(s_NeoPixel_t *sp_NeoPixel);
e_NeoPixel_status_t e_NeoPixel_fadePixel(s_NeoPixel_t *sp_NeoPixel, uint8_t u8_dest_Color[NEOPIXEL_MAX_COLORS]);
e_NeoPixel_status_t e_NeoPixel_fadeChain(s_NeoPixel_t *sp_NeoPixel_head, uint8_t u8_dest_Color[NEOPIXEL_MAX_COLORS]);
e_NeoPixel_status_t e_NeoPixel_SetAll(s_NeoPixel_t *sp_NeoPixel, uint8_t u8_dest_Color[NEOPIXEL_MAX_COLORS]);

#endif /* NEOPIXEL_H_ */
