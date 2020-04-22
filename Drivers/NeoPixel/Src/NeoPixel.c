/*
 * NeoPixel.c
 *
 *  Created on: 01.04.2020
 *      Author: Lukas
 */

#include "NeoPixel.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define NEOPIXEL_RESET_CODE_LEN		35U
#define NEOPIXEL_RESET_CODE			(0x00U)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static e_NeoPixel_status_t e_NeoPixel_write_buffer(s_NeoPixel_t *sp_NeoPixel, s_NeoPixel_buffer_t *s_buffer);
static e_NeoPixel_status_t e_write_color_to_buffer(s_NeoPixel_buffer_t *sp_buffer, uint8_t u8_color);

/* Private user code ---------------------------------------------------------*/
/**
  * @brief init chained list for num of NeoPixel
  * @param 	u16_num:	number of NeoPixel
  * 		e_type:		type of NeoPixel
  * 		spp_head:		list head
  * @retval e_NeoPixel_status_t
  */
e_NeoPixel_status_t e_NeoPixel_init(uint16_t u16_num, e_NeoPixel_Type_t e_type, s_NeoPixel_t **spp_head,s_NeoPixel_buffer_t *sp_buffer){
	s_NeoPixel_t *sp_NeoPixel_last, *sp_NeoPixel_new;

	for(uint16_t i = 0; i < u16_num; i++){
		switch(i){
			case 0:
				// init head
				if(*spp_head != NULL){
					if(e_NeoPixel_deinit(*spp_head) == e_NeoPixel_ERROR){
						return e_NeoPixel_ERROR;
					}
				}
				*spp_head = (s_NeoPixel_t*)malloc(sizeof(s_NeoPixel_t));

				if(*spp_head == NULL) return e_NeoPixel_ERROR;
				(*spp_head)->e_LED_Type = e_type;
				(*spp_head)->sp_NeoPixel_next = (s_NeoPixel_t*)malloc(sizeof(s_NeoPixel_t));

				// create first NeoPixel
				if((*spp_head)->sp_NeoPixel_next == NULL) return e_NeoPixel_ERROR;

				sp_NeoPixel_new = (*spp_head)->sp_NeoPixel_next;
				break;
			default:
				sp_NeoPixel_last->sp_NeoPixel_next = (s_NeoPixel_t*)malloc(sizeof(s_NeoPixel_t));

				if(sp_NeoPixel_last->sp_NeoPixel_next == NULL) return e_NeoPixel_ERROR;

				sp_NeoPixel_new = sp_NeoPixel_last->sp_NeoPixel_next;
				break;
		}
		s_debug = sp_NeoPixel_new;
		sp_NeoPixel_new->sp_NeoPixel_next = NULL;

		sp_NeoPixel_new->e_LED_Type = e_type;
		for(int k = 0; k < NEOPIXEL_MAX_COLORS; k++){
			sp_NeoPixel_new->u8_colors[k] = 0U;
		}
//		sp_NeoPixel_new->u8_colors[0] = 0U;
//		sp_NeoPixel_new->u8_colors[1] = 1U;
//		sp_NeoPixel_new->u8_colors[2] = 0U;
//		sp_NeoPixel_new->u8_colors[3] = 0U;
		sp_NeoPixel_last = sp_NeoPixel_new;
	}

	// allocate output buffer
	if(sp_buffer->u8_Buffer != NULL){
		free(sp_buffer->u8_Buffer);
	}
	switch(e_type){
		case e_SK6812_RGB:
			sp_buffer->size = NEOPIXEL_RESET_CODE_LEN + u16_num * NEOPIXEL_BYTE_PER_PIXEL_SPI(3,8);
			break;
		case e_SK6812_WWA:
			sp_buffer->size = NEOPIXEL_RESET_CODE_LEN + u16_num * NEOPIXEL_BYTE_PER_PIXEL_SPI(3,8);
			break;
		case e_SK6812_RGBW:
			sp_buffer->size = NEOPIXEL_RESET_CODE_LEN + u16_num * NEOPIXEL_BYTE_PER_PIXEL_SPI(4,8);
			break;
		default:
			sp_buffer->size = NEOPIXEL_RESET_CODE_LEN + u16_num * NEOPIXEL_BYTE_PER_PIXEL_SPI(NEOPIXEL_MAX_COLORS,8);
			break;
	}
	sp_buffer->u8_Buffer = (uint8_t*)malloc(sp_buffer->size);
	if (sp_buffer->u8_Buffer == NULL) return e_NeoPixel_ERROR;

	for(sp_buffer->index = 0; sp_buffer->index < NEOPIXEL_RESET_CODE_LEN; sp_buffer->index++){
		sp_buffer->u8_Buffer[sp_buffer->index] = NEOPIXEL_RESET_CODE;
	}

	return e_NeoPixel_OK;
}

/**
  * @brief writes chained list on SPI interface
  * @param 	sp_head:	list head
  * @retval e_NeoPixel_status_t
  */
e_NeoPixel_status_t e_NeoPixel_write(s_NeoPixel_t *sp_head, s_NeoPixel_buffer_t *s_buffer){
	s_buffer->index = NEOPIXEL_RESET_CODE_LEN;
	e_NeoPixel_write_buffer(sp_head->sp_NeoPixel_next, s_buffer);

	return e_NeoPixel_OK;
}

/**
  * @brief writes values to buffer
  * @param 	sp_NeoPixel:	NeoPixel
  * @retval e_NeoPixel_status_t
  */
static e_NeoPixel_status_t e_NeoPixel_write_buffer(s_NeoPixel_t *sp_NeoPixel, s_NeoPixel_buffer_t *s_buffer){
	if(sp_NeoPixel == NULL) return e_NeoPixel_ERROR;

	for(int i = 0; i < NEOPIXEL_MAX_COLORS; i++){
		switch(sp_NeoPixel->e_LED_Type){
			case e_SK6812_RGB:
				switch(i){
					case e_SK6812_RGB_red:
						e_write_color_to_buffer(s_buffer, sp_NeoPixel->u8_colors[e_SK6812_RGB_red]);
						break;
					case e_SK6812_RGB_green:
						e_write_color_to_buffer(s_buffer, sp_NeoPixel->u8_colors[e_SK6812_RGB_green]);
						break;
					case e_SK6812_RGB_blue:
						e_write_color_to_buffer(s_buffer, sp_NeoPixel->u8_colors[e_SK6812_RGB_green]);
						break;
					default:
						break;
				}
				break;
			case e_SK6812_RGBW:
				switch(i){
					case e_SK6812_RGBW_red:
						e_write_color_to_buffer(s_buffer, sp_NeoPixel->u8_colors[e_SK6812_RGBW_red]);
						break;
					case e_SK6812_RGBW_green:
						e_write_color_to_buffer(s_buffer, sp_NeoPixel->u8_colors[e_SK6812_RGBW_green]);
						break;
					case e_SK6812_RGBW_blue:
						e_write_color_to_buffer(s_buffer, sp_NeoPixel->u8_colors[e_SK6812_RGBW_blue]);
						break;
					case e_SK6812_RGBW_white:
						e_write_color_to_buffer(s_buffer, sp_NeoPixel->u8_colors[e_SK6812_RGBW_white]);
						break;
					default:
						break;
				}
				break;
			case e_SK6812_WWA:
				switch(i){
					case e_SK6812_WWA_natural_white:
						e_write_color_to_buffer(s_buffer, sp_NeoPixel->u8_colors[e_SK6812_WWA_natural_white]);
						break;
					case e_SK6812_WWA_warm_white:
						e_write_color_to_buffer(s_buffer, sp_NeoPixel->u8_colors[e_SK6812_WWA_warm_white]);
						break;
					case e_SK6812_WWA_amber:
						e_write_color_to_buffer(s_buffer, sp_NeoPixel->u8_colors[e_SK6812_WWA_amber]);
						break;
					default:
						break;
				}
				break;
			case e_WS2812:
				break;
			default:
				break;
		}
	}
	if(sp_NeoPixel->sp_NeoPixel_next != NULL){
		if(e_NeoPixel_write_buffer(sp_NeoPixel->sp_NeoPixel_next, s_buffer) != e_NeoPixel_OK){
			return e_NeoPixel_ERROR;
		}
	}

	return e_NeoPixel_OK;
}

/**
  * @brief writes color to buffer with pattern
  * @param 	sp_buffer:	NeoPixel to deinit
  * 		, u8_color:	color value
  * @retval e_NeoPixel_status_t
  */
static e_NeoPixel_status_t e_write_color_to_buffer(s_NeoPixel_buffer_t *sp_buffer, uint8_t u8_color){
	uint32_t u32_color_data ;

	u32_color_data = 0;
	for(int j = 0; j < 8; j++){
		u32_color_data = u32_color_data | (1U << ((j * 3) + 2)) |
				((((uint32_t)u8_color >> j) & 1U) << ((j * 3) + 1));
	}
	sp_buffer->u8_Buffer[sp_buffer->index++] = *((uint8_t*)&u32_color_data + 2);
	sp_buffer->u8_Buffer[sp_buffer->index++] = *((uint8_t*)&u32_color_data + 1);
	sp_buffer->u8_Buffer[sp_buffer->index++] = *((uint8_t*)&u32_color_data);
	return e_NeoPixel_OK;
}

/**
  * @brief deletes full chain
  * @param 	sp_NeoPixel:	NeoPixel to deinit
  * @retval
  * e_NeoPixel_status_t
  */
e_NeoPixel_status_t e_NeoPixel_deinit(s_NeoPixel_t *sp_NeoPixel){
	if(sp_NeoPixel->sp_NeoPixel_next != NULL){
		if(e_NeoPixel_deinit(sp_NeoPixel->sp_NeoPixel_next) == e_NeoPixel_OK){
			sp_NeoPixel->sp_NeoPixel_next = NULL;
		}
		else{
			return e_NeoPixel_ERROR;
		}
	}
	free(sp_NeoPixel);
	return e_NeoPixel_OK;
}

/**
  * @brief  fades pixel up/down to a specific value, repeated call (for example timer interrupt) changes value
  * @param  sp_NeoPixel		pointer to pixel to change
  *         u8_dest_Color	array containing destination color
  *         u8_Step			step to increment
  * @retval e_NeoPixel_status_t:	OK if destination color value reched otherwise BUSY
  */
e_NeoPixel_status_t e_NeoPixel_fadePixel(s_NeoPixel_t *sp_NeoPixel, uint8_t u8_dest_Color[NEOPIXEL_MAX_COLORS], uint8_t u8_Step){
	e_NeoPixel_status_t e_status = e_NeoPixel_OK;
	static uint8_t *u8_Color;

	if((u8_dest_Color != NULL) | (u8_Color == NULL)){
		u8_Color = u8_dest_Color;
	}
	for(uint64_t i = 0; i < NEOPIXEL_MAX_COLORS; i++){
		if(sp_NeoPixel->u8_colors[i] < u8_Color[i]){
			if(u8_Step > u8_Color[i] - sp_NeoPixel->u8_colors[i]){
				sp_NeoPixel->u8_colors[i] = u8_Color[i];
			}
			else {
				sp_NeoPixel->u8_colors[i] += u8_Step;
			}
			e_status = e_NeoPixel_BUSY;
		} else if(sp_NeoPixel->u8_colors[i] > u8_Color[i]){
			if(u8_Step > sp_NeoPixel->u8_colors[i] - u8_Color[i]){
				sp_NeoPixel->u8_colors[i] = u8_Color[i];
			}
			else {
				sp_NeoPixel->u8_colors[i] -= u8_Step;
			}
			e_status = e_NeoPixel_BUSY;
		}
	}

	return e_status;
}

/**
  * @brief  fades chain up/down to a specific value, repeated call (for example timer interrupt) changes value
  * @param  sp_NeoPixel		pointer to head of chain
  *         u8_dest_Color	array containing destination color
  *         u8_Step			step to increment
  * @retval e_NeoPixel_status_t:	OK if whole chain finished otherwise BUSY
  */
e_NeoPixel_status_t e_NeoPixel_fadeChain(s_NeoPixel_t *sp_NeoPixel_head, uint8_t u8_dest_Color[NEOPIXEL_MAX_COLORS], uint8_t u8_Step){
	static s_NeoPixel_t *sp_NeoPixel;

	if(sp_NeoPixel == NULL){
		sp_NeoPixel = sp_NeoPixel_head->sp_NeoPixel_next;
	}

	if(e_NeoPixel_fadePixel(sp_NeoPixel, u8_dest_Color, u8_Step) == e_NeoPixel_OK){
		sp_NeoPixel = sp_NeoPixel->sp_NeoPixel_next;

		if(sp_NeoPixel == NULL){
			return e_NeoPixel_OK;
		}
	}

	return e_NeoPixel_BUSY;
}

/**
  * @brief  sets whole chain to the same color
  * @param  sp_NeoPixel		pointer to pixel to change
  *         u8_dest_Color	array containing destination color
  * @retval e_NeoPixel_status_t
  */
e_NeoPixel_status_t e_NeoPixel_SetAll(s_NeoPixel_t *sp_NeoPixel, uint8_t u8_Color[NEOPIXEL_MAX_COLORS]){
	while (sp_NeoPixel != NULL){
		for(uint64_t i = 0; i < NEOPIXEL_MAX_COLORS; i++){
			sp_NeoPixel->u8_colors[i] = u8_Color[i];
		}

		sp_NeoPixel = sp_NeoPixel->sp_NeoPixel_next;
	}

	return e_NeoPixel_OK;
}
