/**
 * original author:  Tilen Majerle<tilen@majerle.eu>
 * modification for STM32f10x: Alexander Lutsai<s.lyra@ya.ru>

   ----------------------------------------------------------------------
   	Copyright (C) Alexander Lutsai, 2016
    Copyright (C) Tilen Majerle, 2015
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.
     
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */
#ifndef OLED_H
#define OLED_H 100

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

/**
 * This OLED LCD uses I2C for communication
 *
 * Library features functions for drawing lines, rectangles and circles.
 *
 * It also allows you to draw texts and characters using appropriate functions provided in library.
 *
 * Default pinout
 *
OLED    |STM32F10x    |DESCRIPTION

VCC        |3.3V         |
GND        |GND          |
SCL        |PB6          |Serial clock line
SDA        |PB7          |Serial data line
 */

#include "stm32wbxx_hal.h"

#include "fonts.h"

#include "stdlib.h"
#include "string.h"


/* I2C address */
#ifndef OLED_I2C_ADDR
#define OLED_I2C_ADDR         0x78
//#define OLED_I2C_ADDR       0x7A
#endif

/* OLED settings */
/* OLED width in pixels */
#ifndef OLED_WIDTH
#define OLED_WIDTH            128
#endif
/* OLED LCD height in pixels */
#ifndef OLED_HEIGHT
#define OLED_HEIGHT           64
#endif

/**
 * @brief  OLED color enumeration
 */
typedef enum {
	OLED_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
	OLED_COLOR_WHITE = 0x01  /*!< Pixel is set. Color depends on LCD */
} OLED_COLOR_t;



/**
 * @brief  Initializes OLED LCD
 * @param  None
 * @retval Initialization status:
 *           - 0: LCD was not detected on I2C port
 *           - > 0: LCD initialized OK and ready to use
 */
uint8_t OLED_Init(void);

void OLED_update_screen(void);
void OLED_toggle_invert(void);
void OLED_fill(OLED_COLOR_t Color);
void OLED_draw_pixel(uint16_t x, uint16_t y, OLED_COLOR_t color);
void OLED_gotoXY(uint16_t x, uint16_t y);
char OLED_putc(char ch, FontDef_t* Font, OLED_COLOR_t color);
char OLED_puts(char* str, FontDef_t* Font, OLED_COLOR_t color);
void OLED_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, OLED_COLOR_t c);
void OLED_draw_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, OLED_COLOR_t c);

void OLED_draw_filled_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, OLED_COLOR_t c);
void OLED_draw_triangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, OLED_COLOR_t color);
void OLED_draw_circle(int16_t x0, int16_t y0, int16_t r, OLED_COLOR_t c);
void OLED_draw_filled_circle(int16_t x0, int16_t y0, int16_t r, OLED_COLOR_t c);

#ifndef OLED_I2C_TIMEOUT
#define OLED_I2C_TIMEOUT					20000
#endif


void OLED_I2C_Init();
void OLED_I2C_Write(uint8_t address, uint8_t reg, uint8_t data);
void OLED_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count);
void OLED_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color);
void OLED_ScrollRight(uint8_t start_row, uint8_t end_row);
void OLED_ScrollLeft(uint8_t start_row, uint8_t end_row);
void OLED_Scrolldiagright(uint8_t start_row, uint8_t end_row);
void OLED_scroll_diag_left(uint8_t start_row, uint8_t end_row);
void OLED_stop_scroll(void);
void OLED_invert_display (int i);






// clear the display

void OLED_Clear (void);


/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
