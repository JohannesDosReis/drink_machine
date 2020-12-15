/*
 * lcd_display.h
 *
 *  Created on: Nov 17, 2020
 *      Author: johannes
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_LCD_DISPLAY_H_
#define INC_LCD_DISPLAY_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
/* Exported types ------------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/
#define LCD_RS GPIO_PIN_10
// #define LCD_RW GPIO_PIN_14
#define LCD_EN GPIO_PIN_11
#define LCD_D4 GPIO_PIN_12
#define LCD_D5 GPIO_PIN_13
#define LCD_D6 GPIO_PIN_14
#define LCD_D7 GPIO_PIN_15
#define LCD_PORT GPIOB
#define LCD_D_ALL (LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7)

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void lcd_sendByte(unsigned char cmd);
void lcd_sendCmd(unsigned char cmd);
void lcd_sendData(uint32_t data);
void lcd_sendText(char *text);
void lcd_setCursor(unsigned char line, unsigned char column);
void lcd_clear(void);
void lcd_init(void);
void lcd_setCustomChar(unsigned char loc, uint16_t key, unsigned char *character);

#endif /* INC_LCD_DISPLAY_H_ */
