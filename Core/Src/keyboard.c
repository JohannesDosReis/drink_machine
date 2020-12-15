/*
 * keyboard.c
 *
 *  Created on: Nov 17, 2020
 *      Author: johan
 */

/* Includes ------------------------------------------------------------------*/
#include "keyboard.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Le se um botão foi prescionado
 *
 * @retval uint8_t 0 - nenhum botão foi prescionado | 1 - 'S' foi prescionado | 2 - '-' foi
 * prescionado | 3 - '+' foi prescionado | 4 - 'C' foi prescionado
 */
uint8_t read_keyboard() {
    static uint8_t state = 0x00;
    uint8_t buttonPressed = 0x00;

    // read buttons state
    state = (state & 0xF0) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) |
            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) << 1 | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) << 2 |
            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) << 3;

    if (state) {
        if (!(state & 0xF0)) { /* button pressed */
            state <<= 4;
        } else if (!(state & 0x0F)) { /* button unpressed */
            buttonPressed = (state & 0x80)
                                ? 4
                                : (state & 0x40) ? 3 : (state & 0x20) ? 2 : (state & 0x10) ? 1 : 0;
            state = 0;
            return buttonPressed;
        }
    }
    return 0;
}
/* External variables --------------------------------------------------------*/
