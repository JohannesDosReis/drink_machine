/*
 * optical_reader.c
 *
 *  Created on: Nov 17, 2020
 *      Author: johan
 */

/* Includes ------------------------------------------------------------------*/
#include "optical_reader.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
/**
 * @brief Le o codigo da capsula inserida
 *
 * @retval uint8_t O valor do codigo da capsula inserida
 */
uint8_t read_optical_code() {
    return ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)) | (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) << 1) |
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) << 2);
}

/* External variables --------------------------------------------------------*/
