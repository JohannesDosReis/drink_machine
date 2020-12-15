/*
 * water_filter.c
 *
 *  Created on: Nov 17, 2020
 *      Author: johan
 */

/* Includes ------------------------------------------------------------------*/
#include "water_filter.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Verifica se o filtro de agua esta saturado
 *
 * @retval uint8_t Se o filtro estiver saturado retorna 1 senao retorna 0
 */
uint8_t water_filter_saturated() { return !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0); }

/* External variables --------------------------------------------------------*/
