/*
 * co2_cylinder.c
 *
 *  Created on: Nov 17, 2020
 *      Author: johan
 */

/* Includes ------------------------------------------------------------------*/
#include "temperature_sensors.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Verifica se o cilindro de co2 esta vazio
 *
 * @retval uint8_t Se o cilindro estiver vazio retorna 1 senao retorna 0
 */
uint8_t co2_cylinder_empty() { return !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1); }

/* External variables --------------------------------------------------------*/
