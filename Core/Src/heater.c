/*
 * heater.c
 *
 *  Created on: Nov 17, 2020
 *      Author: johan
 */


/* Includes ------------------------------------------------------------------*/
#include "heater.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t heater_temperature;
/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Configura a temperatura do aquecedor
 *
 * @param temperature
 */
void set_heater_temperture(uint8_t temperature) {

    // control = erro * kp
    uint16_t control = (temperature - heater_temperature) * K_P_HEATER;

    if (control < 0)
        control = 0;
    else if (control > MAX_VALUE_HEATER)
        control = MAX_VALUE_HEATER;

    //    TIM1->CCR3 = MAX_DUTY_CYCLE_VALUE;
    TIM1->CCR3 = control;
}

/* External variables --------------------------------------------------------*/
