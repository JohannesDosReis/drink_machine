/*
 * cooler.c
 *
 *  Created on: Nov 17, 2020
 *      Author: johan
 */


/* Includes ------------------------------------------------------------------*/
#include "cooler.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t  cooler_temperature;
/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Configura a terpertura do cooler
 *
 * @param temperature
 */
void set_cooler_temperture(uint8_t temperature) {

    // control = erro * kp
    uint16_t control = (cooler_temperature - temperature) * K_P_COOLER;

    if (control < 0)
        control = 0;
    else if (control > MAX_VALUE_COOLER)
        control = MAX_VALUE_COOLER;

    //    TIM1->CCR2 = MAX_DUTY_CYCLE_VALUE;
    TIM1->CCR2 = control;
}



/* External variables --------------------------------------------------------*/
