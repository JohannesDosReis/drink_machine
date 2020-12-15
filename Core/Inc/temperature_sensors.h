/*
 * temperature_sensors.h
 *
 *  Created on: 14 de dez de 2020
 *      Author: johan
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_TEMPERATURE_SENSORS_H_
#define INC_TEMPERATURE_SENSORS_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* Exported types ------------------------------------------------------------*/


/* Exported defines ----------------------------------------------------------*/

#define NUMBER_OF_CONVERSIONS 16
#define CONVERSIONS_BY_CHANNEL 8
#define SAMPLES_NUMBER 32

/* Exported constants --------------------------------------------------------*/
extern uint8_t heater_temperature, cooler_temperature, adc_data_ready;
extern uint32_t adc_data[NUMBER_OF_CONVERSIONS];
/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void read_temperature_sensors();

#endif /* INC_TEMPERATURE_SENSORS_H_ */
