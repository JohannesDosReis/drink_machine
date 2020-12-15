/*
 * heater.h
 *
 *  Created on: Nov 17, 2020
 *      Author: johan
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_HEATER_H_
#define INC_HEATER_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "temperature_sensors.h"
/* Exported types ------------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/
#define MAX_VALUE_HEATER 1120
#define K_P_HEATER 60

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void set_heater_temperture(uint8_t temperature);

#endif /* INC_HEATER_H_ */
