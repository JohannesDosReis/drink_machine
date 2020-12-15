/*
 * cooler.h
 *
 *  Created on: Nov 17, 2020
 *      Author: johan
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_COOLER_H_
#define INC_COOLER_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "temperature_sensors.h"
/* Exported types ------------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/
#define MAX_VALUE_COOLER 112
#define K_P_COOLER 10
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void set_cooler_temperture(uint8_t temperature);


#endif /* INC_COOLER_H_ */
