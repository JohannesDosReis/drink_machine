/*
 * optical_reader.h
 *
 *  Created on: Nov 17, 2020
 *      Author: johan
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_OPTICAL_READER_H_
#define INC_OPTICAL_READER_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
/* Exported types ------------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
uint8_t read_optical_code();

#endif /* INC_OPTICAL_READER_H_ */
