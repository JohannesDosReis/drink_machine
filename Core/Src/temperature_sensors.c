/*
 * temperature_sensors.c
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
uint32_t adc_data[NUMBER_OF_CONVERSIONS];

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Le a temperatura dos sensores presentes no heater e no cooler
 *
 * @retval None
 */
void read_temperature_sensors() {
    static uint8_t data_samples = 0;
    static uint32_t heater_adc_data = 0, cooler_adc_data = 0;
    uint8_t i;

    if (adc_data_ready) {
        adc_data_ready = 0;

        data_samples += CONVERSIONS_BY_CHANNEL;

        for (i = 0; i < CONVERSIONS_BY_CHANNEL; i++) {
            heater_adc_data += adc_data[i];
            cooler_adc_data += adc_data[i + CONVERSIONS_BY_CHANNEL];
        }

        if (data_samples >= SAMPLES_NUMBER) {
            heater_adc_data /= SAMPLES_NUMBER;
            cooler_adc_data /= SAMPLES_NUMBER;

            heater_temperature = ((heater_adc_data * 95) / 4095) + 5;
            cooler_temperature = ((cooler_adc_data * 45) / 4095) + 5;

            data_samples = heater_adc_data = cooler_adc_data = 0;
        }
    }
}

/* External variables --------------------------------------------------------*/
