/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "co2_cylinder.h"
#include "keyboard.h"
#include "lcd_display.h"
#include "optical_reader.h"
#include "temperature_sensors.h"
#include "water_filter.h"
#include "heater.h"
#include "cooler.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    uint16_t liquid_time;
    uint16_t co2_time;
    uint8_t temperature;
    char *description;
} drink_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define MAX_DUTY_CYCLE_VALUE 1120
#define CONTROL_TIME 50
#define ASCENDING_RAMP 200
#define DESCENDING_RAMP 250
#define HEARTBEAT_TIME 300

#define AGUA_GELADA_INDEX 0
#define AGUA_NATURAL_INDEX 1
#define AGUA_QUENTE_INDEX 2
#define AGUA_COM_GAS_INDEX 3
#define CHA_GELADO_INDEX 4
#define CHA_QUENTE_INDEX 5
#define REFRIGERANTE_INDEX 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef sTime;
uint32_t dutyCycle;
uint8_t capsule_id, water_filter, co2_cylinder, warnings, water_option, preparing;
uint8_t heater_temperature, cooler_temperature, adc_data_ready;

drink_t drinks[9] = {
    {.temperature = 15,
     .liquid_time = 3000,
     .co2_time = 0,
     .description = "   água gelada  "}, /* agua gelada */
    {.temperature = 0,
     .liquid_time = 3000,
     .co2_time = 0,
     .description = "  água natural  "}, /* agua natural */
    {.temperature = 60,
     .liquid_time = 3000,
     .co2_time = 0,
     .description = "   água quente  "}, /* agua quente */
    {.temperature = 15,
     .liquid_time = 3000,
     .co2_time = 1500,
     .description = "  água com gás  "}, /* agua com gas */
    {.temperature = 20,
     .liquid_time = 2700,
     .co2_time = 0,
     .description = "   chá gelado   "}, /* cha gelado */
    {.temperature = 60,
     .liquid_time = 2700,
     .co2_time = 0,
     .description = "   chá quente   "}, /* cha quente */
    {.temperature = 20,
     .liquid_time = 4000,
     .co2_time = 2560,
     .description = "  refrigerante  "} /* refrigerante */
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void show_display(uint8_t view);
void prepare_drink(uint8_t index);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */
    uint32_t timer_heart = 0;
    uint8_t key = 0, drink_id = 0;
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_RTC_Init();
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, adc_data, NUMBER_OF_CONVERSIONS);
    lcd_init();
    //    sTime.Hours = 23;
    //    sTime.Minutes = 59;
    //    sTime.Seconds = 0;
    //    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        // le as temperaturas do heater e do cooler
        read_temperature_sensors();

        if (!preparing) {

            // ler o codido da capsula
            if (!capsule_id) capsule_id = read_optical_code();

            // verificar se o filtro de agua esta saturado
            water_filter = water_filter_saturated();

            // verificar se o cilindro de co2 esta vazio
            co2_cylinder = co2_cylinder_empty();

            // configura avisos
            warnings = water_filter | (co2_cylinder << 1);

            // mostrar display
            switch (capsule_id) {
                    // bebidas com agua
                case 1: /* agua  */
                case 3: /* cha gelado */
                case 4: /* cha quente */
                    if (water_filter)
                        show_display(8);
                    else
                        show_display(capsule_id);
                    break;

                    // bebidas com agua e gas
                case 2: /* agua com gas */
                case 5: /* refrigerante */
                    if (water_filter)
                        show_display(8);
                    else if (co2_cylinder)
                        show_display(9);
                    else
                        show_display(capsule_id);
                    break;

                default: show_display(0); break;
            }

            // le o teclado
            key = read_keyboard();

            if (key == 0x04) {
                water_option = capsule_id = key = 0x00;
            } else if (key == 0x03) {
                switch (capsule_id) {
                    case 0x01: water_option = (water_option > 0) ? water_option - 1 : 2; break;
                }
            } else if (key == 0x02) {
                switch (capsule_id) {
                    case 0x01: water_option = (water_option < 2) ? water_option + 1 : 0; break;
                }
            } else if (key == 0x01) {
                switch (capsule_id) {
                        // bebidas com agua
                    case 1: /* agua  */
                        if (!water_filter) {
                            drink_id = water_option;
                            preparing = 1;
                        }
                        break;
                    case 3: /* cha gelado */
                        if (!water_filter) {
                            water_option = 0;
                            drink_id = CHA_GELADO_INDEX;
                            preparing = 1;
                        }
                        break;
                    case 4: /* cha quente */
                        if (!water_filter) {
                            water_option = 2;
                            drink_id = CHA_QUENTE_INDEX;
                            preparing = 1;
                        }
                        break;

                        // bebidas com agua e gas
                    case 2: /* agua com gas */
                        if (!(water_filter | co2_cylinder)) {
                            water_option = 0;
                            drink_id = AGUA_COM_GAS_INDEX;
                            preparing = 1;
                        }
                        break;
                    case 5: /* refrigerante */
                        if (!(water_filter | co2_cylinder)) {
                            water_option = 0;
                            drink_id = REFRIGERANTE_INDEX;
                            preparing = 1;
                        }
                        break;
                }
            }
        } else {
            prepare_drink(drink_id);
        }
        if (timer_heart < HAL_GetTick()) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
            timer_heart = HAL_GetTick() + HEARTBEAT_TIME;
        }

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    lcd_sendText("Acabou");
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 16;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_6;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_7;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_8;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_9;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_10;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_11;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_12;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_13;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_14;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_15;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /** Configure Regular Channel
     */
    sConfig.Rank = ADC_REGULAR_RANK_16;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

    /* USER CODE BEGIN RTC_Init 0 */

    /* USER CODE END RTC_Init 0 */

    /* USER CODE BEGIN RTC_Init 1 */

    /* USER CODE END RTC_Init 1 */
    /** Initialize RTC Only
     */
    hrtc.Instance = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
    hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) { Error_Handler(); }
    /* USER CODE BEGIN RTC_Init 2 */

    /* USER CODE END RTC_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 1120 - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                          GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_4,
                      GPIO_PIN_RESET);

    /*Configure GPIO pins : PA0 PA1 PA11 PA12
                             PA15 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PA6 PA7 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB0 PB1 PB10 PB11
                             PB12 PB13 PB14 PB15
                             PB4 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                          GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PB6 PB7 PB8 PB9 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/**
 * @brief Funcao de callback do dma
 *
 * @param hadc adc handle
 *  @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) { adc_data_ready = 1; }

/**
 * @brief Mostra uma tela no display de acordo com o id
 *
 * @param view A tela a ser mostrada
 */
void show_display(uint8_t view) {
    char lcd_text[2][33];
    switch (view) {
            // escolha de liquido
        case 1:

        	strcpy(lcd_text[0], "      água      ");
            switch (water_option) {
                case 0: strcpy(lcd_text[1], " <   gelada   > "); break;
                case 1: strcpy(lcd_text[1], " <   natural  > "); break;
                case 2: strcpy(lcd_text[1], " <   quente   > "); break;

                default: sprintf(lcd_text[1], " <   natural  > "); break;
            }
            break;

        case 2:
        case 3:
        case 4:
        case 5:
            strcpy(lcd_text[0], drinks[view + 1].description);
            strcpy(lcd_text[1], "   Confirmar ?  ");
            break;

        // avisos
        case 8:
        	strcpy(lcd_text[0], " filtro de agua ");
        	strcpy(lcd_text[1], "    saturado    ");
            break;

        case 9:
        	strcpy(lcd_text[0], "  cilindro Co2  ");
        	strcpy(lcd_text[1], "     vazio      ");
            break;

        case 10:
        	if (capsule_id == 1)
        		strcpy(lcd_text[0], drinks[water_option].description);
			else
        		strcpy(lcd_text[0], drinks[capsule_id + 1].description);
			sprintf(lcd_text[1], " resfriando t=%02d ", cooler_temperature);
            break;

        case 11:
        	if (capsule_id == 1)
				strcpy(lcd_text[0], drinks[water_option].description);
			else
				strcpy(lcd_text[0], drinks[capsule_id + 1].description);
			strcpy(lcd_text[1], "   preparando   ");
            break;

        case 12:
        	if (capsule_id == 1)
				strcpy(lcd_text[0], drinks[water_option].description);
			else
				strcpy(lcd_text[0], drinks[capsule_id + 1].description);
            sprintf(lcd_text[1], " aquecendo t=%02d ", heater_temperature);
            break;

        default:
            HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
            sprintf(lcd_text[0], " hora: %02d:%02d:%02d ", sTime.Hours, sTime.Minutes,
                    sTime.Seconds);
            switch (warnings) {
                case 3: strcpy(lcd_text[1], " !água!   !co2! "); break;
                case 2: strcpy(lcd_text[1], "     !co2!      "); break;
                case 1: strcpy(lcd_text[1], "     !água!     "); break;

                default: strcpy(lcd_text[1], "                "); break;
            }
            break;
    }
    lcd_setCursor(0, 0);
    lcd_sendText(lcd_text[0]);
    lcd_setCursor(1, 0);
    lcd_sendText(lcd_text[1]);
}

/**
 * @brief Prepara a bebida
 *
 * @param index O index da bebida a ser preparada
 */
void prepare_drink(uint8_t index) {
    static uint32_t time0, time1, time2, time3, time4, timer1 = 0, timer;
    uint16_t duty_cycle = 0;
    static uint8_t valve;
    if (preparing) {
        if (timer1 < HAL_GetTick()) {
            timer1 = HAL_GetTick() + CONTROL_TIME;
            switch (water_option) {
                case 0:
                    if (preparing == 1) show_display(10);
                    set_cooler_temperture(drinks[index].temperature);
                    if (preparing == 1 && abs(cooler_temperature - drinks[index].temperature) < 2) {
                        preparing = 2;
                        valve = 0;
                        show_display(11);
                        time0 = HAL_GetTick();
                        time1 = time0 + ASCENDING_RAMP;
                        time2 = time0 + drinks[index].liquid_time - DESCENDING_RAMP;
                        time3 = time0 + drinks[index].liquid_time;
                        time4 = time0 + drinks[index].co2_time;
                    }
                    break;
                case 1:
                    if (preparing == 1) {
                        preparing = 2;
                        valve = 1;
                        show_display(11);
                        time0 = HAL_GetTick();
                        time1 = time0 + ASCENDING_RAMP;
                        time2 = time0 + drinks[index].liquid_time - DESCENDING_RAMP;
                        time3 = time0 + drinks[index].liquid_time;
                        time4 = time0 + drinks[index].co2_time;
                    }

                    break;
                case 2:
                    if (preparing == 1) show_display(12);
                    set_heater_temperture(drinks[index].temperature);
                    if (preparing == 1 && abs(heater_temperature - drinks[index].temperature) < 2) {
                        preparing = 2;
                        valve = 2;
                        show_display(11);
                        time0 = HAL_GetTick();
                        time1 = time0 + ASCENDING_RAMP;
                        time2 = time0 + drinks[index].liquid_time - DESCENDING_RAMP;
                        time3 = time0 + drinks[index].liquid_time;
                        time4 = time0 + drinks[index].co2_time;
                    }
                    break;
            }
        }

        if (preparing == 2) {
            timer = HAL_GetTick();

            // controle valvula co2
            if (timer < time4)
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            switch (valve) {
                case 0:
                    if (timer < time3)
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
                    else
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
                    break;
                case 1:
                    if (timer < time3)
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
                    else
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
                    break;
                case 2:
                    if (timer < time3)
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
                    else
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
                    break;
            }
            //  rampa de subida
            if (timer < time1)
                //            	duty_cycle = MAX_DUTY_CYCLE_VALUE / 2;
                duty_cycle = (timer - time0) * MAX_DUTY_CYCLE_VALUE / ASCENDING_RAMP;
            //
            else if (timer < time2)
                //                duty_cycle = MAX_DUTY_CYCLE_VALUE;
                duty_cycle = MAX_DUTY_CYCLE_VALUE;
            // rampa de descida
            else if (timer < time3)
                //                duty_cycle = MAX_DUTY_CYCLE_VALUE / 2;
                duty_cycle = (timer - time2) * MAX_DUTY_CYCLE_VALUE / DESCENDING_RAMP;
            else {
                TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = capsule_id = preparing = 0;
                return;
            }

            TIM1->CCR1 = duty_cycle;
            //            TIM1->CCR1 = MAX_DUTY_CYCLE_VALUE;
        }
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
