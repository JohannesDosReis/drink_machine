/*
 * lcd_display.c
 *
 *  Created on: Nov 17, 2020
 *      Author: johannes
 */

/* Includes ------------------------------------------------------------------*/
#include "lcd_display.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// caracteres personalizados padrões que serão carregados na CGRAM.
unsigned char UserFont[5][8] = {
    {0x00, 0x0E, 0x10, 0x10, 0x11, 0x0E, 0x04, 0x00}, /* Value for ç */
    {0x02, 0x04, 0x0E, 0x01, 0x0F, 0x11, 0x0F, 0x00}, /* Value for á */
    {0x02, 0x04, 0x0E, 0x11, 0x1F, 0x10, 0x0E, 0x00}, /* Value for é */
    {0x0D, 0x12, 0x0E, 0x01, 0x0F, 0x11, 0x0F, 0x00}, /* Value for ã */
    {0x0D, 0x12, 0x0E, 0x11, 0x11, 0x11, 0x0E, 0x00}, /* Value for õ */
};

// caractres chave para cada um dos caracteres personalizados.
uint16_t UserKeys[5] = {0xC3A7, 0xC3A1, 0xC3A9, 0xC3A3, 0xC3B5};

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Envia um bit para o display lcd
 *
 * @param bts O bit a ser enviado
 * @retval None
 */
void lcd_sendByte(uint8_t bts) {
    uint8_t msbs; /* Os bits mais significativos*/
    uint8_t lsbs; /* Os bits menos significativos */

    HAL_GPIO_WritePin(LCD_PORT, LCD_D_ALL, GPIO_PIN_RESET);

    // Envia os bits mais significativos para o display
    msbs = (bts >> 4) & 0x0F;
    if (msbs & 0x01) HAL_GPIO_WritePin(LCD_PORT, LCD_D4, GPIO_PIN_SET);
    if (msbs & 0x02) HAL_GPIO_WritePin(LCD_PORT, LCD_D5, GPIO_PIN_SET);
    if (msbs & 0x04) HAL_GPIO_WritePin(LCD_PORT, LCD_D6, GPIO_PIN_SET);
    if (msbs & 0x08) HAL_GPIO_WritePin(LCD_PORT, LCD_D7, GPIO_PIN_SET);

    HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LCD_PORT, LCD_D_ALL, GPIO_PIN_RESET);

    // Envia os bits menos significativos para o display
    lsbs = bts & 0x0F;
    if (lsbs & 0x01) HAL_GPIO_WritePin(LCD_PORT, LCD_D4, GPIO_PIN_SET);
    if (lsbs & 0x02) HAL_GPIO_WritePin(LCD_PORT, LCD_D5, GPIO_PIN_SET);
    if (lsbs & 0x04) HAL_GPIO_WritePin(LCD_PORT, LCD_D6, GPIO_PIN_SET);
    if (lsbs & 0x08) HAL_GPIO_WritePin(LCD_PORT, LCD_D7, GPIO_PIN_SET);

    HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_RESET);
}

/**
 * @brief Envia um comando para o display
 *
 * @param cmd O código do comando a ser enviado
 * @retval None
 */
void lcd_sendCmd(uint8_t cmd) {
    HAL_GPIO_WritePin(LCD_PORT, LCD_RS, GPIO_PIN_RESET);
    HAL_Delay(1);
    lcd_sendByte(cmd);
}

/**
 * @brief Escreve um caracter no display
 *
 * @param data O caracter  a ser escrita
 * @retval None
 */
void lcd_sendData(uint32_t data) {
    uint8_t i;
    HAL_GPIO_WritePin(LCD_PORT, LCD_RS, GPIO_PIN_SET);
    HAL_Delay(1);

    // Verifica se é um caracter personalizados
    for (i = 0; i < sizeof(UserKeys); i++) {
        if (data == UserKeys[i]) { data = i; }
    }

    lcd_sendByte(data);
}

/**
 * @brief Escreve um string no displau lcd
 *
 * @param text A strign a ser escrita
 * @retval None
 */
void lcd_sendText(char *text) {
    uint8_t i;
    uint16_t data16;
    while (*text) {
        data16 = ((*text) << 8) | *(text + 1);
        for (i = 0; i < sizeof(UserKeys); i++) {
            if (data16 == UserKeys[i]) {
                lcd_sendData(data16);
                text += 2;
                continue;
            }
        }
        lcd_sendData(*text);
        text++;
    }
}

/**
 * @brief Limpa a tela do display
 *
 * @retval None
 */
void lcd_clear(void) { lcd_sendCmd(0x01); }

/**
 * @brief Coloca o cursor em uma determinada posicao
 *
 * @param line A linha em que o cursor sera colocado
 * @param column A coluna em que o cursor sera colocado
 * @retval None
 */
void lcd_setCursor(unsigned char line, unsigned char column) {
    uint8_t position = 0;

    switch (line) {
        case 0: position = 0x00; break;
        case 1: position = 0x40; break;
        default: position = 0x00; break;
    }
    lcd_sendCmd(0x80 | (position + column));
}

/**
 * @brief Inicializa o display lcd
 *
 * @retval None
 */
void lcd_init(void) {
    uint8_t i;
    unsigned char const *p;

    HAL_GPIO_WritePin(LCD_PORT, LCD_RS | LCD_EN | LCD_D_ALL, GPIO_PIN_RESET);
    HAL_Delay(150);
    for (i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LCD_PORT, LCD_D4 | LCD_D5, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LCD_PORT, LCD_D6 | LCD_D7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_SET);
        HAL_Delay(5);
        HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_RESET);
    }

    // Usar 4 bits
    HAL_GPIO_WritePin(LCD_PORT, LCD_D5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_PORT, LCD_D4 | LCD_D6 | LCD_D7, GPIO_PIN_RESET);

    // Pulso High-to-Low no pino Enable para gravar os dados
    HAL_GPIO_WritePin(LCD_PORT, LCD_EN, 1); // => E = 1
    HAL_Delay(5);
    HAL_GPIO_WritePin(LCD_PORT, LCD_EN, 0); // => E = 0

    lcd_sendCmd(0x28); /* 2 linahs, 5x8 matrix de caracters             */
    lcd_sendCmd(0x0C); /* Display ligado e cursor e blink desligados 	*/
    lcd_sendCmd(0x06); /* Mover cursos para direita após escrita        */
    lcd_sendCmd(0x01); /* Limpar display e colocar cursor no inicio     */

    HAL_Delay(5);

    /* Load user-specific characters into CGRAM */
    lcd_sendCmd(0x40); /* Set CGRAM address counter to 0     */
    p = &UserFont[0][0];
    for (i = 0; i < sizeof(UserFont); i++, p++)
        lcd_sendData(*p);
    lcd_sendCmd(0x80); /* Set DDRAM address counter to 0     */
}

/**
 * @brief Configura um novo caracter customizado
 *
 * @param loc A localizacao na CGRAM em que o caracter sera gravado
 * @param key A chave para o caracter ser escrito
 * @param character O caracter a ser escrito
 */
void lcd_setCustomChar(unsigned char loc, uint16_t key, unsigned char *character) {
    unsigned char i;
    if (loc < sizeof(UserFont)) {
        UserKeys[loc] = key;
        lcd_sendCmd(0x40 + (loc * 8)); /* Set CGRAM address counter */
        for (i = 0; i < 8; i++)        /* Write 8 byte for generation of 1 character */
            lcd_sendData(character[i]);
    }
}

/* External variables --------------------------------------------------------*/
