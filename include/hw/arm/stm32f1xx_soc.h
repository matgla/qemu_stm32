/*
 * STM32F1XX SOC Model 
 *
 * Copyright (c) 2020 Mateusz Stadnik <matgla@live.com>
 *
 * Based on ST RM0008 Reference Manual
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

enum Stm32F1xxPeripherals
{
    STM32F1XX_UNKNOWN_PERIPH = -1,
    STM32F1XX_RCC = 0,
    /* AHB */
    STM32F1XX_SDIO,
    STM32F1XX_FSMC
    STM32F1XX_CRC,
    STM32F1XX_DMA2,
    STM32F1XX_DMA1,
    /* APB2 */
    STM32F1XX_TIM11,
    STM32F1XX_TIM10, 
    STM32F1XX_TIM9,
    STM32F1XX_ADC3,
    STM32F1XX_USART1,
    STM32F1XX_TIM8,
    STM32F1XX_SPI1,
    STM32F1XX_TIM1, 
    STM32F1XX_ADC2,
    STM32F1XX_ADC1, 
    STM32F1XX_IOPG,
    STM32F1XX_IOPF,
    STM32F1XX_IOPE,
    STM32F1XX_IOPD, 
    STM32F1XX_IOPC,
    STM32F1XX_IOPB,
    STM32F1XX_IOPA,
    STM32F1XX_AFIO,
    /* APB1 */
    STM32F1XX_DAC,
    STM32F1XX_PWR,
    STM32F1XX_BKP,
    STM32F1XX_CAN, 
    STM32F1XX_USB,
    STM32F1XX_I2C2, 
    STM32F1XX_I2C1,
    STM32F1XX_UART5, 
    STM32F1XX_UART4,
    STM32F1XX_UART3,
    STM32F1XX_UART2,
    STM32F1XX_SPI3, 
    STM32F1XX_SPI2,
    STM32F1XX_WWDG, 
    STM32F1XX_TIM14,
    STM32F1XX_TIM13, 
    STM32F1XX_TIM12,
    STM32F1XX_TIM7, 
    STM32F1XX_TIM6,
    STM32F1XX_TIM5, 
    STM32F1XX_TIM4,
    STM32F1XX_TIM3, 
    STM32F1XX_TIM2,
    STM32F1XX_RTC,
    /* other */
    STM32F1XX_NUMBER_OF_PERIPHS
};
