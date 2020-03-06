/*
* STM32 Reset & Clock Control Model
*
* Copyright (c) 2020 Mateusz Stadnik <matgla@live.com>
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

// #include <stdint.h>

#include "qemu/osdep.h" // qemu_irq
#include "hw/arm/stm32f1xx_rcc.h"
#include "hw/arm/stm32f1xx_soc.h"
#include "hw/arm/stm32_clk.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "qemu-common.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "hw/qdev-properties.h"

#define STM32F1XX_HSI_FREQUENCY 8000000
#define STM32F1XX_LSI_FREQUENCY 40000
#define STM32F1XX_LSE_FREQUENCY 32768

/* RCC_CR */
#define STM32F1XX_RCC_CR_OFFSET      0x0
#define STM32F1XX_PLL3READY_BIT      29
#define STM32F1XX_PLL3ON_BIT         28
#define STM32F1XX_PLL2READY_BIT      27
#define STM32F1XX_PLL2ON_BIT         26
#define STM32F1XX_PLLREADY_BIT       25
#define STM32F1XX_PLLON_BIT          24
#define STM32F1XX_CSSON_BIT          19
#define STM32F1XX_HSEBYP_BIT         18
#define STM32F1XX_HSERDY_BIT         17
#define STM32F1XX_HSEON_BIT          16
#define STM32F1XX_HSICAL_START_BIT   8
#define STM32F1XX_HSICAL_MASK        0x0000ff00
#define STM32F1XX_HSITRIM_START_BIT  3
#define STM32F1XX_HSITRIM_MASK       0x0000f800
#define STM32F1XX_HSIRDY_BIT         1
#define STM32F1XX_HSION_BIT          0

/* RCC_CFGR */
#define STM32F1XX_RCC_CFGR_OFFSET    0x4
#define STM32F1XX_MCO_START_BIT      24
#define STM32F1XX_MCO_MASK           0x07000000
#define STM32F1XX_OTGFSPRE_BIT       22
#define STM32F1XX_PLLMUL_START_BIT   18
#define STM32F1XX_PLLMUL_MASK        0x003C0000
#define STM32F1XX_PLLXTPRE_BIT       17
#define STM32F1XX_PLLSRC_BIT         16
#define STM32F1XX_ADCPRE_START_BIT   14
#define STM32F1XX_ADCPRE_MASK        0xC0000000
#define STM32F1XX_PPRE2_START_BIT    11
#define STM32F1XX_PPRE2_MASK         0x00003800
#define STM32F1XX_PPRE1_START_BIT    8
#define STM32F1XX_PPRE1_MASK         0x00000700
#define STM32F1XX_HPRE_START_BIT     4
#define STM32F1XX_HPRE_MASK          0x000000F0
#define STM32F1XX_SWS_START_BIT      2
#define STM32F1XX_SWS_MASK           0x0000000C
#define STM32F1XX_SW_START_BIT       0
#define STM32F1XX_SW_MASK            0x00000003

/* RCC_CIR */
#define STM32F1XX_RCC_CIR_OFFSET     0x8
#define STM32F1XX_CSSC_BIT           23
#define STM32F1XX_PLLRDYC_BIT        20
#define STM32F1XX_HSERDYC_BIT        19
#define STM32F1XX_HSIRDYC_BIT        18
#define STM32F1XX_LSERDYC_BIT        17
#define STM32F1XX_LSIRDYC_BIT        16
#define STM32F1XX_PLLEDYIE_BIT       12
#define STM32F1XX_HSERDYIE_BIT       11
#define STM32F1XX_HSIRDYIE_BIT       10
#define STM32F1XX_LSERDYIE_BIT       9
#define STM32F1XX_LSIRDYIE_BIT       8
#define STM32F1XX_CSSF_BIT           7
#define STM32F1XX_PLLRDYF_BIT        4
#define STM32F1XX_HSERDYF_BIT        3
#define STM32F1XX_HSIRDYF_BIT        2
#define STM32F1XX_LSERDYF_BIT        1
#define STM32F1XX_LSIRDYF_BIT        0

/* RCC_APB2RSTR */
#define STM32F1XX_RCC_APB2RSTR_OFFSET 0xC
#define STM32F1XX_TIM11RST_BIT        21
#define STM32F1XX_TIM10RST_BIT        20
#define STM32F1XX_TIM9RST_BIT         19
#define STM32F1XX_ADC3RST_BIT         15
#define STM32F1XX_USART1RST_BIT       14
#define STM32F1XX_TIM8RST_BIT         13
#define STM32F1XX_SPI1RST_BIT         12
#define STM32F1XX_TIM1RST_BIT         11
#define STM32F1XX_ADC2RST_BIT         10
#define STM32F1XX_ADC1RST_BIT         9
#define STM32F1XX_IOPGRST_BIT         8
#define STM32F1XX_IOPFRST_BIT         7
#define STM32F1XX_IOPERST_BIT         6
#define STM32F1XX_IOPDRST_BIT         5
#define STM32F1XX_IOPCRST_BIT         4
#define STM32F1XX_IOPBRST_BIT         3
#define STM32F1XX_IOPARST_BIT         2
#define STM32F1XX_AFIORST_BIT         0

/* RCC_APB1RSTR */
#define STM32F1XX_RCC_APB1RSTR_OFFSET 0x10
#define STM32F1XX_DACRST_BIT          29
#define STM32F1XX_PWRRST_BIT          28
#define STM32F1XX_BKPRST_BIT          27
#define STM32F1XX_CANRST_BIT          25
#define STM32F1XX_USBRST_BIT          23
#define STM32F1XX_I2C2RST_BIT         22
#define STM32F1XX_I2C1RST_BIT         21
#define STM32F1XX_UART5RST_BIT        20
#define STM32F1XX_UART4RST_BIT        19
#define STM32F1XX_UART3RST_BIT        18
#define STM32F1XX_UART2RST_BIT        17
#define STM32F1XX_SPI3RST_BIT         15
#define STM32F1XX_SPI2RST_BIT         14
#define STM32F1XX_WWDGRST_BIT         11
#define STM32F1XX_TIM14RST_BIT        8
#define STM32F1XX_TIM13RST_BIT        7
#define STM32F1XX_TIM12RST_BIT        6
#define STM32F1XX_TIM7RST_BIT         5
#define STM32F1XX_TIM6RST_BIT         4
#define STM32F1XX_TIM5RST_BIT         3
#define STM32F1XX_TIM4RST_BIT         2
#define STM32F1XX_TIM3RST_BIT         1
#define STM32F1XX_TIM2RST_BIT         0

/* RCC_AHBENR */
#define STM32F1XX_RCC_AHBENR_OFFSET  0x14
#define STM32F1XX_SDIOEN_BIT         10
#define STM32F1XX_FSMCEN_BIT         8
#define STM32F1XX_CRCEN_BIT          6
#define STM32F1XX_FLITFEN_BIT        4
#define STM32F1XX_SRAMEN_BIT         2
#define STM32F1XX_DMA2EN_BIT         1
#define STM32F1XX_DMA1EN_BIT         0

/* RCC_APB2ENR */
#define STM32F1XX_RCC_APB2ENR_OFFSET 0x18
#define STM32F1XX_TIM11EN_BIT        21
#define STM32F1XX_TIM10EN_BIT        20
#define STM32F1XX_TIM9EN_BIT         19
#define STM32F1XX_ADC3EN_BIT         15
#define STM32F1XX_USART1EN_BIT       14
#define STM32F1XX_TIM8EN_BIT         13
#define STM32F1XX_SPI1EN_BIT         12
#define STM32F1XX_TIM1EN_BIT         11
#define STM32F1XX_ADC2EN_BIT         10
#define STM32F1XX_ADC1EN_BIT         9
#define STM32F1XX_IOPGEN_BIT         8
#define STM32F1XX_IOPFEN_BIT         7
#define STM32F1XX_IOPEEN_BIT         6
#define STM32F1XX_IOPDEN_BIT         5
#define STM32F1XX_IOPCEN_BIT         4
#define STM32F1XX_IOPBEN_BIT         3
#define STM32F1XX_IOPAEN_BIT         2
#define STM32F1XX_AFIOEN_BIT         0

/* RCC_APB1ENR */
#define STM32F1XX_RCC_APB1ENR_OFFSET 0x1C
#define STM32F1XX_DACEN_BIT          29
#define STM32F1XX_PWREN_BIT          28
#define STM32F1XX_BKPEN_BIT          27
#define STM32F1XX_CANEN_BIT          25
#define STM32F1XX_USBEN_BIT          23
#define STM32F1XX_I2C2EN_BIT         22
#define STM32F1XX_I2C1EN_BIT         21
#define STM32F1XX_USART5EN_BIT       20
#define STM32F1XX_USART4EN_BIT       19
#define STM32F1XX_USART3EN_BIT       18
#define STM32F1XX_USART2EN_BIT       17
#define STM32F1XX_SPI3EN_BIT         15
#define STM32F1XX_SPI2EN_BIT         14
#define STM32F1XX_WWDGEN_BIT         11
#define STM32F1XX_TIM14EN_BIT        8
#define STM32F1XX_TIM13EN_BIT        7
#define STM32F1XX_TIM12EN_BIT        6
#define STM32F1XX_TIM7EN_BIT         5
#define STM32F1XX_TIM6EN_BIT         4
#define STM32F1XX_TIM5EN_BIT         3
#define STM32F1XX_TIM4EN_BIT         2
#define STM32F1XX_TIM3EN_BIT         1
#define STM32F1XX_TIM2EN_BIT         0

/* RCC_BDCR */
#define STM32F1XX_RCC_BDCR_OFFSET    0x20
#define STM32F1XX_BDRST_BIT          16
#define STM32F1XX_RTCEN_BIT          15
#define STM32F1XX_RTCSEL_START_BIT   8
#define STM32F1XX_RTCSEL_MASK        0x00000300
#define STM32F1XX_LSEBYP_BIT         2
#define STM32F1XX_LSERDY_BIT         1
#define STM32F1XX_LSEON_BIT          0

/* RCC_CSR */
#define STM32F1XX_RCC_CSR_OFFSET     0x24
#define STM32F1XX_LPWRRSTF_BIT       31
#define STM32F1XX_WWDGRSTF_BIT       30
#define STM32F1XX_IWDGRSTF_BIT       29
#define STM32F1XX_SFTRSTF_BIT        28
#define STM32F1XX_PORRSTF_BIT        27
#define STM32F1XX_PINRSTF_BIT        26
#define STM32F1XX_RMVF_BIT           24
#define STM32F1XX_LSIRDY_BIT         1
#define STM32F1XX_LSION_BIT          0

#define GET_BIT(from, bit) (from & (1 << bit))
#define SET_BIT(to, bit) (to |= (1 << bit))
#define UNSET_BIT(to, bit) (to &= ~(1 << bit))

#define UNMASK_VALUE(from, start_bit, mask) ((from & mask) >> start_bit)
#define SET_MASKED_VALUE(to, start_bit, mask, value) (to |= ((value << start_bit) & mask))
#define RESET_REGISTER_WITH_MASK(to, start_bit, mask) (to &= ~((1 << start_bit) & mask))

/* STM32F1XX CLOCK TREE */

static const Stm32Prescaler STM32F1XX_PRESCALE_NONE = {
    .name = "DIV1",
    .multiplier = 1,
    .divisor = 1
};

// static struct Stm32Prescaler STM32F1XX_PRESCALE_DIV1_5 {
//     .name = "DIV1.5",
//     .multiplier = 2,
//     .divisor = 3
// };

static const Stm32Prescaler STM32F1XX_PRESCALE_DIV2 = {
    .name = "DIV2",
    .multiplier = 1,
    .divisor = 2
};

// static struct Stm32Prescaler STM32F1XX_PRESCALE_DIV4 {
//     .name = "DIV4",
//     .multiplier = 1,
//     .divisor = 4
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_DIV6 {
//     .name = "DIV6",
//     .multiplier = 1,
//     .divisor = 6
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_DIV8 {
//     .name = "DIV8",
//     .multiplier = 1,
//     .divisor = 8
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_DIV16 {
//     .name = "DIV16",
//     .multiplier = 1,
//     .divisor = 16
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_DIV32 {
//     .name = "DIV32",
//     .multiplier = 1,
//     .divisor = 32
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_DIV64 {
//     .name = "DIV64",
//     .multiplier = 1,
//     .divisor = 64
// };

static const Stm32Prescaler STM32F1XX_PRESCALE_DIV128 = {
    .name = "DIV128",
    .multiplier = 1,
    .divisor = 128
};

// static struct Stm32Prescaler STM32F1XX_PRESCALE_DIV256 {
//     .name = "DIV256",
//     .multiplier = 1,
//     .divisor = 256
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_DIV512 {
//     .name = "DIV512",
//     .multiplier = 1,
//     .divisor = 512
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL2 {
//     .name = "MUL2",
//     .multiplier = 2,
//     .divisor = 1
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL3 {
//     .name = "MUL3",
//     .multiplier = 3,
//     .divisor = 1
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL4 {
//     .name = "MUL4",
//     .multiplier = 4,
//     .divisor = 1
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL5 {
//     .name = "MUL5",
//     .multiplier = 5,
//     .divisor = 1
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL5 {
//     .name = "MUL5",
//     .multiplier = 5,
//     .divisor = 1
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL6 {
//     .name = "MUL6",
//     .multiplier = 6,
//     .divisor = 1
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL7 {
//     .name = "MUL7",
//     .multiplier = 7,
//     .divisor = 1
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL7 {
//     .name = "MUL7",
//     .multiplier = 7,
//     .divisor = 1
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL8 {
//     .name = "MUL8",
//     .multiplier = 8,
//     .divisor = 1
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL9 {
//     .name = "MUL9",
//     .multiplier = 9,
//     .divisor = 1
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL10 {
//     .name = "MUL10",
//     .multiplier = 10,
//     .divisor = 1
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL11 {
//     .name = "MUL11",
//     .multiplier = 11,
//     .divisor = 1
// };

// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL12 {
//     .name = "MUL12",
//     .multiplier = 12,
//     .divisor = 1
// };
// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL13 {
//     .name = "MUL13",
//     .multiplier = 13,
//     .divisor = 1
// };
// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL14 {
//     .name = "MUL14",
//     .multiplier = 14,
//     .divisor = 1
// };
// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL15 {
//     .name = "MUL15",
//     .multiplier = 15,
//     .divisor = 1
// };
// static struct Stm32Prescaler STM32F1XX_PRESCALE_MUL16 {
//     .name = "MUL16",
//     .multiplier = 16,
//     .divisor = 1
// };

static const Stm32Prescaler STM32F1XX_DEFAULT_PRESCALERS[] = {STM32F1XX_PRESCALE_NONE};

#define STM32_CLOCK_FREQUENCY_LIMIT 0xFFFFFFFFFFFFFFFF

#define STM32_CLOCK_TREE_INITIALIZE_LAST(name_param)                      \
    {                                                                     \
        name_param,                                                       \
        .input_frequency = 0,                                             \
        .output_frequency = 0,                                            \
        .max_output_frequency = STM32_CLOCK_FREQUENCY_LIMIT,              \
        .number_of_outputs = 0,                                           \
        .outputs = NULL,                                                  \
        .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS), \
        .prescalers = STM32F1XX_DEFAULT_PRESCALERS,                       \
        .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],           \
        .enabled = false,                                                 \
        .number_of_observers = 0                                          \
    }

#define STM32_CLOCK_TREE_INITIALIZE_SOURCE(name_param, output_frequency, outs)  \
    {                                                                           \
        name_param,                                                             \
        .input_frequency = 0,                                                   \
        output_frequency,                                                       \
        .max_output_frequency = STM32_CLOCK_FREQUENCY_LIMIT,                    \
        .number_of_outputs = ARRAY_SIZE(outs),                                  \
        .outputs = outs,                                                        \
        .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS),       \
        .prescalers = STM32F1XX_DEFAULT_PRESCALERS,                             \
        .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],                 \
        .enabled = false,                                                       \
        .number_of_observers = 0                                                \
    }

// static struct Stm32Clock STM32F1XX_GPIOA_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIOACLK");
// static struct Stm32Clock STM32F1XX_GPIOB_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIOBCLK");
// static struct Stm32Clock STM32F1XX_GPIOC_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIOCCLK");
// static struct Stm32Clock STM32F1XX_GPIOD_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIODCLK");
// static struct Stm32Clock STM32F1XX_GPIOE_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIOECLK");
// static struct Stm32Clock STM32F1XX_GPIOF_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIOFCLK");
// static struct Stm32Clock STM32F1XX_GPIOG_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIOGCLK");
// static struct Stm32Clock STM32F1XX_AFIO_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "AFIOCLK");

// static struct Stm32Clock STM32F1XX_USART1_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "USART1CLK");
// static struct Stm32Clock STM32F1XX_SPI1_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "SPI1CLK");

// static struct Stm32Clock STM32F1XX_TIM11_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM11CLK");
// static struct Stm32Clock STM32F1XX_TIM10_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM10CLK");
// static struct Stm32Clock STM32F1XX_TIM9_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM9CLK");
// static struct Stm32Clock STM32F1XX_TIM8_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM8CLK");
// static struct Stm32Clock STM32F1XX_TIM1_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM1CLK");

// static struct Stm32Clock STM32F1XX_ADC3_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "ADC3CLK");
// static struct Stm32Clock STM32F1XX_ADC2_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "ADC2CLK");
// static struct Stm32Clock STM32F1XX_ADC1_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "ADC1CLK");

// static struct Stm32Clock STM32F1XX_DAC_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "DACCLK");
// static struct Stm32Clock STM32F1XX_PWR_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "PWRCLK");
// static struct Stm32Clock STM32F1XX_BKP_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "BKPCLK");
// static struct Stm32Clock STM32F1XX_CAN_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "CANCLK");

// static struct Stm32Clock STM32F1XX_USBCLK_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "USBCLK");

// static struct Stm32Clock STM32F1XX_I2C2_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "I2C2CLK");
// static struct Stm32Clock STM32F1XX_I2C1_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "I2C1CLK");

// static struct Stm32Clock STM32F1XX_USART5_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "USART5CLK");
// static struct Stm32Clock STM32F1XX_USART4_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "USART4CLK");
// static struct Stm32Clock STM32F1XX_USART3_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "USART3CLK");
// static struct Stm32Clock STM32F1XX_USART2_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "USART2CLK");

// static struct Stm32Clock STM32F1XX_SPI3_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "SPI3CLK");
// static struct Stm32Clock STM32F1XX_SPI2_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "SPI2CLK");
// static Stm32Clock STM32F1XX_WWDG_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "WWDGCLK");
static Stm32Clock STM32F1XX_IWDG_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "IWDGCLK");
static Stm32Clock STM32F1XX_FLITFCLK_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "FLITFCLK");

// static struct Stm32Clock STM32F1XX_TIM14_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM14CLK");
// static struct Stm32Clock STM32F1XX_TIM13_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM13CLK");
// static struct Stm32Clock STM32F1XX_TIM12_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM12CLK");
// static struct Stm32Clock STM32F1XX_TIM7_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM7CLK");
// static struct Stm32Clock STM32F1XX_TIM6_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM6CLK");
// static struct Stm32Clock STM32F1XX_TIM5_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM5CLK");
// static struct Stm32Clock STM32F1XX_TIM4_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM4CLK");
// static struct Stm32Clock STM32F1XX_TIM3_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM3CLK");
// static struct Stm32Clock STM32F1XX_TIM2_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM2CLK");

static Stm32Clock STM32F1XX_RTC_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "RTCCLK");

// static struct Stm32Clock STM32F1XX_FLITFCLK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "FLITFCLK");

// static struct Stm32Prescaler STM32F1XX_USB_PRESCALERS[] = {STM32F1XX_PRESCALE_NONE, STM32F1XX_PRESCALE_DIV1_5};

// static struct Stm32Clock* STM32F1XX_PCLK1_OUTPUTS[] = {

// };

// static struct Stm32Clock STM32F1XX_PCLK1 {
//     .name = "PCLK1",
//     .input_frequency = 0,
//     .output_frequency = 48000000,
//     .max_output_frequency = STM32_CLOCK_FREQUENCY_LIMIT,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_PCLK1_OUTPUTS),
//     .prescalers = STM32F1XX_PCLK1_OUTPUTS,
//     .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

// static struct Stm32Clock STM32F1XX_USB_PRESCALER {
//     .name = "USB_PRESCALER",
//     .input_frequency = 0,
//     .output_frequency = 48000000,
//     .max_output_frequency = 48000000,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_USB_PRESCALERS),
//     .prescalers = STM32F1XX_USB_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

// static struct Stm32Clock STM32F1XX_HCLK {
//     .name = "HCLK",
//     .input_frequency = 0,
//     .output_frequency = 0,
//     .max_output_frequency = 72000000,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS),
//     .prescalers = STM32F1XX_DEFAULT_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

// static struct Stm32Prescaler STM32F1XX_SYSTIMER_PRESCALERS[] = {STM32F1XX_PRESCALE_DIV8};

// static struct Stm32Clock STM32F1XX_SYSTEM_TIMER_CLK {
//     .name = "Cortex System timer",
//     .input_frequency = 0,
//     .output_frequency = 0,
//     .max_output_frequency = 72000000,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_SYSTIMER_PRESCALERS),
//     .prescalers = STM32F1XX_SYSTIMER_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_SYSTIMER_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

// static struct Stm32Prescaler STM32F1XX_APB_PRESCALERS[] = {
//     STM32F1XX_PRESCALE_NONE,
//     STM32F1XX_PRESCALE_DIV2,
//     STM32F1XX_PRESCALE_DIV4,
//     STM32F1XX_PRESCALE_DIV8,
//     STM32F1XX_PRESCALE_DIV16
// };

// static struct Stm32Clock STM32F1XX_APB1 {
//     .name = "APB1",
//     .input_frequency = 0,
//     .output_frequency = 0,
//     .max_output_frequency = 36000000,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_APB_PRESCALERS),
//     .prescalers = STM32F1XX_APB_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_APB_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

// static struct Stm32Prescaler STM32F1XX_TIM_PRESCALERS[] = {
//     STM32F1XX_PRESCALE_NONE,
//     STM32F1XX_PRESCALE_MUL2
// };

// static struct Stm32Clock STM32F1XX_APB1_TIM {
//     .name = "APB1_TIM",
//     .input_frequency = 0,
//     .output_frequency = 0,
//     .max_output_frequency = 72000000,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_TIM_PRESCALERS),
//     .prescalers = STM32F1XX_TIM_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_TIM_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

// static struct Stm32Clock STM32F1XX_APB2_TIM {
//     .name = "APB2_TIM",
//     .input_frequency = 0,
//     .output_frequency = 0,
//     .max_output_frequency = 72000000,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_TIM_PRESCALERS),
//     .prescalers = STM32F1XX_TIM_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_TIM_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

// static struct Stm32Clock STM32F1XX_APB2 {
//     .name = "APB2",
//     .input_frequency = 0,
//     .output_frequency = 0,
//     .max_output_frequency = 72000000,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_APB_PRESCALERS),
//     .prescalers = STM32F1XX_APB_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_APB_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

// static struct Stm32Prescaler STM32F1XX_ADC_PRESCALERS[] = {
//     STM32F1XX_PRESCALE_MUL2,
//     STM32F1XX_PRESCALE_MUL4,
//     STM32F1XX_PRESCALE_MUL6,
//     STM32F1XX_PRESCALE_MUL8
// };

// static struct Stm32Clock STM32F1XX_ADC_PRESCALER {
//     .name = "ADC",
//     .input_frequency = 0,
//     .output_frequency = 0,
//     .max_output_frequency = 14000000,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_ADC_PRESCALERS),
//     .prescalers = STM32F1XX_ADC_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_ADC_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

// static struct Stm32Prescaler STM32F1XX_HCLK_DIV2_PRESCALERS[] = {
//     STM32F1XX_PRESCALE_DIV2
// };

// static struct Stm32Clock STM32F1XX_HCLK_DIV2_PRESCALER {
//     .name = "HCLK/2",
//     .input_frequency = 0,
//     .output_frequency = 0,
//     .max_output_frequency = 36000000,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_HCLK_DIV2_PRESCALERS),
//     .prescalers = STM32F1XX_HCLK_DIV2_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_HCLK_DIV2_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

// static struct Stm32Prescaler STM32F1XX_AHB_PRESCALERS[] = {
//     STM32F1XX_PRESCALE_NONE,
//     STM32F1XX_PRESCALE_DIV2,
//     STM32F1XX_PRESCALE_DIV4,
//     STM32F1XX_PRESCALE_DIV8,
//     STM32F1XX_PRESCALE_DIV16,
//     STM32F1XX_PRESCALE_DIV32,
//     STM32F1XX_PRESCALE_DIV64,
//     STM32F1XX_PRESCALE_DIV128,
//     STM32F1XX_PRESCALE_DIV256,
//     STM32F1XX_PRESCALE_DIV512
// };

// static struct Stm32Clock* STM32F1XX_AHB_OUTPUTS[] = {
//     &STM32F1XX_HCLK,
//     &STM32F1XX_SYSTEM_TIMER_CLK,
//     &STM32F1XX_APB1,
//     &STM32F1XX_APB2,
//     &STM32F1XX_HCLK_DIV2_PRESCALER
// };

// static struct Stm32Clock STM32F1XX_AHB {
//     .name = "AHB",
//     .input_frequency = 0,
//     .output_frequency = 0,
//     .max_output_frequency = 72000000,
//     .number_of_outputs = ARRAY_SIZE(STM32F1XX_AHB_OUTPUTS),
//     .outputs = STM32F1XX_AHB_OUTPUTS,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_AHB_PRESCALERS),
//     .prescalers = STM32F1XX_AHB_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_AHB_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

// static const Stm32Prescaler STM32F1XX_PLLMUL_PRESCALERS[] = {
//     STM32F1XX_PRESCALE_MUL2,
//     STM32F1XX_PRESCALE_MUL3,
//     STM32F1XX_PRESCALE_MUL4,
//     STM32F1XX_PRESCALE_MUL5,
//     STM32F1XX_PRESCALE_MUL6,
//     STM32F1XX_PRESCALE_MUL7,
//     STM32F1XX_PRESCALE_MUL8,
//     STM32F1XX_PRESCALE_MUL9,
//     STM32F1XX_PRESCALE_MUL10,
//     STM32F1XX_PRESCALE_MUL11,
//     STM32F1XX_PRESCALE_MUL12,
//     STM32F1XX_PRESCALE_MUL13,
//     STM32F1XX_PRESCALE_MUL14,
//     STM32F1XX_PRESCALE_MUL15,
//     STM32F1XX_PRESCALE_MUL16
// };

// static struct Stm32Clock STM32F1XX_PLLMUL {
//     .name = "PLLMUL",
//     .input_frequency = 0,
//     .output_frequency = 0,
//     .max_output_frequency = 72000000,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_PLLMUL_PRESCALERS),
//     .prescalers = STM32F1XX_PLLMUL_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_PLLMUL_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };


// static Stm32Clock* STM32F1XX_PLLSRC_OUTPUTS[] = {
//     &STM32F1XX_PLLMUL
// };


// static Stm32Clock STM32F1XX_PLLSRC = {
//     .name = "PLL",
//     .input_frequency = 0,
//     .output_frequency = 0,
//     .max_output_frequency = 72000000,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS),
//     .prescalers = STM32F1XX_DEFAULT_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

/////////////////////////////////////////////////////
//                         SOURCES
////////////////////////////////////////////////////
// static Stm32Clock* STM32F1XX_LSE_OUTPUTS[] = {
//     &STM32F1XX_RTC_CLOCK
// };

// static Stm32Clock STM32F1XX_LSE = STM32_CLOCK_TREE_INITIALIZE_SOURCE(
//     .name = "LSE",
//     .output_frequency = 32768,
//     STM32F1XX_LSE_OUTPUTS
// );

// static Stm32Prescaler STM32F1XX_HSI_DIV2_PRESCALERS[] = {
//     STM32F1XX_PRESCALE_DIV2
// };

// static Stm32Clock* STM32F1XX_HSI_DIV2_OUTPUTS[] = {
//     &STM32F1XX_PLLSRC
// }

// static Stm32Clock STM32F1XX_HSI_DIV2 = {
//     .name = "HSI/2",
//     .input_frequency = 0,
//     .output_frequency = 0,
//     .max_output_frequency = 4000000,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_HSI_DIV2_PRESCALERS),
//     .prescalers = STM32F1XX_HSI_DIV2_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_HSI_DIV2_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

static Stm32Prescaler STM32F1XX_HSE_DIV2_PRESCALERS[] = {
    STM32F1XX_PRESCALE_DIV2
};

static Stm32Clock STM32F1XX_HSE_DIV2 = {
    .name = "HSE/2",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 4000000,
    .number_of_outputs = 0,
    .outputs = NULL,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_HSE_DIV2_PRESCALERS),
    .prescalers = STM32F1XX_HSE_DIV2_PRESCALERS,
    .selected_prescaler = &STM32F1XX_HSE_DIV2_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_HSE_DIV2_OUTPUTS[] = {

};

static Stm32Prescaler STM32F1XX_HSE_DIV128_PRESCALERS[] = {
    STM32F1XX_PRESCALE_DIV128
};

static Stm32Clock STM32F1XX_HSE_DIV128 = {
    .name = "HSE/128",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 4000000,
    .number_of_outputs = 0,
    .outputs = NULL,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_HSE_DIV128_PRESCALERS),
    .prescalers = STM32F1XX_HSE_DIV128_PRESCALERS,
    .selected_prescaler = &STM32F1XX_HSE_DIV128_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_HSE_DIV128_OUTPUTS[] = {

};

static Stm32Clock STM32F1XX_PLLXTPRE = {
    .name = "PLLXTPRE",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 4000000,
    .number_of_outputs = 0,
    .outputs = NULL,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS),
    .prescalers = STM32F1XX_DEFAULT_PRESCALERS,
    .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_PLLXTPRE_OUTPUTS[] = {
};


static Stm32Clock* STM32F1XX_SYSCLK_OUTPUTS[] = {

};

static Stm32Clock STM32F1XX_SYSCLK = {
    .name = "SYSCLK",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 72000000,
    .number_of_outputs = ARRAY_SIZE(STM32F1XX_SYSCLK_OUTPUTS),
    .outputs = STM32F1XX_SYSCLK_OUTPUTS,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS),
    .prescalers = STM32F1XX_DEFAULT_PRESCALERS,
    .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_CSS_OUTPUTS[] = {

};




static Stm32Clock STM32F1XX_CSS = {
    .name = "CSS",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 72000000,
    .number_of_outputs = 0,
    .outputs = NULL,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS),
    .prescalers = STM32F1XX_DEFAULT_PRESCALERS,
    .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_HSE_OUTPUTS[] = {
    &STM32F1XX_SYSCLK,
    &STM32F1XX_CSS,
    &STM32F1XX_PLLXTPRE,
    &STM32F1XX_HSE_DIV2,
    &STM32F1XX_HSE_DIV128
};

static Stm32Clock STM32F1XX_HSE = STM32_CLOCK_TREE_INITIALIZE_SOURCE(
    .name = "HSE",
    .output_frequency = 0,
    STM32F1XX_HSE_OUTPUTS
);

// static Stm32Clock* STM32F1XX_HSI_OUTPUTS[] = {
//     &STM32F1XX_SYSCLK,
//     &STM32F1XX_HSI_DIV2,
//     &STM32F1XX_FLITFCLK_CLOCK
// };

// static Stm32Clock STM32F1XX_HSI = STM32_CLOCK_TREE_INITIALIZE_SOURCE(
//     .name = "HSI",
//     .output_frequency = 8000000,
//     STM32F1XX_HSI_OUTPUTS
// );

// static Stm32Clock* STM32F1XX_LSI_OUTPUTS[] = {
//     &STM32F1XX_RTC_CLOCK,
//     &STM32F1XX_IWDG_CLOCK
// };

// static Stm32Clock STM32F1XX_LSI = STM32_CLOCK_TREE_INITIALIZE_SOURCE(
//     .name = "LSI",
//     .output_frequency = 40000,
//     STM32F1XX_LSI_OUTPUTS
// );

static Stm32Clock* STM32F1XX_CSS_INPUTS[] = {
    &STM32F1XX_HSE
};

static Stm32Clock* STM32F1XX_PLLXTPRE_INPUTS[] = {
    &STM32F1XX_HSE,
    &STM32F1XX_HSE_DIV2
};

static Stm32Clock* STM32F1XX_HSE_DIV2_INPUTS[] = {
    &STM32F1XX_HSE
};

static Stm32Clock* STM32F1XX_SYSCLK_INPUTS[] = {
    &STM32F1XX_HSE
    // TODO complete
};

static Stm32Clock* STM32F1XX_HSE_DIV128_INPUTS[] = {
    &STM32F1XX_HSE
};

typedef struct Stm32F1xxRccRegisters
{
    uint32_t RCC_CR;
    uint32_t RCC_CFGR;
    uint32_t RCC_CIR;
    uint32_t RCC_APB2RSTR;
    uint32_t RCC_APB1RSTR;
    uint32_t RCC_AHBENR;
    uint32_t RCC_APB2ENR;
    uint32_t RCC_APB1ENR;
    uint32_t RCC_BDCR;
    uint32_t RCC_CSR;
} Stm32F1xxRccRegisters;

typedef struct Stm32F1xxRcc
{
    SysBusDevice busdev;

    MemoryRegion iomem;

    Stm32F1xxRccRegisters reg;

    Stm32Clock* hse;
    Stm32Clock* lse;
    Stm32Clock* hsi;
    Stm32Clock* lsi;

    qemu_irq irq;
} Stm32F1xxRcc;



static void stm32f1xx_initialize_hse_tree(Stm32Clock* hse)
{
    stm32_clock_set_outputs(hse, STM32F1XX_HSE_OUTPUTS, ARRAY_SIZE(STM32F1XX_HSE_OUTPUTS));

    Stm32Clock* css = stm32_clock_get_output(hse, "CSS");
    stm32_clock_set_inputs(css, STM32F1XX_CSS_INPUTS, ARRAY_SIZE(STM32F1XX_CSS_INPUTS));
    stm32_clock_select_input(css, hse);
    stm32_clock_set_outputs(css, STM32F1XX_CSS_OUTPUTS, ARRAY_SIZE(STM32F1XX_CSS_OUTPUTS));

    Stm32Clock* pllxtpre = stm32_clock_get_output(hse, "PLLXTPRE");
    stm32_clock_set_inputs(pllxtpre, STM32F1XX_PLLXTPRE_INPUTS, ARRAY_SIZE(STM32F1XX_PLLXTPRE_INPUTS));
    stm32_clock_select_input(pllxtpre, hse);
    stm32_clock_set_outputs(pllxtpre, STM32F1XX_PLLXTPRE_OUTPUTS, ARRAY_SIZE(STM32F1XX_PLLXTPRE_OUTPUTS));

    Stm32Clock* hse_div2 = stm32_clock_get_output(hse, "HSE/2");
    stm32_clock_set_inputs(hse_div2, STM32F1XX_HSE_DIV2_INPUTS, ARRAY_SIZE(STM32F1XX_HSE_DIV2_INPUTS));
    stm32_clock_select_input(hse_div2, hse);
    stm32_clock_set_outputs(hse_div2, STM32F1XX_HSE_DIV2_OUTPUTS, ARRAY_SIZE(STM32F1XX_HSE_DIV2_OUTPUTS));

    Stm32Clock* sysclk = stm32_clock_get_output(hse, "SYSCLK");
    stm32_clock_set_inputs(sysclk, STM32F1XX_SYSCLK_INPUTS, ARRAY_SIZE(STM32F1XX_SYSCLK_INPUTS));
    stm32_clock_set_outputs(sysclk, STM32F1XX_SYSCLK_OUTPUTS, ARRAY_SIZE(STM32F1XX_SYSCLK_OUTPUTS));

    Stm32Clock* hse_div128 = stm32_clock_get_output(hse, "HSE/128");
    stm32_clock_set_inputs(hse_div128, STM32F1XX_HSE_DIV128_INPUTS, ARRAY_SIZE(STM32F1XX_HSE_DIV128_INPUTS));
    stm32_clock_select_input(hse_div128, hse);
    stm32_clock_set_outputs(hse_div128, STM32F1XX_HSE_DIV128_OUTPUTS, ARRAY_SIZE(STM32F1XX_HSE_DIV128_OUTPUTS));
}

static void stm32f1xx_rcc_reset_registers(Stm32F1xxRccRegisters *self)
{
    self->RCC_CR = 0x00000083;
    self->RCC_CFGR = 0x00000000;
    self->RCC_CIR = 0x00000000;
    self->RCC_APB2RSTR = 0x00000000;
    self->RCC_APB1RSTR = 0x00000000;
    self->RCC_AHBENR = 0x00000014;
    self->RCC_APB2ENR = 0x00000000;
    self->RCC_APB1ENR = 0x00000000;
    self->RCC_BDCR = 0x00000000;
    self->RCC_CSR = 0x0C000000;
}

static void stm32f1xx_rcc_init_clock_tree(Stm32F1xxRcc* self)
{
    self->hse = &STM32F1XX_HSE;
    stm32f1xx_initialize_hse_tree(self->hse);
    // self->hsi = &STM32F1XX_HSI;
    // self->lse = &STM32F1XX_LSE;
    // self->lsi = &STM32F1XX_LSI;
}

static uint64_t stm32f1xx_rcc_read_word(Stm32F1xxRcc *self, hwaddr offset)
{
    switch(offset)
    {
        case STM32F1XX_RCC_CR_OFFSET:
        {
            fprintf(stderr, "Reading CR: 0x%x\n", self->reg.RCC_CR);
            return self->reg.RCC_CR;
        } break;
        case STM32F1XX_RCC_CFGR_OFFSET:
        {
            fprintf(stderr, "Reading RCC_CFGR: 0x%x\n", self->reg.RCC_CFGR);
            return self->reg.RCC_CFGR;
        } break;
        case STM32F1XX_RCC_CIR_OFFSET:
        {
            fprintf(stderr, "Reading RCC_CIR: 0x%x\n", self->reg.RCC_CIR);
            return self->reg.RCC_CIR;
        } break;
        case STM32F1XX_RCC_APB2RSTR_OFFSET:
        {
            fprintf(stderr, "Reading RCC_APB2RSTR: 0x%x\n", self->reg.RCC_APB2RSTR);
            return self->reg.RCC_APB2RSTR;
        } break;
        case STM32F1XX_RCC_APB1RSTR_OFFSET:
        {
            fprintf(stderr, "Reading RCC_APB1RSTR: 0x%x\n", self->reg.RCC_APB1RSTR);
            return self->reg.RCC_APB1RSTR;
        } break;
        case STM32F1XX_RCC_AHBENR_OFFSET:
        {
            fprintf(stderr, "Reading RCC_AHBENR: 0x%x\n", self->reg.RCC_AHBENR);
            return self->reg.RCC_AHBENR;
        } break;
        case STM32F1XX_RCC_APB2ENR_OFFSET:
        {
            fprintf(stderr, "Reading RCC_APB2ENR: 0x%x\n", self->reg.RCC_APB2ENR);
            return self->reg.RCC_APB2ENR;
        } break;
        case STM32F1XX_RCC_APB1ENR_OFFSET:
        {
            fprintf(stderr, "Reading RCC_APB1ENR: 0x%x\n", self->reg.RCC_APB1ENR);
            return self->reg.RCC_APB1ENR;
        } break;
        case STM32F1XX_RCC_BDCR_OFFSET:
        {
            fprintf(stderr, "Reading RCC_BDCR: 0x%x\n", self->reg.RCC_BDCR);
            return self->reg.RCC_BDCR;
        } break;
        case STM32F1XX_RCC_CSR_OFFSET:
        {
            fprintf(stderr, "Reading RCC_CSR: 0x%x\n", self->reg.RCC_CSR);
            return self->reg.RCC_CSR;
        } break;
    }
    return 0;
}

static uint64_t stm32f1xx_rcc_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    Stm32F1xxRcc *self = STM32F1XX_RCC(opaque);
    if (size == 4)
    {
        return stm32f1xx_rcc_read_word(self, offset);
    }
    else
    {
        // TODO: NOT IMPLEMENTED
    }
    return 0;
}

static void stm32f1xx_rcc_write_cr(Stm32F1xxRcc *self, uint64_t value)
{
    fprintf(stderr, "Write to CR: 0x%lx\n", value);

    if (GET_BIT(value, STM32F1XX_HSION_BIT))
    {
        fprintf(stderr, "Enabling HSI \n");
        SET_BIT(self->reg.RCC_CR, STM32F1XX_HSION_BIT);
        SET_BIT(self->reg.RCC_CR, STM32F1XX_HSIRDY_BIT);
        SET_BIT(self->reg.RCC_CIR, STM32F1XX_HSIRDYF_BIT);
        // enable source clk
    }
    if (value & STM32F1XX_HSITRIM_MASK)
    {
        fprintf(stderr, "Setting HSITRIM, unimp!\n");
    }
    if (GET_BIT(value, STM32F1XX_HSEON_BIT))
    {
        fprintf(stderr, "Enabling HSE \n");
        SET_BIT(self->reg.RCC_CR, STM32F1XX_HSEON_BIT);
        SET_BIT(self->reg.RCC_CR, STM32F1XX_HSERDY_BIT);
        SET_BIT(self->reg.RCC_CIR, STM32F1XX_HSERDYF_BIT);
    }
    if (GET_BIT(value, STM32F1XX_HSEBYP_BIT))
    {
        fprintf(stderr, "Setting HSEBYP, unimp!\n");
    }
    if (GET_BIT(value, STM32F1XX_CSSON_BIT))
    {
        fprintf(stderr, "Setting CSSON, unimp!\n");
    }
    if (GET_BIT(value, STM32F1XX_PLLON_BIT))
    {
        fprintf(stderr, "Setting PLLON!\n");
        SET_BIT(self->reg.RCC_CR, STM32F1XX_PLLREADY_BIT);

    }
    if (GET_BIT(value, STM32F1XX_PLL2ON_BIT))
    {
        fprintf(stderr, "Setting PLL2ON, unimp!\n");
    }
    if (GET_BIT(value, STM32F1XX_PLL3ON_BIT))
    {
        fprintf(stderr, "Setting PLL3ON, unimp!\n");
    }
}

static void stm32f1xx_rcc_write_cfgr(Stm32F1xxRcc *self, uint64_t value)
{
    fprintf(stderr, "Write to CFGR: 0x%lx\n", value);

    if (value & STM32F1XX_SW_MASK)
    {
        fprintf(stderr, "Setting system clock\n");
        uint64_t source = UNMASK_VALUE(value, STM32F1XX_SW_START_BIT, STM32F1XX_SW_MASK);
        switch (source)
        {
            case 0: {
                fprintf(stderr, "Setting system clock to HSI\n");
                RESET_REGISTER_WITH_MASK(self->reg.RCC_CFGR, STM32F1XX_SWS_START_BIT, STM32F1XX_SWS_MASK);
                SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_SWS_START_BIT, STM32F1XX_SWS_MASK, source);
                SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_SW_START_BIT, STM32F1XX_SW_MASK, source);
            } break;
            case 1: {
                fprintf(stderr, "Setting system clock to HSE\n");
                RESET_REGISTER_WITH_MASK(self->reg.RCC_CFGR, STM32F1XX_SWS_START_BIT, STM32F1XX_SWS_MASK);
                SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_SWS_START_BIT, STM32F1XX_SWS_MASK, source);
                SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_SW_START_BIT, STM32F1XX_SW_MASK, source);
            } break;
            case 2: {
                fprintf(stderr, "Setting system clock to PLL\n");
                RESET_REGISTER_WITH_MASK(self->reg.RCC_CFGR, STM32F1XX_SWS_START_BIT, STM32F1XX_SWS_MASK);
                SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_SWS_START_BIT, STM32F1XX_SWS_MASK, source);
                SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_SW_START_BIT, STM32F1XX_SW_MASK, source);
            } break;
            case 3: {
                fprintf(stderr, "Setting system clock to NotAllowed\n");

            } break;
        }

    }

    if (value & STM32F1XX_HPRE_MASK)
    {
        fprintf(stderr, "Setting HPRE, unimp!\n");

    }

    if (value & STM32F1XX_PPRE1_MASK)
    {
        self->reg.RCC_CFGR &= ~STM32F1XX_PPRE1_MASK;
        self->reg.RCC_CFGR |= (value & STM32F1XX_PPRE1_MASK);
    }

    if (value & STM32F1XX_PPRE2_MASK)
    {
        fprintf(stderr, "Setting PPRE2, unimp!\n");

    }

    if (value & STM32F1XX_ADCPRE_MASK)
    {
        fprintf(stderr, "Setting ADCPRE, unimp!\n");

    }
    if (GET_BIT(value, STM32F1XX_PLLSRC_BIT))
    {
        fprintf(stderr, "Setting PLLSRC\n");
        SET_BIT(self->reg.RCC_CFGR, STM32F1XX_PLLSRC_BIT);
    }
    if (GET_BIT(value, STM32F1XX_PLLXTPRE_BIT))
    {
        fprintf(stderr, "Setting PLLXTPRE, unimp!\n");

    }
    if (value & STM32F1XX_PLLMUL_MASK)
    {
        fprintf(stderr, "Setting PLLMUL\n");
        self->reg.RCC_CFGR &= ~STM32F1XX_PLLMUL_MASK;
        self->reg.RCC_CFGR |= (value & STM32F1XX_PLLMUL_MASK);

    }
    if (GET_BIT(value, STM32F1XX_OTGFSPRE_BIT))
    {
        fprintf(stderr, "Setting OTGFSPRE, unimp!\n");

    }
    if (value & STM32F1XX_MCO_MASK)
    {
        fprintf(stderr, "Setting MCO, unimp!\n");

    }
}


static void stm32f1xx_rcc_write_word(Stm32F1xxRcc *self, hwaddr offset,
                       uint64_t value)
{
    switch(offset)
    {
        case STM32F1XX_RCC_CR_OFFSET:
        {
            stm32f1xx_rcc_write_cr(self, value);
        } break;
        case STM32F1XX_RCC_CFGR_OFFSET:
        {
            stm32f1xx_rcc_write_cfgr(self, value);
        } break;
        case STM32F1XX_RCC_CIR_OFFSET:
        {
            fprintf(stderr, "Write to CIR\n");
        } break;
        case STM32F1XX_RCC_APB2RSTR_OFFSET:
        {
            fprintf(stderr, "Write to APB2RSTR\n");
        } break;
        case STM32F1XX_RCC_APB1RSTR_OFFSET:
        {
            fprintf(stderr, "Write to APB1RSTR\n");
        } break;
        case STM32F1XX_RCC_AHBENR_OFFSET:
        {
            fprintf(stderr, "Write to AHBENR\n");
        } break;
        case STM32F1XX_RCC_APB2ENR_OFFSET:
        {
            fprintf(stderr, "Write to APB2ENR\n");
        } break;
        case STM32F1XX_RCC_APB1ENR_OFFSET:
        {
            fprintf(stderr, "Write to APB1ENR\n");
        } break;
        case STM32F1XX_RCC_BDCR_OFFSET:
        {
            fprintf(stderr, "Write to BDCR\n");
        } break;
        case STM32F1XX_RCC_CSR_OFFSET:
        {
            fprintf(stderr, "Write to CSR\n");
        } break;
    }
}

static void stm32f1xx_rcc_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    fprintf(stderr, "RCC write at 0x%lx: 0x%lx\n", offset, value);
    Stm32F1xxRcc *self = STM32F1XX_RCC(opaque);
    if (size == 4)
    {
        stm32f1xx_rcc_write_word(self, offset, value);
        return;
    }

    // TODO: not implemented
}

static const MemoryRegionOps stm32f1xx_rcc_operations = {
    .read = stm32f1xx_rcc_read,
    .write = stm32f1xx_rcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32f1xx_rcc_init(Object *device)
{
    Stm32F1xxRcc *rcc = STM32F1XX_RCC(device);

    memory_region_init_io(&rcc->iomem, OBJECT(rcc), &stm32f1xx_rcc_operations, rcc, "stm32f1xx-rcc", 0x3FF);
    sysbus_init_mmio(SYS_BUS_DEVICE(rcc), &rcc->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(rcc), &rcc->irq);
    stm32f1xx_rcc_reset_registers(&rcc->reg);
    stm32f1xx_rcc_init_clock_tree(rcc);
}

static void stm32f1xx_rcc_reset(DeviceState *device)
{
    Stm32F1xxRcc *rcc = STM32F1XX_RCC(device);

    stm32f1xx_rcc_reset_registers(&rcc->reg);
}

static void stm32f1xx_rcc_class_init(ObjectClass *klass, void* data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32f1xx_rcc_reset;
}

static TypeInfo stm32f1xx_rcc_info = {
    .name = TYPE_STM32F1XX_RCC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32F1xxRcc),
    .instance_init = stm32f1xx_rcc_init,
    .class_init = stm32f1xx_rcc_class_init
};

static void stm32f1xx_rcc_register_type(void)
{
    type_register_static(&stm32f1xx_rcc_info);
}

type_init(stm32f1xx_rcc_register_type);

