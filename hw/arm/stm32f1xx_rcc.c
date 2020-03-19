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
#include "qemu/bitops.h"
#include "qemu/error-report.h"
#include "hw/timer/armv7m_systick.h"

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

#define GET_BIT(from, bit) ((from & (1 << bit)) >> (bit))
#define UNSET_BIT(to, bit) (to &= ~(1 << bit))
#define SET_BIT(to, bit) (to |= (1 << bit))
#define CHANGE_BIT(to, bit, value) (to = (to & ~(1 << bit)) | ((value > 0 ? 1 : 0) << bit))

#define UNMASK_VALUE(from, start_bit, mask) ((from & mask) >> start_bit)
#define RESET_REGISTER_WITH_MASK(to, start_bit, mask) (to &= ~((1 << start_bit) & mask))
#define SET_MASKED_VALUE(to, start_bit, mask, value) (to = (to & ~(mask)) | ((value << start_bit) & mask))

/* STM32F1XX CLOCK TREE */

static const Stm32Prescaler STM32F1XX_PRESCALE_NONE = {
    .name = "DIV1",
    .multiplier = 1,
    .divisor = 1
};

static const Stm32Prescaler STM32F1XX_PRESCALE_DIV1_5 = {
    .name = "DIV1.5",
    .multiplier = 2,
    .divisor = 3
};

static const Stm32Prescaler STM32F1XX_PRESCALE_DIV2 = {
    .name = "DIV2",
    .multiplier = 1,
    .divisor = 2
};

static const Stm32Prescaler STM32F1XX_PRESCALE_DIV4 = {
    .name = "DIV4",
    .multiplier = 1,
    .divisor = 4
};

static const Stm32Prescaler STM32F1XX_PRESCALE_DIV6 = {
    .name = "DIV6",
    .multiplier = 1,
    .divisor = 6
};

static const Stm32Prescaler STM32F1XX_PRESCALE_DIV8 = {
    .name = "DIV8",
    .multiplier = 1,
    .divisor = 8
};

static const Stm32Prescaler STM32F1XX_PRESCALE_DIV16 = {
    .name = "DIV16",
    .multiplier = 1,
    .divisor = 16
};

static const Stm32Prescaler STM32F1XX_PRESCALE_DIV32 = {
    .name = "DIV32",
    .multiplier = 1,
    .divisor = 32
};

static const Stm32Prescaler STM32F1XX_PRESCALE_DIV64 = {
    .name = "DIV64",
    .multiplier = 1,
    .divisor = 64
};

static const Stm32Prescaler STM32F1XX_PRESCALE_DIV128 = {
    .name = "DIV128",
    .multiplier = 1,
    .divisor = 128
};

static const Stm32Prescaler STM32F1XX_PRESCALE_DIV256 = {
    .name = "DIV256",
    .multiplier = 1,
    .divisor = 256
};

static const Stm32Prescaler STM32F1XX_PRESCALE_DIV512 = {
    .name = "DIV512",
    .multiplier = 1,
    .divisor = 512
};

static const Stm32Prescaler STM32F1XX_PRESCALE_MUL2 = {
    .name = "MUL2",
    .multiplier = 2,
    .divisor = 1
};

static const Stm32Prescaler STM32F1XX_PRESCALE_MUL3 = {
    .name = "MUL3",
    .multiplier = 3,
    .divisor = 1
};

static const Stm32Prescaler STM32F1XX_PRESCALE_MUL4 = {
    .name = "MUL4",
    .multiplier = 4,
    .divisor = 1
};

static const Stm32Prescaler STM32F1XX_PRESCALE_MUL5 = {
    .name = "MUL5",
    .multiplier = 5,
    .divisor = 1
};

static const Stm32Prescaler STM32F1XX_PRESCALE_MUL6 = {
    .name = "MUL6",
    .multiplier = 6,
    .divisor = 1
};

static const Stm32Prescaler STM32F1XX_PRESCALE_MUL7 = {
    .name = "MUL7",
    .multiplier = 7,
    .divisor = 1
};

static const Stm32Prescaler STM32F1XX_PRESCALE_MUL8 = {
    .name = "MUL8",
    .multiplier = 8,
    .divisor = 1
};

static const Stm32Prescaler STM32F1XX_PRESCALE_MUL9 = {
    .name = "MUL9",
    .multiplier = 9,
    .divisor = 1
};

static const Stm32Prescaler STM32F1XX_PRESCALE_MUL10 = {
    .name = "MUL10",
    .multiplier = 10,
    .divisor = 1
};

static const Stm32Prescaler STM32F1XX_PRESCALE_MUL11 = {
    .name = "MUL11",
    .multiplier = 11,
    .divisor = 1
};

static const Stm32Prescaler STM32F1XX_PRESCALE_MUL12 = {
    .name = "MUL12",
    .multiplier = 12,
    .divisor = 1
};
static const Stm32Prescaler STM32F1XX_PRESCALE_MUL13 = {
    .name = "MUL13",
    .multiplier = 13,
    .divisor = 1
};
static const Stm32Prescaler STM32F1XX_PRESCALE_MUL14 = {
    .name = "MUL14",
    .multiplier = 14,
    .divisor = 1
};
static const Stm32Prescaler STM32F1XX_PRESCALE_MUL15 = {
    .name = "MUL15",
    .multiplier = 15,
    .divisor = 1
};
static const Stm32Prescaler STM32F1XX_PRESCALE_MUL16 = {
    .name = "MUL16",
    .multiplier = 16,
    .divisor = 1
};

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

static Stm32Clock STM32F1XX_GPIOA_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIOACLK");
static Stm32Clock STM32F1XX_GPIOB_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIOBCLK");
static Stm32Clock STM32F1XX_GPIOC_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIOCCLK");
static Stm32Clock STM32F1XX_GPIOD_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIODCLK");
static Stm32Clock STM32F1XX_GPIOE_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIOECLK");
static Stm32Clock STM32F1XX_GPIOF_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIOFCLK");
static Stm32Clock STM32F1XX_GPIOG_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "GPIOGCLK");
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

static Stm32Clock STM32F1XX_USBCLK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "USBCLK");

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
static Stm32Clock STM32F1XX_FLITFCLK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "FLITFCLK");

// static struct Stm32Clock STM32F1XX_TIM14_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM14CLK");
// static struct Stm32Clock STM32F1XX_TIM13_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM13CLK");
// static struct Stm32Clock STM32F1XX_TIM12_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM12CLK");
// static struct Stm32Clock STM32F1XX_TIM7_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM7CLK");
// static struct Stm32Clock STM32F1XX_TIM6_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM6CLK");
// static struct Stm32Clock STM32F1XX_TIM5_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM5CLK");
// static struct Stm32Clock STM32F1XX_TIM4_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM4CLK");
// static struct Stm32Clock STM32F1XX_TIM3_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM3CLK");
// static struct Stm32Clock STM32F1XX_TIM2_CLOCK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "TIM2CLK");

static Stm32Clock STM32F1XX_I2S3 = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "I2S3CLK");
static Stm32Clock STM32F1XX_I2S2 = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "I2S2CLK");


static Stm32Clock STM32F1XX_RTC_CLK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "RTCCLK");
static Stm32Clock STM32F1XX_SDIO_CLK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "SDIOCLK");
static Stm32Clock STM32F1XX_FSMC_CLK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "FSMCCLK");
static Stm32Clock STM32F1XX_HCLK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "HCLK");
static Stm32Clock STM32F1XX_FCLK = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "FCLK");
static Stm32Clock STM32F1XX_CORTEX_SYSTIM = STM32_CLOCK_TREE_INITIALIZE_LAST(.name = "SYSTIM");

static Stm32Prescaler STM32F1XX_USB_PRESCALERS[] = {STM32F1XX_PRESCALE_DIV1_5, STM32F1XX_PRESCALE_NONE};

static Stm32Prescaler STM32F1XX_AHB_DIV8_PRESCALERS[] = {
    STM32F1XX_PRESCALE_DIV8
};

static Stm32Prescaler STM32F1XX_AHB_DIV2_PRESCALERS[] = {
    STM32F1XX_PRESCALE_DIV8
};

static Stm32Clock* STM32F1XX_AHB_DIV8_OUTPUTS[] = {
    &STM32F1XX_CORTEX_SYSTIM
};

static Stm32Clock STM32F1XX_AHB_DIV8 = {
    .name = "AHB/8",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 4000000,
    .number_of_outputs = 0,
    .outputs = NULL,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_AHB_DIV8_PRESCALERS),
    .prescalers = STM32F1XX_AHB_DIV8_PRESCALERS,
    .selected_prescaler = &STM32F1XX_AHB_DIV8_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_AHB_DIV2_OUTPUTS[] = {
    &STM32F1XX_CORTEX_SYSTIM
};

static Stm32Clock STM32F1XX_AHB_DIV2 = {
    .name = "AHB/2",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 4000000,
    .number_of_outputs = 0,
    .outputs = NULL,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_AHB_DIV2_PRESCALERS),
    .prescalers = STM32F1XX_AHB_DIV2_PRESCALERS,
    .selected_prescaler = &STM32F1XX_AHB_DIV2_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_PCLK1_OUTPUTS[] = {
};

static Stm32Clock STM32F1XX_PCLK1 = {
    .name = "PCLK1",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 36000000,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS),
    .prescalers = STM32F1XX_DEFAULT_PRESCALERS,
    .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};



static Stm32Clock STM32F1XX_APB1_TIMERS = {
    .name = "APB1_TIM",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 36000000,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS),
    .prescalers = STM32F1XX_DEFAULT_PRESCALERS,
    .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_APB1_TIMER_CLK_OUTPUTS[] = {
};

static Stm32Clock STM32F1XX_APB1_TIMER_CLK = {
    .name = "APB1_TIMCLK",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 36000000,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS),
    .prescalers = STM32F1XX_DEFAULT_PRESCALERS,
    .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};


static Stm32Clock* STM32F1XX_APB1_TIMERS_OUTPUTS[] = {
    &STM32F1XX_APB1_TIMER_CLK
};

static Stm32Clock* STM32F1XX_PCLK2_OUTPUTS[] = {
};

static Stm32Clock STM32F1XX_PCLK2 = {
    .name = "PCLK2",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 36000000,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS),
    .prescalers = STM32F1XX_DEFAULT_PRESCALERS,
    .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};




static Stm32Clock STM32F1XX_APB2_TIMERS = {
    .name = "APB2_TIM",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 36000000,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS),
    .prescalers = STM32F1XX_DEFAULT_PRESCALERS,
    .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_APB2_TIMER_CLK_OUTPUTS[] = {
};

static Stm32Clock STM32F1XX_APB2_TIMER_CLK = {
    .name = "APB2_TIMCLK",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 36000000,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS),
    .prescalers = STM32F1XX_DEFAULT_PRESCALERS,
    .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_APB2_TIMERS_OUTPUTS[] = {
    &STM32F1XX_APB2_TIMER_CLK
};

static Stm32Prescaler STM32F1XX_ADC_PRESCALERS[] = {
    STM32F1XX_PRESCALE_DIV2,
    STM32F1XX_PRESCALE_DIV4,
    STM32F1XX_PRESCALE_DIV6,
    STM32F1XX_PRESCALE_DIV8
};

static Stm32Clock* STM32F1XX_ADC_OUTPUTS[] = {
};

static Stm32Clock STM32F1XX_ADC = {
    .name = "ADC",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 14000000,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_ADC_PRESCALERS),
    .prescalers = STM32F1XX_ADC_PRESCALERS,
    .selected_prescaler = &STM32F1XX_ADC_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};



// static Stm32Clock STM32F1XX_AHB_DIV8 = {
//     .name = "AHB/8",
//     .input_frequency = 0,
//     .output_frequency = 0,
//     .max_output_frequency = 4000000,
//     .number_of_outputs = 0,
//     .outputs = NULL,
//     .number_of_prescalers = ARRAY_SIZE(STM32F1XX_AHB_DIV8_PRESCALERS),
//     .prescalers = STM32F1XX_AHB_DIV8_PRESCALERS,
//     .selected_prescaler = &STM32F1XX_AHB_DIV8_PRESCALERS[0],
//     .enabled = false,
//     .number_of_observers = 0
// };

static Stm32Clock STM32F1XX_USB_PRESCALER = {
    .name = "USB_PRESCALER",
    .input_frequency = 0,
    .output_frequency = 48000000,
    .max_output_frequency = STM32_CLOCK_FREQUENCY_LIMIT,
    .number_of_outputs = 0,
    .outputs = NULL,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_USB_PRESCALERS),
    .prescalers = STM32F1XX_USB_PRESCALERS,
    .selected_prescaler = &STM32F1XX_USB_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_USB_PRESCALER_OUTPUTS[] = {
    &STM32F1XX_USBCLK
};


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

static struct Stm32Prescaler STM32F1XX_APB_PRESCALERS[] = {
    STM32F1XX_PRESCALE_NONE,
    STM32F1XX_PRESCALE_DIV2,
    STM32F1XX_PRESCALE_DIV4,
    STM32F1XX_PRESCALE_DIV8,
    STM32F1XX_PRESCALE_DIV16
};

static Stm32Clock STM32F1XX_APB1 = {
    .name = "APB1",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 36000000,
    .number_of_outputs = 0,
    .outputs = NULL,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_APB_PRESCALERS),
    .prescalers = STM32F1XX_APB_PRESCALERS,
    .selected_prescaler = &STM32F1XX_APB_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_APB1_OUTPUTS[] = {
    &STM32F1XX_APB1_TIMERS,
    &STM32F1XX_PCLK1
};

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

static Stm32Clock STM32F1XX_APB2 = {
    .name = "APB2",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 72000000,
    .number_of_outputs = 0,
    .outputs = NULL,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_APB_PRESCALERS),
    .prescalers = STM32F1XX_APB_PRESCALERS,
    .selected_prescaler = &STM32F1XX_APB_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_APB2_OUTPUTS[] = {
    &STM32F1XX_APB2_TIMERS,
    &STM32F1XX_PCLK2,
    &STM32F1XX_ADC
};



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

static const Stm32Prescaler STM32F1XX_AHB_PRESCALERS[] = {
    STM32F1XX_PRESCALE_NONE,
    STM32F1XX_PRESCALE_DIV2,
    STM32F1XX_PRESCALE_DIV4,
    STM32F1XX_PRESCALE_DIV8,
    STM32F1XX_PRESCALE_DIV16,
    STM32F1XX_PRESCALE_DIV32,
    STM32F1XX_PRESCALE_DIV64,
    STM32F1XX_PRESCALE_DIV128,
    STM32F1XX_PRESCALE_DIV256,
    STM32F1XX_PRESCALE_DIV512
};

static struct Stm32Clock* STM32F1XX_AHB_OUTPUTS[] = {
    &STM32F1XX_SDIO_CLK,
    &STM32F1XX_FSMC_CLK,
    &STM32F1XX_HCLK,
    &STM32F1XX_AHB_DIV8,
    &STM32F1XX_FCLK,
    &STM32F1XX_APB1,
    &STM32F1XX_APB2,
    &STM32F1XX_AHB_DIV2
};

static Stm32Clock STM32F1XX_AHB = {
    .name = "AHB",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 72000000,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_AHB_PRESCALERS),
    .prescalers = STM32F1XX_AHB_PRESCALERS,
    .selected_prescaler = &STM32F1XX_AHB_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static const Stm32Prescaler STM32F1XX_PLLCLK_PRESCALERS[] = {
    STM32F1XX_PRESCALE_MUL2,
    STM32F1XX_PRESCALE_MUL3,
    STM32F1XX_PRESCALE_MUL4,
    STM32F1XX_PRESCALE_MUL5,
    STM32F1XX_PRESCALE_MUL6,
    STM32F1XX_PRESCALE_MUL7,
    STM32F1XX_PRESCALE_MUL8,
    STM32F1XX_PRESCALE_MUL9,
    STM32F1XX_PRESCALE_MUL10,
    STM32F1XX_PRESCALE_MUL11,
    STM32F1XX_PRESCALE_MUL12,
    STM32F1XX_PRESCALE_MUL13,
    STM32F1XX_PRESCALE_MUL14,
    STM32F1XX_PRESCALE_MUL15,
    STM32F1XX_PRESCALE_MUL16
};

static Stm32Clock* STM32F1XX_SYSCLK_OUTPUTS[] = {
    &STM32F1XX_I2S2,
    &STM32F1XX_I2S3,
    &STM32F1XX_AHB
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
    &STM32F1XX_SYSCLK
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


static struct Stm32Clock STM32F1XX_PLLCLK = {
    .name = "PLLCLK",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 72000000,
    .number_of_outputs = 0,
    .outputs = NULL,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_PLLCLK_PRESCALERS),
    .prescalers = STM32F1XX_PLLCLK_PRESCALERS,
    .selected_prescaler = &STM32F1XX_PLLCLK_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_PLLCLK_OUTPUTS[] = {
    &STM32F1XX_SYSCLK,
    &STM32F1XX_USB_PRESCALER
};


static Stm32Clock* STM32F1XX_PLLSRC_OUTPUTS[] = {
    &STM32F1XX_PLLCLK
};


static Stm32Clock STM32F1XX_PLLSRC = {
    .name = "PLLSRC",
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

/////////////////////////////////////////////////////
//                         SOURCES
////////////////////////////////////////////////////
static Stm32Clock* STM32F1XX_LSE_OUTPUTS[] = {
    &STM32F1XX_RTC_CLK
};

static Stm32Clock STM32F1XX_LSE = STM32_CLOCK_TREE_INITIALIZE_SOURCE(
    .name = "LSE",
    .output_frequency = 32768,
    STM32F1XX_LSE_OUTPUTS
);

static Stm32Prescaler STM32F1XX_HSI_DIV2_PRESCALERS[] = {
    STM32F1XX_PRESCALE_DIV2
};

static Stm32Clock* STM32F1XX_HSI_DIV2_OUTPUTS[] = {
    &STM32F1XX_PLLSRC
};

static Stm32Clock STM32F1XX_HSI_DIV2 = {
    .name = "HSI/2",
    .input_frequency = 0,
    .output_frequency = 0,
    .max_output_frequency = 4000000,
    .number_of_outputs = 0,
    .outputs = NULL,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_HSI_DIV2_PRESCALERS),
    .prescalers = STM32F1XX_HSI_DIV2_PRESCALERS,
    .selected_prescaler = &STM32F1XX_HSI_DIV2_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

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
    .max_output_frequency = 16000000,
    .number_of_outputs = 0,
    .outputs = NULL,
    .number_of_prescalers = ARRAY_SIZE(STM32F1XX_DEFAULT_PRESCALERS),
    .prescalers = STM32F1XX_DEFAULT_PRESCALERS,
    .selected_prescaler = &STM32F1XX_DEFAULT_PRESCALERS[0],
    .enabled = false,
    .number_of_observers = 0
};

static Stm32Clock* STM32F1XX_PLLXTPRE_OUTPUTS[] = {
    &STM32F1XX_PLLSRC
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

static Stm32Clock* STM32F1XX_HSI_OUTPUTS[] = {
    &STM32F1XX_SYSCLK,
    &STM32F1XX_HSI_DIV2,
    &STM32F1XX_FLITFCLK
};

static Stm32Clock STM32F1XX_HSI = STM32_CLOCK_TREE_INITIALIZE_SOURCE(
    .name = "HSI",
    .output_frequency = 8000000,
    STM32F1XX_HSI_OUTPUTS
);

static Stm32Clock* STM32F1XX_LSI_OUTPUTS[] = {
    &STM32F1XX_RTC_CLK,
    &STM32F1XX_IWDG_CLOCK
};

static Stm32Clock STM32F1XX_LSI = STM32_CLOCK_TREE_INITIALIZE_SOURCE(
    .name = "LSI",
    .output_frequency = 40000,
    STM32F1XX_LSI_OUTPUTS
);

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
    &STM32F1XX_HSI,
    &STM32F1XX_HSE,
    &STM32F1XX_PLLCLK,
    &STM32F1XX_CSS
};

static Stm32Clock* STM32F1XX_HSE_DIV128_INPUTS[] = {
    &STM32F1XX_HSE
};

static Stm32Clock* STM32F1XX_FLITFCLK_INPUTS[] = {
    &STM32F1XX_HSI
};

static Stm32Clock* STM32F1XX_HSI_DIV2_INPUTS[] = {
    &STM32F1XX_HSI
};

static Stm32Clock* STM32F1XX_RTC_INPUTS[] = {
    &STM32F1XX_HSE_DIV128,
    &STM32F1XX_LSE,
    &STM32F1XX_LSI
};

static Stm32Clock* STM32F1XX_IWDG_INPUTS[] = {
    &STM32F1XX_LSI
};

static Stm32Clock* STM32F1XX_PLLSRC_INPUTS[] = {
    &STM32F1XX_HSI_DIV2,
    &STM32F1XX_PLLXTPRE
};

static Stm32Clock* STM32F1XX_PLLCLK_INPUTS[] = {
    &STM32F1XX_PLLSRC,
};

static Stm32Clock* STM32F1XX_USB_PRESCALER_INPUTS[] = {
    &STM32F1XX_PLLCLK
};

static Stm32Clock* STM32F1XX_I2S3_INPUTS[] = {
    &STM32F1XX_SYSCLK
};

static Stm32Clock* STM32F1XX_I2S2_INPUTS[] = {
    &STM32F1XX_SYSCLK
};

static Stm32Clock* STM32F1XX_AHB_INPUTS[] = {
    &STM32F1XX_SYSCLK
};

static Stm32Clock* STM32F1XX_SDIO_INPUTS[] = {
    &STM32F1XX_AHB
};

static Stm32Clock* STM32F1XX_FSMC_INPUTS[] = {
    &STM32F1XX_AHB
};

static Stm32Clock* STM32F1XX_HCLK_INPUTS[] = {
    &STM32F1XX_AHB
};

static Stm32Clock* STM32F1XX_FCLK_INPUTS[] = {
    &STM32F1XX_AHB
};

static Stm32Clock* STM32F1XX_AHB_DIV8_INPUTS[] = {
    &STM32F1XX_AHB
};

static Stm32Clock* STM32F1XX_AHB_DIV2_INPUTS[] = {
    &STM32F1XX_AHB
};

static Stm32Clock* STM32F1XX_SYSTIM_INPUTS[] = {
    &STM32F1XX_AHB
};

static Stm32Clock* STM32F1XX_APB1_INPUTS[] = {
    &STM32F1XX_AHB
};

static Stm32Clock* STM32F1XX_APB2_INPUTS[] = {
    &STM32F1XX_AHB
};

static Stm32Clock* STM32F1XX_PCLK1_INPUTS[] = {
    &STM32F1XX_APB1
};

static Stm32Clock* STM32F1XX_APB1_TIMERS_INPUTS[] = {
    &STM32F1XX_APB1
};

static Stm32Clock* STM32F1XX_APB1_TIMCLK_INPUTS[] = {
    &STM32F1XX_APB1_TIMERS
};

static Stm32Clock* STM32F1XX_PCLK2_INPUTS[] = {
    &STM32F1XX_APB2
};

static Stm32Clock* STM32F1XX_APB2_TIMERS_INPUTS[] = {
    &STM32F1XX_APB2
};

static Stm32Clock* STM32F1XX_APB2_TIMCLK_INPUTS[] = {
    &STM32F1XX_APB2_TIMERS
};

static Stm32Clock* STM32F1XX_ADC_INPUTS[] = {
    &STM32F1XX_APB2
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

    Stm32Clock* selected_clock;

    uint32_t hse_frequency;
    uint32_t lse_frequency;

    qemu_irq irq;
} Stm32F1xxRcc;

static void stm32f1xx_rcc_initialize_tree(Stm32Clock* hse, Stm32Clock* hsi, Stm32Clock* lse, Stm32Clock* lsi)
{
    /* HSE configuration */
    stm32_clock_set_outputs(hse, STM32F1XX_HSE_OUTPUTS, ARRAY_SIZE(STM32F1XX_HSE_OUTPUTS));

    Stm32Clock* css = stm32_clock_get_output(hse, "CSS");
    assert(css != NULL);
    stm32_clock_set_inputs(css, STM32F1XX_CSS_INPUTS, ARRAY_SIZE(STM32F1XX_CSS_INPUTS));
    stm32_clock_set_outputs(css, STM32F1XX_CSS_OUTPUTS, ARRAY_SIZE(STM32F1XX_CSS_OUTPUTS));

    Stm32Clock* pllxtpre = stm32_clock_get_output(hse, "PLLXTPRE");
    assert(pllxtpre != NULL);
    stm32_clock_set_inputs(pllxtpre, STM32F1XX_PLLXTPRE_INPUTS, ARRAY_SIZE(STM32F1XX_PLLXTPRE_INPUTS));
    stm32_clock_set_outputs(pllxtpre, STM32F1XX_PLLXTPRE_OUTPUTS, ARRAY_SIZE(STM32F1XX_PLLXTPRE_OUTPUTS));

    Stm32Clock* hse_div2 = stm32_clock_get_output(hse, "HSE/2");
    assert(hse_div2 != NULL);
    stm32_clock_set_inputs(hse_div2, STM32F1XX_HSE_DIV2_INPUTS, ARRAY_SIZE(STM32F1XX_HSE_DIV2_INPUTS));
    stm32_clock_set_outputs(hse_div2, STM32F1XX_HSE_DIV2_OUTPUTS, ARRAY_SIZE(STM32F1XX_HSE_DIV2_OUTPUTS));

    Stm32Clock* sysclk = stm32_clock_get_output(hse, "SYSCLK");
    assert(sysclk != NULL);
    stm32_clock_set_inputs(sysclk, STM32F1XX_SYSCLK_INPUTS, ARRAY_SIZE(STM32F1XX_SYSCLK_INPUTS));
    stm32_clock_set_outputs(sysclk, STM32F1XX_SYSCLK_OUTPUTS, ARRAY_SIZE(STM32F1XX_SYSCLK_OUTPUTS));
    sysclk->enabled = true;

    Stm32Clock* hse_div128 = stm32_clock_get_output(hse, "HSE/128");
    assert(hse_div128 != NULL);
    stm32_clock_set_inputs(hse_div128, STM32F1XX_HSE_DIV128_INPUTS, ARRAY_SIZE(STM32F1XX_HSE_DIV128_INPUTS));
    stm32_clock_set_outputs(hse_div128, STM32F1XX_HSE_DIV128_OUTPUTS, ARRAY_SIZE(STM32F1XX_HSE_DIV128_OUTPUTS));

    Stm32Clock* pllsrc = stm32_clock_get_output(pllxtpre, "PLLSRC");
    assert(pllsrc != NULL);
    stm32_clock_set_inputs(pllsrc, STM32F1XX_PLLSRC_INPUTS, ARRAY_SIZE(STM32F1XX_PLLSRC_INPUTS));
    stm32_clock_set_outputs(pllsrc, STM32F1XX_PLLSRC_OUTPUTS, ARRAY_SIZE(STM32F1XX_PLLSRC_OUTPUTS));

    Stm32Clock* pllclk = stm32_clock_get_output(pllsrc, "PLLCLK");
    assert(pllclk != NULL);
    stm32_clock_set_inputs(pllclk, STM32F1XX_PLLCLK_INPUTS, ARRAY_SIZE(STM32F1XX_PLLCLK_INPUTS));
    stm32_clock_set_outputs(pllclk, STM32F1XX_PLLCLK_OUTPUTS, ARRAY_SIZE(STM32F1XX_PLLCLK_OUTPUTS));

    Stm32Clock* usb = stm32_clock_get_output(pllclk, "USB_PRESCALER");
    assert(usb != NULL);
    stm32_clock_set_inputs(usb, STM32F1XX_USB_PRESCALER_INPUTS, ARRAY_SIZE(STM32F1XX_USB_PRESCALER_INPUTS));
    stm32_clock_set_outputs(usb, STM32F1XX_USB_PRESCALER_OUTPUTS, ARRAY_SIZE(STM32F1XX_USB_PRESCALER_OUTPUTS));
    /* HSI configuration */
    stm32_clock_set_outputs(hsi, STM32F1XX_HSI_OUTPUTS, ARRAY_SIZE(STM32F1XX_HSI_OUTPUTS));

    Stm32Clock* flitfclk = stm32_clock_get_output(hsi, "FLITFCLK");
    assert(flitfclk != NULL);
    stm32_clock_set_inputs(flitfclk, STM32F1XX_FLITFCLK_INPUTS, ARRAY_SIZE(STM32F1XX_FLITFCLK_INPUTS));

    Stm32Clock* hsi_div2 = stm32_clock_get_output(hsi, "HSI/2");
    assert(hsi_div2 != NULL);
    stm32_clock_set_inputs(hsi_div2, STM32F1XX_HSI_DIV2_INPUTS, ARRAY_SIZE(STM32F1XX_HSI_DIV2_INPUTS));
    stm32_clock_set_outputs(hsi_div2, STM32F1XX_HSI_DIV2_OUTPUTS, ARRAY_SIZE(STM32F1XX_HSI_DIV2_OUTPUTS));

    /* LSE configuration */
    stm32_clock_set_outputs(lse, STM32F1XX_LSE_OUTPUTS, ARRAY_SIZE(STM32F1XX_LSE_OUTPUTS));

    Stm32Clock* rtc = stm32_clock_get_output(lse, "RTCCLK");
    assert(rtc != NULL);
    stm32_clock_set_inputs(rtc, STM32F1XX_RTC_INPUTS, ARRAY_SIZE(STM32F1XX_RTC_INPUTS));

    /* LSI configuration */
    stm32_clock_set_outputs(lsi, STM32F1XX_LSI_OUTPUTS, ARRAY_SIZE(STM32F1XX_LSI_OUTPUTS));

    Stm32Clock* iwdg = stm32_clock_get_output(lsi, "IWDGCLK");
    assert(iwdg != NULL);
    stm32_clock_set_inputs(iwdg, STM32F1XX_IWDG_INPUTS, ARRAY_SIZE(STM32F1XX_IWDG_INPUTS));

    /* SYSCLK */
    Stm32Clock* i2s3 = stm32_clock_get_output(sysclk, "I2S3CLK");
    assert(i2s3 != NULL);
    stm32_clock_set_inputs(i2s3, STM32F1XX_I2S3_INPUTS, ARRAY_SIZE(STM32F1XX_I2S3_INPUTS));

    Stm32Clock* i2s2 = stm32_clock_get_output(sysclk, "I2S2CLK");
    assert(i2s2 != NULL);
    stm32_clock_set_inputs(i2s2, STM32F1XX_I2S2_INPUTS, ARRAY_SIZE(STM32F1XX_I2S2_INPUTS));

    Stm32Clock* ahb = stm32_clock_get_output(sysclk, "AHB");
    assert(ahb != NULL);
    stm32_clock_set_inputs(ahb, STM32F1XX_AHB_INPUTS, ARRAY_SIZE(STM32F1XX_AHB_INPUTS));
    stm32_clock_set_outputs(ahb, STM32F1XX_AHB_OUTPUTS, ARRAY_SIZE(STM32F1XX_AHB_OUTPUTS));

    Stm32Clock* sdio = stm32_clock_get_output(ahb, "SDIOCLK");
    assert(sdio != NULL);
    stm32_clock_set_inputs(sdio, STM32F1XX_SDIO_INPUTS, ARRAY_SIZE(STM32F1XX_SDIO_INPUTS));

    Stm32Clock* fsmc = stm32_clock_get_output(ahb, "FSMCCLK");
    assert(fsmc != NULL);
    stm32_clock_set_inputs(fsmc, STM32F1XX_FSMC_INPUTS, ARRAY_SIZE(STM32F1XX_FSMC_INPUTS));

    Stm32Clock* ahb_div8 = stm32_clock_get_output(ahb, "AHB/8");
    assert(ahb_div8 != NULL);
    stm32_clock_set_inputs(ahb_div8, STM32F1XX_AHB_DIV8_INPUTS, ARRAY_SIZE(STM32F1XX_AHB_DIV8_INPUTS));
    stm32_clock_set_outputs(ahb_div8, STM32F1XX_AHB_DIV8_OUTPUTS, ARRAY_SIZE(STM32F1XX_AHB_DIV8_OUTPUTS));

    Stm32Clock* systim = stm32_clock_get_output(ahb_div8, "SYSTIM");
    assert(systim != NULL);
    stm32_clock_set_inputs(systim, STM32F1XX_SYSTIM_INPUTS, ARRAY_SIZE(STM32F1XX_SYSTIM_INPUTS));

    Stm32Clock* apb1 = stm32_clock_get_output(ahb, "APB1");
    assert(apb1 != NULL);
    stm32_clock_set_inputs(apb1, STM32F1XX_APB1_INPUTS, ARRAY_SIZE(STM32F1XX_APB1_INPUTS));
    stm32_clock_set_outputs(apb1, STM32F1XX_APB1_OUTPUTS, ARRAY_SIZE(STM32F1XX_APB1_OUTPUTS));

    Stm32Clock* pclk1 = stm32_clock_get_output(apb1, "PCLK1");
    assert(pclk1 != NULL);
    stm32_clock_set_inputs(pclk1, STM32F1XX_PCLK1_INPUTS, ARRAY_SIZE(STM32F1XX_PCLK1_INPUTS));
    stm32_clock_set_outputs(pclk1, STM32F1XX_PCLK1_OUTPUTS, ARRAY_SIZE(STM32F1XX_PCLK1_OUTPUTS));

    Stm32Clock* apb1_tim = stm32_clock_get_output(apb1, "APB1_TIM");
    assert(apb1_tim != NULL);
    stm32_clock_set_inputs(apb1_tim, STM32F1XX_APB1_TIMERS_INPUTS, ARRAY_SIZE(STM32F1XX_APB1_TIMERS_INPUTS));
    stm32_clock_set_outputs(apb1_tim, STM32F1XX_APB1_TIMERS_OUTPUTS, ARRAY_SIZE(STM32F1XX_APB1_TIMERS_OUTPUTS));

    Stm32Clock* apb1_timclk = stm32_clock_get_output(apb1_tim, "APB1_TIMCLK");
    assert(apb1_timclk != NULL);
    stm32_clock_set_inputs(apb1_timclk, STM32F1XX_APB1_TIMCLK_INPUTS, ARRAY_SIZE(STM32F1XX_APB1_TIMCLK_INPUTS));
    stm32_clock_set_outputs(apb1_timclk, STM32F1XX_APB1_TIMER_CLK_OUTPUTS, ARRAY_SIZE(STM32F1XX_APB1_TIMER_CLK_OUTPUTS));

    Stm32Clock* apb2 = stm32_clock_get_output(ahb, "APB2");
    assert(apb2 != NULL);
    stm32_clock_set_inputs(apb2, STM32F1XX_APB2_INPUTS, ARRAY_SIZE(STM32F1XX_APB2_INPUTS));
    stm32_clock_set_outputs(apb2, STM32F1XX_APB2_OUTPUTS, ARRAY_SIZE(STM32F1XX_APB2_OUTPUTS));

    Stm32Clock* pclk2 = stm32_clock_get_output(apb2, "PCLK2");
    assert(pclk2 != NULL);
    stm32_clock_set_inputs(pclk2, STM32F1XX_PCLK2_INPUTS, ARRAY_SIZE(STM32F1XX_PCLK2_INPUTS));
    stm32_clock_set_outputs(pclk2, STM32F1XX_PCLK2_OUTPUTS, ARRAY_SIZE(STM32F1XX_PCLK2_OUTPUTS));

    Stm32Clock* apb2_tim = stm32_clock_get_output(apb2, "APB2_TIM");
    assert(apb2_tim != NULL);
    stm32_clock_set_inputs(apb2_tim, STM32F1XX_APB2_TIMERS_INPUTS, ARRAY_SIZE(STM32F1XX_APB2_TIMERS_INPUTS));
    stm32_clock_set_outputs(apb2_tim, STM32F1XX_APB2_TIMERS_OUTPUTS, ARRAY_SIZE(STM32F1XX_APB2_TIMERS_OUTPUTS));

    Stm32Clock* apb2_timclk = stm32_clock_get_output(apb2_tim, "APB2_TIMCLK");
    assert(apb2_timclk != NULL);
    stm32_clock_set_inputs(apb2_timclk, STM32F1XX_APB2_TIMCLK_INPUTS, ARRAY_SIZE(STM32F1XX_APB2_TIMCLK_INPUTS));
    stm32_clock_set_outputs(apb2_timclk, STM32F1XX_APB2_TIMER_CLK_OUTPUTS, ARRAY_SIZE(STM32F1XX_APB2_TIMER_CLK_OUTPUTS));

    Stm32Clock* adc = stm32_clock_get_output(apb2, "ADC");
    assert(adc != NULL);
    stm32_clock_set_inputs(adc, STM32F1XX_ADC_INPUTS, ARRAY_SIZE(STM32F1XX_ADC_INPUTS));
    stm32_clock_set_outputs(adc, STM32F1XX_ADC_OUTPUTS, ARRAY_SIZE(STM32F1XX_ADC_OUTPUTS));

    Stm32Clock* fclk = stm32_clock_get_output(ahb, "FCLK");
    assert(fclk != NULL);
    stm32_clock_set_inputs(fclk, STM32F1XX_FCLK_INPUTS, ARRAY_SIZE(STM32F1XX_FCLK_INPUTS));

    Stm32Clock* hclk = stm32_clock_get_output(ahb, "HCLK");
    assert(hclk != NULL);
    stm32_clock_set_inputs(hclk, STM32F1XX_HCLK_INPUTS, ARRAY_SIZE(STM32F1XX_HCLK_INPUTS));

    Stm32Clock* ahb_div2 = stm32_clock_get_output(ahb, "AHB/2");
    assert(ahb_div2 != NULL);
    stm32_clock_set_inputs(ahb_div2, STM32F1XX_AHB_DIV2_INPUTS, ARRAY_SIZE(STM32F1XX_AHB_DIV2_INPUTS));
    stm32_clock_set_outputs(ahb_div2, STM32F1XX_AHB_DIV2_OUTPUTS, ARRAY_SIZE(STM32F1XX_AHB_DIV2_OUTPUTS));
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

static bool stm32f1xx_rcc_is_hsi_enabled(Stm32F1xxRcc* self)
{
    return GET_BIT(self->reg.RCC_CR, STM32F1XX_HSION_BIT);
}

static bool stm32f1xx_rcc_is_hse_enabled(Stm32F1xxRcc* self)
{
    return GET_BIT(self->reg.RCC_CR, STM32F1XX_HSEON_BIT);
}

static bool stm32f1xx_rcc_is_pll_enabled(Stm32F1xxRcc* self)
{
    return GET_BIT(self->reg.RCC_CR, STM32F1XX_PLLON_BIT);
}


static void stm32f1xx_rcc_recalculate_clock_tree(Stm32F1xxRcc* self)
{
    stm32_clock_update_clocks(self->selected_clock);
    Stm32Clock* sysclk = stm32_clock_get_descendant(self->selected_clock, "SYSCLK");
    system_clock_scale = NANOSECONDS_PER_SECOND / sysclk->output_frequency;
}

static void stm32f1xx_rcc_init_clock_tree(Stm32F1xxRcc* self)
{
    // fprintf(stderr, "HSE: %u, LSE: %u\n", self->hse_frequency, self->lse_frequency);
    self->hse = &STM32F1XX_HSE;
    self->hse->output_frequency = self->hse_frequency;

    self->hsi = &STM32F1XX_HSI;
    self->lse = &STM32F1XX_LSE;
    self->lse->output_frequency = self->lse_frequency;
    self->lsi = &STM32F1XX_LSI;
    stm32f1xx_rcc_initialize_tree(self->hse, self->hsi, self->lse, self->lsi);
    self->selected_clock = self->hsi;
    self->hsi->enabled = true;
    stm32f1xx_rcc_recalculate_clock_tree(self);
}

static uint64_t stm32f1xx_rcc_read_word(Stm32F1xxRcc *self, hwaddr offset)
{
    switch(offset)
    {
        case STM32F1XX_RCC_CR_OFFSET:
        {
            return self->reg.RCC_CR;
        } break;
        case STM32F1XX_RCC_CFGR_OFFSET:
        {
            return self->reg.RCC_CFGR;
        } break;
        case STM32F1XX_RCC_CIR_OFFSET:
        {
            return self->reg.RCC_CIR;
        } break;
        case STM32F1XX_RCC_APB2RSTR_OFFSET:
        {
            return self->reg.RCC_APB2RSTR;
        } break;
        case STM32F1XX_RCC_APB1RSTR_OFFSET:
        {
            return self->reg.RCC_APB1RSTR;
        } break;
        case STM32F1XX_RCC_AHBENR_OFFSET:
        {
            return self->reg.RCC_AHBENR;
        } break;
        case STM32F1XX_RCC_APB2ENR_OFFSET:
        {
            return self->reg.RCC_APB2ENR;
        } break;
        case STM32F1XX_RCC_APB1ENR_OFFSET:
        {
            return self->reg.RCC_APB1ENR;
        } break;
        case STM32F1XX_RCC_BDCR_OFFSET:
        {
            return self->reg.RCC_BDCR;
        } break;
        case STM32F1XX_RCC_CSR_OFFSET:
        {
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
    if (GET_BIT(value, STM32F1XX_HSION_BIT))
    {
        if (!self->hsi->enabled)
        {
            self->hsi->enabled = true;
            SET_BIT(self->reg.RCC_CR, STM32F1XX_HSION_BIT);
            SET_BIT(self->reg.RCC_CR, STM32F1XX_HSIRDY_BIT);
            SET_BIT(self->reg.RCC_CIR, STM32F1XX_HSIRDYF_BIT);
        }
    }
    if (value & STM32F1XX_HSITRIM_MASK)
    {
        qemu_log_mask(LOG_UNIMP, "Writing to HSITRIM unimplemented");
    }
    if (GET_BIT(value, STM32F1XX_HSEON_BIT))
    {
        if (!self->hse->enabled)
        {
            self->hse->enabled = true;
            SET_BIT(self->reg.RCC_CR, STM32F1XX_HSEON_BIT);
            SET_BIT(self->reg.RCC_CR, STM32F1XX_HSERDY_BIT);
            SET_BIT(self->reg.RCC_CIR, STM32F1XX_HSERDYF_BIT);
        }
    }
    if (GET_BIT(value, STM32F1XX_HSEBYP_BIT))
    {
        qemu_log_mask(LOG_UNIMP, "Writing to HSEBYP unimplemented");
    }
    if (GET_BIT(value, STM32F1XX_CSSON_BIT))
    {
        qemu_log_mask(LOG_UNIMP, "Writing to CSSON unimplemented");
    }
    if (GET_BIT(value, STM32F1XX_PLLON_BIT))
    {
        Stm32Clock* pllsrc = stm32_clock_get_descendant(self->hse, "PLLSRC");
        Stm32Clock* pllclk = stm32_clock_get_descendant(self->hse, "PLLCLK");
        Stm32Clock* pllxtpre = stm32_clock_get_descendant(self->hse, "PLLXTPRE");

        if (!pllsrc->enabled || !pllclk->enabled || !pllxtpre->enabled)
        {
            pllsrc->enabled = true;
            pllclk->enabled = true;
            pllxtpre->enabled = true;
            stm32_clock_update_clocks(pllxtpre);
            SET_BIT(self->reg.RCC_CR, STM32F1XX_PLLREADY_BIT);
        }

    }
    if (GET_BIT(value, STM32F1XX_PLL2ON_BIT))
    {
        qemu_log_mask(LOG_UNIMP, "Writing to PLL2ON unimplemented");
    }
    if (GET_BIT(value, STM32F1XX_PLL3ON_BIT))
    {
        qemu_log_mask(LOG_UNIMP, "Writing to PLL3ON unimplemented");
    }
}

static void stm32f1xx_rcc_write_cfgr(Stm32F1xxRcc *self, uint64_t value)
{
    if ((self->reg.RCC_CFGR & STM32F1XX_PLLMUL_MASK) != (value & STM32F1XX_PLLMUL_MASK))
    {
        Stm32Clock *pllclk = stm32_clock_get_descendant(self->hse, "PLLCLK");
        if (pllclk->enabled)
        {
            warn_report("Can't change PLLMUL while PLL is enabled");
        }

        uint32_t pllmul_value = UNMASK_VALUE(value, STM32F1XX_PLLMUL_START_BIT, STM32F1XX_PLLMUL_MASK);
        SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_PLLMUL_START_BIT, STM32F1XX_PLLMUL_MASK, pllmul_value);
        pllclk->selected_prescaler = &pllclk->prescalers[pllmul_value];
    }

    uint64_t source = UNMASK_VALUE(value, STM32F1XX_SW_START_BIT, STM32F1XX_SW_MASK);
    Stm32Clock* sysclk = stm32_clock_get_descendant(self->hse, "SYSCLK");
    switch (source)
    {
        case 0: {
            SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_SWS_START_BIT, STM32F1XX_SWS_MASK, source);
            SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_SW_START_BIT, STM32F1XX_SW_MASK, source);
            sysclk->selected_input = self->hsi;
            self->selected_clock = self->hsi;
        } break;
        case 1: {
            SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_SWS_START_BIT, STM32F1XX_SWS_MASK, source);
            SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_SW_START_BIT, STM32F1XX_SW_MASK, source);
            sysclk->selected_input = self->hse;
            self->selected_clock = self->hse;

        } break;
        case 2: {
            SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_SWS_START_BIT, STM32F1XX_SWS_MASK, source);
            SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_SW_START_BIT, STM32F1XX_SW_MASK, source);
            Stm32Clock* pllclk = stm32_clock_get_descendant(self->hse, "PLLCLK");
            sysclk->selected_input = pllclk;
            self->selected_clock = pllclk;
        } break;
        case 3: {
            warn_report("Value 0x3 is not allowed for SYSCLK input, see RM0008");
        } break;

    }
    if ((self->reg.RCC_CFGR & STM32F1XX_HPRE_MASK) != (value & STM32F1XX_HPRE_MASK))
    {
        uint32_t hpre_value = UNMASK_VALUE(value, STM32F1XX_HPRE_START_BIT, STM32F1XX_HPRE_MASK);
        SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_HPRE_START_BIT, STM32F1XX_HPRE_MASK, hpre_value);
        Stm32Clock* ahb = stm32_clock_get_descendant(self->hse, "AHB");
        if (hpre_value & 0x8)
        {
            hpre_value -= 7;
        }
        else
        {
            hpre_value = 0;
        }

        ahb->selected_prescaler = &ahb->prescalers[hpre_value];
    }

    if ((self->reg.RCC_CFGR & STM32F1XX_PPRE1_MASK) != (value & STM32F1XX_PPRE1_MASK))
    {
        uint32_t ppre1_value = UNMASK_VALUE(value, STM32F1XX_PPRE1_START_BIT, STM32F1XX_PPRE1_MASK);
        SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_PPRE1_START_BIT, STM32F1XX_PPRE1_MASK, ppre1_value);
        Stm32Clock* apb1 = stm32_clock_get_descendant(self->hse, "APB1");
        if (ppre1_value & 0x4)
        {
            ppre1_value = 0;
        }
        else
        {
            ppre1_value -= 3;
        }

        apb1->selected_prescaler = &apb1->prescalers[ppre1_value];
    }

    if ((self->reg.RCC_CFGR & STM32F1XX_PPRE2_MASK) != (value & STM32F1XX_PPRE2_MASK))
    {
        uint32_t ppre2_value = UNMASK_VALUE(value, STM32F1XX_PPRE2_START_BIT, STM32F1XX_PPRE2_MASK);
        SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_PPRE2_START_BIT, STM32F1XX_PPRE2_MASK, ppre2_value);
        Stm32Clock* apb2 = stm32_clock_get_descendant(self->hse, "APB2");
        if (ppre2_value & 0x4)
        {
            ppre2_value = 0;
        }
        else
        {
            ppre2_value -= 3;
        }

        apb2->selected_prescaler = &apb2->prescalers[ppre2_value];
    }

    if ((self->reg.RCC_CFGR & STM32F1XX_ADCPRE_MASK) != (value & STM32F1XX_ADCPRE_MASK))
    {
        uint32_t adcpre_value = UNMASK_VALUE(value, STM32F1XX_ADCPRE_START_BIT, STM32F1XX_ADCPRE_MASK);
        SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_ADCPRE_START_BIT, STM32F1XX_ADCPRE_MASK, adcpre_value);
        Stm32Clock* adc = stm32_clock_get_descendant(self->hse, "ADC");

        adc->selected_prescaler = &adc->prescalers[adcpre_value];
    }

    if (GET_BIT(value, STM32F1XX_PLLSRC_BIT) != GET_BIT(self->reg.RCC_CFGR, STM32F1XX_PLLSRC_BIT))
    {
        uint32_t pllsrc_bit = GET_BIT(value, STM32F1XX_PLLSRC_BIT);
        CHANGE_BIT(self->reg.RCC_CFGR, STM32F1XX_PLLSRC_BIT, pllsrc_bit);
        Stm32Clock* pllsrc = stm32_clock_get_descendant(self->hse, "PLLSRC");
        pllsrc->selected_input = pllsrc->inputs[pllsrc_bit];
    }

    if (GET_BIT(value, STM32F1XX_PLLXTPRE_BIT) != GET_BIT(self->reg.RCC_CFGR, STM32F1XX_PLLXTPRE_BIT))
    {
        uint32_t pllxtpre_bit = GET_BIT(value, STM32F1XX_PLLXTPRE_BIT);
        CHANGE_BIT(self->reg.RCC_CFGR, STM32F1XX_PLLXTPRE_BIT, pllxtpre_bit);
        Stm32Clock* pllxtpre = stm32_clock_get_descendant(self->hse, "PLLXTPRE");
        pllxtpre->selected_input = pllxtpre->inputs[pllxtpre_bit];
    }

    if ((self->reg.RCC_CFGR & STM32F1XX_PLLMUL_MASK) != (value & STM32F1XX_PLLMUL_MASK))
    {
        uint32_t pllmul_value = UNMASK_VALUE(value, STM32F1XX_PLLMUL_START_BIT, STM32F1XX_PLLMUL_MASK);
        SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_PLLMUL_START_BIT, STM32F1XX_PLLMUL_MASK, pllmul_value);
        Stm32Clock* pllclk = stm32_clock_get_descendant(self->hse, "PLLCLK");
        pllclk->selected_prescaler = &pllclk->prescalers[pllmul_value];
    }

    if (GET_BIT(value, STM32F1XX_OTGFSPRE_BIT))
    {
        uint32_t otgfspre_bit = GET_BIT(value, STM32F1XX_OTGFSPRE_BIT);
        CHANGE_BIT(self->reg.RCC_CFGR, STM32F1XX_OTGFSPRE_BIT, otgfspre_bit);
        Stm32Clock *usb_prescaler = stm32_clock_get_descendant(self->hse, "USB_PRESCALER");
        usb_prescaler->selected_prescaler = &usb_prescaler->prescalers[otgfspre_bit];
    }

    if ((self->reg.RCC_CFGR & STM32F1XX_MCO_MASK) != (value & STM32F1XX_MCO_MASK))
    {
        uint32_t mco_value = UNMASK_VALUE(value, STM32F1XX_MCO_START_BIT, STM32F1XX_MCO_MASK);
        SET_MASKED_VALUE(self->reg.RCC_CFGR, STM32F1XX_MCO_START_BIT, STM32F1XX_MCO_MASK, mco_value);
        qemu_log_mask(LOG_UNIMP, "Selecting MCO - unimplemented");
    }

    stm32f1xx_rcc_recalculate_clock_tree(self);
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
            qemu_log_mask(LOG_UNIMP, "Write to CIR - unimplemented");
        } break;
        case STM32F1XX_RCC_APB2RSTR_OFFSET:
        {
            qemu_log_mask(LOG_UNIMP, "Write to ABP2RSTR - unimplemented");
        } break;
        case STM32F1XX_RCC_APB1RSTR_OFFSET:
        {
            qemu_log_mask(LOG_UNIMP, "Write to APB1RSTR - unimplemented");
        } break;
        case STM32F1XX_RCC_AHBENR_OFFSET:
        {
            qemu_log_mask(LOG_UNIMP, "Write to AHBENR - unimplemented");
        } break;
        case STM32F1XX_RCC_APB2ENR_OFFSET:
        {
            qemu_log_mask(LOG_UNIMP, "Write to APB2ENR - unimplemented");
        } break;
        case STM32F1XX_RCC_APB1ENR_OFFSET:
        {
            qemu_log_mask(LOG_UNIMP, "Write to APB1ENR - unimplemented");
        } break;
        case STM32F1XX_RCC_BDCR_OFFSET:
        {
            qemu_log_mask(LOG_UNIMP, "Write to BDCR - unimplemented");
        } break;
        case STM32F1XX_RCC_CSR_OFFSET:
        {
            qemu_log_mask(LOG_UNIMP, "Write to CSR - unimplemented");
        } break;
    }
}

static void stm32f1xx_rcc_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    Stm32F1xxRcc *self = STM32F1XX_RCC(opaque);
    if (size == 4)
    {
        stm32f1xx_rcc_write_word(self, offset, value);
        return;
    }
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

}

static void stm32f1xx_rcc_realize(DeviceState *device, Error **errp)
{
    Stm32F1xxRcc *rcc = STM32F1XX_RCC(device);
    stm32f1xx_rcc_reset_registers(&rcc->reg);
    stm32f1xx_rcc_init_clock_tree(rcc);
}

static void stm32f1xx_rcc_reset(DeviceState *device)
{
    Stm32F1xxRcc *rcc = STM32F1XX_RCC(device);

    stm32f1xx_rcc_reset_registers(&rcc->reg);
}

static Property stm32f1xx_rcc_properties[] = {
    DEFINE_PROP_UINT32("hse-frequency", Stm32F1xxRcc, hse_frequency, 0),
    DEFINE_PROP_UINT32("lse-frequency", Stm32F1xxRcc, lse_frequency, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32f1xx_rcc_class_init(ObjectClass *klass, void* data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_props(dc, stm32f1xx_rcc_properties);
    dc->reset = stm32f1xx_rcc_reset;
    dc->realize = stm32f1xx_rcc_realize;
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

