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

#include <stdint.h>

#include <stm32f1xx_soc.h>
#include <stm32_clk.h>

#define STM32F1XX_HSI_FREQUENCY 8000000
#define STM32F1XX_LSI_FREQUENCY 40000
#define STM32F1XX_LSE_FREQUENCY 32768

/* RCC_CR */
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
#define STM32F1XX_MCO_START_BIT      24
#define STM32F1XX_MCO_MASK           0x07000000
#define STM32F1XX_USBPRE_BIT         22 
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
#define STM32F1XX_TIM11RST_BIT       21
#define STM32F1XX_TIM10RST_BIT       20
#define STM32F1XX_TIM9RST_BIT        19
#define STM32F1XX_ADC3RST_BIT        15
#define STM32F1XX_USART1RST_BIT      14
#define STM32F1XX_TIM8RST_BIT        13
#define STM32F1XX_SPI1RST_BIT        12
#define STM32F1XX_TIM1RST_BIT        11
#define STM32F1XX_ADC2RST_BIT        10
#define STM32F1XX_ADC1RST_BIT        9
#define STM32F1XX_IOPGRST_BIT        8
#define STM32F1XX_IOPFRST_BIT        7     
#define STM32F1XX_IOPERST_BIT        6
#define STM32F1XX_IOPDRST_BIT        5
#define STM32F1XX_IOPCRST_BIT        4
#define STM32F1XX_IOPBRST_BIT        3
#define STM32F1XX_IOPARST_BIT        2
#define STM32F1XX_AFIORST_BIT        0

/* RCC_APB1RSTR */
#define STM32F1XX_DACRST_BIT         29
#define STM32F1XX_PWRRST_BIT         28
#define STM32F1XX_BKPRST_BIT         27
#define STM32F1XX_CANRST_BIT         25
#define STM32F1XX_USBRST_BIT         23
#define STM32F1XX_I2C2RST_BIT        22
#define STM32F1XX_I2C1RST_BIT        21
#define STM32F1XX_UART5RST_BIT       20
#define STM32F1XX_UART4RST_BIT       19
#define STM32F1XX_UART3RST_BIT       18
#define STM32F1XX_UART2RST_BIT       17
#define STM32F1XX_SPI3RST_BIT        15
#define STM32F1XX_SPI2RST_BIT        14
#define STM32F1XX_WWDGRST_BIT        11
#define STM32F1XX_TIM14RST_BIT       8
#define STM32F1XX_TIM13RST_BIT       7
#define STM32F1XX_TIM12RST_BIT       6
#define STM32F1XX_TIM7RST_BIT        5
#define STM32F1XX_TIM6RST_BIT        4
#define STM32F1XX_TIM5RST_BIT        3
#define STM32F1XX_TIM4RST_BIT        2
#define STM32F1XX_TIM3RST_BIT        1
#define STM32F1XX_TIM2RST_BIT        0

/* RCC_AHBENR */
#define STM32F1XX_SDIOEN_BIT         10
#define STM32F1XX_FSMCEN_BIT         8
#define STM32F1XX_CRCEN_BIT          6
#define STM32F1XX_FLITFEN_BIT        4
#define STM32F1XX_SRAMEN_BIT         2
#define STM32F1XX_DMA2EN_BIT         1
#define STM32F1XX_DMA1EN_BIT         0

/* RCC_APB2ENR */
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
#define STM32F1XX_BDRST_BIT          16
#define STM32F1XX_RTCEN_BIT          15
#define STM32F1XX_RTCSEL_START_BIT   8
#define STM32F1XX_RTCSEL_MASK        0x00000300
#define STM32F1XX_LSEBYP_BIT         2
#define STM32F1XX_LSERDY_BIT         1
#define STM32F1XX_LSEON_BIT          0

/* RCC_CSR */
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
#define SET_BIT(to, bit) (from | (1 << bit))

struct Stm32F1XXRccRegisters
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
};

struct Stm32F1XXRcc
{
    SysBusDevice busdev;

    MemoryRegion iomem;

    Stm32F1XXRccRegisters reg;

    Stm32Clock hse;
    Stm32Clock lse;
    Stm32Clock hsi;
    Stm32Clock lsi;
    Stm32Clock sysclk;
    Stm32Clock pllclk;
    Stm32Clock mco;
    Stm32Clock periph_clk[STM32F1XX_NUMBER_OF_PERIPHS];

    qemu_irq irq;
};

static void stm32_f1xx_rcc_registers_reset(Stm32F1XXRccRegisters* self)
{
    self->RCC_CR       = 0x00000083;
    self->RCC_CFGR     = 0x00000000;
    self->RCC_CIR      = 0x00000000;
    self->RCC_APB2RSTR = 0x00000000;
    self->RCC_APB1RSTR = 0x00000000;
    self->RCC_AHBENR   = 0x00000014;
    self->RCC_APB2ENR  = 0x00000000;
    self->RCC_APB1ENR  = 0x00000000;
    self->RCC_BDCR     = 0x00000000;
    self->RCC_CSR      = 0x0C000000;
}

static void stm32_f1xx_init_clock_tree(Stm32F1XXRcc* self)
{
    int i = 0; 
    for (i = 0; i < STM32F1XX_NUMBER_OF_PERIPHS; ++i)
    {
        self->periph_clk[i] = NULL:
    }

    // init sources HSI, LSI, HSE(4-16MHz), LSE(32768HZ) ( requires data from upper leve) 
    
    // create childs AHBCLK 
    // AHB -> APB1, APB2
    // USB
    
}

static void stm32_f1xx_rcc_init(SysBusDevice *device)
{
    Stm32F1XXRcc *rcc = STM32_F1XX_RCC(device);

    memory_region_init_io(&rcc->iomem, OBJECT(rcc), &stm32_f1xx_operations, rcc, "stm32_f1xx_rcc", 0x3FF);
    sysbus_init_mmio(device, &rcc->iomem);
    sysbus_init_irq(device, &rcc->irq);
    stm32_f1xx_init_clock_tree();
}

static const MemoryRegionOps stm32_f1xx_operations = {
    .read = stm32_f1xx_rcc_read,
    .write = stm32_f1xx_rcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static Property stm32_f1xx_rcc_properties[] = {
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_f1xx_rcc_class_init(ObjectClass *klass, void* data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDevice *sbd = SYS_BUS_DEVICE_CLASS(klass);

    sbd->init = stm32_f1xx_init;
    dc->reset = stm32_f1xx_reset;
    dc->props = stm32_f1xx_rcc_properties;
}



static TypeInfo stm32_f1xx_rcc_info = {
    .name = "stm32_f1xx_rcc",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32F1XXRcc),
    .class_init = stm32_f1xx_rcc_class_init
};

static void stm32_f1xx_rcc_register_type(void)
{
    type_register_static(&stm32_f1xx_rcc_info);
}

type_init(stm32_f1xx_rcc_register_type);

