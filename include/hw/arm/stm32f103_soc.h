/*
 * STM32F103 SoC
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

#ifndef HW_ARM_STM32F103_SOC_H
#define HW_ARM_STM32F103_SOC_H

#include "hw/misc/stm32f2xx_syscfg.h"
#include "hw/timer/stm32f2xx_timer.h"
#include "hw/char/stm32f2xx_usart.h"
#include "hw/adc/stm32f2xx_adc.h"
#include "hw/ssi/stm32f2xx_spi.h"
#include "hw/or-irq.h"
#include "hw/arm/armv7m.h"

// STM32F103

#define STM32F103_FLASH_BASE_ADDRESS 0x08000000
#define STM32F103_SRAM_BASE_ADDRESS 0x20000000

// STM32F103C8

#define TYPE_STM32F103C8_SOC "stm32f103c8-soc"
#define STM32F103C8_SOC(obj) \
    OBJECT_CHECK(STM32F103C8State, (obj), TYPE_STM32F103C8_SOC)

#define STM32F103C8_NUM_USARTS 3
#define STM32F103C8_NUM_TIMERS 4
#define STM32F103C8_NUM_ADC 2
#define STM32F103C8_NUM_SPI 2

#define STM32F103C8_FLASH_SIZE (1024 * 64)
#define STM32F103C8_SRAM_SIZE (1024 * 20)

typedef struct STM32F103C8State {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    char *cpu_type;

    ARMv7MState armv7m;

    STM32F2XXSyscfgState syscfg;
    STM32F2XXUsartState usart[STM32F103C8_NUM_USARTS];
    qemu_or_irq *adc_irqs;
    STM32F2XXADCState adc[STM32F103C8_NUM_ADC];
    STM32F2XXSPIState spi[STM32F103C8_NUM_SPI];
    STM32F2XXTimerState timer[STM32F103C8_NUM_TIMERS];

    MemoryRegion sram;
    MemoryRegion flash;
    MemoryRegion flash_alias;
} STM32F103C8State;

#endif

