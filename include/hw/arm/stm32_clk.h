/*
 * STM32 Clock Model
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

#ifndef HW_ARM_STM32_CLOCK_H
#define HW_ARM_STM32_CLOCK_H

#include <stdint.h>

#include "qemu/osdep.h" // qemu_irq
#include "hw/irq.h"

#define STM32_CLOCK_MAX_OBSERVERS 16

typedef struct Stm32Prescaler
{
    const char *name;
    uint64_t multiplier;
    uint64_t divisor;
} Stm32Prescaler;

typedef struct Stm32Clock
{
    const char *name;

    /* parameters */
    uint64_t input_frequency;
    uint64_t output_frequency;
    uint64_t max_output_frequency;

    /* input */
    struct Stm32Clock *selected_input;

    uint32_t number_of_inputs;
    struct Stm32Clock **inputs;

    uint32_t number_of_outputs;
    struct Stm32Clock **outputs;

    /* block */
    uint32_t number_of_prescalers;
    const Stm32Prescaler *prescalers;
    const Stm32Prescaler *selected_prescaler;

    /* state */
    bool enabled;
    bool initialized;

    uint32_t number_of_observers;
    qemu_irq observers[STM32_CLOCK_MAX_OBSERVERS];
} Stm32Clock;

Stm32Clock* stm32_clock_get_output(Stm32Clock* self, const char* name);
Stm32Clock* stm32_clock_get_input(Stm32Clock* self, const char* name);
void stm32_clock_select_input(Stm32Clock* self, Stm32Clock* input);
Stm32Clock* stm32_clock_get_descendant(Stm32Clock* self, const char* name);

void stm32_clock_set_inputs(Stm32Clock* self, Stm32Clock** inputs, int number_of_inputs);
void stm32_clock_set_outputs(Stm32Clock* self, Stm32Clock** outputs, int number_of_outputs);

bool stm32_clock_is_initialized(Stm32Clock* self);
void stm32_clock_set_initialized(Stm32Clock* self);

void stm32_clock_update_clocks(Stm32Clock* self);

#endif
