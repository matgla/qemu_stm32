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

#include <stdint.h>

#include "hw/irq.h"

struct Stm32Clock
{
public:
    const char *name;

    /* parameters */
    uint64_t input_freqency;
    uint64_t output_freqency;
    uint64_t max_output_frequency;
    uint64_t multiplier;
    uint64_t divisor;

    /* state */
    int enabled;

    uint32_t number_of_observers;
    qemu_irq* observers;

    uint32_t number_of_inputs;
    Stm32Clock* inputs;

    uint32_t number_of_outputs;
    Stm32Clock* outputs;
};

void stm32_process_clock_parameters_change(Stm32Clock* self)
{
    uint64_t new_output_frequency = 0;

    if (self->enabled)
    {
        new_output_frequency = self->input_freqency * self->multiplier / self->divisor;
    }

    if (self->output_freqency != new_output_frequency)
    {
        if (new_output_frequency > self->max_output_frequency)
        {
            // SOME ERROR handling procedure
        }
        self->output_freqency = new_output_frequency;
        stm32_notify_frequency_change(self);
        stm32_propagate_to_child_clocks(self);
    }


}

void stm32_change_clock_input_frequency(Stm32Clock* self, uint32_t new_frequency)
{
    self->input_freqency = new_frequency;

    stm32_process_clock_parameters_change(self);
}

void stm32_notify_frequency_change(Stm32Clock* self)
{
    int i;
    for (i = 0; i < self->number_of_observers; ++i)
    {
        qemu_set_irq(self->observers[i], 1);
    }
}

Stm32Clock* stm32_get_selected_input(Stm32Clock* self)
{

}

void stm32_propagate_to_child_clocks(Stm32Clock* self)
{
    int i;
    Stm32Clock *next, *input;

    for (i = 0; i < self->number_of_outputs; ++i)
    {
        next = &self->outputs[i];
        input = stm32_get_selected_input(next);

        if (input == self)
        {
            stm32_change_clock_input_frequency(next, self->output_freqency);
        }
    }
}



