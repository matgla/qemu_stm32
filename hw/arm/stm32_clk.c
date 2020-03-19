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

#include "hw/arm/stm32_clk.h"

#include "qemu/error-report.h"

#include <string.h>


Stm32Clock* stm32_clock_get_output(Stm32Clock* self, const char* name)
{
    int i;

    assert (self != NULL);

    for (i = 0; i < self->number_of_outputs; ++i)
    {
        if (strcmp(self->outputs[i]->name, name) == 0)
        {
            return self->outputs[i];
        }
    }

    warn_report("Can't find output clock: %s, under: %s\n", name, self->name);
    return NULL;
}



static Stm32Clock* stm32_clock_recurse_get(Stm32Clock* self, const char* name)
{
    int i;

    assert (self != NULL);

    for (i = 0; i < self->number_of_outputs; ++i)
    {
        if (strcmp(self->outputs[i]->name, name) == 0)
        {
            return self->outputs[i];
        }

        if (self->outputs[i]->number_of_outputs != 0)
        {
            Stm32Clock* clock = stm32_clock_recurse_get(self->outputs[i], name);
            if (clock != NULL)
            {
                return clock;
            }
        }
    }

    return NULL;
}

Stm32Clock* stm32_clock_get_descendant(Stm32Clock* self, const char* name)
{
    Stm32Clock* clock = stm32_clock_recurse_get(self, name);
    if (clock == NULL)
    {
        warn_report("Can't find clock %s", name);
    }
    return clock;
}

Stm32Clock* stm32_clock_get_input(Stm32Clock* self, const char* name)
{
    int i;

    assert (self != NULL);

    for (i = 0; i < self->number_of_inputs; ++i)
    {
        if (strcmp(self->inputs[i]->name, name) == 0)
        {
            return self->inputs[i];
        }
    }

    warn_report("Can't find input clock: %s, under: %s", name, self->name);
    return NULL;
}

void stm32_clock_select_input(Stm32Clock* self, Stm32Clock* input)
{
    assert (self != NULL);

    self->selected_input = input;
}

void stm32_clock_set_inputs(Stm32Clock* self, Stm32Clock** inputs, int number_of_inputs)
{
    assert (self != NULL);

    self->number_of_inputs = number_of_inputs;
    self->inputs = inputs;
    if (self->number_of_inputs > 0)
    {
        self->selected_input = inputs[0];
        return;
    }
    self->selected_input = NULL;
}

void stm32_clock_set_outputs(Stm32Clock* self, Stm32Clock** outputs, int number_of_outputs)
{
    assert (self != NULL);

    self->number_of_outputs = number_of_outputs;
    self->outputs = outputs;
}

bool stm32_clock_is_initialized(Stm32Clock* self)
{
    return self->initialized;
}

void stm32_clock_set_initialized(Stm32Clock* self)
{
    self->initialized = true;
}


void stm32_clock_update_clocks(Stm32Clock* self)
{
    int i = 0;
    if (self->enabled)
    {
        if (self->selected_input != NULL)
        {
            uint64_t new_freq = (self->selected_input->output_frequency * self->selected_prescaler->multiplier) / self->selected_prescaler->divisor;
            assert (new_freq <= self->max_output_frequency);
            self->output_frequency = new_freq;
        }

        for (i = 0; i < self->number_of_outputs; ++i)
        {
            stm32_clock_update_clocks(self->outputs[i]);
        }
    }
}
