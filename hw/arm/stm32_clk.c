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

    warn_report("Can't find input clock: %s, under: %s\n", name, self->name);
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
}

void stm32_clock_set_outputs(Stm32Clock* self, Stm32Clock** outputs, int number_of_outputs)
{
    assert (self != NULL);

    self->number_of_outputs = number_of_outputs;
    self->outputs = outputs;
}


// #include <stdlib.h>
// #include <stdint.h>
// #include <string.h>

// #include "stm32_clk.h"

// static struct Stm32DeviceTreeNode *stm32_clock_new_node(struct Stm32Clock *clock)
// {
//     struct Stm32DeviceTreeNode *new_node = (struct Stm32DeviceTreeNode *)malloc(sizeof(struct Stm32DeviceTreeNode));
//     new_node->next_node = NULL;
//     new_node->next_input = NULL;
//     new_node->next_output = NULL;
//     new_node->clock = clock;
//     return new_node;
// }

// static struct Stm32ClockTree *stm32_clock_create_tree()
// {
//     struct Stm32ClockTree *device_tree = (struct Stm32ClockTree *) malloc(sizeof(struct Stm32ClockTree));
//     device_tree->data = NULL;
// }

// Stm32Clock* stm32_clock_create_source(const char* name, uint64_t frequency)
// {
//     Stm32Clock* clock = stm32_clock_create_default(name);
//     clock->input_freqency = frequency;
//     stm32_clock_recalculate(clock);
//     return clock;
// }

// static void stm32_clock_add_input_clock(
//     Stm32Clock* parent, Stm32Clock clock)
// {
//     struct Stm32Clock* new_clock = (Stm32Clock*)malloc(sizeof(Stm32Clock));
//     struct Stm32DeviceTreeNode *new_node;

//     memcpy(new_clock, &clock, sizeof(Stm32Clock));
//     new_node = stm32_clock_new_node(new_clock);
//     struct Stm32DeviceTreeNode *source_node = parent->next_input;
//     while (source_node != NULL)
//     {
//         source_node = source_node->next_input;
//     } while (source_node != NULL)
//     parent
// }

// static void stm32_clock_find_clock(Stm32Clock* root, const char* name)
// {
//     int i, j;

//     for (j = 0; i <
// }

// void stm32_clock_register_input(Stm32Clock* self, Stm32Clock* input)
// {
//     if (self->number_of_inputs >= STM32_CLOCK_MAX_INPUTS)
//     {
//         ++(self->number_of_inputs);
//         self->inputs[self->number_of_inputs] = input;
//         return;
//     }
//     fprintf(stderr, "There is no space for new input, please increase STM32_CLOCK_MAX_INPUTS define.\n");
// }

// void stm32_clock_register_output(Stm32Clock* self, Stm32Clock* output)
// {
//     if (self->number_of_outputs >= STM32_CLOCK_MAX_OUTPUTS)
//     {
//         ++(self->number_of_outputs);
//         self->inputs[self->number_of_outputs] = output;
//         return;
//     }
//     fprintf(stderr, "There is no space for new output, please increase STM32_CLOCK_MAX_OUTPUTS define.\n");
// }

// void stm32_clock_register_observer(Stm32Clock* self, qemu_irq observer)
// {
//     if (self->number_of_outputs >= STM32_CLOCK_MAX_OBSERVERS)
//     {
//         ++(self->number_of_observers);
//         self->inputs[self->number_of_observers] = observer;
//         return;
//     }
//     fprintf(stderr, "There is no space for new output, please increase STM32_CLOCK_MAX_OBSERVERS define.\n");

// }

// void stm32_clock_print_state(Stm32Clock* self)
// {
//     Stm32Clock* input = stm32_clock_get_selected_input(self);
//     fprintf(stderr, "Clock %s: {enabled: %s, input: %s, input frequency: %llu, output_freqency: %llu, prescaler: %llu/%llu}\n"
//             , self->name
//             , self->enabled ? "true" : "false"
//             , input ? input->name : "Unknown"
//             , self->input_freqency
//             , self->output_freqency
//             , self->prescaler.multiplier
//             , self->prescaler.divisor
//             );
// }

// void stm32_clock_recalculate(Stm32Clock* self)
// {
//     uint64_t new_output_frequency = 0;

//     if (self->enabled)
//     {
//         new_output_frequency = self->input_freqency * self->multiplier / self->divisor;
//     }

//     if (self->output_freqency != new_output_frequency)
//     {
//         stm32_clock_print_state(self);
//         if (new_output_frequency > self->max_output_frequency)
//         {
//             // SOME ERROR handling procedure
//         }
//         self->output_freqency = new_output_frequency;
//         stm32_clock_notify_frequency_change(self);
//         stm32_clock_propagate(self);
//     }

// }

// void stm32_clock_set_input_frequency(Stm32Clock* self, uint32_t new_frequency)
// {
//     self->input_freqency = new_frequency;
// }

// void stm32_clock_notify_frequency_change(Stm32Clock* self)
// {
//     int i;
//     for (i = 0; i < self->number_of_observers; ++i)
//     {
//         qemu_set_irq(self->observers[i], 1);
//     }
// }

// Stm32Clock* stm32_clock_get_selected_input(Stm32Clock* self)
// {
//     if (self->selected_input <= 0)
//     {
//         return NULL;
//     }
//     return self->inputs[self->selected_input];
// }

// void stm32_propagate_to_child_clocks(Stm32Clock* self)
// {
//     int i;
//     Stm32Clock *next, *input;

//     for (i = 0; i < self->number_of_outputs; ++i)
//     {
//         next = &self->outputs[i];
//         input = stm32_get_selected_input(next);

//         if (input == self)
//         {
//             stm32_change_clock_input_frequency(next, self->output_freqency);
//         }
//     }
// }

// void stm32_clock_set_input(Stm32Clock* self, int input)
// {
//     if (input < -1 || input > self->number_of_inputs)
//     {
//         fprintf(stderr, "Trying to select wrong input (%d), %s has %d inputs\n"
//                 , input
//                 , self->name
//                 , self->number_of_inputs);
//         return;
//     }

//     self->selected_input = input;

//     if (self->selected_input != -1)
//     {
//         Stm32Clock* input_clock = stm32_clock_get_selected_input(self);
//         stm32_clock_set_input_frequency(self, input_clock->output_freqency);
//         return;
//     }
//     stm32_clock_set_input_frequency(self, 0);
// }

// Stm32Clock* stm32_clock_create_default(const char* name)
// {
//     Stm32Clock* clock = (Stm32Clock*)g_malloc(sizeof(struct Stm32Clock));
//     clock->name = name;
//     clock->input_freqency = 0;
//     clock->output_freqency = 0;
//     clock->max_output_frequency = 0;
//     clock->number_of_inputs = 0;
//     clock->inputs = NULL;
//     clock->selected_input = -1;
//     clock->prescaler.divisor = 1;
//     clock->prescaler.multiplier = 1;
//     clock->number_of_outputs = 0;
//     clock->ouputs = NULL;
//     clock->enabled = 0;
//     clock->number_of_observers = 0;
//     clock->observers = NULL;

//     return clock;
// }
