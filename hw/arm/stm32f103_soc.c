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

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "hw/arm/stm32f103_soc.h"
#include "hw/misc/unimp.h"

#define SYSCFG_ADD                     0x40013800 
static const uint32_t usart_addr[] = { 0x40011000, 0x40004400, 0x40004800,
                                       0x40004C00, 0x40005000, 0x40011400, 
                                       0x40007800, 0x40007C00 };

#define SYSCFG_IRQ                     71
static const int usart_irq[] = { 37, 38, 39 };

static void stm32f103c8_soc_initfn(Object *obj)
{
    int i;
    STM32F103C8State *s = STM32F103C8_SOC(obj);

    sysbus_init_child_obj(obj, "armv7m", &s->armv7m, sizeof(s->armv7m), TYPE_ARMV7M);
    sysbus_init_child_obj(obj, "syscfg", &s->syscfg, sizeof(s->syscfg), TYPE_STM32F2XX_SYSCFG);

    for (i = 0; i < STM32F103C8_NUM_USARTS; ++i) {
        sysbus_init_child_obj(obj, "usart[*]", &s->usart[i], 
                              sizeof(s->usart[i]), TYPE_STM32F2XX_USART);
    }

    //for (i = 0; i < STM32F103C8_NUM_TIMERS; ++i) {
        
    //}
}

static Property stm32f103c8_soc_properties[] = {
    DEFINE_PROP_STRING("cpu-type", STM32F103C8State, cpu_type),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32f103c8_soc_realize(DeviceState *dev_soc, Error **errp)
{
    STM32F103C8State *s = STM32F103C8_SOC(dev_soc);
    MemoryRegion *system_memory = get_system_memory();
    DeviceState *dev, *armv7m;
    SysBusDevice *busdev;
    Error *err = NULL;
    int i = 0;

    memory_region_init_ram(&s->flash, NULL, "STM32F103C8.flash", STM32F103C8_FLASH_SIZE, &err);

    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }

    memory_region_init_alias(&s->flash_alias, NULL, "STM32F103C8.flash.alias",
                             &s->flash, 0, STM32F103C8_FLASH_SIZE);

    memory_region_set_readonly(&s->flash, true);
    memory_region_set_readonly(&s->flash_alias, true);

    memory_region_add_subregion(system_memory, STM32F103_FLASH_BASE_ADDRESS, &s->flash);
    memory_region_add_subregion(system_memory, 0, &s->flash_alias);

    memory_region_init_ram(&s->sram, NULL, "STM32F103.sram", STM32F103C8_SRAM_SIZE,
                           &err);

    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(system_memory, STM32F103_SRAM_BASE_ADDRESS, &s->sram);

    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", 96);
    qdev_prop_set_string(armv7m, "cpu-type", s->cpu_type);
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    object_property_set_link(OBJECT(&s->armv7m), OBJECT(system_memory),
                             "memory", &error_abort);
    object_property_set_bool(OBJECT(&s->armv7m), true, "realized", &err);

    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }

    /* System configuration controller */
    dev = DEVICE(&s->syscfg);
    object_property_set_bool(OBJECT(&s->syscfg), true, "realized", &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, SYSCFG_ADD);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, SYSCFG_IRQ));

    /* attach UART and USART */
    for (i = 0; i < STM32F103C8_NUM_USARTS; ++i) {
        dev = DEVICE(&(s->usart[i]));
        qdev_prop_set_chr(dev, "chardev", serial_hd(i));
        object_property_set_bool(OBJECT(&s->usart[i]), true, "realized", &err);
        if (err != NULL) {
            error_propagate(errp, err);
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, usart_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, usart_irq[i]));
    }

}

static void stm32f103c8_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32f103c8_soc_realize;
    device_class_set_props(dc, stm32f103c8_soc_properties);
}

static const TypeInfo stm32f103c8_soc_info = {
    .name          = TYPE_STM32F103C8_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE, 
    .instance_size = sizeof(STM32F103C8State),
    .instance_init = stm32f103c8_soc_initfn,
    .class_init    = stm32f103c8_soc_class_init,
};

static void stm32f103c8_soc_types(void)
{
    type_register_static(&stm32f103c8_soc_info);
}

type_init(stm32f103c8_soc_types);


