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

#include "hw/arm/stm32f1xx_rcc.h"

#define SYSCFG_ADD 0x40013800
static const uint32_t usart_addr[] = {0x40011000, 0x40004400, 0x40004800,
                                      0x40004C00, 0x40005000, 0x40011400,
                                      0x40007800, 0x40007C00};

static const uint32_t timer_addr[] = {0x40000000, 0x40000400,
                                      0x40000800, 0x40000C00};
#define ADC_ADDR 0x40012000
static const uint32_t spi_addr[] = {0x40013000, 0x40003800, 0x40003C00,
                                    0x40013400, 0x40015000, 0x40015400};
static const uint32_t adc_addr[] = {0x40012000, 0x40012100,
                                    0x40012200};

#define SYSCFG_IRQ 71
#define RCC_IRQ 5
static const int usart_irq[] = {37, 38, 39};
static const int timer_irq[] = {28, 29, 30, 50};
#define ADC_IRQ 18
static const int spi_irq[] = {35, 36, 51, 0, 0, 0};

static void stm32f103c8_soc_initfn(Object *obj)
{
    int i;
    STM32F103C8State *s = STM32F103C8_SOC(obj);

    sysbus_init_child_obj(obj, "armv7m", &s->armv7m, sizeof(s->armv7m), TYPE_ARMV7M);
    sysbus_init_child_obj(obj, "syscfg", &s->syscfg, sizeof(s->syscfg), TYPE_STM32F2XX_SYSCFG);

    // for (i = 0; i < STM32F103C8_NUM_USARTS; ++i)
    // {
    //     sysbus_init_child_obj(obj, "usart[*]", &s->usart[i],
    //                           sizeof(s->usart[i]), TYPE_STM32F2XX_USART);
    // }

    // for (i = 0; i < STM32F103C8_NUM_TIMERS; i++) {
    //     sysbus_init_child_obj(obj, "timer[*]", &s->timer[i],
    //                           sizeof(s->timer[i]), TYPE_STM32F2XX_TIMER);
    // }

    // s->adc_irqs = OR_IRQ(object_new(TYPE_OR_IRQ));

    // for (i = 0; i < STM32F103C8_NUM_ADC; i++) {
    //     sysbus_init_child_obj(obj, "adc[*]", &s->adc[i], sizeof(s->adc[i]),
    //                           TYPE_STM32F2XX_ADC);
    // }

    // for (i = 0; i < STM32F103C8_NUM_SPI; i++) {
    //     sysbus_init_child_obj(obj, "spi[*]", &s->spi[i], sizeof(s->spi[i]),
    //                           TYPE_STM32F2XX_SPI);
    // }
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

    if (err != NULL)
    {
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

    if (err != NULL)
    {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(system_memory, STM32F103_SRAM_BASE_ADDRESS, &s->sram);

    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", 91);
    qdev_prop_set_string(armv7m, "cpu-type", s->cpu_type);
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    object_property_set_link(OBJECT(&s->armv7m), OBJECT(system_memory),
                             "memory", &error_abort);
    object_property_set_bool(OBJECT(&s->armv7m), true, "realized", &err);

    if (err != NULL)
    {
        error_propagate(errp, err);
        return;
    }

    /* System configuration controller */
    dev = DEVICE(&s->syscfg);
    object_property_set_bool(OBJECT(&s->syscfg), true, "realized", &err);
    if (err != NULL)
    {
        error_propagate(errp, err);
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, SYSCFG_ADD);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, SYSCFG_IRQ));

    /* RCC */
    DeviceState *rcc_device = qdev_create(NULL, TYPE_STM32F1XX_RCC);
    qdev_init_nofail(rcc_device);
    fprintf(stderr, "RCC initialized\n");
    sysbus_mmio_map(SYS_BUS_DEVICE(rcc_device), 0, 0x40021000);
    // qdev_get_
    // sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, RCC_IRQ));

    // object_property_add_child()

    // /* attach UART and USART */
    // for (i = 0; i < STM32F103C8_NUM_USARTS; ++i)
    // {
    //     dev = DEVICE(&(s->usart[i]));
    //     qdev_prop_set_chr(dev, "chardev", serial_hd(i));
    //     object_property_set_bool(OBJECT(&s->usart[i]), true, "realized", &err);
    //     if (err != NULL)
    //     {
    //         error_propagate(errp, err);
    //         return;
    //     }
    //     busdev = SYS_BUS_DEVICE(dev);
    //     sysbus_mmio_map(busdev, 0, usart_addr[i]);
    //     sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, usart_irq[i]));
    // }

    // /* Timer 2 to 5 */
    // for (i = 0; i < STM32F103C8_NUM_TIMERS; i++)
    // {
    //     dev = DEVICE(&(s->timer[i]));
    //     qdev_prop_set_uint64(dev, "clock-frequency", 1000000000);
    //     object_property_set_bool(OBJECT(&s->timer[i]), true, "realized", &err);
    //     if (err != NULL)
    //     {
    //         error_propagate(errp, err);
    //         return;
    //     }
    //     busdev = SYS_BUS_DEVICE(dev);
    //     sysbus_mmio_map(busdev, 0, timer_addr[i]);
    //     sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, timer_irq[i]));
    // }

    // /* ADC 1 to 3 */
    // object_property_set_int(OBJECT(s->adc_irqs), STM32F103C8_NUM_ADC,
    //                         "num-lines", &err);
    // object_property_set_bool(OBJECT(s->adc_irqs), true, "realized", &err);
    // if (err != NULL)
    // {
    //     error_propagate(errp, err);
    //     return;
    // }
    // qdev_connect_gpio_out(DEVICE(s->adc_irqs), 0,
    //                       qdev_get_gpio_in(armv7m, ADC_IRQ));

    // for (i = 0; i < STM32F103C8_NUM_ADC; i++)
    // {
    //     dev = DEVICE(&(s->adc[i]));
    //     object_property_set_bool(OBJECT(&s->adc[i]), true, "realized", &err);
    //     if (err != NULL)
    //     {
    //         error_propagate(errp, err);
    //         return;
    //     }
    //     busdev = SYS_BUS_DEVICE(dev);
    //     sysbus_mmio_map(busdev, 0, adc_addr[i]);
    //     sysbus_connect_irq(busdev, 0,
    //                        qdev_get_gpio_in(DEVICE(s->adc_irqs), i));
    // }

    // /* SPI 1 and 2 */
    // for (i = 0; i < STM32F103C8_NUM_SPI; i++)
    // {
    //     dev = DEVICE(&(s->spi[i]));
    //     object_property_set_bool(OBJECT(&s->spi[i]), true, "realized", &err);
    //     if (err != NULL)
    //     {
    //         error_propagate(errp, err);
    //         return;
    //     }
    //     busdev = SYS_BUS_DEVICE(dev);
    //     sysbus_mmio_map(busdev, 0, spi_addr[i]);
    //     sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, spi_irq[i]));
    // }
    create_unimplemented_device("FSMC",                     0xA0000000, 0xFFF);
    create_unimplemented_device("USB OTG FS",               0x50000000, 0x3FFFF);
    create_unimplemented_device("Ethernet",                 0x40028000, 0x1FFF);
    create_unimplemented_device("CRC",                      0x40023000, 0x3FF);
    create_unimplemented_device("Flash memory interface",   0x40022000, 0x3FF);
    // create_unimplemented_device("RCC",                      0x40021000, 0x3FF);
    create_unimplemented_device("DMA2",                     0x40020400, 0x3FF);
    create_unimplemented_device("DMA1",                     0x40020000, 0x3FF);
    create_unimplemented_device("SDIO",                     0x40018000, 0x3FF);
    create_unimplemented_device("TIM11",                    0x40015400, 0x3FF);
    create_unimplemented_device("TIM10",                    0x40015000, 0x3FF);
    create_unimplemented_device("TIM9",                     0x40014C00, 0x3FF);
    create_unimplemented_device("ADC3",                     0x40013C00, 0x3FF);
    create_unimplemented_device("USART1",                   0x40013800, 0x3FF);
    create_unimplemented_device("TIM8",                     0x40013400, 0x3FF);
    create_unimplemented_device("SPI1",                     0x40013000, 0x3FF);
    create_unimplemented_device("TIM1",                     0x40012C00, 0x3FF);
    create_unimplemented_device("ADC2",                     0x40012800, 0x3FF);
    create_unimplemented_device("ADC1",                     0x40012400, 0x3FF);
    create_unimplemented_device("GPIOG",                    0x40012000, 0x3FF);
    create_unimplemented_device("GPIOF",                    0x40011C00, 0x3FF);
    create_unimplemented_device("GPIOE",                    0x40011800, 0x3FF);
    create_unimplemented_device("GPIOD",                    0x40011400, 0x3FF);
    create_unimplemented_device("GPIOC",                    0x40011000, 0x3FF);
    create_unimplemented_device("GPIOB",                    0x40010C00, 0x3FF);
    create_unimplemented_device("GPIOB",                    0x40010800, 0x3FF);
    create_unimplemented_device("EXTI",                     0x40010400, 0x3FF);
    create_unimplemented_device("AFIO",                     0x40010000, 0x3FF);
    create_unimplemented_device("DAC",                      0x40007400, 0x3FF);
    create_unimplemented_device("PWR",                      0x40007000, 0x3FF);
    create_unimplemented_device("BKP",                      0x40006C00, 0x3FF);
    create_unimplemented_device("CAN2",                     0x40006800, 0x3FF);
    create_unimplemented_device("CAN1",                     0x40006400, 0x3FF);
    create_unimplemented_device("USB/CAN shared SRAM",      0x40006000, 0x3FF);
    create_unimplemented_device("USB device FS",            0x40005C00, 0x3FF);
    create_unimplemented_device("I2C2",                     0x40005800, 0x3FF);
    create_unimplemented_device("I2C1",                     0x40005400, 0x3FF);
    create_unimplemented_device("UART5",                    0x40005000, 0x3FF);
    create_unimplemented_device("UART4",                    0x40004C00, 0x3FF);
    create_unimplemented_device("USART3",                   0x40004800, 0x3FF);
    create_unimplemented_device("USART2",                   0x40004400, 0x3FF);
    create_unimplemented_device("SPI2/I2S2",                0x40003C00, 0x3FF);
    create_unimplemented_device("SPI1/I2S1",                0x40003800, 0x3FF);
    create_unimplemented_device("IWDG",                     0x40003000, 0x3FF);
    create_unimplemented_device("WWDG",                     0x40002C00, 0x3FF);
    create_unimplemented_device("RTC",                      0x40002400, 0x3FF);
    create_unimplemented_device("TIM14",                    0x40002000, 0x3FF);
    create_unimplemented_device("TIM13",                    0x40001C00, 0x3FF);
    create_unimplemented_device("TIM12",                    0x40001800, 0x3FF);
    create_unimplemented_device("TIM7",                     0x40001400, 0x3FF);
    create_unimplemented_device("TIM6",                     0x40001000, 0x3FF);
    create_unimplemented_device("TIM5",                     0x40000C00, 0x3FF);
    create_unimplemented_device("TIM4",                     0x40000800, 0x3FF);
    create_unimplemented_device("TIM3",                     0x40000400, 0x3FF);
    create_unimplemented_device("TIM2",                     0x40000000, 0x3FF);
}

static void stm32f103c8_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32f103c8_soc_realize;
    device_class_set_props(dc, stm32f103c8_soc_properties);
}

static const TypeInfo stm32f103c8_soc_info = {
    .name = TYPE_STM32F103C8_SOC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F103C8State),
    .instance_init = stm32f103c8_soc_initfn,
    .class_init = stm32f103c8_soc_class_init,
};

static void stm32f103c8_soc_types(void)
{
    type_register_static(&stm32f103c8_soc_info);
}

type_init(stm32f103c8_soc_types);
