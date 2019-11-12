#include "hw/arm/stm32.h"
#include "qemu/bitops.h"
#include <stdio.h>

#ifdef DEBUG_STM32_BKP
#define DPRINTF(fmt, ...)                                       \
    do { fprintf(stderr, "STM32_BKP: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

typedef struct Stm32Bkp {
    SysBusDevice busdev;
    MemoryRegion iomem;

    uint16_t idr;
} Stm32Bkp;

static uint64_t stm32_bkp_read(void* arg, hwaddr address, uint32_t size)
{
    DPRINTF("Reading data from address: %x\n", address);

    Stm32Bkp* bkp_register = arg;

    if (size != 2)
    {
        qemu_log_mask(LOG_GUEST_ERROR, "STM32 BKP supports only 2-byte read()");
    }

    return bkp_register->idr;
}

static void stm32_bkp_write(void* arg, hwaddr address, uint64_t data, uint32_t size)
{
    DPRINTF("Writting data on address: %x -> %d\n", address, data);
    Stm32Bkp* bkp_register = arg;

    // TODO: check peripheral clock and power clock are active
    // TODO: check if backup register write enabled

    if (size != 2)
    {
        qemu_log_mask(LOG_GUEST_ERROR, "STM32 BKP supports only 2-byte write()");
        return;
    }

    // TODO: check if write inside address

    bkp_register->idr = data;
}

static const MemoryRegionOps stm32_bkp_ops = {
    .read = stm32_bkp_read,
    .write = stm32_bkp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 2
    }
};

static int
stm32_bkp_init(SysBusDevice* device)
{
    DPRINTF("Initialized bkp\n");
    Stm32Bkp* backup_device = STM32_BKP(device);

    // TODO: pass memory size, it's depends on specific CPU
    memory_region_init_io(&backup_device->iomem, OBJECT(backup_device), &stm32_bkp_ops, backup_device, "bkp", 0x3FF);
    sysbus_init_mmio(device, &backup_device->iomem);
    return 0;
}

static void stm32_bkp_reset(DeviceState* device_state)
{
}

static Property stm32_bkp_properties[] = {
    DEFINE_PROP_END_OF_LIST()
};

static void
stm32_bkp_class_init(ObjectClass *object, void *data)
{
    DeviceClass *device = DEVICE_CLASS(object);
    SysBusDeviceClass *sysbus_device = SYS_BUS_DEVICE_CLASS(object);
    sysbus_device->init  = stm32_bkp_init;
    device->reset = stm32_bkp_reset;
    device->props = stm32_bkp_properties;
    DPRINTF("STM32 backup registers class initialized!\n");
}

static const TypeInfo
stm32_bkp_info = {
    .name           = "stm32-bkp",
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32Bkp),
    .class_init     = stm32_bkp_class_init
};

static void
stm32_bkp_register_types(void)
{
    type_register_static(&stm32_bkp_info);
}

type_init(stm32_bkp_register_types);

