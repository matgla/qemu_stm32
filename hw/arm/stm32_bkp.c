#include "hw/arm/stm32.h"
#include "hw/arm/stm32_bkp.h"
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

    stm32_periph_t periph;
    void *stm32_rcc_prop;

    uint16_t register_size;
    uint16_t data_buffer[255];

    uint32_t bkp_rtccr;
    uint32_t bkp_cr;
    uint32_t bkp_csr;

    Stm32Rcc *stm32_rcc;

} Stm32Bkp;

static uint64_t stm32_bkp_read(void* arg, hwaddr address, uint32_t size)
{
    DPRINTF("Reading data from address: %lx\n", address);

    Stm32Bkp* bkp_register = arg;

    return bkp_register->data_buffer[address / 4];
}

enum BKP_REGISTERS
{
    BKP_RTCCR = 0x2c,
    BKP_CR = 0x30,
    BKP_CSR = 0x34
};

#define BKP_RTCCR_ASOS_BIT 9
#define BKP_RTCCR_ASOE_BIT 8
#define BKP_RTCCR_CCO_BIT 7

#define BKP_CR_TPAL_BIT 1
#define BKP_CR_TPE_BIT 0

#define BKP_CSR_TIF_BIT 9
#define BKP_CSR_TEF_BIT 8
#define BKP_CSR_TPIE_BIT 2
#define BKP_CSR_CTI_BIT 1
#define BKP_CSR_CTE_BIT 0



static void stm32_bkp_write(void* arg, hwaddr address, uint64_t data, uint32_t size)
{
    DPRINTF("Writting data on address: 0x%08lx -> 0x%04lx\n", address, data);

    Stm32Bkp* bkp_register = arg;

    stm32_rcc_check_periph_clk((Stm32Rcc *)bkp_register->stm32_rcc, STM32_BKP);
    stm32_rcc_check_periph_clk((Stm32Rcc *)bkp_register->stm32_rcc, STM32_PWR);

    if (bkp_register->register_size == 84)
    {
        if (!((address >= 0x04 && address <= 0x34) || (address >= 0x40 && address <= 0xBC)))
        {
            hw_error("Attempted to write non existing backup register. Target available addresses: from 0x04 to 0x28 and from 0x40 to 0xBC\n");
            return;
        }
    }

    if (bkp_register->register_size == 20)
    {
        if (!((address >= 0x04 && address <= 0x34)))
        {
            hw_error("Attempted to write non existing backup register. Target available addresses: from 0x04 to 0x28\n");
            return;
        }
    }

    switch (address)
    {
        case BKP_CR:
        {
            qemu_log_mask(LOG_UNIMP, "STM32 TAMPER not implemented\n");
            return;
        }
        case BKP_RTCCR:
        {
            qemu_log_mask(LOG_UNIMP, "STM32 RTC calibration not implemented\n");
            return;
        }
        case BKP_CSR:
        {
            qemu_log_mask(LOG_UNIMP, "STM32 RTC calibration not implemented\n");
            return;
        }
    }
    // TODO: check if backup register write enabled

    // TODO: check if write inside address

    // RM0008 Rev 20 -> half word reserved and half word for user
    bkp_register->data_buffer[address / 4] = data & 0x0000ffff;
}

static const MemoryRegionOps stm32_bkp_ops = {
    .read = stm32_bkp_read,
    .write = stm32_bkp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static int
stm32_bkp_init(SysBusDevice* device)
{
    DPRINTF("Initialized bkp\n");
    Stm32Bkp* backup_device = STM32_BKP(device);
    backup_device->stm32_rcc = (Stm32Rcc*)backup_device->stm32_rcc_prop;
    memory_region_init_io(&backup_device->iomem, OBJECT(backup_device), &stm32_bkp_ops, backup_device, "bkp", 0x3FF);
    sysbus_init_mmio(device, &backup_device->iomem);
    return 0;
}

void stm32_bkp_reset(DeviceState* device_state)
{
    Stm32Bkp* backup_device = STM32_BKP(device_state);
    DPRINTF("Resetting BKP: %d\n", backup_device->register_size);
    for (int i = 0; i < 255; ++i)
    {
        backup_device->data_buffer[i] = 0;
    }
    backup_device->bkp_cr = 0;
    backup_device->bkp_csr = 0;
    backup_device->bkp_rtccr = 0;
}

static Property stm32_bkp_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32Bkp, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_PTR("stm32_rcc", Stm32Bkp, stm32_rcc_prop),
    DEFINE_PROP_UINT16("stm32_bkp_register_size", Stm32Bkp, register_size, 0x0),
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

