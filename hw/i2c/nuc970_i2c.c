/*
 */

#include "qemu/osdep.h"
#include "hw/i2c/nuc970_i2c.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "hw/i2c/i2c.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "sysemu/blockdev.h"
#include "hw/qdev-properties.h"

#ifndef DEBUG_NUC970_I2C
#define DEBUG_NUC970_I2C 1
#endif

#define DPRINTF(fmt, args...) \
    do { \
        if (DEBUG_NUC970_I2C) { \
            fprintf(stderr, "[%s]%s: " fmt , TYPE_NUC970_I2C, \
                                             __func__, ##args); \
        } \
    } while (0)

static const char* imx_i2c_get_regname(unsigned offset)
{
    switch (offset) {
    case I2C_CSR:
        return "I2C_CSR";
    case I2C_DIVIDER:
        return "I2C_DIVIDER";
    case I2C_CMDR:
        return "I2C_CMDR";
    case I2C_SWR:
        return "I2C_SWR";
    case I2C_RxR:
        return "I2C_RxR";
    case I2C_TxR:
        return "I2C_TxR";
    default:
        return "[?]";
    }
}

static inline bool imx_i2c_is_enabled(NUC970I2CState* s)
{
    return s->csr & I2CR_IEN;
}

static inline bool imx_i2c_interrupt_is_enabled(NUC970I2CState* s)
{
    return s->csr & I2CR_IIEN;
}

static inline bool imx_i2c_is_master(NUC970I2CState* s)
{
    //return s->csr & I2CR_MSTA;
    return true;
}

static void imx_i2c_reset(DeviceState* dev)
{
    NUC970I2CState* s = NUC970_I2C(dev);

    //if (s->address != ADDR_RESET) {
        i2c_end_transfer(s->bus);
    //}
/*
    s->address = ADDR_RESET;
    s->iadr = IADR_RESET;
    s->ifdr = IFDR_RESET;
    s->i2cr = I2CR_RESET;
    s->i2sr = I2SR_RESET;
    s->i2dr_read = I2DR_RESET;
    s->i2dr_write = I2DR_RESET;
    */
        s->swr = 0x3f;
}

static inline void imx_i2c_raise_interrupt(NUC970I2CState* s)
{
    /*
     * raise an interrupt if the device is enabled and it is configured
     * to generate some interrupts.
     */
    if (imx_i2c_is_enabled(s) && imx_i2c_interrupt_is_enabled(s)) {
        s->csr |= I2SR_IIF;
        qemu_irq_raise(s->irq);
    }
}

static uint64_t imx_i2c_read(void* opaque, hwaddr offset,
    unsigned size)
{
    uint16_t value;
    NUC970I2CState* s = NUC970_I2C(opaque);

    switch (offset) {
    case I2C_CSR:
        value = s->csr;
        break;
    case I2C_DIVIDER:
        value = s->divider;
        break;
    case I2C_CMDR:
        value = s->cmdr;
        break;
    case I2C_SWR:
        value = s->swr;
        break;
    case I2C_RxR:
        value = s->rxr;
        break;
    case I2C_TxR:
        value = s->txr;

        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "[%s]%s: Bad address at offset 0x%"
            HWADDR_PRIx "\n", TYPE_NUC970_I2C, __func__, offset);
        value = 0;
        break;
    }

    DPRINTF("read  %s [0x%" HWADDR_PRIx "] -> 0x%02x\n",
        imx_i2c_get_regname(offset), offset, value);

    return (uint64_t)value;
}

static void imx_i2c_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    NUC970I2CState* s = NUC970_I2C(opaque);

    DPRINTF("write %s [0x%" HWADDR_PRIx "] <- 0x%02x\n",
        imx_i2c_get_regname(offset), offset, (int)value);

    value &= 0xff;

    switch (offset) {
    
    case I2C_CSR:
        
        s->csr = value & 0x33;  // only save I2C_EN, IE, Tx_NUM
        if ((value & I2SR_IIF)) {
            s->csr &= ~I2SR_IIF;
            qemu_irq_lower(s->irq);
        }        

        break;
    case I2C_DIVIDER:
        s->divider = value;
        break;
    case I2C_CMDR:
        if (!imx_i2c_is_enabled(s)) {
            break;
        }
        if ((value & I2C_CMD_START) && 
            (value & I2C_CMD_WRITE)) {
            if (i2c_start_transfer(s->bus, extract32(s->txr, 1, 7),
                extract32(s->txr, 0, 1))) {
                /* if non zero is returned, the address is not valid */
                s->csr |= (1 << 11);
            }
            else {
                
                s->csr &= ~(1 << 11);
                imx_i2c_raise_interrupt(s);
            }
        }
        else if (value & I2C_CMD_WRITE) { /* This is a normal data write */
            if (i2c_send(s->bus, s->txr)) {
                /* if the target return non zero then end the transfer */
                s->csr |= (1<<11);
                i2c_end_transfer(s->bus);
            }
            else {
                s->csr &= ~(1<<11);
                imx_i2c_raise_interrupt(s);
            }
        }
        else if (value & I2C_CMD_READ) {
            s->rxr = i2c_recv(s->bus);

            imx_i2c_raise_interrupt(s);
        }
        break;
    case I2C_SWR:
        //
        break;
    case I2C_TxR:
        /* if the device is not enabled, nothing to do */
        if (!imx_i2c_is_enabled(s)) {
            break;
        }
        s->txr = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "[%s]%s: Bad address at offset 0x%"
            HWADDR_PRIx "\n", TYPE_NUC970_I2C, __func__, offset);
        break;
    }
}

static const MemoryRegionOps imx_i2c_ops = {
    .read = imx_i2c_read,
    .write = imx_i2c_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription imx_i2c_vmstate = {
    .name = TYPE_NUC970_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16(csr, NUC970I2CState),
        VMSTATE_UINT16(divider, NUC970I2CState),
        VMSTATE_UINT8(cmdr, NUC970I2CState),
        VMSTATE_UINT8(swr, NUC970I2CState),
        VMSTATE_UINT8(rxr, NUC970I2CState),
        VMSTATE_UINT32(txr, NUC970I2CState),
        VMSTATE_END_OF_LIST()
    }
};

static void imx_i2c_realize(DeviceState* dev, Error** errp)
{
    NUC970I2CState* s = NUC970_I2C(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &imx_i2c_ops, s, TYPE_NUC970_I2C,
        0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->bus = i2c_init_bus(dev, NULL);

    I2CSlave* i2c_dev = i2c_slave_new("at24c-eeprom", 0x50);
    

    DriveInfo* dinfo = drive_get_by_index(IF_NONE, 0);
    BlockBackend* blk = dinfo ? blk_by_legacy_dinfo(dinfo) : NULL;
    if (blk) {
        qdev_prop_set_drive(DEVICE(i2c_dev), "drive", blk);
    }
    qdev_prop_set_uint32(DEVICE(i2c_dev), "rom-size", 2 * KiB);

    i2c_slave_realize_and_unref(i2c_dev, s->bus, &error_abort);

}

static void imx_i2c_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->vmsd = &imx_i2c_vmstate;
    dc->reset = imx_i2c_reset;
    dc->realize = imx_i2c_realize;
    dc->desc = "NUC970 I2C Controller";
}

static const TypeInfo imx_i2c_type_info = {
    .name = TYPE_NUC970_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC970I2CState),
    .class_init = imx_i2c_class_init,
};

static void imx_i2c_register_types(void)
{
    type_register_static(&imx_i2c_type_info);
}

type_init(imx_i2c_register_types)