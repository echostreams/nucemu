/*
 */

#include "qemu/osdep.h"
#include "hw/i2c/nuc980_i2c.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "hw/i2c/i2c.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "sysemu/blockdev.h"
#include "hw/qdev-properties.h"

#ifndef DEBUG_NUC980_I2C
#define DEBUG_NUC980_I2C 1
#endif

#define DPRINTF(fmt, args...) \
    do { \
        if (DEBUG_NUC980_I2C) { \
            fprintf(stderr, "[%s]%s: " fmt , TYPE_NUC980_I2C, \
                                             __func__, ##args); \
        } \
    } while (0)

static const char* nuc980_i2c_get_regname(unsigned offset)
{
    switch (offset) {
    case I2C_CTL0:
        return "I2C_CTL0";
    case I2C_ADDR0:
        return "I2C_ADDR0";
    case I2C_DAT:
        return "I2C_DAT";
    case I2C_STATUS0:
        return "I2C_STATUS0";
    case I2C_CLKDIV:
        return "I2C_CLKDIV";
    case I2C_TOCTL:
        return "I2C_TOCTL";
    default:
        return "[?]";
    }
}

static inline bool nuc980_i2c_is_enabled(NUC980I2CState* s)
{
    return s->CTL0 & I2C_CTL0_I2CEN_Msk;
}

static inline bool nuc980_i2c_interrupt_is_enabled(NUC980I2CState* s)
{
    return s->CTL0 & I2C_CTL0_INTEN_Msk;
}

/*
static inline bool nuc980_i2c_is_master(NUC980I2CState* s)
{
    //return s->csr & I2CR_MSTA;
    return true;
}
*/

static void nuc980_i2c_reset(DeviceState* dev)
{
    NUC980I2CState* s = NUC980_I2C(dev);

    //if (s->address != ADDR_RESET) {
        i2c_end_transfer(s->bus);
    //}

    s->STATUS0 = 0xF8;
}

static inline void nuc980_i2c_raise_interrupt(NUC980I2CState* s)
{
    /*
     * raise an interrupt if the device is enabled and it is configured
     * to generate some interrupts.
     */
    if (nuc980_i2c_is_enabled(s) && nuc980_i2c_interrupt_is_enabled(s)) {
        s->CTL0 |= I2C_CTL_SI;
        qemu_irq_raise(s->irq);
    }
}

static uint64_t nuc980_i2c_read(void* opaque, hwaddr offset,
    unsigned size)
{
    uint16_t value;
    NUC980I2CState* s = NUC980_I2C(opaque);

    switch (offset) {
    case I2C_CTL0:
        value = s->CTL0;
        break;
    case I2C_ADDR0:
        value = s->ADDR0;
        break;
    case I2C_DAT:
        value = s->DAT;
        break;
    case I2C_STATUS0:
        value = s->STATUS0;
        break;
    case I2C_CLKDIV:
        value = s->CLKDIV;
        break;
    case I2C_TOCTL:
        value = s->TOCTL;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "[%s]%s: Bad address at offset 0x%"
            HWADDR_PRIx "\n", TYPE_NUC980_I2C, __func__, offset);
        value = 0;
        break;
    }

    DPRINTF("read  %s [0x%" HWADDR_PRIx "] -> 0x%08x\n",
        nuc980_i2c_get_regname(offset), offset, value);

    return (uint64_t)value;
}

static void nuc980_i2c_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    NUC980I2CState* s = NUC980_I2C(opaque);

    DPRINTF("write %s [0x%" HWADDR_PRIx "] <- 0x%02x\n",
        nuc980_i2c_get_regname(offset), offset, (int)value);

    value &= 0xff;

    switch (offset) {
    case I2C_CLKDIV:
        s->CLKDIV = value;
        break;

    case I2C_CTL0:
        if (!nuc980_i2c_is_enabled(s)) {
            break;
        }
        s->CTL0 = value;
        if (value & I2C_CTL_SI) {
            s->CTL0 &= ~I2C_CTL_SI;
            qemu_irq_lower(s->irq);
        }
        else if (value & I2C_CTL_SI_AA) {
            // enter SLV mode
        }
        else if (value & I2C_CTL_STA) {
            s->STATUS0 = 0x08;
            nuc980_i2c_raise_interrupt(s);
        }
/*
        else if ((value & I2C_CMD_START) && 
            (value & I2C_CMD_WRITE)) {
            if (i2c_start_transfer(s->bus, extract32(s->DAT, 1, 7),
                extract32(s->DAT, 0, 1))) {
                // if non zero is returned, the address is not valid
                //s->csr |= (1 << 11);
            }
            else {                
                //s->csr &= ~(1 << 11);
                nuc980_i2c_raise_interrupt(s);
            }
        }
        else if (value & I2C_CMD_WRITE) { // This is a normal data write
            if (i2c_send(s->bus, s->DAT)) {
                // if the target return non zero then end the transfer
                //s->csr |= (1<<11);
                i2c_end_transfer(s->bus);
            }
            else {
                //s->csr &= ~(1<<11);
                nuc980_i2c_raise_interrupt(s);
            }
        }
        else if (value & I2C_CMD_READ) {
            s->DAT = i2c_recv(s->bus);
            nuc980_i2c_raise_interrupt(s);
        }
*/
        break;

    case I2C_DAT:
        /* if the device is not enabled, nothing to do */
        if (!nuc980_i2c_is_enabled(s)) {
            break;
        }
        s->DAT = value;
        if (s->STATUS0 == 0x08 || s->STATUS0 == 0x10) {
            if (i2c_start_transfer(s->bus, extract32(s->DAT, 1, 7),
                extract32(s->DAT, 0, 1))) {
                // if non zero is returned, the address is not valid
                s->STATUS0 = 0x20;
            }
            else {
                s->STATUS0 = 0x18;
                
            }
            nuc980_i2c_raise_interrupt(s);
        }
        else { // This is a normal data write
            if (i2c_send(s->bus, s->DAT)) {
                // if the target return non zero then end the transfer
                s->STATUS0 = 0x30;
                i2c_end_transfer(s->bus);
            }
            else {
                s->STATUS0 = 0x28;                
            }
            nuc980_i2c_raise_interrupt(s);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "[%s]%s: Bad address at offset 0x%"
            HWADDR_PRIx "\n", TYPE_NUC980_I2C, __func__, offset);
        break;
    }
}

static const MemoryRegionOps nuc980_i2c_ops = {
    .read = nuc980_i2c_read,
    .write = nuc980_i2c_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription nuc980_i2c_vmstate = {
    .name = TYPE_NUC980_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        /*
        VMSTATE_UINT16(csr, NUC980I2CState),
        VMSTATE_UINT16(divider, NUC980I2CState),
        VMSTATE_UINT8(cmdr, NUC980I2CState),
        VMSTATE_UINT8(swr, NUC980I2CState),
        VMSTATE_UINT8(rxr, NUC980I2CState),
        VMSTATE_UINT32(txr, NUC980I2CState),
        */
        VMSTATE_END_OF_LIST()
    }
};

static void nuc980_i2c_realize(DeviceState* dev, Error** errp)
{
    NUC980I2CState* s = NUC980_I2C(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &nuc980_i2c_ops, s, TYPE_NUC980_I2C,
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

static void nuc980_i2c_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->vmsd = &nuc980_i2c_vmstate;
    dc->reset = nuc980_i2c_reset;
    dc->realize = nuc980_i2c_realize;
    dc->desc = "NUC980 I2C Controller";
}

static const TypeInfo nuc980_i2c_type_info = {
    .name = TYPE_NUC980_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980I2CState),
    .class_init = nuc980_i2c_class_init,
};

static void nuc980_i2c_register_types(void)
{
    type_register_static(&nuc980_i2c_type_info);
}

type_init(nuc980_i2c_register_types)