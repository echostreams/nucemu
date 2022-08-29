#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/fifo8.h"

#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"
#include "qapi/error.h"
#include "sysemu/blockdev.h"
#include "hw/block/flash.h"
#include "qemu/error-report.h"
#include "qemu/log.h"

#include "trace.h"


#define NUC970_SPI_ERR_DEBUG 0

#if NUC970_SPI_ERR_DEBUG
#define DB_PRINT(...) do { \
    fprintf(stderr,  ": %s: ", __func__); \
    fprintf(stderr, ## __VA_ARGS__); \
    } while (0)
#else
#define DB_PRINT(...)
#endif

#define TYPE_NUC970_SPI "nuc970-spi"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970SPI, NUC970_SPI)

struct NUC970SPI {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    qemu_irq irq;
    int irqline;

    uint8_t num_cs;
    qemu_irq* cs_lines;

    SSIBus* spi;

    //Fifo8 rx_fifo;
    //Fifo8 tx_fifo;
    //uint32_t regs[R_MAX];

    uint32_t cntrl;
    uint32_t divider;
    uint32_t ssr;
    /*
    The RxX and TxX registers share the same flip-flops, which means that what is
    received from the input data line in one transfer will be transmitted on the output data line in
    the next transfer if no write access to the TxX register is executed between the transfers.
    */
    //uint32_t rxtx[4];
    uint32_t rx[4];
    uint32_t tx[4];
    int transfer_busy;
    int active_cs;
};

static const VMStateDescription vmstate_nuc970_spi = {
    .name = "nuc970_spi",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        //VMSTATE_FIFO8(tx_fifo, NUC970SPI),
        //VMSTATE_FIFO8(rx_fifo, NUC970SPI),
        //VMSTATE_UINT32_ARRAY(regs, NUC970SPI, R_MAX),
        VMSTATE_UINT32(cntrl, NUC970SPI),
        VMSTATE_UINT32(divider, NUC970SPI),
        VMSTATE_UINT32(ssr, NUC970SPI),
        //VMSTATE_UINT32_ARRAY(rxtx, NUC970SPI, 4),
        VMSTATE_UINT32_ARRAY(rx, NUC970SPI, 4),
        VMSTATE_UINT32_ARRAY(tx, NUC970SPI, 4),
        VMSTATE_END_OF_LIST()
    }
};


static Property nuc970_spi_properties[] = {
    DEFINE_PROP_UINT8("num-ss-bits", NUC970SPI, num_cs, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void
update_interrupt(NUC970SPI* spi)
{
    
    if ((spi->cntrl & (1 << 17)) && (spi->cntrl & (1 << 16))) {
        //fprintf(stderr, " spi update irq 1, %x\n", spi->tx[0]);
        qemu_set_irq(spi->irq, 1);
    }
    else {
        //fprintf(stderr, " spi update irq 0, %x\n", spi->tx[0]);
        qemu_set_irq(spi->irq, 0);
    }
}

/* Assert the chip select specified in the UMA Control/Status Register. */
static void nuc970_spi_select(NUC970SPI* s, unsigned cs_id)
{
    trace_npcm7xx_fiu_select(DEVICE(s)->canonical_path, cs_id);

    if (cs_id < s->num_cs) {
        //qemu_irq_lower(s->cs_lines[cs_id]);
        qemu_irq_raise(s->cs_lines[cs_id]);
        s->active_cs = cs_id;
    }
    else {
        qemu_log_mask(LOG_GUEST_ERROR,
            "%s: UMA to CS%d; this module has only %d chip selects",
            DEVICE(s)->canonical_path, cs_id, s->num_cs);
        s->active_cs = -1;
    }
}

/* Deassert the currently active chip select. */
static void nuc970_spi_deselect(NUC970SPI* s)
{
    if (s->active_cs < 0) {
        return;
    }

    trace_npcm7xx_fiu_deselect(DEVICE(s)->canonical_path, s->active_cs);

    //qemu_irq_raise(s->cs_lines[s->active_cs]);
    qemu_irq_lower(s->cs_lines[s->active_cs]);
    s->active_cs = -1;
}

static void nuc970_spi_do_reset(NUC970SPI* s)
{
    //memset(s->regs, 0, sizeof s->regs);

    //rxfifo_reset(s);
    //txfifo_reset(s);

    //s->regs[R_SPISSR] = ~0;
    //xlx_spi_update_irq(s);
    //xlx_spi_update_cs(s);
    s->cntrl = 0x00000004;
    s->divider = 0;
    s->ssr = 0;
    for (int i = 0; i < 4; i++) {
        //s->rxtx[i] = 0;
        s->rx[i] = s->tx[i] = 0;
    }
    s->transfer_busy = 0;
}

static void nuc970_spi_reset(DeviceState* d)
{
    nuc970_spi_do_reset(NUC970_SPI(d));
}

static uint64_t
spi_read(void* opaque, hwaddr addr, unsigned int size)
{
    NUC970SPI* s = opaque;
    uint32_t r = 0;
    switch (addr) {
    case 0x00:
        r = (s->cntrl & (~1)) | (s->transfer_busy);
        break;
    case 0x04:
        r = s->divider;
        break;
    case 0x08:
        r = s->ssr;
        break;
    case 0x10:
        r = s->rx[0];
        break;
    case 0x14:
        r = s->rx[1];
        break;
    case 0x18:
        r = s->rx[2];
        break;
    case 0x1c:
        r = s->rx[3];
        break;
    }

    DB_PRINT("addr=" TARGET_FMT_plx " = %x\n", addr, r);
    //if (s->cntrl >> 17 & 0x01) {
        //fprintf(stderr, "addr=" TARGET_FMT_plx " = %x\n", addr, r);
    //}
    //if (addr >= 0x10 && addr <= 0x1c) {
    //    fprintf(stderr, "addr=" TARGET_FMT_plx " = %x\n", addr, r);
    //}
    return r;

}

static void
run_transfer(void* clientData)
{
    NUC970SPI* cspi = (NUC970SPI*)clientData;

    int Tx_NUM = (cspi->cntrl >> 8) & 0x03;		// [9:8] Tx_NUM
    int Tx_BIT_LEN = (cspi->cntrl >> 3) & 0x1f; // [7:3] Tx_BIT_LEN
    int i;
    uint32_t rx[4] = { 0, 0, 0, 0 };
    for (i = 0; i < (Tx_NUM + 1); i++)
    {
        if (!Tx_BIT_LEN) // 32 bit
        {
            uint8_t ret = ssi_transfer(cspi->spi, cspi->tx[i] >> 0);
            //fprintf(stderr, " %02x => %08x\n", cspi->SPI_TxRx[i] >> 0 & 0xff, ret);
            rx[i] |= (ret & 0xff) << 24;

            ret = ssi_transfer(cspi->spi, cspi->tx[i] >> 8);
            //fprintf(stderr, " %02x => %08x\n", cspi->SPI_TxRx[i] >> 8 & 0xff, ret);
            rx[i] |= (ret & 0xff) << 16;

            ret = ssi_transfer(cspi->spi, cspi->tx[i] >> 16);
            //fprintf(stderr, " %02x => %08x\n", cspi->SPI_TxRx[i] >> 16 & 0xff, ret);
            rx[i] |= (ret & 0xff) << 8;

            ret = ssi_transfer(cspi->spi, cspi->tx[i] >> 24);
            //fprintf(stderr, " %02x => %08x\n", cspi->SPI_TxRx[i] >> 24 & 0xff, ret);
            rx[i] |= (ret & 0xff) << 0;

            cspi->rx[i] = rx[i];
        }
        else
        {
            rx[i] = ssi_transfer(cspi->spi, cspi->tx[i]);
            cspi->rx[i] = rx[i];
        }
    }

    cspi->transfer_busy = 0;
    if (cspi->cntrl >> 17 & 0x01) { // Interrupt Enabled
        cspi->cntrl |= 1 << 16;
        update_interrupt(cspi);		// transfer done
    }


}

static void
spi_write(void* opaque, hwaddr addr,
    uint64_t val64, unsigned int size)
{
    NUC970SPI* s = opaque;
    uint32_t value = val64;
    switch (addr)
    {
    case 0x00:
        s->cntrl = value;
        if ((value >> 17) & 0x01) {
            //fprintf(stderr, "\033[;32m[%ld] Enable SPI Interrupt %08x \033[0m\n", CycleCounter_Get(), value);

            if ((value >> 16) & 0x01) { // This bit is read only, but can be cleared by writing 1 to this bit.
                s->cntrl &= ~(1 << 16); // clear interrupt flag
                //fprintf(stderr, "...clear SPI IF...\n");
                update_interrupt(s);
            }
        }

        if (value & 1) {	// GO BUSY
            //fprintf(stderr, "Start XCH ...\n");
            int Tx_NUM = (value >> 8) & 0x03;		// [9:8] Tx_NUM
            int Tx_BIT_LEN = (value >> 3) & 0x1f; // [7:3] Tx_BIT_LEN
            int t;

            s->transfer_busy = 1;
            //s->cntrl &= ~(1 << 16); // clear interrupt flag
            
            run_transfer(s);
            
        }        
        break;
    case 0x04:
        s->divider = value;
        break;
    case 0x08:
        s->ssr = value;
        if (value & 0x01)
            nuc970_spi_select(s, 0);
        else
            nuc970_spi_deselect(s);
        if (value & 0x02)
            nuc970_spi_select(s, 1);
        else
            nuc970_spi_deselect(s);
        break;
    case 0x10:
        s->tx[0] = value;
        break;
    case 0x14:
        s->tx[1] = value;
        break;
    case 0x18:
        s->tx[2] = value;
        break;
    case 0x1c:
        s->tx[3] = value;
        break;

    }

    DB_PRINT("addr=" TARGET_FMT_plx " = %x\n", addr, value);
}

static const MemoryRegionOps spi_ops = {
    .read = spi_read,
    .write = spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

void nuc970_connect_flash(DeviceState* dev, int cs_no,
    const char* flash_type, DriveInfo* dinfo)
{
    DeviceState* flash;
    qemu_irq flash_cs;

    NUC970SPI* s = NUC970_SPI(dev);
    flash = qdev_new(flash_type);

    if (dinfo) {
        BlockBackend* blk = blk_by_legacy_dinfo(dinfo);
        int64_t size;

        /*
         * The block backend size should have already been 'validated' by
         * the creation of the m25p80 object.
         */
        size = blk_getlength(blk);
        if (size <= 0) {
            error_report("failed to get flash size");
            return;
        }
        else {
            info_report("blk_getlength: %lx\n", size);
        }

        qdev_prop_set_drive_err(flash, "drive", blk, &error_fatal);
    }
    qdev_realize_and_unref(flash, BUS(s->spi), &error_fatal);

    flash_cs = qdev_get_gpio_in_named(flash, SSI_GPIO_CS, 0);
    qdev_connect_gpio_out_named(DEVICE(s), "cs", cs_no, flash_cs);
}


static void nuc970_spi_realize(DeviceState* dev, Error** errp)
{
    SysBusDevice* sbd = SYS_BUS_DEVICE(dev);
    NUC970SPI* s = NUC970_SPI(dev);
    int i;

    DB_PRINT("\n");

    s->spi = ssi_create_bus(dev, "spi");

    sysbus_init_irq(sbd, &s->irq);
    s->cs_lines = g_new0(qemu_irq, s->num_cs);
    for (i = 0; i < s->num_cs; ++i) {
        sysbus_init_irq(sbd, &s->cs_lines[i]);
    }
    qdev_init_gpio_out_named(DEVICE(s), s->cs_lines, "cs", s->num_cs);

    memory_region_init_io(&s->mmio, OBJECT(s), &spi_ops, s,
        "nuc970-spi", 0x100);
    sysbus_init_mmio(sbd, &s->mmio);

    s->irqline = -1;

    //fifo8_create(&s->tx_fifo, FIFO_CAPACITY);
    //fifo8_create(&s->rx_fifo, FIFO_CAPACITY);

    nuc970_connect_flash(dev, 0, "w25q256", drive_get(IF_MTD, 0, 0));
    //nuc970_connect_flash(dev, 0, "s25sl12801", drive_get(IF_MTD, 0, 0));
    //nuc970_connect_flash(dev, 0, "sst25vf032b", drive_get(IF_MTD, 0, 0));
    //nuc970_connect_flash(dev, 0, "W25Q128BV", drive_get(IF_MTD, 0, 0));
    //nuc970_connect_flash(dev, 0, "s25fl016k", drive_get(IF_MTD, 0, 0));
    
    
}

static void nuc970_spi_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->realize = nuc970_spi_realize;
    dc->reset = nuc970_spi_reset;
    device_class_set_props(dc, nuc970_spi_properties);
    dc->vmsd = &vmstate_nuc970_spi;
}

static const TypeInfo nuc970_spi_info = {
    .name = TYPE_NUC970_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC970SPI),
    .class_init = nuc970_spi_class_init,
};

static void nuc970_spi_register_types(void)
{
    type_register_static(&nuc970_spi_info);
}

type_init(nuc970_spi_register_types)





