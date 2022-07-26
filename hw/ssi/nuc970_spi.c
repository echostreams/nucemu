#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/fifo8.h"

#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"

#define NUC970_SPI_ERR_DEBUG 1

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
    uint32_t rx[4];
    uint32_t tx[4];
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
        VMSTATE_UINT32_ARRAY(rx, NUC970SPI, 4),
        VMSTATE_UINT32_ARRAY(tx, NUC970SPI, 4),
        VMSTATE_END_OF_LIST()
    }
};


static Property nuc970_spi_properties[] = {
    DEFINE_PROP_UINT8("num-ss-bits", NUC970SPI, num_cs, 1),
    DEFINE_PROP_END_OF_LIST(),
};

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
        s->rx[i] = 0;
        s->tx[i] = 0;
    }

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

    DB_PRINT("addr=" TARGET_FMT_plx " = %x\n", addr, r);
    return r;

}

static void
spi_write(void* opaque, hwaddr addr,
    uint64_t val64, unsigned int size)
{
    NUC970SPI* s = opaque;
    uint32_t value = val64;

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

static void nuc970_spi_realize(DeviceState* dev, Error** errp)
{
    SysBusDevice* sbd = SYS_BUS_DEVICE(dev);
    NUC970SPI* s = NUC970_SPI(dev);
    int i;

    DB_PRINT("\n");

    s->spi = ssi_create_bus(dev, "spi");

    sysbus_init_irq(sbd, &s->irq);
    //s->cs_lines = g_new0(qemu_irq, s->num_cs);
    //for (i = 0; i < s->num_cs; ++i) {
    //    sysbus_init_irq(sbd, &s->cs_lines[i]);
    //}

    memory_region_init_io(&s->mmio, OBJECT(s), &spi_ops, s,
        "nuc970-spi", 11 * 4);
    sysbus_init_mmio(sbd, &s->mmio);

    s->irqline = -1;

    //fifo8_create(&s->tx_fifo, FIFO_CAPACITY);
    //fifo8_create(&s->rx_fifo, FIFO_CAPACITY);
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


