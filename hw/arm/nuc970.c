/*
 * nuc970 emulation.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "hw/arm/boot.h"
#include "net/net.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/char/serial.h"
#include "qemu/timer.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-clock.h"
#include "hw/block/flash.h"
#include "ui/console.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/or-irq.h"
#include "hw/audio/wm8750.h"
#include "sysemu/block-backend.h"
#include "sysemu/runstate.h"
#include "sysemu/dma.h"
#include "ui/pixel_ops.h"
#include "qemu/cutils.h"
#include "qom/object.h"
#include "hw/misc/unimp.h"
#include "qemu/units.h"
#include "hw/net/mv88w8618_eth.h"
#include "hw/char/nuc970_uart.h"
#include "hw/intc/nuc970_aic.h"
#include "hw/timer/nuc970_timer.h"
#include "hw/net/nuc970_emc.h"
#include "hw/i2c/nuc970_i2c.h"
#include "hw/arm/nuc970.h"
#include "hw/sd/sd.h"
#include "hw/sd/sdhci.h"
#include "hw/loader.h"

#define MP_MISC_BASE            0x80002000
#define MP_MISC_SIZE            0x00001000

#define MP_ETH_BASE             0x80008000

#define MP_WLAN_BASE            0x8000C000
#define MP_WLAN_SIZE            0x00000800

#define MP_UART1_BASE           0x8000C840
#define MP_UART2_BASE           0x8000C940

#define MP_GPIO_BASE            0xb8003000
#define MP_GPIO_SIZE            0x00001000

#define MP_FLASHCFG_BASE        0x90006000
#define MP_FLASHCFG_SIZE        0x00001000

#define MP_AUDIO_BASE           0x90007000

#define MP_PIC_BASE             0x90008000
#define MP_PIC_SIZE             0x00001000

#define MP_PIT_BASE             0x90009000
#define MP_PIT_SIZE             0x00001000

#define MP_LCD_BASE             0x9000c000
#define MP_LCD_SIZE             0x00001000

#define MP_SRAM_BASE            0x3c000000
#define MP_SRAM_SIZE            0x0000e000

#define MP_RAM_DEFAULT_SIZE     64*1024*1024
#define MP_FLASH_SIZE_MAX       32*1024*1024

#define MP_TIMER1_IRQ           4
#define MP_TIMER2_IRQ           5
#define MP_TIMER3_IRQ           6
#define MP_TIMER4_IRQ           7
#define MP_EHCI_IRQ             8
#define MP_ETH_IRQ              9
#define MP_UART_SHARED_IRQ      11
#define MP_GPIO_IRQ             57
#define MP_RTC_IRQ              15
#define MP_AUDIO_IRQ            30

 /* Wolfson 8750 I2C address */
#define MP_WM_ADDR              0x1A

/* LCD register offsets */
#define MP_LCD_IRQCTRL          0x180
#define MP_LCD_IRQSTAT          0x184
#define MP_LCD_SPICTRL          0x1ac
#define MP_LCD_INST             0x1bc
#define MP_LCD_DATA             0x1c0

/* Mode magics */
#define MP_LCD_SPI_DATA         0x00100011
#define MP_LCD_SPI_CMD          0x00104011
#define MP_LCD_SPI_INVALID      0x00000000

/* Commmands */
#define MP_LCD_INST_SETPAGE0    0xB0
/* ... */
#define MP_LCD_INST_SETPAGE7    0xB7

#define MP_LCD_TEXTCOLOR        0xe0e0ff /* RRGGBB */

#define TYPE_NUC970_LCD "nuc970_lcd"
OBJECT_DECLARE_SIMPLE_TYPE(nuc970_lcd_state, NUC970_LCD)

struct nuc970_lcd_state {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t brightness;
    uint32_t mode;
    uint32_t irqctrl;
    uint32_t page;
    uint32_t page_off;
    QemuConsole* con;
    //uint8_t video_ram[128 * 64 / 8];
    uint8_t video_ram[800 * 480 * 2];

    uint32_t crtc_size; // CRTC_SIZE
    uint32_t crtc_dend; // CRTC_DEND

    uint32_t va_baddr0; // VA_BADDR0
    uint32_t va_baddr1; // VA_BADDR1
    uint32_t osd_wins;  // starting coordinates register
    uint32_t osd_wine;  // ending coordinates register

};
/*
static uint8_t scale_lcd_color(nuc970_lcd_state* s, uint8_t col)
{
    switch (s->brightness) {
    case 7:
        return col;
    case 0:
        return 0;
    default:
        return (col * s->brightness) / 7;
    }
}
*/

static inline void set_lcd_pixel32(nuc970_lcd_state* s,
    int x, int y, uint32_t col)
{
    int dx, dy;
    DisplaySurface* surface = qemu_console_surface(s->con);
    uint32_t* pixel =
        &((uint32_t*)surface_data(surface))[(y * 128 * 3 + x) * 3];

    for (dy = 0; dy < 3; dy++, pixel += 127 * 3) {
        for (dx = 0; dx < 3; dx++, pixel++) {
            *pixel = col;
        }
    }
}

static void lcd_refresh(void* opaque)
{
    int x, y;
    //int col;

    nuc970_lcd_state* s = opaque;    
    DisplaySurface* surface = qemu_console_surface(s->con);
    uint8_t* data_display;
    data_display = surface_data(surface);

    //col = rgb_to_pixel32(scale_lcd_color(s, (MP_LCD_TEXTCOLOR >> 16) & 0xff),
    //    scale_lcd_color(s, (MP_LCD_TEXTCOLOR >> 8) & 0xff),
    //    scale_lcd_color(s, MP_LCD_TEXTCOLOR & 0xff));

    dma_memory_read(&address_space_memory, s->va_baddr0, s->video_ram, sizeof(s->video_ram), MEMTXATTRS_UNSPECIFIED);

    for (y = 0; y < 480; y++) {
        for (x = 0; x < 800; x++, data_display += 4) {
            uint16_t color = 
                s->video_ram[y * 800 * 2 + x * 2] | 
                s->video_ram[y * 800 * 2 + x * 2 + 1] << 8;
        
            uint8_t r = ((color >> 11) & 0x1F);
            uint8_t g = ((color >> 5) & 0x3F);
            uint8_t b = (color & 0x1F);

            uint32_t dest_color = rgb_to_pixel32(r, g, b);
            *(uint32_t*)data_display = dest_color;

        }
    }

    dpy_gfx_update(s->con, 0, 0, 800, 480);
}

static void lcd_invalidate(void* opaque)
{
}

static void nuc970_lcd_gpio_brightness_in(void* opaque, int irq, int level)
{
    nuc970_lcd_state* s = opaque;
    s->brightness &= ~(1 << irq);
    s->brightness |= level << irq;
}

static uint64_t nuc970_lcd_read(void* opaque, hwaddr offset,
    unsigned size)
{
    nuc970_lcd_state* s = opaque;

    switch (offset) {
    case MP_LCD_IRQCTRL:
        return s->irqctrl;

    default:
        return 0;
    }
}

static void nuc970_lcd_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    nuc970_lcd_state* s = opaque;
    fprintf(stderr, "lcd_write: %08lx %lx\n", offset, value);

    switch (offset) {
    case 0x10:
        s->crtc_size = value;
        break;
    case 0x14:
        s->crtc_dend = value;

        qemu_console_resize(s->con, (value & 0x7ff),  ((value >> 16) & 0x7ff));

        break;
    case 0x24:
        s->va_baddr0 = value;
        break;
    case 0x28:
        s->va_baddr1 = value;
        break;
    case 0x40:
        s->osd_wins = value;
        break;
    case 0x44:
        s->osd_wine = value;
        break;
        

    case MP_LCD_IRQCTRL:
        s->irqctrl = value;
        break;

    case MP_LCD_SPICTRL:
        if (value == MP_LCD_SPI_DATA || value == MP_LCD_SPI_CMD) {
            s->mode = value;
        }
        else {
            s->mode = MP_LCD_SPI_INVALID;
        }
        break;

    case MP_LCD_INST:
        if (value >= MP_LCD_INST_SETPAGE0 && value <= MP_LCD_INST_SETPAGE7) {
            s->page = value - MP_LCD_INST_SETPAGE0;
            s->page_off = 0;
        }
        break;

    case MP_LCD_DATA:
        if (s->mode == MP_LCD_SPI_CMD) {
            if (value >= MP_LCD_INST_SETPAGE0 &&
                value <= MP_LCD_INST_SETPAGE7) {
                s->page = value - MP_LCD_INST_SETPAGE0;
                s->page_off = 0;
            }
        }
        else if (s->mode == MP_LCD_SPI_DATA) {
            s->video_ram[s->page * 128 + s->page_off] = value;
            s->page_off = (s->page_off + 1) & 127;
        }
        break;
    }
}

static const MemoryRegionOps nuc970_lcd_ops = {
    .read = nuc970_lcd_read,
    .write = nuc970_lcd_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const GraphicHwOps nuc970_gfx_ops = {
    .invalidate = lcd_invalidate,
    .gfx_update = lcd_refresh,
};

static void nuc970_lcd_realize(DeviceState* dev, Error** errp)
{
    nuc970_lcd_state* s = NUC970_LCD(dev);
    s->con = graphic_console_init(dev, 0, &nuc970_gfx_ops, s);
    qemu_console_resize(s->con, 800, 480);
}

static void nuc970_lcd_init(Object* obj)
{
    SysBusDevice* sbd = SYS_BUS_DEVICE(obj);
    DeviceState* dev = DEVICE(sbd);
    nuc970_lcd_state* s = NUC970_LCD(dev);

    s->brightness = 7;

    memory_region_init_io(&s->iomem, obj, &nuc970_lcd_ops, s,
        "nuc970-lcd", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_in(dev, nuc970_lcd_gpio_brightness_in, 3);
}

static const VMStateDescription nuc970_lcd_vmsd = {
    .name = "nuc970_lcd",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(brightness, nuc970_lcd_state),
        VMSTATE_UINT32(mode, nuc970_lcd_state),
        VMSTATE_UINT32(irqctrl, nuc970_lcd_state),
        VMSTATE_UINT32(page, nuc970_lcd_state),
        VMSTATE_UINT32(page_off, nuc970_lcd_state),
        VMSTATE_BUFFER(video_ram, nuc970_lcd_state),
        VMSTATE_END_OF_LIST()
    }
};

static void nuc970_lcd_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->vmsd = &nuc970_lcd_vmsd;
    dc->realize = nuc970_lcd_realize;
}

static const TypeInfo nuc970_lcd_info = {
    .name = TYPE_NUC970_LCD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nuc970_lcd_state),
    .instance_init = nuc970_lcd_init,
    .class_init = nuc970_lcd_class_init,
};

/* PIC register offsets */
#define MP_PIC_STATUS           0x00
#define MP_PIC_ENABLE_SET       0x08
#define MP_PIC_ENABLE_CLR       0x0C

#define TYPE_MV88W8618_PIC "nuc970_aic"
OBJECT_DECLARE_SIMPLE_TYPE(mv88w8618_pic_state, MV88W8618_PIC)

struct mv88w8618_pic_state {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t level;
    uint32_t enabled;
    qemu_irq parent_irq;
};

static void mv88w8618_pic_update(mv88w8618_pic_state* s)
{
    qemu_set_irq(s->parent_irq, (s->level & s->enabled));
}

static void mv88w8618_pic_set_irq(void* opaque, int irq, int level)
{
    mv88w8618_pic_state* s = opaque;

    if (level) {
        s->level |= 1 << irq;
    }
    else {
        s->level &= ~(1 << irq);
    }
    mv88w8618_pic_update(s);
}

static uint64_t mv88w8618_pic_read(void* opaque, hwaddr offset,
    unsigned size)
{
    mv88w8618_pic_state* s = opaque;

    switch (offset) {
    case MP_PIC_STATUS:
        return s->level & s->enabled;

    default:
        return 0;
    }
}

static void mv88w8618_pic_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    mv88w8618_pic_state* s = opaque;

    switch (offset) {
    case MP_PIC_ENABLE_SET:
        s->enabled |= value;
        break;

    case MP_PIC_ENABLE_CLR:
        s->enabled &= ~value;
        s->level &= ~value;
        break;
    }
    mv88w8618_pic_update(s);
}

static void mv88w8618_pic_reset(DeviceState* d)
{
    mv88w8618_pic_state* s = MV88W8618_PIC(d);

    s->level = 0;
    s->enabled = 0;
}

static const MemoryRegionOps mv88w8618_pic_ops = {
    .read = mv88w8618_pic_read,
    .write = mv88w8618_pic_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void mv88w8618_pic_init(Object* obj)
{
    SysBusDevice* dev = SYS_BUS_DEVICE(obj);
    mv88w8618_pic_state* s = MV88W8618_PIC(dev);

    qdev_init_gpio_in(DEVICE(dev), mv88w8618_pic_set_irq, 32);
    sysbus_init_irq(dev, &s->parent_irq);
    memory_region_init_io(&s->iomem, obj, &mv88w8618_pic_ops, s,
        "nuc970-pic", MP_PIC_SIZE);
    sysbus_init_mmio(dev, &s->iomem);
}

static const VMStateDescription mv88w8618_pic_vmsd = {
    .name = "mv88w8618_pic",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(level, mv88w8618_pic_state),
        VMSTATE_UINT32(enabled, mv88w8618_pic_state),
        VMSTATE_END_OF_LIST()
    }
};

static void mv88w8618_pic_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->reset = mv88w8618_pic_reset;
    dc->vmsd = &mv88w8618_pic_vmsd;
}

static const TypeInfo mv88w8618_pic_info = {
    .name = TYPE_MV88W8618_PIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(mv88w8618_pic_state),
    .instance_init = mv88w8618_pic_init,
    .class_init = mv88w8618_pic_class_init,
};

/* PIT register offsets */
#define MP_PIT_TIMER1_LENGTH    0x00
/* ... */
#define MP_PIT_TIMER4_LENGTH    0x0C
#define MP_PIT_CONTROL          0x10
#define MP_PIT_TIMER1_VALUE     0x14
/* ... */
#define MP_PIT_TIMER4_VALUE     0x20
#define MP_BOARD_RESET          0x34

/* Magic board reset value (probably some watchdog behind it) */
#define MP_BOARD_RESET_MAGIC    0x10000

typedef struct mv88w8618_timer_state {
    ptimer_state* ptimer;
    uint32_t limit;
    int freq;
    qemu_irq irq;
} mv88w8618_timer_state;

#define TYPE_MV88W8618_PIT "nuc970_pit"
OBJECT_DECLARE_SIMPLE_TYPE(mv88w8618_pit_state, MV88W8618_PIT)

struct mv88w8618_pit_state {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    mv88w8618_timer_state timer[4];
};

static void mv88w8618_timer_tick(void* opaque)
{
    mv88w8618_timer_state* s = opaque;

    qemu_irq_raise(s->irq);
}

static void mv88w8618_timer_init(SysBusDevice* dev, mv88w8618_timer_state* s,
    uint32_t freq)
{
    sysbus_init_irq(dev, &s->irq);
    s->freq = freq;

    s->ptimer = ptimer_init(mv88w8618_timer_tick, s, PTIMER_POLICY_DEFAULT);
}

static uint64_t mv88w8618_pit_read(void* opaque, hwaddr offset,
    unsigned size)
{
    mv88w8618_pit_state* s = opaque;
    mv88w8618_timer_state* t;

    switch (offset) {
    case MP_PIT_TIMER1_VALUE ... MP_PIT_TIMER4_VALUE:
        t = &s->timer[(offset - MP_PIT_TIMER1_VALUE) >> 2];
        return ptimer_get_count(t->ptimer);

    default:
        return 0;
    }
}

static void mv88w8618_pit_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    mv88w8618_pit_state* s = opaque;
    mv88w8618_timer_state* t;
    int i;

    switch (offset) {
    case MP_PIT_TIMER1_LENGTH ... MP_PIT_TIMER4_LENGTH:
        t = &s->timer[offset >> 2];
        t->limit = value;
        ptimer_transaction_begin(t->ptimer);
        if (t->limit > 0) {
            ptimer_set_limit(t->ptimer, t->limit, 1);
        }
        else {
            ptimer_stop(t->ptimer);
        }
        ptimer_transaction_commit(t->ptimer);
        break;

    case MP_PIT_CONTROL:
        for (i = 0; i < 4; i++) {
            t = &s->timer[i];
            ptimer_transaction_begin(t->ptimer);
            if (value & 0xf && t->limit > 0) {
                ptimer_set_limit(t->ptimer, t->limit, 0);
                ptimer_set_freq(t->ptimer, t->freq);
                ptimer_run(t->ptimer, 0);
            }
            else {
                ptimer_stop(t->ptimer);
            }
            ptimer_transaction_commit(t->ptimer);
            value >>= 4;
        }
        break;

    case MP_BOARD_RESET:
        if (value == MP_BOARD_RESET_MAGIC) {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        break;
    }
}

static void mv88w8618_pit_reset(DeviceState* d)
{
    mv88w8618_pit_state* s = MV88W8618_PIT(d);
    int i;

    for (i = 0; i < 4; i++) {
        mv88w8618_timer_state* t = &s->timer[i];
        ptimer_transaction_begin(t->ptimer);
        ptimer_stop(t->ptimer);
        ptimer_transaction_commit(t->ptimer);
        t->limit = 0;
    }
}

static const MemoryRegionOps mv88w8618_pit_ops = {
    .read = mv88w8618_pit_read,
    .write = mv88w8618_pit_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void mv88w8618_pit_init(Object* obj)
{
    SysBusDevice* dev = SYS_BUS_DEVICE(obj);
    mv88w8618_pit_state* s = MV88W8618_PIT(dev);
    int i;

    /* Letting them all run at 1 MHz is likely just a pragmatic
     * simplification. */
    for (i = 0; i < 4; i++) {
        mv88w8618_timer_init(dev, &s->timer[i], 1000000);
    }

    memory_region_init_io(&s->iomem, obj, &mv88w8618_pit_ops, s,
        "nuc970-pit", MP_PIT_SIZE);
    sysbus_init_mmio(dev, &s->iomem);
}

static void mv88w8618_pit_finalize(Object* obj)
{
    SysBusDevice* dev = SYS_BUS_DEVICE(obj);
    mv88w8618_pit_state* s = MV88W8618_PIT(dev);
    int i;

    for (i = 0; i < 4; i++) {
        ptimer_free(s->timer[i].ptimer);
    }
}

static const VMStateDescription mv88w8618_timer_vmsd = {
    .name = "timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PTIMER(ptimer, mv88w8618_timer_state),
        VMSTATE_UINT32(limit, mv88w8618_timer_state),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription mv88w8618_pit_vmsd = {
    .name = "mv88w8618_pit",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(timer, mv88w8618_pit_state, 4, 1,
                             mv88w8618_timer_vmsd, mv88w8618_timer_state),
        VMSTATE_END_OF_LIST()
    }
};

static void mv88w8618_pit_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->reset = mv88w8618_pit_reset;
    dc->vmsd = &mv88w8618_pit_vmsd;
}

static const TypeInfo mv88w8618_pit_info = {
    .name = TYPE_MV88W8618_PIT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(mv88w8618_pit_state),
    .instance_init = mv88w8618_pit_init,
    .instance_finalize = mv88w8618_pit_finalize,
    .class_init = mv88w8618_pit_class_init,
};

/* Flash config register offsets */
#define MP_FLASHCFG_CFGR0    0x04

#define TYPE_MV88W8618_FLASHCFG "nuc970_flashcfg"
OBJECT_DECLARE_SIMPLE_TYPE(mv88w8618_flashcfg_state, MV88W8618_FLASHCFG)

struct mv88w8618_flashcfg_state {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t cfgr0;
};

static uint64_t mv88w8618_flashcfg_read(void* opaque,
    hwaddr offset,
    unsigned size)
{
    mv88w8618_flashcfg_state* s = opaque;

    switch (offset) {
    case MP_FLASHCFG_CFGR0:
        return s->cfgr0;

    default:
        return 0;
    }
}

static void mv88w8618_flashcfg_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    mv88w8618_flashcfg_state* s = opaque;

    switch (offset) {
    case MP_FLASHCFG_CFGR0:
        s->cfgr0 = value;
        break;
    }
}

static const MemoryRegionOps mv88w8618_flashcfg_ops = {
    .read = mv88w8618_flashcfg_read,
    .write = mv88w8618_flashcfg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void mv88w8618_flashcfg_init(Object* obj)
{
    SysBusDevice* dev = SYS_BUS_DEVICE(obj);
    mv88w8618_flashcfg_state* s = MV88W8618_FLASHCFG(dev);

    s->cfgr0 = 0xfffe4285; /* Default as set by U-Boot for 8 MB flash */
    memory_region_init_io(&s->iomem, obj, &mv88w8618_flashcfg_ops, s,
        "nuc970-flashcfg", MP_FLASHCFG_SIZE);
    sysbus_init_mmio(dev, &s->iomem);
}

static const VMStateDescription mv88w8618_flashcfg_vmsd = {
    .name = "mv88w8618_flashcfg",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(cfgr0, mv88w8618_flashcfg_state),
        VMSTATE_END_OF_LIST()
    }
};

static void mv88w8618_flashcfg_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->vmsd = &mv88w8618_flashcfg_vmsd;
}

static const TypeInfo mv88w8618_flashcfg_info = {
    .name = TYPE_MV88W8618_FLASHCFG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(mv88w8618_flashcfg_state),
    .instance_init = mv88w8618_flashcfg_init,
    .class_init = mv88w8618_flashcfg_class_init,
};

/* Misc register offsets */
#define MP_MISC_BOARD_REVISION  0x18

#define MP_BOARD_REVISION       0x31

struct NUC970MiscState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
};

#define TYPE_NUC970_MISC "nuc970-misc"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970MiscState, NUC970_MISC)

static uint64_t nuc970_misc_read(void* opaque, hwaddr offset,
    unsigned size)
{
    switch (offset) {
    case MP_MISC_BOARD_REVISION:
        return MP_BOARD_REVISION;

    default:
        return 0;
    }
}

static void nuc970_misc_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
}

static const MemoryRegionOps nuc970_misc_ops = {
    .read = nuc970_misc_read,
    .write = nuc970_misc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void nuc970_misc_init(Object* obj)
{
    SysBusDevice* sd = SYS_BUS_DEVICE(obj);
    NUC970MiscState* s = NUC970_MISC(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &nuc970_misc_ops, NULL,
        "nuc970-misc", MP_MISC_SIZE);
    sysbus_init_mmio(sd, &s->iomem);
}

static const TypeInfo nuc970_misc_info = {
    .name = TYPE_NUC970_MISC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = nuc970_misc_init,
    .instance_size = sizeof(NUC970MiscState),
};

// nuc970 system global control registers

struct NUC970SysState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[67];      // 0x0 ~ 0x108
};

#define TYPE_NUC970_SYS "nuc970-sys"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970SysState, NUC970_SYS)

static uint64_t nuc970_sys_read(void* opaque, hwaddr offset,
    unsigned size)
{
    NUC970SysState* s = (NUC970SysState*)opaque;
    uint32_t r = 0;
    switch (offset) {
    case 0x1fc:
        r = 0x00000001;
        break;
    case 0:
        r = 0x1230D008;
        break;
    case 4:
        //r = 0x1F0007D7;
        r = 0x1E0007C7;
        break;
    default:
        r = s->regs[offset / 4];
        break;
    }

    //fprintf(stderr, "sys_read(offset=%lx, value=%08x)\n", offset, r);
    return r;
}

static void nuc970_sys_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    NUC970SysState* s = (NUC970SysState*)opaque;
    //fprintf(stderr, "sys_write(offset=%lx, value=%08x)\n", offset, value);
    switch (offset)
    {
    case 0x1fc:
        break;
    default:
        s->regs[offset / 4] = value;
        break;
    }
}

static const MemoryRegionOps nuc970_sys_ops = {
    .read = nuc970_sys_read,
    .write = nuc970_sys_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void nuc970_sys_init(Object* obj)
{
    SysBusDevice* sd = SYS_BUS_DEVICE(obj);
    NUC970SysState* s = NUC970_SYS(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &nuc970_sys_ops, s,
        "nuc970-sys", 0x200);
    sysbus_init_mmio(sd, &s->iomem);
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x68/4] = 0xffffffff;   // SYS_APBIPRST1
}

static const TypeInfo nuc970_sys_info = {
    .name = TYPE_NUC970_SYS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = nuc970_sys_init,
    .instance_size = sizeof(NUC970SysState),
};

// WDT

struct NUC970WdtState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    /*
    WDT_CTL WDT_BA + 0x00 R / W WDT Control Register 0x0000_0700
    WDT_ALTCTL WDT_BA + 0x04 R / W WDT Alternative Control Register 0x0000_0000
    */
    uint32_t ctl;      // 00
    uint32_t altctl;   // 04
};

#define TYPE_NUC970_WDT "nuc970-wdt"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970WdtState, NUC970_WDT)

static uint64_t nuc970_wdt_read(void* opaque, hwaddr offset,
    unsigned size)
{
    NUC970WdtState* s = (NUC970WdtState*)opaque;
    uint32_t r = 0;
    switch (offset) {
    case 0:
        r = s->ctl & ~(0x01);   // clear RSTCNT
        break;
    case 4:
        r = s->altctl;
        break;
    default:
        break;
    }

    //fprintf(stderr, "wdt_read(offset=%lx, value=%08x)\n", offset, r);
    return r;
}

static void nuc970_wdt_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    NUC970WdtState* s = (NUC970WdtState*)opaque;
    //fprintf(stderr, "wdt_write(offset=%lx, value=%08x)\n", offset, value);
    switch (offset)
    {
    case 0:
        s->ctl = value;
        break;
    case 4:
        s->altctl = value;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps nuc970_wdt_ops = {
    .read = nuc970_wdt_read,
    .write = nuc970_wdt_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void nuc970_wdt_init(Object* obj)
{
    SysBusDevice* sd = SYS_BUS_DEVICE(obj);
    NUC970WdtState* s = NUC970_WDT(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &nuc970_wdt_ops, s,
        "nuc970-wdt", 0x100);
    sysbus_init_mmio(sd, &s->iomem);
    s->ctl = 0x00000700;
    s->altctl = 0;
}

static const TypeInfo nuc970_wdt_info = {
    .name = TYPE_NUC970_WDT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = nuc970_wdt_init,
    .instance_size = sizeof(NUC970WdtState),
};


// SDRAM (SDR/DDR/DDR2) Control Register
struct NUC970SdicState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
};

#define TYPE_NUC970_SDIC "nuc970-sdic"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970SdicState, NUC970_SDIC)

static uint64_t nuc970_sdic_read(void* opaque, hwaddr offset,
    unsigned size)
{
    switch (offset) {
    case 0x10:  // SDIC_SIZE0
        return 0x00000006;  // 6: 64M

    default:
        return 0;
    }
}

static void nuc970_sdic_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
}

static const MemoryRegionOps nuc970_sdic_ops = {
    .read = nuc970_sdic_read,
    .write = nuc970_sdic_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void nuc970_sdic_init(Object* obj)
{
    SysBusDevice* sd = SYS_BUS_DEVICE(obj);
    NUC970SdicState* s = NUC970_SDIC(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &nuc970_sdic_ops, NULL,
        "nuc970-sdic", 0x800);
    sysbus_init_mmio(sd, &s->iomem);
}

static const TypeInfo nuc970_sdic_info = {
    .name = TYPE_NUC970_SDIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = nuc970_sdic_init,
    .instance_size = sizeof(NUC970SdicState),
};

// CLK_CTL
#define CLK_PMCON 0x000  //R/W Power Management Control Register 0xFFFF_FF03
#define CLK_HCLKEN 0x010 //R/W AHB Devices Clock Enable Control Register 0x0000_0527
#define CLK_PCLKEN0 0x018 //R / W APB Devices Clock Enable Control Register 0 0x0000_0000
#define CLK_PCLKEN1 0x01C //R / W APB Devices Clock Enable Control Register 1 0x0000_0000
#define CLK_DIVCTL0 0x020 //R / W Clock Divider Control Register 0 0x0100_00XX
#define CLK_DIVCTL1 0x024 //R / W Clock Divider Control Register 1 0x0000_0000
#define CLK_DIVCTL2 0x028 //R / W Clock Divider Control Register 2 0x0000_0000
#define CLK_DIVCTL3 0x02C //R / W Clock Divider Control Register 3 0x0000_0000
#define CLK_DIVCTL4 0x030 //R / W Clock Divider Control Register 4 0x0000_0000
#define CLK_DIVCTL5 0x034 //R / W Clock Divider Control Register 5 0x0000_0000
#define CLK_DIVCTL6 0x038 //R / W Clock Divider Control Register 6 0x0000_0000
#define CLK_DIVCTL7 0x03C //R / W Clock Divider Control Register 7 0x0000_0000
#define CLK_DIVCTL8 0x040 //R / W Clock Divider Control Register 8 0x0000_0500
#define CLK_DIVCTL9 0x044 //R / W Clock Divider Control Register 9 0x0000_0000
#define CLK_APLLCON 0x060 //R / W APLL Control Register 0x1000_0015
#define CLK_UPLLCON 0x064 //R / W UPLL Control Register 0xX000_0015
#define CLK_PLLSTBCNTR 0x080 //R/W PLL Stable Counter and Test Clock Control Register 0x0000_1800

struct NUC970ClkState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t pmcon;
    uint32_t hclk;
    uint32_t pclk0;
    uint32_t pclk1;
    uint32_t divctl[10];
    uint32_t apll;
    uint32_t upll;
    uint32_t pll;
};

#define TYPE_NUC970_CLK "nuc970-clk"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970ClkState, NUC970_CLK)

static uint64_t nuc970_clk_read(void* opaque, hwaddr offset,
    unsigned size)
{
    NUC970ClkState* s = (NUC970ClkState*)opaque;
    switch (offset) {
    case CLK_PMCON:
        return s->pmcon;
    case CLK_HCLKEN:
        return s->hclk;
    case CLK_PCLKEN0:
        return s->pclk0;
    case CLK_PCLKEN1:
        return s->pclk1;
    case CLK_DIVCTL0:
    case CLK_DIVCTL1:
    case CLK_DIVCTL2:
    case CLK_DIVCTL3:
    case CLK_DIVCTL4:
    case CLK_DIVCTL5:
    case CLK_DIVCTL6:
    case CLK_DIVCTL7:
    case CLK_DIVCTL8:
    case CLK_DIVCTL9:
        return s->divctl[(offset - CLK_DIVCTL0) / 4];
    case CLK_APLLCON:
        return s->apll;
    case CLK_UPLLCON:
        return s->upll;
    case CLK_PLLSTBCNTR:
        return s->pll;
    default:
        return 0;
    }
}

static void nuc970_clk_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
}

static const MemoryRegionOps nuc970_clk_ops = {
    .read = nuc970_clk_read,
    .write = nuc970_clk_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void nuc970_clk_init(Object* obj)
{
    SysBusDevice* sd = SYS_BUS_DEVICE(obj);
    NUC970ClkState* s = NUC970_CLK(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &nuc970_clk_ops, s,
        "nuc970-clk", 0x100);
    sysbus_init_mmio(sd, &s->iomem);

    s->pmcon = 0xffffff03;
    s->hclk = 0x00000527;
    s->pclk0 = 0x00000000;
    s->pclk1 = 0x00000000;
    s->divctl[0] = 0x01000018;
    s->divctl[1] = 0x00000000;
    s->divctl[2] = 0x00000000;
    s->divctl[3] = 0x00000100;
    s->divctl[4] = 0x00000000;
    s->divctl[5] = 0x00000000;
    s->divctl[6] = 0x00000000;
    s->divctl[7] = 0x00000000;
    s->divctl[8] = 0x00000295;
    s->divctl[9] = 0x0000011a;
    s->apll = 0x50000015;
    s->upll = 0xc0000018;
    s->pll = 0x00001800;
    

}

static const VMStateDescription nuc970_clk_vmsd = {
    .name = "nuc970_clk",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(pmcon, NUC970ClkState),
        VMSTATE_UINT32(hclk, NUC970ClkState),
        VMSTATE_UINT32(pclk0, NUC970ClkState),
        VMSTATE_UINT32(pclk1, NUC970ClkState),
        VMSTATE_UINT32_ARRAY(divctl, NUC970ClkState, 10),
        VMSTATE_UINT32(apll, NUC970ClkState),
        VMSTATE_UINT32(upll, NUC970ClkState),
        VMSTATE_UINT32(pll, NUC970ClkState),
        VMSTATE_END_OF_LIST()
    }
};

static void nuc970_clk_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->vmsd = &nuc970_clk_vmsd;
}

static const TypeInfo nuc970_clk_info = {
    .name = TYPE_NUC970_CLK,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = nuc970_clk_init,
    .instance_size = sizeof(NUC970ClkState),
    .class_init = nuc970_clk_class_init,
};




//////////////////

/* WLAN register offsets */
#define MP_WLAN_MAGIC1          0x11c
#define MP_WLAN_MAGIC2          0x124

static uint64_t mv88w8618_wlan_read(void* opaque, hwaddr offset,
    unsigned size)
{
    switch (offset) {
        /* Workaround to allow loading the binary-only wlandrv.ko crap
         * from the original Freecom firmware. */
    case MP_WLAN_MAGIC1:
        return ~3;
    case MP_WLAN_MAGIC2:
        return -1;

    default:
        return 0;
    }
}

static void mv88w8618_wlan_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
}

static const MemoryRegionOps mv88w8618_wlan_ops = {
    .read = mv88w8618_wlan_read,
    .write = mv88w8618_wlan_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void mv88w8618_wlan_realize(DeviceState* dev, Error** errp)
{
    MemoryRegion* iomem = g_new(MemoryRegion, 1);

    memory_region_init_io(iomem, OBJECT(dev), &mv88w8618_wlan_ops, NULL,
        "nuc970-wlan", MP_WLAN_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), iomem);
}

/* GPIO register offsets */
#define MP_GPIO_OE_LO           0x008
#define MP_GPIO_OUT_LO          0x00c
#define MP_GPIO_IN_LO           0x010
#define MP_GPIO_IER_LO          0x014
#define MP_GPIO_IMR_LO          0x018
#define MP_GPIO_ISR_LO          0x020
#define MP_GPIO_OE_HI           0x508
#define MP_GPIO_OUT_HI          0x50c
#define MP_GPIO_IN_HI           0x510
#define MP_GPIO_IER_HI          0x514
#define MP_GPIO_IMR_HI          0x518
#define MP_GPIO_ISR_HI          0x520

/* GPIO bits & masks */
#define MP_GPIO_LCD_BRIGHTNESS  0x00070000
#define MP_GPIO_I2C_DATA_BIT    29
#define MP_GPIO_I2C_CLOCK_BIT   30

/* LCD brightness bits in GPIO_OE_HI */
#define MP_OE_LCD_BRIGHTNESS    0x0007

#define TYPE_NUC970_GPIO "nuc970_gpio"
OBJECT_DECLARE_SIMPLE_TYPE(nuc970_gpio_state, NUC970_GPIO)

struct nuc970_gpio_group
{
    uint16_t dir;       // 00
    uint16_t dataout;   // 04
    uint16_t datain;    // 08
    uint16_t imd;       // 0c
    uint16_t iren;      // 10
    uint16_t ifen;      // 14
    uint16_t isr;       // 18
    uint16_t dben;      // 1c
    uint16_t puen;      // 20
    uint16_t pden;      // 24
    uint16_t icen;      // 28
    uint16_t isen;      // 2c
};

struct nuc970_gpio_state {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;

    uint32_t lcd_brightness;
    uint32_t out_state;
    uint32_t in_state;
    uint32_t ier;
    uint32_t imr;
    uint32_t isr;
    
    qemu_irq irq;
    qemu_irq out[5];

    uint16_t regs[256]; /* GPIOA - GPIOJ */
};

#if 0
static void nuc970_gpio_brightness_update(nuc970_gpio_state* s) {
    int i;
    uint32_t brightness;

    /* compute brightness ratio */
    switch (s->lcd_brightness) {
    case 0x00000007:
        brightness = 0;
        break;

    case 0x00020000:
        brightness = 1;
        break;

    case 0x00020001:
        brightness = 2;
        break;

    case 0x00040000:
        brightness = 3;
        break;

    case 0x00010006:
        brightness = 4;
        break;

    case 0x00020005:
        brightness = 5;
        break;

    case 0x00040003:
        brightness = 6;
        break;

    case 0x00030004:
    default:
        brightness = 7;
    }

    /* set lcd brightness GPIOs  */
    for (i = 0; i <= 2; i++) {
        qemu_set_irq(s->out[i], (brightness >> i) & 1);
    }
}
#endif

static void nuc970_gpio_pin_event(void* opaque, int pin, int level)
{
    nuc970_gpio_state* s = opaque;
    uint32_t mask = 1 << pin;
    uint32_t delta = level << pin;
    uint32_t old = s->in_state & mask;

    s->in_state &= ~mask;
    s->in_state |= delta;

    if ((old ^ delta) &&
        ((level && (s->imr & mask)) || (!level && (s->ier & mask)))) {
        s->isr = mask;
        qemu_irq_raise(s->irq);
    }
}

static uint64_t nuc970_gpio_read(void* opaque, hwaddr offset,
    unsigned size)
{
    nuc970_gpio_state* s = opaque;
    uint32_t r = 0;
    //int group = offset / 0x40;
    //int idx = offset % 0x40;

    switch (offset) {   
    default:
        r = s->regs[offset / 4];
        break;
    }
    fprintf(stderr, "gpio read %08lx = %08x\n", offset, r);
    return r;
}

static void nuc970_gpio_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    nuc970_gpio_state* s = opaque;
    fprintf(stderr, "gpio write %08lx = %08lx\n", offset, value);
    switch (offset) {
    default:
        s->regs[offset / 4] = value & 0xffff;
        break;
    }
}

static const MemoryRegionOps nuc970_gpio_ops = {
    .read = nuc970_gpio_read,
    .write = nuc970_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void nuc970_gpio_reset(DeviceState* d)
{
    int i;
    nuc970_gpio_state* s = NUC970_GPIO(d);

    s->lcd_brightness = 0;
    s->out_state = 0;
    s->in_state = 0xffffffff;
    s->ier = 0;
    s->imr = 0;
    s->isr = 0;
    memset(s->regs, 0, sizeof s->regs);

    s->regs[0x3f0 / 4] = 0x0020;
    for (i = 0; i < 10; i++) {
        s->regs[i * 0x10 + 10] = 0xffff;    // ICEN
    }
    
}

static void nuc970_gpio_init(Object* obj)
{
    SysBusDevice* sbd = SYS_BUS_DEVICE(obj);
    DeviceState* dev = DEVICE(sbd);
    nuc970_gpio_state* s = NUC970_GPIO(dev);

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init_io(&s->iomem, obj, &nuc970_gpio_ops, s,
        "nuc970-gpio", MP_GPIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_out(dev, s->out, ARRAY_SIZE(s->out));

    qdev_init_gpio_in(dev, nuc970_gpio_pin_event, 32);
}

static const VMStateDescription nuc970_gpio_vmsd = {
    .name = "nuc970_gpio",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, nuc970_gpio_state, 256),
        VMSTATE_END_OF_LIST()
    }
};

static void nuc970_gpio_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->reset = nuc970_gpio_reset;
    dc->vmsd = &nuc970_gpio_vmsd;
}

static const TypeInfo nuc970_gpio_info = {
    .name = TYPE_NUC970_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nuc970_gpio_state),
    .instance_init = nuc970_gpio_init,
    .class_init = nuc970_gpio_class_init,
};

/* Keyboard codes & masks */
#define KEY_RELEASED            0x80
#define KEY_CODE                0x7f

#define KEYCODE_TAB             0x0f
#define KEYCODE_ENTER           0x1c
#define KEYCODE_F               0x21
#define KEYCODE_M               0x32

#define KEYCODE_EXTENDED        0xe0
#define KEYCODE_UP              0x48
#define KEYCODE_DOWN            0x50
#define KEYCODE_LEFT            0x4b
#define KEYCODE_RIGHT           0x4d

#define MP_KEY_WHEEL_VOL       (1 << 0)
#define MP_KEY_WHEEL_VOL_INV   (1 << 1)
#define MP_KEY_WHEEL_NAV       (1 << 2)
#define MP_KEY_WHEEL_NAV_INV   (1 << 3)
#define MP_KEY_BTN_FAVORITS    (1 << 4)
#define MP_KEY_BTN_MENU        (1 << 5)
#define MP_KEY_BTN_VOLUME      (1 << 6)
#define MP_KEY_BTN_NAVIGATION  (1 << 7)

#define TYPE_NUC970_KEY "nuc970_key"
OBJECT_DECLARE_SIMPLE_TYPE(nuc970_key_state, NUC970_KEY)

struct nuc970_key_state {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t kbd_extended;
    uint32_t pressed_keys;
    qemu_irq out[8];
};

static void nuc970_key_event(void* opaque, int keycode)
{
    nuc970_key_state* s = opaque;
    uint32_t event = 0;
    int i;

    if (keycode == KEYCODE_EXTENDED) {
        s->kbd_extended = 1;
        return;
    }

    if (s->kbd_extended) {
        switch (keycode & KEY_CODE) {
        case KEYCODE_UP:
            event = MP_KEY_WHEEL_NAV | MP_KEY_WHEEL_NAV_INV;
            break;

        case KEYCODE_DOWN:
            event = MP_KEY_WHEEL_NAV;
            break;

        case KEYCODE_LEFT:
            event = MP_KEY_WHEEL_VOL | MP_KEY_WHEEL_VOL_INV;
            break;

        case KEYCODE_RIGHT:
            event = MP_KEY_WHEEL_VOL;
            break;
        }
    }
    else {
        switch (keycode & KEY_CODE) {
        case KEYCODE_F:
            event = MP_KEY_BTN_FAVORITS;
            break;

        case KEYCODE_TAB:
            event = MP_KEY_BTN_VOLUME;
            break;

        case KEYCODE_ENTER:
            event = MP_KEY_BTN_NAVIGATION;
            break;

        case KEYCODE_M:
            event = MP_KEY_BTN_MENU;
            break;
        }
        /* Do not repeat already pressed buttons */
        if (!(keycode & KEY_RELEASED) && (s->pressed_keys & event)) {
            event = 0;
        }
    }

    if (event) {
        /* Raise GPIO pin first if repeating a key */
        if (!(keycode & KEY_RELEASED) && (s->pressed_keys & event)) {
            for (i = 0; i <= 7; i++) {
                if (event & (1 << i)) {
                    qemu_set_irq(s->out[i], 1);
                }
            }
        }
        for (i = 0; i <= 7; i++) {
            if (event & (1 << i)) {
                qemu_set_irq(s->out[i], !!(keycode & KEY_RELEASED));
            }
        }
        if (keycode & KEY_RELEASED) {
            s->pressed_keys &= ~event;
        }
        else {
            s->pressed_keys |= event;
        }
    }

    s->kbd_extended = 0;
}

static void nuc970_key_init(Object* obj)
{
    SysBusDevice* sbd = SYS_BUS_DEVICE(obj);
    DeviceState* dev = DEVICE(sbd);
    nuc970_key_state* s = NUC970_KEY(dev);

    memory_region_init(&s->iomem, obj, "dummy", 0);
    sysbus_init_mmio(sbd, &s->iomem);

    s->kbd_extended = 0;
    s->pressed_keys = 0;

    qdev_init_gpio_out(dev, s->out, ARRAY_SIZE(s->out));

    qemu_add_kbd_event_handler(nuc970_key_event, s);
}

static const VMStateDescription nuc970_key_vmsd = {
    .name = "nuc970_key",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(kbd_extended, nuc970_key_state),
        VMSTATE_UINT32(pressed_keys, nuc970_key_state),
        VMSTATE_END_OF_LIST()
    }
};

static void nuc970_key_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->vmsd = &nuc970_key_vmsd;
}

static const TypeInfo nuc970_key_info = {
    .name = TYPE_NUC970_KEY,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nuc970_key_state),
    .instance_init = nuc970_key_init,
    .class_init = nuc970_key_class_init,
};

// rtc

struct NUC970RtcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    /*
    WDT_CTL WDT_BA + 0x00 R / W WDT Control Register 0x0000_0700
    WDT_ALTCTL WDT_BA + 0x04 R / W WDT Alternative Control Register 0x0000_0000
    */
    uint32_t time;     // 0c
    uint32_t cal;   // 10
    uint32_t timefmt; // 14

    uint32_t inten; //RTC_BA + 0x028 R / W RTC Interrupt Enable Register 0x0000_0000
    uint32_t intsts; //RTC_BA + 0x02C
};

#define TYPE_NUC970_RTC "nuc970-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970RtcState, NUC970_RTC)

static uint64_t nuc970_rtc_read(void* opaque, hwaddr offset,
    unsigned size)
{
    NUC970RtcState* s = (NUC970RtcState*)opaque;
    uint32_t r = 0;
    switch (offset) {
    case 0x0c:
        r = s->time;
        break;
    case 0x10:
        r = s->cal;
        break;
    case 0x14:
        r = s->timefmt;
        break;
    case 0x28:
        r = s->inten;
        break;
    case 0x2c:
        r = s->intsts;
        break;
    default:
        break;
    }

    //fprintf(stderr, "rtc_read(offset=%lx, value=%08x)\n", offset, r);
    return r;
}

static void nuc970_rtc_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    NUC970RtcState* s = (NUC970RtcState*)opaque;
    //fprintf(stderr, "rtc_write(offset=%lx, value=%08x)\n", offset, value);
    switch (offset)
    {
    case 0x14:
        s->timefmt = value;
        break;
    case 0x28:
        s->inten = value;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps nuc970_rtc_ops = {
    .read = nuc970_rtc_read,
    .write = nuc970_rtc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void nuc970_rtc_init(Object* obj)
{
    SysBusDevice* sd = SYS_BUS_DEVICE(obj);
    NUC970RtcState* s = NUC970_RTC(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &nuc970_rtc_ops, s,
        "nuc970-rtc", 0x1000);
    sysbus_init_mmio(sd, &s->iomem);
    s->time = 0x0;
    s->cal = 0x00050101;
    s->timefmt = 0x1;
    s->inten = 0;
    s->intsts = 0;
}

static const TypeInfo nuc970_rtc_info = {
    .name = TYPE_NUC970_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = nuc970_rtc_init,
    .instance_size = sizeof(NUC970RtcState),
};

// Flash Memory Interface (FMI)
// DMA unit and FMI unit (eMMC or NAND flash)

struct NUC970FmiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    DeviceState* nand;
    uint8_t manf_id;
    uint8_t chip_id;
    ECCState ecc;

    uint32_t FMI_CTL;           // 0x800
    uint32_t FMI_NANDCTL;       // 0x8a0
    uint32_t FMI_NANDTMCTL;     // 0x8a4
    uint32_t FMI_NANDINTSTS;    // 0x8ac
    uint32_t FMI_NANDCMD;       // 0x8b0
    uint32_t FMI_NANDADDR;      // 0x8b4
    uint32_t FMI_NANDDATA;      // 0x8b8
    uint32_t FMI_NANDRACTL;     // 0x8bc
    uint32_t FMI_NANDECTL;      // 0x8c0
};

#define TYPE_NUC970_FMI "nuc970-fmi"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970FmiState, NUC970_FMI)

static uint64_t nuc970_fmi_read(void* opaque, hwaddr offset,
    unsigned size)
{
    NUC970FmiState* fmi = (NUC970FmiState*)opaque;
    uint32_t r = 0;
    switch (offset) {
    case 0x800: r = fmi->FMI_CTL; break;
    case 0x8a0:
        //fprintf(stderr, "FMI_NANDCTL: %08x\n", fmi->FMI_NANDCTL);
        r = fmi->FMI_NANDCTL & ~(1); // clear SR_RST BIT
        break;
    case 0x8a4: r = fmi->FMI_NANDTMCTL; break;
    case 0x8ac:
        //fprintf(stderr, "FMI_NANDINTSTS: %08x\n", fmi->FMI_NANDINTSTS);
        r = fmi->FMI_NANDINTSTS; break;
    case 0x8b0: r = fmi->FMI_NANDCMD; break;
    case 0x8b4: r = fmi->FMI_NANDADDR; break;
    case 0x8b8: r = fmi->FMI_NANDDATA; break;
    case 0x8bc: r = fmi->FMI_NANDRACTL; break;
    case 0x8c0: r = fmi->FMI_NANDECTL; break;
    default: r = 0; break;
    }

    fprintf(stderr, "fmi_read(offset=%lx, value=%08x)\n", offset, r);
    return r;
}

static void nuc970_fmi_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    NUC970FmiState* fmi = (NUC970FmiState*)opaque;
    fprintf(stderr, "fmi_write(offset=%lx, value=%08lx)\n", offset, value);
    switch (offset)
    {
    case 0x800:
        fmi->FMI_CTL = value;
        break;
    case 0x8a0:
        fmi->FMI_NANDCTL = value;
        break;
    case 0x8a4:
        fmi->FMI_NANDTMCTL = value;
        break;
    case 0x8ac:
        fmi->FMI_NANDINTSTS = value;
        break;
    case 0x8b0:
        fmi->FMI_NANDCMD = value;
        break;
    case 0x8b4:
        fmi->FMI_NANDADDR = value;
        break;
    case 0x8b8:
        fmi->FMI_NANDDATA = value;
        break;
    case 0x8bc:
        fmi->FMI_NANDRACTL = value;
        break;
    case 0x8c0:
        fmi->FMI_NANDECTL = value;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps nuc970_fmi_ops = {
    .read = nuc970_fmi_read,
    .write = nuc970_fmi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void nuc970_fmi_init(Object* obj)
{
    SysBusDevice* sd = SYS_BUS_DEVICE(obj);
    NUC970FmiState* fmi = NUC970_FMI(obj);

    memory_region_init_io(&fmi->iomem, OBJECT(fmi), &nuc970_fmi_ops, fmi,
        "nuc970-fmi", 0x1000);
    sysbus_init_mmio(sd, &fmi->iomem);
    fmi->FMI_CTL = 0x0;
    fmi->FMI_NANDTMCTL = 0x00010105;
    fmi->FMI_NANDCTL = 0x1E880090;
    fmi->FMI_NANDINTSTS = 0x00040000;
    fmi->FMI_NANDCMD = 0x0;
    fmi->FMI_NANDADDR = 0x0;
    fmi->FMI_NANDECTL = 0x0;
}

static void nuc970_fmi_realize(DeviceState* dev, Error** errp)
{
    NUC970FmiState* s = NUC970_FMI(dev);
    DriveInfo* nand;

    /* FIXME use a qdev drive property instead of drive_get() */
    nand = drive_get_by_index(IF_MTD, 1);
    s->nand = nand_init(nand ? blk_by_legacy_dinfo(nand) : NULL,
        s->manf_id, s->chip_id);
}

static const VMStateDescription vmstate_nuc970_fmi_info = {
    .name = "nuc970-fmi",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(FMI_CTL, NUC970FmiState),
        VMSTATE_STRUCT(ecc, NUC970FmiState, 0, vmstate_ecc_state, ECCState),
        VMSTATE_END_OF_LIST(),
    },
};

static Property nuc970_fmi_properties[] = {
    DEFINE_PROP_UINT8("manf_id", NUC970FmiState, manf_id, NAND_MFR_SAMSUNG),
    DEFINE_PROP_UINT8("chip_id", NUC970FmiState, chip_id, 0xf1), // 0x73:128M 0xf1:1024M
    DEFINE_PROP_END_OF_LIST(),
};

static void nuc970_fmi_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_nuc970_fmi_info;
    device_class_set_props(dc, nuc970_fmi_properties);
    dc->realize = nuc970_fmi_realize;
    /* Reason: init() method uses drive_get() */
    dc->user_creatable = false;
}

static const TypeInfo nuc970_fmi_info = {
    .name = TYPE_NUC970_FMI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = nuc970_fmi_init,
    .instance_size = sizeof(NUC970FmiState),
    .class_init = nuc970_fmi_class_init,
};

// SDH : Secure Digital Host Controller

struct NUC970SdhState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    
    /** Memory region where DMA transfers are done */
    MemoryRegion* dma_mr;

    /** Address space used internally for DMA transfers */
    AddressSpace dma_as;

    /** Number of bytes left in current DMA transfer */
    uint32_t transfer_cnt;

    qemu_irq irq;
    BlockBackend* blk;
    SDBus sdbus;

    uint32_t SDH_FB[32];    //
    uint32_t SDH_DMACTL;    // 0x400 0x0000_0000
    uint32_t SDH_DMASA;
    uint32_t SDH_DMABCNT;
    uint32_t SDH_DMAINTEN;
    uint32_t SDH_DMAINTSTS;

    uint32_t SDH_GCTL;      // 0x800 0x0000_0000
    uint32_t SDH_GINTEN;
    uint32_t SDH_GINTSTS;
    uint32_t SDH_CTL;       // 0x820 0x0101_0000
    uint32_t SDH_CMD;
    uint32_t SDH_INTEN;     // 0x828 0x0000_0A00
    uint32_t SDH_INTSTS;    // 0x82c 0x0000_008C
    uint32_t SDH_RESP0;
    uint32_t SDH_RESP1;
    uint32_t SDH_BLEN;
    uint32_t SDH_TMOUT;
    uint32_t SDH_ECTL;      // 0x840 0x0000_0003
};

#define TYPE_NUC970_SDH "nuc970-sdh"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970SdhState, NUC970_SDH)

static void nuc970_sdhost_send_command(NUC970SdhState* s)
{
    SDRequest request;
    uint8_t resp[16];
    int rlen;

    /* Auto clear load flag */
    //s->command &= ~SD_CMDR_LOAD;

    /* Clock change does not actually interact with the SD bus */
    //if (!(s->command & SD_CMDR_CLKCHANGE)) 
    {

        /* Prepare request */
        request.cmd = (s->SDH_CTL >> 8) & 0x3f;
        request.arg = s->SDH_CMD;

        /* Send request to SD bus */
        rlen = sdbus_do_command(&s->sdbus, &request, resp);
        fprintf(stderr, "sdbus_do_command: %d %x\n", rlen, resp[0]);
        if (rlen < 0) {
            //goto error;
        }

        /* If the command has a response, store it in the response registers */
        if ((s->SDH_CTL & (1 << 1))) {
            s->SDH_RESP0 = ldl_be_p(&resp[0]);
            /*
            if (rlen == 4 && !(s->command & SD_CMDR_RESPONSE_LONG)) {
                s->response[0] = ldl_be_p(&resp[0]);
                s->response[1] = s->response[2] = s->response[3] = 0;

            }
            else if (rlen == 16 && (s->command & SD_CMDR_RESPONSE_LONG)) {
                s->response[0] = ldl_be_p(&resp[12]);
                s->response[1] = ldl_be_p(&resp[8]);
                s->response[2] = ldl_be_p(&resp[4]);
                s->response[3] = ldl_be_p(&resp[0]);
            }
            else {
                goto error;
            }
            */
        }
    }

    /* Set interrupt status bits */
    //s->irq_status |= SD_RISR_CMD_COMPLETE;
    return;

//error:
    //s->irq_status |= SD_RISR_NO_RESPONSE;
}


static uint64_t nuc970_sdh_read(void* opaque, hwaddr offset,
    unsigned size)
{
    NUC970SdhState* sdh = (NUC970SdhState*)opaque;
    uint32_t r = 0;
    switch (offset) {
    case 0x400: r = sdh->SDH_DMACTL & ~(1<<1); break;
    case 0x800: r = sdh->SDH_GCTL & ~(1<<0); break;
    case 0x820:
        //fprintf(stderr, "SDH_CTL: %08x\n", sdh->SDH_CTL);
        r = sdh->SDH_CTL;
        //r &= ~(1 << 6); // clear CLK8_OE
        //r &= ~(1 << 5); // clear CLK74_OE
        //r &= ~(1 << 14);
        r &= ~(0x407f);
        break;
    case 0x828: r = sdh->SDH_INTEN; break;
    case 0x82c: r = sdh->SDH_INTSTS | (1 << 2); break;
    case 0x830: r = sdh->SDH_RESP0; break;
    case 0x840: r = sdh->SDH_ECTL; break;
    
    default: r = 0; break;
    }

    fprintf(stderr, "sdh_read(offset=%lx, value=%08x)\n", offset, r);
    return r;
}

static void nuc970_sdh_write(void* opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    NUC970SdhState* sdh = (NUC970SdhState*)opaque;
    fprintf(stderr, "sdh_write(offset=%lx, value=%08lx)\n", offset, value);
    switch (offset)
    {
    case 0x400:
        sdh->SDH_DMACTL = value;
        break;
    case 0x800:
        sdh->SDH_GCTL = value;
        break;
    case 0x820:
        sdh->SDH_CTL = value;
        fprintf(stderr, "CMD_CODE: %02lx\n", (value >> 8) & 0x3f);
        if ((value & 0x01) && (value & 0x02))
        {
            nuc970_sdhost_send_command(sdh);
        }
        break;
    case 0x828:
        sdh->SDH_INTEN = value;
        break;
    case 0x82c:
        sdh->SDH_INTSTS = value;
        break;
    case 0x840:
        sdh->SDH_ECTL = value;
        break;
    
    default:
        break;
    }
}

static const MemoryRegionOps nuc970_sdh_ops = {
    .read = nuc970_sdh_read,
    .write = nuc970_sdh_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void nuc970_sdh_init(Object* obj)
{
    SysBusDevice* sd = SYS_BUS_DEVICE(obj);
    NUC970SdhState* sdh = NUC970_SDH(obj);
    DriveInfo* di;
    BlockBackend* blk;
    DeviceState* carddev;
    BusState* bus;

    memory_region_init_io(&sdh->iomem, OBJECT(sdh), &nuc970_sdh_ops, sdh,
        "nuc970-sdh", 0x1000);
    sysbus_init_mmio(sd, &sdh->iomem);

    sysbus_init_irq(sd, &sdh->irq);

    qbus_init(&sdh->sdbus, sizeof(sdh->sdbus), TYPE_SD_BUS,
        DEVICE(sd), "sd-bus");

    /* Retrieve SD bus */
    di = drive_get(IF_SD, 0, 0);
    blk = di ? blk_by_legacy_dinfo(di) : NULL;

    /* Plug in SD card */
    carddev = qdev_new(TYPE_SD_CARD);
    qdev_prop_set_drive_err(carddev, "drive", blk, &error_fatal);
    bus = qdev_get_child_bus(DEVICE(sd), "sd-bus");
    qdev_realize_and_unref(carddev, bus, &error_fatal);

    sdh->SDH_DMACTL = 0x00000000;
    sdh->SDH_GCTL = 0x00000000;
    sdh->SDH_CTL = 0x01010000;
    sdh->SDH_INTEN = 0x00000A00;
    sdh->SDH_INTSTS = 0x0000008C;
    sdh->SDH_ECTL = 0x00000003;

    /*
# devmem 0xb000c000
0x000081A4
# devmem 0xb000c004
0x00003CCF
# devmem 0xb000c008
0x41D60995

# devmem 0xb000c400
0x00000001
# devmem 0xb000c408
0x03A80400
# devmem 0xb000c40c
0x00000000
# devmem 0xb000c410
0x00000001
# devmem 0xb000c414
0x00000000
# devmem 0xb000c418
0x00000300
# devmem 0xb000c800
0x00000002
# devmem 0xb000c804
0x00000001
# devmem 0xb000c808
0x00000000
# devmem 0xb000c820
0x09018D80
# devmem 0xb000c824
0x00010000
# devmem 0xb000c828
0x40000100
# devmem 0xb000c82c
0x013E00AC
# devmem 0xb000c830
0x0D000009
# devmem 0xb000c834
0x00000000
# devmem 0xb000c838
0x000001FF
# devmem 0xb000c83c
0x0000FFFF
# devmem 0xb000c840
0x00000000
    */
}

static const TypeInfo nuc970_sdh_info = {
    .name = TYPE_NUC970_SDH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = nuc970_sdh_init,
    .instance_size = sizeof(NUC970SdhState),
};

static struct arm_boot_info nuc970_binfo = {
    .loader_start = 0x0,
    .board_id = 0x20e,
};

/* Register base address for each Timer Module */
static const hwaddr nuc970_tim_addr[] = {
    TMR0_BA,
    TMR1_BA,
    TMR2_BA,
    TMR3_BA,
    TMR4_BA
};

/* Register base address for each Timer Module */
static const int nuc970_tim_irq[] = {
    TMR0_IRQn,
    TMR1_IRQn,
    TMR2_IRQn,
    TMR3_IRQn,
    TMR4_IRQn
};

#define INIT_SHADOW_REGION 1

/*
static void nuc970_eeprom_init(I2CBus* bus, uint8_t addr, uint32_t rsize)
{
    I2CSlave* i2c_dev = i2c_slave_new("at24c-eeprom", addr);
    DeviceState* dev = DEVICE(i2c_dev);

    qdev_prop_set_uint32(dev, "rom-size", rsize);
    i2c_slave_realize_and_unref(i2c_dev, bus, &error_abort);
}
*/

static void nuc970_init(MachineState* machine)
{
    ARMCPU* cpu;
    DeviceState* dev;
    //DeviceState* pic;
    DeviceState* aic;
    //DeviceState* uart_orgate;
    DeviceState* i2c_dev;
    //DeviceState* lcd_dev;
    //DeviceState* key_dev;
    //DeviceState* uart[2];
    NPCM7xxTimerCtrlState tmr[1];

    //I2CSlave* wm8750_dev;
    SysBusDevice* s;
    //I2CBus* i2c;
    int i;
    //unsigned long flash_size;
    //DriveInfo* dinfo;
    MemoryRegion* ram_alias;
    MemoryRegion* sram_alias;

    MachineClass* mc = MACHINE_GET_CLASS(machine);
    MemoryRegion* address_space_mem = get_system_memory();
    MemoryRegion* sram = g_new(MemoryRegion, 1);
       

    /* For now we use a fixed - the original - RAM size */
    if (machine->ram_size != mc->default_ram_size) {
        char* sz = size_to_str(mc->default_ram_size);
        error_report("Invalid RAM size, should be %s", sz);
        g_free(sz);
        exit(EXIT_FAILURE);
    }

    cpu = ARM_CPU(cpu_create(machine->cpu_type));

    memory_region_add_subregion(address_space_mem, 0x00000000, machine->ram);
    memory_region_init_ram(sram, NULL, "nuc970.sram", MP_SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(address_space_mem, MP_SRAM_BASE, sram);

#if INIT_SHADOW_REGION
    ram_alias = g_new(MemoryRegion, 1);
    memory_region_init_alias(ram_alias, NULL, "nuc970.ram.alias", machine->ram, 0, MP_RAM_DEFAULT_SIZE);
    memory_region_add_subregion(address_space_mem, 0x80000000, ram_alias);
    sram_alias = g_new(MemoryRegion, 1);
    memory_region_init_alias(sram_alias, NULL, "nuc970.sram.alias", sram, 0, MP_SRAM_SIZE);
    memory_region_add_subregion(address_space_mem, MP_SRAM_BASE + 0x80000000, sram_alias);
#endif

    

    /*
    pic = sysbus_create_simple(TYPE_MV88W8618_PIC, MP_PIC_BASE,
        qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));
    sysbus_create_varargs(TYPE_MV88W8618_PIT, MP_PIT_BASE,
        qdev_get_gpio_in(pic, MP_TIMER1_IRQ),
        qdev_get_gpio_in(pic, MP_TIMER2_IRQ),
        qdev_get_gpio_in(pic, MP_TIMER3_IRQ),
        qdev_get_gpio_in(pic, MP_TIMER4_IRQ), NULL);
    */

    aic = sysbus_create_simple(TYPE_NUC970_AIC, AIC_BA, qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));

    /* Logically OR both UART IRQs together */
#if 0
    uart_orgate = DEVICE(object_new(TYPE_OR_IRQ));
    object_property_set_int(OBJECT(uart_orgate), "num-lines", 2, &error_fatal);
    qdev_realize_and_unref(uart_orgate, NULL, &error_fatal);
    qdev_connect_gpio_out(DEVICE(uart_orgate), 0,
        qdev_get_gpio_in(pic, MP_UART_SHARED_IRQ));

    serial_mm_init(address_space_mem, MP_UART1_BASE, 2,
        qdev_get_gpio_in(uart_orgate, 0),
        1825000, serial_hd(0), DEVICE_NATIVE_ENDIAN);
    serial_mm_init(address_space_mem, MP_UART2_BASE, 2,
        qdev_get_gpio_in(uart_orgate, 1),
        1825000, serial_hd(1), DEVICE_NATIVE_ENDIAN);

    /* Register flash */
    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (dinfo) {
        BlockBackend* blk = blk_by_legacy_dinfo(dinfo);

        flash_size = blk_getlength(blk);
        if (flash_size != 8 * 1024 * 1024 && flash_size != 16 * 1024 * 1024 &&
            flash_size != 32 * 1024 * 1024) {
            error_report("Invalid flash image size");
            exit(1);
        }

        /*
         * The original U-Boot accesses the flash at 0xFE000000 instead of
         * 0xFF800000 (if there is 8 MB flash). So remap flash access if the
         * image is smaller than 32 MB.
         */
        pflash_cfi02_register(0x100000000ULL - MP_FLASH_SIZE_MAX,
            "nuc970.flash", flash_size,
            blk, 0x10000,
            MP_FLASH_SIZE_MAX / flash_size,
            2, 0x00BF, 0x236D, 0x0000, 0x0000,
            0x5555, 0x2AAA, 0);
    }
    sysbus_create_simple(TYPE_MV88W8618_FLASHCFG, MP_FLASHCFG_BASE, NULL);

    qemu_check_nic_model(&nd_table[0], "mv88w8618");
    dev = qdev_new(TYPE_MV88W8618_ETH);
    qdev_set_nic_properties(dev, &nd_table[0]);
    object_property_set_link(OBJECT(dev), "dma-memory",
        OBJECT(get_system_memory()), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, MP_ETH_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
        qdev_get_gpio_in(pic, MP_ETH_IRQ));

    sysbus_create_simple("nuc970_wlan", MP_WLAN_BASE, NULL);

    sysbus_create_simple(TYPE_NUC970_MISC, MP_MISC_BASE, NULL);
#endif


    dev = sysbus_create_simple(TYPE_NUC970_GPIO, GPIO_BA, qdev_get_gpio_in(aic, GPIO_IRQn));

    i2c_dev = sysbus_create_simple(TYPE_NUC970_I2C, I2C0_BA, NULL);
    //i2c = (I2CBus*)qdev_get_child_bus(i2c_dev, "i2c");

    s = SYS_BUS_DEVICE(i2c_dev);
    sysbus_connect_irq(s, 0, qdev_get_gpio_in(aic, I2C0_IRQn));

    //nuc970_eeprom_init(i2c, 0x50, 32 * KiB);

    //lcd_dev = 
    sysbus_create_simple(TYPE_NUC970_LCD, LCM_BA, NULL);
#if 0
    key_dev = sysbus_create_simple(TYPE_NUC970_KEY, -1, NULL);

    /* I2C read data */
    qdev_connect_gpio_out(i2c_dev, 0,
        qdev_get_gpio_in(dev, MP_GPIO_I2C_DATA_BIT));
    /* I2C data */
    qdev_connect_gpio_out(dev, 3, qdev_get_gpio_in(i2c_dev, 0));
    /* I2C clock */
    qdev_connect_gpio_out(dev, 4, qdev_get_gpio_in(i2c_dev, 1));

    for (i = 0; i < 3; i++) {
        qdev_connect_gpio_out(dev, i, qdev_get_gpio_in(lcd_dev, i));
    }
    for (i = 0; i < 4; i++) {
        qdev_connect_gpio_out(key_dev, i, qdev_get_gpio_in(dev, i + 8));
    }
    for (i = 4; i < 8; i++) {
        qdev_connect_gpio_out(key_dev, i, qdev_get_gpio_in(dev, i + 15));
    }

    wm8750_dev = i2c_slave_create_simple(i2c, TYPE_WM8750, MP_WM_ADDR);
    dev = qdev_new(TYPE_MV88W8618_AUDIO);
    s = SYS_BUS_DEVICE(dev);
    object_property_set_link(OBJECT(dev), "wm8750", OBJECT(wm8750_dev),
        NULL);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, MP_AUDIO_BASE);
    sysbus_connect_irq(s, 0, qdev_get_gpio_in(pic, MP_AUDIO_IRQ));
#endif

    sysbus_create_simple(TYPE_NUC970_SYS, SYS_BA, NULL);
    sysbus_create_simple(TYPE_NUC970_CLK, CLK_BA, NULL);
    sysbus_create_simple(TYPE_NUC970_SDIC, SDIC_BA, NULL);
    sysbus_create_simple(TYPE_NUC970_RTC, RTC_BA, NULL);

//#define NUC970_TIMER2

#ifdef NUC970_TIMER2
    dev = qdev_new("nuc970-sys-timer");
    SysBusDevice* sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize(sbd, &error_abort);
    sysbus_mmio_map(sbd, 0, TMR0_BA);
    int j;
    for (j = 0; j < 5; j++) {
        //qemu_irq irq = npcm7xx_irq(s, first_irq + j);
        //sysbus_connect_irq(sbd, j, irq);
        sysbus_connect_irq(sbd, j, qdev_get_gpio_in(aic, nuc970_tim_irq[j]));
    }
#else
    {
        //dev = qdev_new(TYPE_NPCM7XX_TIMER);
        for (i = 0; i < ARRAY_SIZE(tmr); i++) {
            //object_initialize_child(OBJECT(machine), "tmr[*]", &tmr[i], TYPE_NPCM7XX_TIMER);

            dev = qdev_new(TYPE_NPCM7XX_TIMER);

            SysBusDevice* sbd = SYS_BUS_DEVICE(dev);
            //int first_irq;
            int j;

            /* Connect the timer clock. */
            //qdev_connect_clock_in(DEVICE(&tmr[i]), "clock", qdev_get_clock_out(
            //    DEVICE(&s->clk), "timer-clock"));

            sysbus_realize(sbd, &error_abort);
            sysbus_mmio_map(sbd, 0, nuc970_tim_addr[i]);

            for (j = 0; j < NPCM7XX_TIMERS_PER_CTRL; j++) {
                //qemu_irq irq = npcm7xx_irq(s, first_irq + j);
                //sysbus_connect_irq(sbd, j, irq);
                sysbus_connect_irq(sbd, j, qdev_get_gpio_in(aic, nuc970_tim_irq[j]));
            }         
                        
        }
    }
#endif
    
    /*** UARTs ***/
    //uart[0] = 
    nuc970_uart_create(UART0_BA, 64, 0, serial_hd(0),
        qdev_get_gpio_in(aic, UART0_IRQn));
    //uart[1] = 
    nuc970_uart_create(UART1_BA, 64, 1, serial_hd(1),
        qdev_get_gpio_in(aic, UART1_IRQn));

    nuc970_uart_create(UART5_BA, 64, 5, serial_hd(5), qdev_get_gpio_in(aic, UART5_IRQn));
    nuc970_uart_create(UART6_BA, 64, 6, serial_hd(6), qdev_get_gpio_in(aic, UART6_IRQn));
    nuc970_uart_create(UART7_BA, 64, 7, serial_hd(7), qdev_get_gpio_in(aic, UART7_IRQn));
    nuc970_uart_create(UART8_BA, 64, 8, serial_hd(8), qdev_get_gpio_in(aic, UART8_IRQn));
    nuc970_uart_create(UARTA_BA, 64, 10, serial_hd(10), qdev_get_gpio_in(aic, UART10_IRQn));

    /*** SPI ***/
    dev = qdev_new("nuc970-spi");
    qdev_prop_set_uint8(dev, "num-ss-bits", 2);

    s = SYS_BUS_DEVICE(dev);

    sysbus_realize(s, &error_abort);
    sysbus_mmio_map(s, 0, SPI0_BA);

    sysbus_connect_irq(s, 0, qdev_get_gpio_in(aic, SPI0_IRQn));
    
    sysbus_create_simple(TYPE_NUC970_WDT, WDT_BA, NULL);
    sysbus_create_simple("nuc970.rng", CRPT_BA, NULL);
    sysbus_create_simple(TYPE_NUC970_FMI, FMI_BA, NULL);

    dev = qdev_new(TYPE_NPCM7XX_EMC);
    s = SYS_BUS_DEVICE(dev);
    if (nd_table[0].used) {
        qemu_check_nic_model(&nd_table[0], TYPE_NPCM7XX_EMC);
        qdev_set_nic_properties(DEVICE(s), &nd_table[0]);
    }
    /*
     * The device exists regardless of whether it's connected to a QEMU
     * netdev backend. So always instantiate it even if there is no
     * backend.
     */
    sysbus_realize(s, &error_abort);
    sysbus_mmio_map(s, 0, EMC0_BA);
    //int tx_irq = i == 0 ? NPCM7XX_EMC1TX_IRQ : NPCM7XX_EMC2TX_IRQ;
    //int rx_irq = i == 0 ? NPCM7XX_EMC1RX_IRQ : NPCM7XX_EMC2RX_IRQ;
    /*
     * N.B. The values for the second argument sysbus_connect_irq are
     * chosen to match the registration order in npcm7xx_emc_realize.
     */
    sysbus_connect_irq(s, 0, qdev_get_gpio_in(aic, EMC0_TX_IRQn));
    sysbus_connect_irq(s, 1, qdev_get_gpio_in(aic, EMC0_RX_IRQn));
    

    // test shadow memory region
    /*
    AddressSpace* as = cpu_get_address_space(cpu, ARMASIdx_NS);
    address_space_stl_notdirty(as, 0x00000000, 0x01234567, MEMTXATTRS_UNSPECIFIED, NULL);
    address_space_stl_notdirty(as, 0x80000004, 0x89abcdef, MEMTXATTRS_UNSPECIFIED, NULL);
    address_space_stl_notdirty(as, 0x3c000000, 0x01234567, MEMTXATTRS_UNSPECIFIED, NULL);
    address_space_stl_notdirty(as, 0xbc000004, 0x89abcdef, MEMTXATTRS_UNSPECIFIED, NULL);
    */

    
    sysbus_create_simple(TYPE_NUC970_SDH, SDH_BA, NULL);
    
    create_unimplemented_device("nuc970.gdma", GDMA_BA, 0x1000);
    create_unimplemented_device("nuc970.ebi", EBI_BA, 0x800);
    create_unimplemented_device("nuc970.emc1", EMC1_BA, 0x1000);
    create_unimplemented_device("nuc970.ehci", USBH_BA, 0x1000);
    create_unimplemented_device("nuc970.usbd", USBD_BA, 0x1000);
    create_unimplemented_device("nuc970.ohci", USBO_BA, 0x1000);
    create_unimplemented_device("nuc970.i2s", ACTL_BA, 0x1000);
    create_unimplemented_device("nuc970.jpeg", JPEG_BA, 0x1000);
    create_unimplemented_device("nuc970.ge2d", GE_BA, 0x1000);
    create_unimplemented_device("nuc970.cap", CAP_BA, 0x1000);
    create_unimplemented_device("nuc970.etimer0", ETMR0_BA, 0x100);
    create_unimplemented_device("nuc970.etimer1", ETMR1_BA, 0x100);
    create_unimplemented_device("nuc970.etimer2", ETMR2_BA, 0x100);
    create_unimplemented_device("nuc970.etimer3", ETMR3_BA, 0x100);
    create_unimplemented_device("nuc970.wwdt", WWDT_BA, 0x100);
    create_unimplemented_device("nuc970.sc0", SC0_BA, 0x400);
    create_unimplemented_device("nuc970.sc1", SC1_BA, 0x400);
    create_unimplemented_device("nuc970.i2c1", I2C1_BA, 0x100);
    create_unimplemented_device("nuc970.spi1", SPI1_BA, 0x100);
    create_unimplemented_device("nuc970.pwm", PWM_BA, 0x1000);
    create_unimplemented_device("nuc970.kpi", 0xb8008000, 0x1000);
    create_unimplemented_device("nuc970.adc", ADC_BA, 0x1000);
    create_unimplemented_device("nuc970.can0", CAN0_BA, 0x400);
    create_unimplemented_device("nuc970.can1", CAN1_BA, 0x400);
    create_unimplemented_device("nuc970.mtp", MTP_BA, 0x1000);

    /* If the user specified a -bios image, we put it to 0xe00000 and bypass
     * the normal Linux boot process. 
     */
    if (machine->firmware) {
        hwaddr firmware_addr = 0xe00000;
        /* load the firmware image */
        int r = load_image_targphys(machine->firmware, firmware_addr,
            MP_RAM_DEFAULT_SIZE - firmware_addr);
        if (r < 0) {
            error_report("Failed to load firmware from %s", machine->firmware);
            exit(1);
        }

        nuc970_binfo.entry = firmware_addr;
        nuc970_binfo.firmware_loaded = true;
    }

    nuc970_binfo.ram_size = MP_RAM_DEFAULT_SIZE;
    arm_load_kernel(cpu, machine, &nuc970_binfo);

    info_report("Loading NUC970 kernel...\n");
}

static void nuc970_machine_init(MachineClass* mc)
{
    mc->desc = "NUC970 (ARM926EJ-S)";
    mc->init = nuc970_init;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
    mc->default_ram_size = MP_RAM_DEFAULT_SIZE;
    mc->default_ram_id = "nuc970.ram";
}

DEFINE_MACHINE("nuc970", nuc970_machine_init)

static void mv88w8618_wlan_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->realize = mv88w8618_wlan_realize;
}

static const TypeInfo mv88w8618_wlan_info = {
    .name = "nuc970_wlan",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SysBusDevice),
    .class_init = mv88w8618_wlan_class_init,
};

static void nuc970_register_types(void)
{
    type_register_static(&mv88w8618_pic_info);
    type_register_static(&mv88w8618_pit_info);
    type_register_static(&mv88w8618_flashcfg_info);
    type_register_static(&mv88w8618_wlan_info);
    type_register_static(&nuc970_lcd_info);
    type_register_static(&nuc970_gpio_info);
    type_register_static(&nuc970_key_info);
    type_register_static(&nuc970_misc_info);
    type_register_static(&nuc970_sys_info);
    type_register_static(&nuc970_sdic_info);
    type_register_static(&nuc970_clk_info);
    type_register_static(&nuc970_wdt_info);
    type_register_static(&nuc970_rtc_info);
    type_register_static(&nuc970_fmi_info);
    type_register_static(&nuc970_sdh_info);
}

type_init(nuc970_register_types)