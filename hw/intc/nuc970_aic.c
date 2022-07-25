/*
 * NUC970 Advanced Interrupt Controller
 * 
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 *
 * TODO: implement vectors.
 */

#include "qemu/osdep.h"
#include "hw/intc/nuc970_aic.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#ifndef DEBUG_NUC970_AIC
#define DEBUG_NUC970_AIC 0
#endif

#define DPRINTF(fmt, args...) \
    do { \
        if (DEBUG_NUC970_AIC) { \
            fprintf(stderr, "[%s]%s: " fmt , TYPE_NUC970_AIC, \
                                             __func__, ##args); \
        } \
    } while (0)

static const VMStateDescription vmstate_nuc970_aic = {
    .name = TYPE_NUC970_AIC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT64(pending, NUC970AicState),
        VMSTATE_UINT64(enabled, NUC970AicState),
        VMSTATE_UINT64(is_fiq, NUC970AicState),
        VMSTATE_UINT32(intcntl, NUC970AicState),
        VMSTATE_UINT32(intmask, NUC970AicState),
        VMSTATE_UINT32_ARRAY(prio, NUC970AicState, PRIO_WORDS),
        VMSTATE_UINT64(current, NUC970AicState),
        VMSTATE_END_OF_LIST()
    },
};

static inline int nuc970_aic_prio(NUC970AicState* s, int irq)
{
    int idx = irq / 4;
    int shift = (irq % 4) * 8;
    //int nIntType = (s->scr[idx] >> shift) & 0xc0;
    int nIntLevel = (s->scr[idx] >> shift) & 0x07;
    return nIntLevel;
}

/* Update interrupts.  */
static void nuc970_aic_update(NUC970AicState* s)
{
    int i;
    uint64_t new = s->pending & s->enabled;
    uint64_t flags;

    DPRINTF("update irq(pending=%" HWADDR_PRIx ", current=%" HWADDR_PRIx ", enabled=%" HWADDR_PRIx ", is_fiq=%" HWADDR_PRIx ")\n",
        s->pending, s->current, s->enabled, s->is_fiq);

    flags = new & s->is_fiq;
    qemu_set_irq(s->fiq, !!flags);
    if (flags)
        s->oisr |= 1;
    else
        s->oisr &= ~1;

    flags = new & ~s->is_fiq;
    if (!flags/* || (s->intmask == 0x1f)*/) {
        s->oisr &= ~(1 << 1);
        qemu_set_irq(s->irq, !!flags);
        return;
    }

    /*
     * Take interrupt if there's a pending interrupt with
     * priority higher than the value of intmask
     */
    for (i = 1; i < NUC970_AIC_NUM_IRQS + 1; i++) {
        if (flags & (1UL << i)) {
            //if (nuc970_aic_prio(s, i) > s->intmask) {
                s->current |= 1UL << i;
                s->oisr |= 1 << 1;
                DPRINTF("set IRQ %d\n", i);
                qemu_set_irq(s->irq, 1);
                return;
            //}
        }
    }
    s->oisr &= ~(1 << 1);
    qemu_set_irq(s->irq, 0);
    
}

static void nuc970_aic_set_irq(void* opaque, int irq, int level)
{
    NUC970AicState* s = (NUC970AicState*)opaque;

    if (level) {
        DPRINTF("Raising IRQ %d, prio %d\n",
            irq, nuc970_aic_prio(s, irq));
        s->pending |= (1ULL << irq);
    }
    else {
        //DPRINTF("Clearing IRQ %d, prio %d\n", irq, nuc970_aic_prio(s, irq));
        s->pending &= ~(1ULL << irq);
    }

    nuc970_aic_update(s);
}

static int nuc970_aic_get_irq(NUC970AicState* s) {
    /*
     * This returns the highest priority
     * outstanding interrupt.  Where there is more than
     * one pending IRQ with the same priority,
     * take the highest numbered one.
     */
    uint64_t flags = s->current & s->enabled & ~s->is_fiq;
    int i;
    int prio = 7;
    int irq = 0;
    for (i = 1; i <= NUC970_AIC_NUM_IRQS; i++) {
        if (flags & (1ULL << i)) {
            /*
            int irq_prio = nuc970_aic_prio(s, i);
            if (irq_prio < prio) {
                irq = i;
                prio = irq_prio;
            }
            */
            irq = i;
            return irq;
        }
    }
    DPRINTF("get irq(pending=%" HWADDR_PRIx ", current=%" HWADDR_PRIx ", enabled=%" HWADDR_PRIx ", is_fiq=%" HWADDR_PRIx ")\n",
        s->pending, s->current, s->enabled, s->is_fiq);
    return irq;
}

static uint64_t nuc970_aic_read(void* opaque,
    hwaddr offset, unsigned size)
{
    NUC970AicState* s = (NUC970AicState*)opaque;

    DPRINTF("read(offset = 0x%" HWADDR_PRIx ")\n", offset);

    switch (offset >> 2) {
    case 0: 
    case 1:
    case 2:
    case 3:
    case 4:
    case 5: 
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
        return s->scr[(offset >> 2)];

    case 0x100/4:   // Interrupt Raw Status
        return s->pending & 0xffffffffULL;
    case 0x104/4:   // Interrupt Raw Status (High)
        return s->pending >> 32;
    case 0x108/4:   // Interrupt Active Status Register
        return s->current & 0xffffffffULL;
    case 0x10c/4:   // Interrupt Active Status Register (High)
        return s->current >> 32;
    case 0x110/4:   // Interrupt Status Register
        return (s->current & s->enabled) & 0xffffffffULL;
    case 0x114/4:   // Interrupt Status Register (High)
        return (s->current & s->enabled) >> 32;
    case 0x118/4:   // IPER
        return nuc970_aic_get_irq(s) << 2;
    case 0x120/4:
        return nuc970_aic_get_irq(s);
    case 0x124/4:   // Output Interrupt Status Register
        return s->oisr;
    case 0x128/4:
        return s->enabled & 0xffffffffULL;
    case 0x12c/4:
        return s->enabled >> 32;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "[%s]%s: Bad register at offset 0x%"
            HWADDR_PRIx "\n", TYPE_NUC970_AIC, __func__, offset);
        return 0;
    }
}

static void nuc970_aic_write(void* opaque, hwaddr offset,
    uint64_t val, unsigned size)
{
    NUC970AicState* s = (NUC970AicState*)opaque;

    DPRINTF("(0x%" HWADDR_PRIx ") = 0x%x\n", offset, (unsigned int)val);

    switch (offset >> 2) {
    case 0: 
    case 1: 
    case 2:
    case 3: 
    case 4:
    case 5: 
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
        s->scr[(offset >> 2)] = val;
        break;

    case 0x130/4:
        //DPRINTF(" MECR : %08x\n", val);
        s->enabled = (s->enabled & 0xffffffff00000000ULL) | val;
        break;
    case 0x134/4:
        //DPRINTF(" MECRH: %08x\n", val);
        s->enabled = (s->enabled & 0xffffffffULL) | (val << 32);
        break;
    case 0x138/4:
        s->enabled = s->enabled & ~val;
        break;
    case 0x13c/4:
        s->enabled = s->enabled & ~(val << 32);
        break;
    case 0x150/4:
    {
        int irq = nuc970_aic_get_irq(s);
        if (irq > 0)
            s->current &= ~(1 << irq);
    }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "[%s]%s: Bad register at offset 0x%"
            HWADDR_PRIx "\n", TYPE_NUC970_AIC, __func__, offset);
    }
    nuc970_aic_update(s);
}

static const MemoryRegionOps nuc970_aic_ops = {
    .read = nuc970_aic_read,
    .write = nuc970_aic_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void nuc970_aic_reset(DeviceState* dev)
{
    NUC970AicState* s = NUC970_AIC(dev);

    s->pending = 0;
    s->enabled = 0;
    s->is_fiq = 0;
    s->intmask = 0x1f;
    s->intcntl = 0;
    s->current = 0;
    s->oisr = 0;
    memset(s->prio, 0, sizeof s->prio);
    memset(s->scr, 0, sizeof s->scr);
    for (int i = 1; i <= NUC970_AIC_NUM_IRQS; i++) {
        int idx = i / 4;
        int shift = (i % 4) * 8;
        s->scr[idx] |= (IRQ_LEVEL_7 | HIGH_LEVEL_SENSITIVE) << shift;
    }
}

static void nuc970_aic_init(Object* obj)
{
    DeviceState* dev = DEVICE(obj);
    NUC970AicState* s = NUC970_AIC(obj);
    SysBusDevice* sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &nuc970_aic_ops, s,
        TYPE_NUC970_AIC, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_in(dev, nuc970_aic_set_irq, NUC970_AIC_NUM_IRQS);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
}


static void nuc970_aic_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_nuc970_aic;
    dc->reset = nuc970_aic_reset;
    dc->desc = "NUC970 Advanced Interrupt Controller";
}

static const TypeInfo nuc970_aic_info = {
    .name = TYPE_NUC970_AIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC970AicState),
    .instance_init = nuc970_aic_init,
    .class_init = nuc970_aic_class_init,
};

static void nuc970_aic_register_types(void)
{
    type_register_static(&nuc970_aic_info);
}

type_init(nuc970_aic_register_types)