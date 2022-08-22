
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/timer/nuc970_timer2.h"
#include "hw/registerfields.h"
#include "migration/vmstate.h"
#include "trace.h"
#include "hw/qdev-clock.h"

#ifndef DEBUG_NUC970_ETMR
#define DEBUG_NUC970_ETMR 0
#endif

#define DPRINTF(fmt, args...) \
    do { \
        if (DEBUG_NUC970_ETMR) { \
            fprintf(stderr, "[%s]%s: " fmt , TYPE_NUC970_ETIMER, \
                                             __func__, ##args); \
        } \
    } while (0)

REG32(CTRL_STATUS, 0x00)
REG32(COUNTER_LOW, 0x04)
REG32(COUNTER_HIGH, 0x08)
REG32(COMPARE0, 0x0c)
REG32(COMPARE1, 0x10)
REG32(COMPARE2, 0x14)
REG32(COMPARE3, 0x18)

/* Convert a timer cycle count to a time interval in nanoseconds. */
static int64_t nuc970_etimer_count_to_ns(NUC970ETimerState* t, uint32_t count)
{
    int64_t ticks = count;
    uint64_t ns_low, ns_high;

    ticks *= (t->precnt & 0xff + 1);

    //fprintf(stderr, "timer count to ns: ticks %ld, period %ld\n", ticks, t->clock->period);

    //return clock_ticks_to_ns(t->ctrl->clock, ticks);

    /*
     * clk->period is the period in units of 2^-32 ns, so
     * (clk->period * ticks) is the required length of time in those
     * units, and we can convert to nanoseconds by multiplying by
     * 2^32, which is the same as shifting the 128-bit multiplication
     * result right by 32.
     */
     //12 MHz = 83.333333333333 ns(p)
     //clock->period: 4294967296 × 83.3333333333333 = 357,913,941,333
    mulu64(&ns_low, &ns_high, t->clock->period, ticks);
    if (ns_high & MAKE_64BIT_MASK(31, 33)) {
        return INT64_MAX;
    }
    return ns_low >> 32 | ns_high << 32;

}

/* Convert a time interval in nanoseconds to a timer cycle count. */
static uint32_t nuc970_etimer_ns_to_count(NUC970ETimerState* t, int64_t ns)
{
    return clock_ns_to_ticks(t->clock, ns) / (t->precnt + 1);
}

/*
 * Raise the interrupt line if there's a pending interrupt and interrupts are
 * enabled for this timer. If not, lower it.
 */
static void nuc970_etimer_check_interrupt(NUC970ETimerState* t)
{
    bool pending = (t->isr) & (t->ier);
    qemu_set_irq(t->irq, pending ? 1 : 0);
}

/* Start or resume the timer. */
static void nuc970_etimer_start(NUC970ETimerState* t)
{
    int64_t now;

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    t->expires_ns = now + t->remaining_ns;

    DPRINTF("etmr start: now %d, remaining_ns %d, expires_ns %d\n", now, t->remaining_ns, t->expires_ns);
    timer_mod(&t->timer, t->expires_ns);
}

/* Stop counting. Record the time remaining so we can continue later. */
static void nuc970_etimer_pause(NUC970ETimerState* t)
{
    int64_t now;

    timer_del(&t->timer);
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    t->remaining_ns = t->expires_ns - now;
}

/* Delete the timer and reset it to default state. */
static void nuc970_etimer_clear(NUC970ETimerState* t)
{
    timer_del(&t->timer);
    t->expires_ns = 0;
    t->remaining_ns = 0;
}

/*
 * Restart the timer from its initial value. If the timer was enabled and stays
 * enabled, adjust the QEMU timer according to the new count. If the timer is
 * transitioning from disabled to enabled, the caller is expected to start the
 * timer later.
 */
static void nuc970_etimer_restart(NUC970ETimerState* t, uint32_t old_tcsr)
{
    t->remaining_ns = nuc970_etimer_count_to_ns(t, t->cmpr);

    //fprintf(stderr, "timer_restart: ticr %d, remaining_ns %ld\n", t->ticr, t->base_timer.remaining_ns);

    if (old_tcsr & t->ctl & (1 << 0)) {
        nuc970_etimer_start(t);
    }
}

/*
 * Called when the counter reaches zero. Sets the interrupt flag, and either
 * restarts or disables the timer.
 */
static void nuc970_etimer_reached_zero(NUC970ETimerState* t)
{
    if (t->ctl & ETIMER_PERIODIC_MODE) {
        t->remaining_ns = nuc970_etimer_count_to_ns(t, t->dr);
        if (t->ctl & (1 << 0)) {
            nuc970_etimer_start(t);
        }
        t->isr |= BIT(0);
    }
    else {
        t->ctl &= ~(1 << 0 | 1 << 1);
    }

    nuc970_etimer_check_interrupt(t);
}

static void nuc970_etmr_timer_expire(void* opaque)
{
    NUC970ETimerState* t = opaque;
    if (t->ctl & (1 << 0)) {
        nuc970_etimer_reached_zero(t);
    }
    
}

static uint64_t nuc970_etmr_read(void* opaque, hwaddr offset,
    unsigned size)
{
    NUC970ETimerState* s = NUC970_ETIMER(opaque);
    uint64_t r = 0;

    switch (offset) {
    case 0:
        r = s->ctl;
        break;
    case 4:
        r = s->precnt;
        break;
    case 8:
        r = s->cmpr;
        break;
    case 0x0c:
        r = s->ier;
        break;
    case 0x10:
        r = s->isr;
        break;
    case 0x14:
        if (s->ctl & (1 << 0)) {
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

            r = nuc970_etimer_ns_to_count(s, s->expires_ns - now);
            //fprintf(stderr, "timer_read_tdr: %d, now: %ld, exp: %ld, %ld\n", r, now, 
            //    t->base_timer.expires_ns, t->base_timer.expires_ns - now);
            r = (s->cmpr & 0xffffff) - r;
            
        }
        else
            r = nuc970_etimer_ns_to_count(s, s->remaining_ns);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%" HWADDR_PRIx "\n",
            __func__, offset);
        break;
    }
    trace_bcm2835_systmr_read(offset, r);

    DPRINTF("etmr read  [0x%" HWADDR_PRIx "] -> 0x%02x\n",
        offset, r);

    return r;
}

static void nuc970_etmr_write(void* opaque, hwaddr offset,
    uint64_t value64, unsigned size)
{
    NUC970ETimerState* s = NUC970_ETIMER(opaque);
    //int index;
    uint32_t value = value64;
    //uint32_t triggers_delay_us;
    uint64_t now;

    trace_bcm2835_systmr_write(offset, value);

    DPRINTF("write [0x%" HWADDR_PRIx "] <- 0x%02x\n",
        offset, (int)value);

    switch (offset) {
    case 0:
        s->ctl = value;
        if (value & BIT(0))
            nuc970_etimer_start(s);
        break;
    case 4:
        s->precnt = value;
        break;
    case 8:
        s->cmpr = value;
        s->remaining_ns = nuc970_etimer_count_to_ns(s, s->cmpr);
        if (s->ctl & BIT(0)) {
            nuc970_etimer_start(s);
        }
        break;
    case 0x0c:
        s->ier = value;
        break;
    case 0x10:
        if (value & BIT(0))
            value &= ~BIT(0);
        s->isr = value;
        nuc970_etimer_check_interrupt(s);
        break;
    case 0x14:
    case 0x18:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read-only ofs 0x%" HWADDR_PRIx "\n",
            __func__, offset);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%" HWADDR_PRIx "\n",
            __func__, offset);
        break;
    }
}

static const MemoryRegionOps nuc970_etmr_ops = {
    .read = nuc970_etmr_read,
    .write = nuc970_etmr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void nuc970_etmr_reset(DeviceState* dev)
{
    NUC970ETimerState* s = NUC970_ETIMER(dev);
    s->expires_ns = 0;
    s->remaining_ns = 0;
    /*
    uint32_t ctl;       // 0x00
    uint32_t precnt;    // 0x04
    uint32_t cmpr;      // 0x08
    uint32_t ier;       // 0x0c
    uint32_t isr;       // 0x10
    uint32_t dr;        // 0x14
    uint32_t tcap;      // 0x18
    */
}

static void nuc970_etmr_realize(DeviceState* dev, Error** errp)
{
    NUC970ETimerState* s = NUC970_ETIMER(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &nuc970_etmr_ops,
        s, "nuc970-etimer", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem); 

    s->clock = qdev_init_clock_in(dev, "clock", NULL, NULL, 0);

    clock_set_hz(s->clock, 12 * 1000 * 1000); /* 12MHz */
    //clock_set_hz(s->clock, 1 * 1000 * 1000); /* 1MHz */

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL,
            nuc970_etmr_timer_expire, s);
    
}

static const VMStateDescription nuc970_etmr_vmstate = {
    .name = "nuc970_etimer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_CLOCK(clock, NUC970ETimerState),
        VMSTATE_UINT32(ctl, NUC970ETimerState),
        VMSTATE_UINT32(precnt, NUC970ETimerState),
        VMSTATE_UINT32(cmpr, NUC970ETimerState),
        VMSTATE_UINT32(ier, NUC970ETimerState),
        VMSTATE_UINT32(isr, NUC970ETimerState),
        VMSTATE_UINT32(dr, NUC970ETimerState),
        VMSTATE_UINT32(tcap, NUC970ETimerState),
        VMSTATE_END_OF_LIST()
    }
};

static void nuc970_etmr_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->desc = "NUC970 Enhanced Timer Controller";
    dc->realize = nuc970_etmr_realize;
    dc->reset = nuc970_etmr_reset;
    dc->vmsd = &nuc970_etmr_vmstate;
}

static const TypeInfo nuc970_etmr_info = {
    .name = TYPE_NUC970_ETIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC970ETimerState),
    .class_init = nuc970_etmr_class_init,
};

static void nuc970_etmr_register_types(void)
{
    type_register_static(&nuc970_etmr_info);
}

type_init(nuc970_etmr_register_types);