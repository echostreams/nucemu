
#ifndef NUC970_SYSTIMER_H
#define NUC970_SYSTIMER_H

#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_NUC970_SYSTIMER "nuc970-sys-timer"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970SystemTimerState, NUC970_SYSTIMER)

#define NUC970_SYSTIMER_COUNT 5

typedef struct {
    unsigned id;
    QEMUTimer timer;
    qemu_irq irq;
    NUC970SystemTimerState* state;
} NUC970SystemTimerCompare;

struct NUC970SystemTimerState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    struct {
        uint32_t ctrl_status;
        uint32_t compare[NUC970_SYSTIMER_COUNT];
    } reg;
    NUC970SystemTimerCompare tmr[NUC970_SYSTIMER_COUNT];
};

#endif