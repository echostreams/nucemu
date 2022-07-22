/*
 * NUC970 Advanced Interrupt Controller
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 *
 * TODO: implement vectors.
 */

#ifndef NUC970_AIC_H
#define NUC970_AIC_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_NUC970_AIC "nuc970.aic"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970AicState, NUC970_AIC)

#define NUC970_AIC_NUM_IRQS 61

/* Interrupt Control Bits */
#define ABFLAG (1<<25)
#define ABFEN  (1<<24)
#define NIDIS  (1<<22) /* Normal Interrupt disable */
#define FIDIS  (1<<21) /* Fast interrupt disable */
#define NIAD   (1<<20) /* Normal Interrupt Arbiter Rise ARM level */
#define FIAD   (1<<19) /* Fast Interrupt Arbiter Rise ARM level */
#define NM     (1<<18) /* Normal interrupt mode */

#define PRIO_PER_WORD (sizeof(uint32_t) * 8 / 4)
#define PRIO_WORDS (NUC970_AIC_NUM_IRQS/PRIO_PER_WORD)

struct NUC970AicState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    uint64_t pending;
    uint64_t enabled;
    uint64_t is_fiq;
    uint32_t intcntl;
    uint32_t intmask;
    qemu_irq irq;
    qemu_irq fiq;
    uint32_t prio[PRIO_WORDS]; /* Priorities are 4-bits each */
};

#endif /* IMX_AVIC_H */
