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

#define NUC970_AIC_NUM_IRQS 61 /* 1 ~ 61 */

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

/* The parameters for sysSetInterruptType() use */
/* AIC_SCR1 ~ AIC_SCR16 [7:6] TYPE */
#define LOW_LEVEL_SENSITIVE        0x00
#define HIGH_LEVEL_SENSITIVE       0x40
#define NEGATIVE_EDGE_TRIGGER      0x80
#define POSITIVE_EDGE_TRIGGER      0xC0

/* The parameters for sysSetInterruptPriorityLevel() and
   sysInstallISR() use */
#define FIQ_LEVEL_0     0       /*!< FIQ Level 0 */
#define IRQ_LEVEL_1     1       /*!< IRQ Level 1 */
#define IRQ_LEVEL_2     2       /*!< IRQ Level 2 */
#define IRQ_LEVEL_3     3       /*!< IRQ Level 3 */
#define IRQ_LEVEL_4     4       /*!< IRQ Level 4 */
#define IRQ_LEVEL_5     5       /*!< IRQ Level 5 */
#define IRQ_LEVEL_6     6       /*!< IRQ Level 6 */
#define IRQ_LEVEL_7     7       /*!< IRQ Level 7 */

/*
AIC_BA=0xB800_2000
AIC_SCR1...16   0x0...0x03C R/W AIC Source Control Register 1...16 0x4747_4747
AIC_IRSR        0x100 R AIC Interrupt Raw Status Register 0x0000_0000
AIC_IRSRH       0x104 R AIC Interrupt Raw Status Register (High) 0x0000_0000
                        0 = Interrupt channel is in the voltage level 0.
                        1 = Interrupt channel is in the voltage level 1.
AIC_IASR        0x108 R AIC Interrupt Active Status Register 0x0000_0000
AIC_IASRH       0x10C R AIC Interrupt Active Status Register (High) 0x0000_0000
                        This register indicates the status of each interrupt channel in consideration of the interrupt source type
                        as defined in the corresponding Source Control Register, but regardless of its mask setting.
                        0 = Inactive
                        1 = Active
AIC_ISR         0x110 R AIC Interrupt Status Register 0x0000_0000
AIC_ISRH        0x114 R AIC Interrupt Status Register (High) 0x0000_0000
                        0 = Inactive or active but not enabled
                        1 = Both active and enabled
AIC_IPER        0x118 R AIC Interrupt Priority Encoding Register 0x0000_0000
AIC_ISNR        0x120 R AIC Interrupt Source Number Register 0x0000_0000
AIC_OISR        0x124 R AIC Output Interrupt Status Register 0x0000_0000
                        This register indicates whether the asserted interrupt is FIQ or IRQ. 
                        If both IRQ and FIQ are equal to 0, it means there is no interrupt occurred.
AIC_IMR         0x128 R AIC Interrupt Mask Register 0x0000_0000
AIC_IMRH        0x12C R AIC Interrupt Mask Register (High) 0x0000_0000

AIC_MECR        0x130 W AIC Mask Enable Command Register Undefined
AIC_MECRH       0x134 W AIC Mask Enable Command Register (High) Undefined
AIC_MDCR        0x138 W AIC Mask Disable Command Register Undefined
AIC_MDCRH       0x13C W AIC Mask Disable Command Register (High) Undefined

AIC_SSCR        0x140 W AIC Source Set Command Register Undefined
AIC_SSCRH       0x144 W AIC Source Set Command Register (High) Undefined
AIC_SCCR        0x148 W AIC Source Clear Command Register Undefined
AIC_SCCRH       0x14C W AIC Source Clear Command Register (High) Undefined
                        When the chip is under debugging or verification, 
                        software can activate/deactivate any interrupt channel by setting the corresponding bit. 
                        This feature is useful in hardware verification or software debugging.
AIC_EOSCR       0x150 W AIC End of Service Command Register Undefined
*/

struct NUC970AicState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    uint64_t raw_status; /* IRSR: the intrinsic state within each interrupt channel. */
    uint64_t scr_type;   /* 1=high 0=low */
    uint64_t level_sensitive;   /* 1=level sensitive, 0=edge trigger */
    uint64_t enabled;    /* IMR: Interrupt Mask Register */
    uint64_t is_fiq;    /* promoted to fiq */

    uint8_t vector;     /* represents the interrupt channel number that is 
                        active, enabled, and has the highest priority. */

    uint8_t  oisr;      /* Output Interrupt Status Register */
    uint32_t scr[16];   /* Source Control Register 1...16 */

    qemu_irq irq;
    qemu_irq fiq;
    
};

#endif /* IMX_AVIC_H */
