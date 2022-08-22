
#ifndef NUC970_ETIMER_H
#define NUC970_ETIMER_H

#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "qom/object.h"

/** @addtogroup NUC970_ETIMER_EXPORTED_CONSTANTS ETIMER Exported Constants
  @{
*/

#define ETIMER_ONESHOT_MODE                      (0UL)          /*!< Timer working in one shot mode   */
#define ETIMER_PERIODIC_MODE                     (1UL << 4)     /*!< Timer working in periodic mode   */
#define ETIMER_TOGGLE_MODE                       (2UL << 4)     /*!< Timer working in toggle mode     */
#define ETIMER_CONTINUOUS_MODE                   (3UL << 4)     /*!< Timer working in continuous mode */

#define ETIMER_CAPTURE_FREE_COUNTING_MODE        (0UL)          /*!< Free counting mode    */
#define ETIMER_CAPTURE_TRIGGER_COUNTING_MODE     (1UL << 20)    /*!< Trigger counting mode */
#define ETIMER_CAPTURE_COUNTER_RESET_MODE        (1UL << 17)    /*!< Counter reset mode    */

#define ETIMER_CAPTURE_FALLING_EDGE              (0UL)          /*!< Falling edge trigger timer capture */
#define ETIMER_CAPTURE_RISING_EDGE               (1UL << 18)    /*!< Rising edge trigger timer capture  */
#define ETIMER_CAPTURE_FALLING_THEN_RISING_EDGE  (2UL << 18)    /*!< Falling edge then rising edge trigger timer capture */
#define ETIMER_CAPTURE_RISING_THEN_FALLING_EDGE  (3UL << 18)    /*!< Rising edge then falling edge trigger timer capture */


/*@}*/ /* end of group NUC970_ETIMER_EXPORTED_CONSTANTS */

#define TYPE_NUC970_ETIMER "nuc970-etimer"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970ETimerState, NUC970_ETIMER)

struct NUC970ETimerState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    QEMUTimer timer;
    int64_t     expires_ns;
    int64_t     remaining_ns;
    qemu_irq irq;
    Clock* clock;
    uint32_t ctl;       // 0x00
    uint32_t precnt;    // 0x04
    uint32_t cmpr;      // 0x08
    uint32_t ier;       // 0x0c
    uint32_t isr;       // 0x10
    uint32_t dr;        // 0x14
    uint32_t tcap;      // 0x18
};

#endif