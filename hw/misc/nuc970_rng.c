/*
 *  Exynos4210 Pseudo Random Nubmer Generator Emulation
 *
 *  Copyright (c) 2017 Krzysztof Kozlowski <krzk@kernel.org>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/guest-random.h"
#include "qemu/module.h"
#include "qom/object.h"

#define DEBUG_NUC970_RNG 1

#define DPRINTF(fmt, ...) \
    do { \
        if (DEBUG_NUC970_RNG) { \
            printf("nuc970_rng: " fmt, ## __VA_ARGS__); \
        } \
    } while (0)

#define TYPE_NUC970_RNG             "nuc970.rng"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970RngState, NUC970_RNG)

/*
 * Exynos4220, PRNG, only polling mode is supported.
 */

 /* RNG_CONTROL_1 register bitfields, reset value: 0x0 */
#define NUC970_RNG_CONTROL_1_PRNG           0x8
#define NUC970_RNG_CONTROL_1_START_INIT     BIT(4)
/* RNG_STATUS register bitfields, reset value: 0x1 */
#define NUC970_RNG_STATUS_PRNG_ERROR        BIT(7)
#define NUC970_RNG_STATUS_PRNG_DONE         BIT(5)
#define NUC970_RNG_STATUS_MSG_DONE          BIT(4)
#define NUC970_RNG_STATUS_PARTIAL_DONE      BIT(3)
#define NUC970_RNG_STATUS_PRNG_BUSY         BIT(2)
#define NUC970_RNG_STATUS_SEED_SETTING_DONE BIT(1)
#define NUC970_RNG_STATUS_BUFFER_READY      BIT(0)
#define NUC970_RNG_STATUS_WRITE_MASK   (NUC970_RNG_STATUS_PRNG_DONE \
                                           | NUC970_RNG_STATUS_MSG_DONE \
                                           | NUC970_RNG_STATUS_PARTIAL_DONE)

#define NUC970_RNG_CONTROL_1                  0x0
#define NUC970_RNG_STATUS                    0x8
#define NUC970_RNG_SEED_IN                  0x140
#define NUC970_RNG_SEED_IN_OFFSET(n)   (NUC970_RNG_SEED_IN + (n * 0x4))
#define NUC970_RNG_PRNG                     0x10
#define NUC970_RNG_PRNG_OFFSET(n)      (NUC970_RNG_PRNG + (n * 0x4))

#define NUC970_RNG_PRNG_NUM                 8

#define NUC970_RNG_REGS_MEM_SIZE            0x200

struct NUC970RngState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    int32_t randr_value[NUC970_RNG_PRNG_NUM];
    /* bits from 0 to NUC970_RNG_PRNG_NUM if given seed register was set */
    uint32_t seed_set;

    bool busy;		// 8	
    int keysz;		// 2-3
    bool reload;	// 1
    bool start;		// 0

    /* Register values */
    uint32_t reg_control;
    uint32_t reg_status;
};

static bool exynos4210_rng_seed_ready(const NUC970RngState* s)
{
    uint32_t mask = MAKE_64BIT_MASK(0, NUC970_RNG_PRNG_NUM);

    /* Return true if all the seed-set bits are set. */
    return (s->seed_set & mask) == mask;
}

static void exynos4210_rng_set_seed(NUC970RngState* s, unsigned int i,
    uint64_t val)
{
    /*
     * We actually ignore the seed and always generate true random numbers.
     * Theoretically this should not match the device as Exynos has
     * a Pseudo Random Number Generator but testing shown that it always
     * generates random numbers regardless of the seed value.
     */
    s->seed_set |= BIT(i);

    /* If all seeds were written, update the status to reflect it */
    if (exynos4210_rng_seed_ready(s)) {
        s->reg_status |= NUC970_RNG_STATUS_SEED_SETTING_DONE;
    }
    else {
        s->reg_status &= ~NUC970_RNG_STATUS_SEED_SETTING_DONE;
    }
}

static int getKeySize(NUC970RngState* s)
{
    switch (s->keysz)
    {
    case 0:
        return 8;
    case 1:
        return 16;
    case 2:
        return 24;
    case 3:
        return 32;
    default:
        return 8;
    }    
}

static void exynos4210_rng_run_engine(NUC970RngState* s)
{
    Error* err = NULL;
    /* Seed set? */
    //if ((s->reg_status & NUC970_RNG_STATUS_SEED_SETTING_DONE) == 0) {
    //    goto out;
    //}

    /* PRNG engine chosen? */
    //if ((s->reg_control & NUC970_RNG_CONTROL_1_PRNG) == 0) {
    //    goto out;
    //}

    /* PRNG engine started? */
    //if ((s->reg_control & NUC970_RNG_CONTROL_1_START_INIT) == 0) {
    //    goto out;
    //}

    /* Get randoms */
    /*PRNG Generate Key Size
        0 = 64 bits.     8 bytes
        1 = 128 bits.   16 bytes;
        2 = 192 bits.   24 bytes;
        3 = 256 bits.   32 bytes;
    */
    if (qemu_guest_getrandom(s->randr_value, (s->keysz + 1) * 8, &err)) {
        error_report_err(err);
    }
    else {
        /* Notify that PRNG is ready */
        s->reg_status |= NUC970_RNG_STATUS_PRNG_DONE;
        s->busy = 0;
    }

out:
    /* Always clear start engine bit */
    s->reg_control &= ~NUC970_RNG_CONTROL_1_START_INIT;
}

static uint64_t exynos4210_rng_read(void* opaque, hwaddr offset,
    unsigned size)
{
    NUC970RngState* s = (NUC970RngState*)opaque;
    uint32_t val = 0;

    assert(size == 4);

    switch (offset) {
    case NUC970_RNG_CONTROL_1:
        val = s->reg_control;
        break;

    case NUC970_RNG_STATUS:
        //val = s->reg_status;
        val = s->start | s->reload << 1 | s->keysz << 2 | s->busy << 8;
        break;

    case NUC970_RNG_PRNG_OFFSET(0):
    case NUC970_RNG_PRNG_OFFSET(1):
    case NUC970_RNG_PRNG_OFFSET(2):
    case NUC970_RNG_PRNG_OFFSET(3):
    case NUC970_RNG_PRNG_OFFSET(4):
        val = s->randr_value[(offset - NUC970_RNG_PRNG_OFFSET(0)) / 4];
        DPRINTF("returning random @0x%" HWADDR_PRIx ": 0x%" PRIx32 "\n",
            offset, val);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
            "%s: bad read offset 0x%" HWADDR_PRIx "\n",
            __func__, offset);
    }
    DPRINTF("returning @0x%" HWADDR_PRIx ": 0x%" PRIx32 "\n",
        offset, val);
    return val;
}

static void exynos4210_rng_write(void* opaque, hwaddr offset,
    uint64_t val, unsigned size)
{
    NUC970RngState* s = (NUC970RngState*)opaque;
    DPRINTF("writing @0x%" HWADDR_PRIx ": 0x%" PRIx32 "\n",
        offset, val);
    assert(size == 4);

    switch (offset) {
    case NUC970_RNG_CONTROL_1:
        DPRINTF("RNG_CONTROL_1 = 0x%" PRIx64 "\n", val);
        s->reg_control = val;
        
        break;

    case NUC970_RNG_STATUS:
        /* For clearing status fields */
        //s->reg_status &= ~NUC970_RNG_STATUS_WRITE_MASK;
        //s->reg_status |= val & NUC970_RNG_STATUS_WRITE_MASK;
        s->keysz = (val >> 2) & 0x03;
        s->reload = (val >> 1) & 0x01;
        if (s->reload) {
            srand(s->seed_set);
        }
        if (val & 0x01) {
            s->busy = 1;
            exynos4210_rng_run_engine(s);
        }
        break;
    case 0xc:
        s->seed_set = val;
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
            "%s: bad write offset 0x%" HWADDR_PRIx "\n",
            __func__, offset);
    }
}

static const MemoryRegionOps exynos4210_rng_ops = {
    .read = exynos4210_rng_read,
    .write = exynos4210_rng_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void exynos4210_rng_reset(DeviceState* dev)
{
    NUC970RngState* s = NUC970_RNG(dev);

    s->reg_control = 0;
    s->reg_status = NUC970_RNG_STATUS_BUFFER_READY;
    memset(s->randr_value, 0, sizeof(s->randr_value));
    s->seed_set = 0;
    s->busy = 0;
    s->start = 0;
    s->keysz = 0;
    s->reload = 0;
}

static void exynos4210_rng_init(Object* obj)
{
    NUC970RngState* s = NUC970_RNG(obj);
    SysBusDevice* dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &exynos4210_rng_ops, s,
        TYPE_NUC970_RNG, 0x1000);
    sysbus_init_mmio(dev, &s->iomem);
}

static const VMStateDescription exynos4210_rng_vmstate = {
    .name = TYPE_NUC970_RNG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_INT32_ARRAY(randr_value, NUC970RngState,
                            NUC970_RNG_PRNG_NUM),
        VMSTATE_UINT32(seed_set, NUC970RngState),
        VMSTATE_UINT32(reg_status, NUC970RngState),
        VMSTATE_UINT32(reg_control, NUC970RngState),
        VMSTATE_END_OF_LIST()
    }
};

static void exynos4210_rng_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->reset = exynos4210_rng_reset;
    dc->vmsd = &exynos4210_rng_vmstate;
}

static const TypeInfo exynos4210_rng_info = {
    .name = TYPE_NUC970_RNG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC970RngState),
    .instance_init = exynos4210_rng_init,
    .class_init = exynos4210_rng_class_init,
};

static void exynos4210_rng_register(void)
{
    type_register_static(&exynos4210_rng_info);
}

type_init(exynos4210_rng_register)