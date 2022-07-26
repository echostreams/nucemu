/*
 */

#ifndef NUC980_I2C_H
#define NUC980_I2C_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"

#define TYPE_NUC980_I2C "nuc980.i2c"
OBJECT_DECLARE_SIMPLE_TYPE(NUC980I2CState, NUC980_I2C)

#define NUC970_I2C_MEM_SIZE           0x50

/* NUC980 I2C memory map */
/*
	i2c register offset
*/
#define     I2C_CTL0	    (0x00)
#define     I2C_ADDR0	    (0x04)
#define     I2C_DAT 	    (0x08)
#define     I2C_STATUS0	    (0x0c)
#define     I2C_CLKDIV	    (0x10)
#define     I2C_TOCTL	    (0x14)
#define     I2C_ADDR1       (0x18)
#define     I2C_ADDR2       (0x1c)
#define     I2C_ADDR3       (0x20)
#define     I2C_ADDRMSK0    (0x24)
#define     I2C_ADDRMSK1    (0x28)
#define     I2C_ADDRMSK2    (0x2c)
#define     I2C_ADDRMSK3    (0x30)
#define     I2C_WKCTL       (0x3c)
#define     I2C_WKSTS       (0x40)
#define     I2C_CTL1        (0x44)
#define     I2C_STATUS1     (0x48)
#define     I2C_TMCTL       (0x4c)

#define I2C_BUSCTL      0x50
#define I2C_BUSTCTL     0x54
#define I2C_BUSSTS      0x58
#define I2C_PKTSIZE     0x5C
#define I2C_PKTCRC      0x60
#define I2C_BUSTOUT     0x64
#define I2C_CLKTOUT     0x68

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C_CTL constant definitions.                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define I2C_CTL_STA_SI            0x28UL /*!< I2C_CTL setting for I2C control bits. It would set STA and SI bits          \hideinitializer */
#define I2C_CTL_STA_SI_AA         0x2CUL /*!< I2C_CTL setting for I2C control bits. It would set STA, SI and AA bits      \hideinitializer */
#define I2C_CTL_STO_SI            0x18UL /*!< I2C_CTL setting for I2C control bits. It would set STO and SI bits          \hideinitializer */
#define I2C_CTL_STO_SI_AA         0x1CUL /*!< I2C_CTL setting for I2C control bits. It would set STO, SI and AA bits      \hideinitializer */
#define I2C_CTL_SI                0x08UL /*!< I2C_CTL setting for I2C control bits. It would set SI bit                   \hideinitializer */
#define I2C_CTL_SI_AA             0x0CUL /*!< I2C_CTL setting for I2C control bits. It would set SI and AA bits           \hideinitializer */
#define I2C_CTL_STA               0x20UL /*!< I2C_CTL setting for I2C control bits. It would set STA bit                  \hideinitializer */
#define I2C_CTL_STO               0x10UL /*!< I2C_CTL setting for I2C control bits. It would set STO bit                  \hideinitializer */
#define I2C_CTL_AA                0x04UL /*!< I2C_CTL setting for I2C control bits. It would set AA bit                   \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C GCMode constant definitions.                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define I2C_GCMODE_ENABLE           1    /*!< Enable  I2C GC Mode                                                         \hideinitializer */
#define I2C_GCMODE_DISABLE          0    /*!< Disable I2C GC Mode                                                         \hideinitializer */

#define I2C_CTL0_AA_Pos                  (2)                                               /*!< I2C_T::CTL: AA Position                */
#define I2C_CTL0_AA_Msk                  (0x1ul << I2C_CTL0_AA_Pos)                        /*!< I2C_T::CTL: AA Mask                    */

#define I2C_CTL0_SI_Pos                  (3)                                               /*!< I2C_T::CTL: SI Position                */
#define I2C_CTL0_SI_Msk                  (0x1ul << I2C_CTL0_SI_Pos)                        /*!< I2C_T::CTL: SI Mask                    */

#define I2C_CTL0_STO_Pos                 (4)                                               /*!< I2C_T::CTL: STO Position               */
#define I2C_CTL0_STO_Msk                 (0x1ul << I2C_CTL0_STO_Pos)                       /*!< I2C_T::CTL: STO Mask                   */

#define I2C_CTL0_STA_Pos                 (5)                                               /*!< I2C_T::CTL: STA Position               */
#define I2C_CTL0_STA_Msk                 (0x1ul << I2C_CTL0_STA_Pos)                       /*!< I2C_T::CTL: STA Mask                   */

#define I2C_CTL0_I2CEN_Pos               (6)                                               /*!< I2C_T::CTL: I2CEN Position             */
#define I2C_CTL0_I2CEN_Msk               (0x1ul << I2C_CTL0_I2CEN_Pos)                     /*!< I2C_T::CTL: I2CEN Mask                 */

#define I2C_CTL0_INTEN_Pos               (7)                                               /*!< I2C_T::CTL: INTEN Position             */
#define I2C_CTL0_INTEN_Msk               (0x1ul << I2C_CTL0_INTEN_Pos)                     /*!< I2C_T::CTL: INTEN Mask                 */

#define I2C_ADDR0_GC_Pos                 (0)                                               /*!< I2C_T::ADDR0: GC Position              */
#define I2C_ADDR0_GC_Msk                 (0x1ul << I2C_ADDR0_GC_Pos)                       /*!< I2C_T::ADDR0: GC Mask                  */

#define I2C_ADDR0_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR0: ADDR Position            */
#define I2C_ADDR0_ADDR_Msk               (0x3fful << I2C_ADDR0_ADDR_Pos)                   /*!< I2C_T::ADDR0: ADDR Mask                */

#define I2C_DAT_DAT_Pos                  (0)                                               /*!< I2C_T::DAT: DAT Position               */
#define I2C_DAT_DAT_Msk                  (0xfful << I2C_DAT_DAT_Pos)                       /*!< I2C_T::DAT: DAT Mask                   */

#define I2C_STATUS0_STATUS_Pos           (0)                                               /*!< I2C_T::STATUS: STATUS Position         */
#define I2C_STATUS0_STATUS_Msk           (0xfful << I2C_STATUS_STATUS0_Pos)                /*!< I2C_T::STATUS: STATUS Mask             */

#define I2C_CLKDIV_DIVIDER_Pos           (0)                                               /*!< I2C_T::CLKDIV: DIVIDER Position        */
#define I2C_CLKDIV_DIVIDER_Msk           (0x3fful << I2C_CLKDIV_DIVIDER_Pos)               /*!< I2C_T::CLKDIV: DIVIDER Mask            */

#define I2C_TOCTL_TOIF_Pos               (0)                                               /*!< I2C_T::TOCTL: TOIF Position            */
#define I2C_TOCTL_TOIF_Msk               (0x1ul << I2C_TOCTL_TOIF_Pos)                     /*!< I2C_T::TOCTL: TOIF Mask                */

#define I2C_TOCTL_TOCDIV4_Pos            (1)                                               /*!< I2C_T::TOCTL: TOCDIV4 Position         */
#define I2C_TOCTL_TOCDIV4_Msk            (0x1ul << I2C_TOCTL_TOCDIV4_Pos)                  /*!< I2C_T::TOCTL: TOCDIV4 Mask             */

#define I2C_TOCTL_TOCEN_Pos              (2)                                               /*!< I2C_T::TOCTL: TOCEN Position           */
#define I2C_TOCTL_TOCEN_Msk              (0x1ul << I2C_TOCTL_TOCEN_Pos)                    /*!< I2C_T::TOCTL: TOCEN Mask               */

#define I2C_ADDR1_GC_Pos                 (0)                                               /*!< I2C_T::ADDR1: GC Position              */
#define I2C_ADDR1_GC_Msk                 (0x1ul << I2C_ADDR1_GC_Pos)                       /*!< I2C_T::ADDR1: GC Mask                  */

#define I2C_ADDR1_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR1: ADDR Position            */
#define I2C_ADDR1_ADDR_Msk               (0x3fful << I2C_ADDR1_ADDR_Pos)                   /*!< I2C_T::ADDR1: ADDR Mask                */

#define I2C_ADDR2_GC_Pos                 (0)                                               /*!< I2C_T::ADDR2: GC Position              */
#define I2C_ADDR2_GC_Msk                 (0x1ul << I2C_ADDR2_GC_Pos)                       /*!< I2C_T::ADDR2: GC Mask                  */

#define I2C_ADDR2_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR2: ADDR Position            */
#define I2C_ADDR2_ADDR_Msk               (0x3fful << I2C_ADDR2_ADDR_Pos)                   /*!< I2C_T::ADDR2: ADDR Mask                */

#define I2C_ADDR3_GC_Pos                 (0)                                               /*!< I2C_T::ADDR3: GC Position              */
#define I2C_ADDR3_GC_Msk                 (0x1ul << I2C_ADDR3_GC_Pos)                       /*!< I2C_T::ADDR3: GC Mask                  */

#define I2C_ADDR3_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR3: ADDR Position            */
#define I2C_ADDR3_ADDR_Msk               (0x3fful << I2C_ADDR3_ADDR_Pos)                   /*!< I2C_T::ADDR3: ADDR Mask                */

#define I2C_ADDRMSK0_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK0: ADDRMSK Position      */
#define I2C_ADDRMSK0_ADDRMSK_Msk         (0x3fful << I2C_ADDRMSK0_ADDRMSK_Pos)             /*!< I2C_T::ADDRMSK0: ADDRMSK Mask          */

#define I2C_ADDRMSK1_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK1: ADDRMSK Position      */
#define I2C_ADDRMSK1_ADDRMSK_Msk         (0x3fful << I2C_ADDRMSK1_ADDRMSK_Pos)             /*!< I2C_T::ADDRMSK1: ADDRMSK Mask          */

#define I2C_ADDRMSK2_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK2: ADDRMSK Position      */
#define I2C_ADDRMSK2_ADDRMSK_Msk         (0x3fful << I2C_ADDRMSK2_ADDRMSK_Pos)             /*!< I2C_T::ADDRMSK2: ADDRMSK Mask          */

#define I2C_ADDRMSK3_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK3: ADDRMSK Position      */
#define I2C_ADDRMSK3_ADDRMSK_Msk         (0x3fful << I2C_ADDRMSK3_ADDRMSK_Pos)             /*!< I2C_T::ADDRMSK3: ADDRMSK Mask          */

#define I2C_WKCTL_WKEN_Pos               (0)                                               /*!< I2C_T::WKCTL: WKEN Position            */
#define I2C_WKCTL_WKEN_Msk               (0x1ul << I2C_WKCTL_WKEN_Pos)                     /*!< I2C_T::WKCTL: WKEN Mask                */

#define I2C_WKCTL_NHDBUSEN_Pos           (7)                                               /*!< I2C_T::WKCTL: NHDBUSEN Position        */
#define I2C_WKCTL_NHDBUSEN_Msk           (0x1ul << I2C_WKCTL_NHDBUSEN_Pos)                 /*!< I2C_T::WKCTL: NHDBUSEN Mask            */

#define I2C_WKSTS_WKIF_Pos               (0)                                               /*!< I2C_T::WKSTS: WKIF Position            */
#define I2C_WKSTS_WKIF_Msk               (0x1ul << I2C_WKSTS_WKIF_Pos)                     /*!< I2C_T::WKSTS: WKIF Mask                */

#define I2C_WKSTS_WKAKDONE_Pos           (1)                                               /*!< I2C_T::WKSTS: WKAKDONE Position        */
#define I2C_WKSTS_WKAKDONE_Msk           (0x1ul << I2C_WKSTS_WKAKDONE_Pos)                 /*!< I2C_T::WKSTS: WKAKDONE Mask            */

#define I2C_WKSTS_WRSTSWK_Pos            (2)                                               /*!< I2C_T::WKSTS: WRSTSWK Position         */
#define I2C_WKSTS_WRSTSWK_Msk            (0x1ul << I2C_WKSTS_WRSTSWK_Pos)                  /*!< I2C_T::WKSTS: WRSTSWK Mask             */

#define I2C_CTL1_TXPDMAEN_Pos            (0)                                               /*!< I2C_T::CTL1: TXPDMAEN Position         */
#define I2C_CTL1_TXPDMAEN_Msk            (0x1ul << I2C_CTL1_TXPDMAEN_Pos)                  /*!< I2C_T::CTL1: TXPDMAEN Mask             */

#define I2C_CTL1_RXPDMAEN_Pos            (1)                                               /*!< I2C_T::CTL1: RXPDMAEN Position         */
#define I2C_CTL1_RXPDMAEN_Msk            (0x1ul << I2C_CTL1_RXPDMAEN_Pos)                  /*!< I2C_T::CTL1: RXPDMAEN Mask             */

#define I2C_CTL1_PDMARST_Pos             (2)                                               /*!< I2C_T::CTL1: PDMARST Position          */
#define I2C_CTL1_PDMARST_Msk             (0x1ul << I2C_CTL1_PDMARST_Pos)                   /*!< I2C_T::CTL1: PDMARST Mask              */

#define I2C_CTL1_PDMASTR_Pos             (8)                                               /*!< I2C_T::CTL1: PDMASTR Position          */
#define I2C_CTL1_PDMASTR_Msk             (0x1ul << I2C_CTL1_PDMASTR_Pos)                   /*!< I2C_T::CTL1: PDMASTR Mask              */

#define I2C_CTL1_ADDR10EN_Pos            (9)                                               /*!< I2C_T::CTL1: ADDR10EN Position         */
#define I2C_CTL1_ADDR10EN_Msk            (0x1ul << I2C_CTL1_ADDR10EN_Pos)                  /*!< I2C_T::CTL1: ADDR10EN Mask             */

#define I2C_STATUS1_ADMAT0_Pos           (0)                                               /*!< I2C_T::STATUS1: ADMAT0 Position        */
#define I2C_STATUS1_ADMAT0_Msk           (0x1ul << I2C_STATUS1_ADMAT0_Pos)                 /*!< I2C_T::STATUS1: ADMAT0 Mask            */

#define I2C_STATUS1_ADMAT1_Pos           (1)                                               /*!< I2C_T::STATUS1: ADMAT1 Position        */
#define I2C_STATUS1_ADMAT1_Msk           (0x1ul << I2C_STATUS1_ADMAT1_Pos)                 /*!< I2C_T::STATUS1: ADMAT1 Mask            */

#define I2C_STATUS1_ADMAT2_Pos           (2)                                               /*!< I2C_T::STATUS1: ADMAT2 Position        */
#define I2C_STATUS1_ADMAT2_Msk           (0x1ul << I2C_STATUS1_ADMAT2_Pos)                 /*!< I2C_T::STATUS1: ADMAT2 Mask            */

#define I2C_STATUS1_ADMAT3_Pos           (3)                                               /*!< I2C_T::STATUS1: ADMAT3 Position        */
#define I2C_STATUS1_ADMAT3_Msk           (0x1ul << I2C_STATUS1_ADMAT3_Pos)                 /*!< I2C_T::STATUS1: ADMAT3 Mask            */

#define I2C_STATUS1_ONBUSY_Pos           (8)                                               /*!< I2C_T::STATUS1: ONBUSY Position        */
#define I2C_STATUS1_ONBUSY_Msk           (0x1ul << I2C_STATUS1_ONBUSY_Pos)                 /*!< I2C_T::STATUS1: ONBUSY Mask            */

#define I2C_TMCTL_STCTL_Pos              (0)                                               /*!< I2C_T::TMCTL: STCTL Position           */
#define I2C_TMCTL_STCTL_Msk              (0x1fful << I2C_TMCTL_STCTL_Pos)                  /*!< I2C_T::TMCTL: STCTL Mask               */

#define I2C_TMCTL_HTCTL_Pos              (16)                                              /*!< I2C_T::TMCTL: HTCTL Position           */
#define I2C_TMCTL_HTCTL_Msk              (0x1fful << I2C_TMCTL_HTCTL_Pos)                  /*!< I2C_T::TMCTL: HTCTL Mask               */

#define I2C_BUSCTL_ACKMEN_Pos            (0)                                               /*!< I2C_T::BUSCTL: ACKMEN Position         */
#define I2C_BUSCTL_ACKMEN_Msk            (0x1ul << I2C_BUSCTL_ACKMEN_Pos)                  /*!< I2C_T::BUSCTL: ACKMEN Mask             */

#define I2C_BUSCTL_PECEN_Pos             (1)                                               /*!< I2C_T::BUSCTL: PECEN Position          */
#define I2C_BUSCTL_PECEN_Msk             (0x1ul << I2C_BUSCTL_PECEN_Pos)                   /*!< I2C_T::BUSCTL: PECEN Mask              */

#define I2C_BUSCTL_BMDEN_Pos             (2)                                               /*!< I2C_T::BUSCTL: BMDEN Position          */
#define I2C_BUSCTL_BMDEN_Msk             (0x1ul << I2C_BUSCTL_BMDEN_Pos)                   /*!< I2C_T::BUSCTL: BMDEN Mask              */

#define I2C_BUSCTL_BMHEN_Pos             (3)                                               /*!< I2C_T::BUSCTL: BMHEN Position          */
#define I2C_BUSCTL_BMHEN_Msk             (0x1ul << I2C_BUSCTL_BMHEN_Pos)                   /*!< I2C_T::BUSCTL: BMHEN Mask              */

#define I2C_BUSCTL_ALERTEN_Pos           (4)                                               /*!< I2C_T::BUSCTL: ALERTEN Position        */
#define I2C_BUSCTL_ALERTEN_Msk           (0x1ul << I2C_BUSCTL_ALERTEN_Pos)                 /*!< I2C_T::BUSCTL: ALERTEN Mask            */

#define I2C_BUSCTL_SCTLOSTS_Pos          (5)                                               /*!< I2C_T::BUSCTL: SCTLOSTS Position       */
#define I2C_BUSCTL_SCTLOSTS_Msk          (0x1ul << I2C_BUSCTL_SCTLOSTS_Pos)                /*!< I2C_T::BUSCTL: SCTLOSTS Mask           */

#define I2C_BUSCTL_SCTLOEN_Pos           (6)                                               /*!< I2C_T::BUSCTL: SCTLOEN Position        */
#define I2C_BUSCTL_SCTLOEN_Msk           (0x1ul << I2C_BUSCTL_SCTLOEN_Pos)                 /*!< I2C_T::BUSCTL: SCTLOEN Mask            */

#define I2C_BUSCTL_BUSEN_Pos             (7)                                               /*!< I2C_T::BUSCTL: BUSEN Position          */
#define I2C_BUSCTL_BUSEN_Msk             (0x1ul << I2C_BUSCTL_BUSEN_Pos)                   /*!< I2C_T::BUSCTL: BUSEN Mask              */

#define I2C_BUSCTL_PECTXEN_Pos           (8)                                               /*!< I2C_T::BUSCTL: PECTXEN Position        */
#define I2C_BUSCTL_PECTXEN_Msk           (0x1ul << I2C_BUSCTL_PECTXEN_Pos)                 /*!< I2C_T::BUSCTL: PECTXEN Mask            */

#define I2C_BUSCTL_TIDLE_Pos             (9)                                               /*!< I2C_T::BUSCTL: TIDLE Position          */
#define I2C_BUSCTL_TIDLE_Msk             (0x1ul << I2C_BUSCTL_TIDLE_Pos)                   /*!< I2C_T::BUSCTL: TIDLE Mask              */

#define I2C_BUSCTL_PECCLR_Pos            (10)                                              /*!< I2C_T::BUSCTL: PECCLR Position         */
#define I2C_BUSCTL_PECCLR_Msk            (0x1ul << I2C_BUSCTL_PECCLR_Pos)                  /*!< I2C_T::BUSCTL: PECCLR Mask             */

#define I2C_BUSCTL_ACKM9SI_Pos           (11)                                              /*!< I2C_T::BUSCTL: ACKM9SI Position        */
#define I2C_BUSCTL_ACKM9SI_Msk           (0x1ul << I2C_BUSCTL_ACKM9SI_Pos)                 /*!< I2C_T::BUSCTL: ACKM9SI Mask            */

#define I2C_BUSCTL_BCDIEN_Pos            (12)                                              /*!< I2C_T::BUSCTL: BCDIEN Position         */
#define I2C_BUSCTL_BCDIEN_Msk            (0x1ul << I2C_BUSCTL_BCDIEN_Pos)                  /*!< I2C_T::BUSCTL: BCDIEN Mask             */

#define I2C_BUSCTL_PECDIEN_Pos           (13)                                              /*!< I2C_T::BUSCTL: PECDIEN Position        */
#define I2C_BUSCTL_PECDIEN_Msk           (0x1ul << I2C_BUSCTL_PECDIEN_Pos)                 /*!< I2C_T::BUSCTL: PECDIEN Mask            */

#define I2C_BUSTCTL_BUSTOEN_Pos          (0)                                               /*!< I2C_T::BUSTCTL: BUSTOEN Position       */
#define I2C_BUSTCTL_BUSTOEN_Msk          (0x1ul << I2C_BUSTCTL_BUSTOEN_Pos)                /*!< I2C_T::BUSTCTL: BUSTOEN Mask           */

#define I2C_BUSTCTL_CLKTOEN_Pos          (1)                                               /*!< I2C_T::BUSTCTL: CLKTOEN Position       */
#define I2C_BUSTCTL_CLKTOEN_Msk          (0x1ul << I2C_BUSTCTL_CLKTOEN_Pos)                /*!< I2C_T::BUSTCTL: CLKTOEN Mask           */

#define I2C_BUSTCTL_BUSTOIEN_Pos         (2)                                               /*!< I2C_T::BUSTCTL: BUSTOIEN Position      */
#define I2C_BUSTCTL_BUSTOIEN_Msk         (0x1ul << I2C_BUSTCTL_BUSTOIEN_Pos)               /*!< I2C_T::BUSTCTL: BUSTOIEN Mask          */

#define I2C_BUSTCTL_CLKTOIEN_Pos         (3)                                               /*!< I2C_T::BUSTCTL: CLKTOIEN Position      */
#define I2C_BUSTCTL_CLKTOIEN_Msk         (0x1ul << I2C_BUSTCTL_CLKTOIEN_Pos)               /*!< I2C_T::BUSTCTL: CLKTOIEN Mask          */

#define I2C_BUSTCTL_TORSTEN_Pos          (4)                                               /*!< I2C_T::BUSTCTL: TORSTEN Position       */
#define I2C_BUSTCTL_TORSTEN_Msk          (0x1ul << I2C_BUSTCTL_TORSTEN_Pos)                /*!< I2C_T::BUSTCTL: TORSTEN Mask           */

#define I2C_BUSSTS_BUSY_Pos              (0)                                               /*!< I2C_T::BUSSTS: BUSY Position           */
#define I2C_BUSSTS_BUSY_Msk              (0x1ul << I2C_BUSSTS_BUSY_Pos)                    /*!< I2C_T::BUSSTS: BUSY Mask               */

#define I2C_BUSSTS_BCDONE_Pos            (1)                                               /*!< I2C_T::BUSSTS: BCDONE Position         */
#define I2C_BUSSTS_BCDONE_Msk            (0x1ul << I2C_BUSSTS_BCDONE_Pos)                  /*!< I2C_T::BUSSTS: BCDONE Mask             */

#define I2C_BUSSTS_PECERR_Pos            (2)                                               /*!< I2C_T::BUSSTS: PECERR Position         */
#define I2C_BUSSTS_PECERR_Msk            (0x1ul << I2C_BUSSTS_PECERR_Pos)                  /*!< I2C_T::BUSSTS: PECERR Mask             */

#define I2C_BUSSTS_ALERT_Pos             (3)                                               /*!< I2C_T::BUSSTS: ALERT Position          */
#define I2C_BUSSTS_ALERT_Msk             (0x1ul << I2C_BUSSTS_ALERT_Pos)                   /*!< I2C_T::BUSSTS: ALERT Mask              */

#define I2C_BUSSTS_SCTLDIN_Pos           (4)                                               /*!< I2C_T::BUSSTS: SCTLDIN Position        */
#define I2C_BUSSTS_SCTLDIN_Msk           (0x1ul << I2C_BUSSTS_SCTLDIN_Pos)                 /*!< I2C_T::BUSSTS: SCTLDIN Mask            */

#define I2C_BUSSTS_BUSTO_Pos             (5)                                               /*!< I2C_T::BUSSTS: BUSTO Position          */
#define I2C_BUSSTS_BUSTO_Msk             (0x1ul << I2C_BUSSTS_BUSTO_Pos)                   /*!< I2C_T::BUSSTS: BUSTO Mask              */

#define I2C_BUSSTS_CLKTO_Pos             (6)                                               /*!< I2C_T::BUSSTS: CLKTO Position          */
#define I2C_BUSSTS_CLKTO_Msk             (0x1ul << I2C_BUSSTS_CLKTO_Pos)                   /*!< I2C_T::BUSSTS: CLKTO Mask              */

#define I2C_BUSSTS_PECDONE_Pos           (7)                                               /*!< I2C_T::BUSSTS: PECDONE Position        */
#define I2C_BUSSTS_PECDONE_Msk           (0x1ul << I2C_BUSSTS_PECDONE_Pos)                 /*!< I2C_T::BUSSTS: PECDONE Mask            */

#define I2C_PKTSIZE_PLDSIZE_Pos          (0)                                               /*!< I2C_T::PKTSIZE: PLDSIZE Position       */
#define I2C_PKTSIZE_PLDSIZE_Msk          (0x1fful << I2C_PKTSIZE_PLDSIZE_Pos)              /*!< I2C_T::PKTSIZE: PLDSIZE Mask           */

#define I2C_PKTCRC_PECCRC_Pos            (0)                                               /*!< I2C_T::PKTCRC: PECCRC Position         */
#define I2C_PKTCRC_PECCRC_Msk            (0xfful << I2C_PKTCRC_PECCRC_Pos)                 /*!< I2C_T::PKTCRC: PECCRC Mask             */

#define I2C_BUSTOUT_BUSTO_Pos            (0)                                               /*!< I2C_T::BUSTOUT: BUSTO Position         */
#define I2C_BUSTOUT_BUSTO_Msk            (0xfful << I2C_BUSTOUT_BUSTO_Pos)                 /*!< I2C_T::BUSTOUT: BUSTO Mask             */

#define I2C_CLKTOUT_CLKTO_Pos            (0)                                               /*!< I2C_T::CLKTOUT: CLKTO Position         */
#define I2C_CLKTOUT_CLKTO_Msk            (0xfful << I2C_CLKTOUT_CLKTO_Pos)                 /*!< I2C_T::CLKTOUT: CLKTO Mask             */

struct NUC980I2CState {
	/*< private >*/
	SysBusDevice parent_obj;

	/*< public >*/
	MemoryRegion iomem;
	I2CBus* bus;
	qemu_irq irq;

	uint32_t CTL0;                  /*!< [0x0000] I2C Control Register 0                                           */
	uint32_t ADDR0;                 /*!< [0x0004] I2C Slave Address Register0                                      */
	uint32_t DAT;                   /*!< [0x0008] I2C Data Register                                                */
	uint32_t STATUS0;               /*!< [0x000c] I2C Status Register 0                                            */
	uint32_t CLKDIV;                /*!< [0x0010] I2C Clock Divided Register                                       */
	uint32_t TOCTL;                 /*!< [0x0014] I2C Time-out Control Register                                    */
	uint32_t ADDR1;                 /*!< [0x0018] I2C Slave Address Register1                                      */
	uint32_t ADDR2;                 /*!< [0x001c] I2C Slave Address Register2                                      */
	uint32_t ADDR3;                 /*!< [0x0020] I2C Slave Address Register3                                      */
	uint32_t ADDRMSK0;              /*!< [0x0024] I2C Slave Address Mask Register0                                 */
	uint32_t ADDRMSK1;              /*!< [0x0028] I2C Slave Address Mask Register1                                 */
	uint32_t ADDRMSK2;              /*!< [0x002c] I2C Slave Address Mask Register2                                 */
	uint32_t ADDRMSK3;              /*!< [0x0030] I2C Slave Address Mask Register3                                 */
	//uint32_t RESERVE0[2];
	uint32_t WKCTL;                 /*!< [0x003c] I2C Wake-up Control Register                                     */
	uint32_t WKSTS;                 /*!< [0x0040] I2C Wake-up Status Register                                      */
	uint32_t CTL1;                  /*!< [0x0044] I2C Control Register 1                                           */
	uint32_t STATUS1;               /*!< [0x0048] I2C Status Register 1                                            */
	uint32_t TMCTL;                 /*!< [0x004c] I2C Timing Configure Control Register                            */
	uint32_t BUSCTL;                /*!< [0x0050] I2C Bus Management Control Register                              */
	uint32_t BUSTCTL;               /*!< [0x0054] I2C Bus Management Timer Control Register                        */
	uint32_t BUSSTS;                /*!< [0x0058] I2C Bus Management Status Register                               */
	uint32_t PKTSIZE;               /*!< [0x005c] I2C Packet Error Checking Byte Number Register                   */
	uint32_t PKTCRC;                /*!< [0x0060] I2C Packet Error Checking Byte Value Register                    */
	uint32_t BUSTOUT;               /*!< [0x0064] I2C Bus Management Timer Register                                */
	uint32_t CLKTOUT;               /*!< [0x0068] I2C Bus Management Clock Low Timer Register*/

	bool is_slave;
	bool is_recv;
	bool ack;
	QEMUTimer* dat_timer;
	int64_t dat_time;
};

/** @addtogroup I2C_EXPORTED_FUNCTIONS I2C Exported Functions
  @{
*/
/**
 *    @brief        The macro is used to set I2C bus condition at One Time
 *
 *    @param[in]    i2c        Specify I2C port
 *    @param[in]    u8Ctrl     A byte writes to I2C control register
 *
 *    @return       None
 *
 *    @details      Set I2C_CTL register to control I2C bus conditions of START, STOP, SI, ACK.
 *    \hideinitializer
 */
#define I2C_SET_CONTROL_REG(i2c, u8Ctrl) ((i2c)->CTL0 = ((i2c)->CTL0 & ~0x3c) | (u8Ctrl))

/**
*    @brief        The macro is used to set START condition of I2C Bus
*
*    @param[in]    i2c        Specify I2C port
*
*    @return       None
*
*    @details      Set the I2C bus START condition in I2C_CTL register.
*    \hideinitializer
*/
#define I2C_START(i2c)  ((i2c)->CTL0 = ((i2c)->CTL0 & ~I2C_CTL0_SI_Msk) | I2C_CTL0_STA_Msk)

/**
*    @brief        The macro is used to wait I2C bus status get ready
*
*    @param[in]    i2c        Specify I2C port
*
*    @return       None
*
*    @details      When a new status is presented of I2C bus, the SI flag will be set in I2C_CTL register.
*    \hideinitializer
*/
#define I2C_WAIT_READY(i2c)     while(!((i2c)->CTL0 & I2C_CTL0_SI_Msk))

/**
*    @brief        The macro is used to Read I2C Bus Data Register
*
*    @param[in]    i2c        Specify I2C port
*
*    @return       A byte of I2C data register
*
*    @details      I2C controller read data from bus and save it in I2CDAT register.
*    \hideinitializer
*/
#define I2C_GET_DATA(i2c)   ((i2c)->DAT)

/**
*    @brief        Write a Data to I2C Data Register
*
*    @param[in]    i2c         Specify I2C port
*    @param[in]    u8Data      A byte that writes to data register
*
*    @return       None
*
*    @details      When write a data to I2C_DAT register, the I2C controller will shift it to I2C bus.
*    \hideinitializer
*/
#define I2C_SET_DATA(i2c, u8Data) ((i2c)->DAT = (u8Data))

/**
*    @brief        Get I2C Bus status code
*
*    @param[in]    i2c        Specify I2C port
*
*    @return       I2C status code
*
*    @details      To get this status code to monitor I2C bus event.
*    \hideinitializer
*/
#define I2C_GET_STATUS(i2c) ((i2c)->STATUS0)

	  /**
	   *    @brief        Get Time-out flag from I2C Bus
	   *
	   *    @param[in]    i2c     Specify I2C port
	   *
	   *    @retval       0       I2C Bus time-out is not happened
	   *    @retval       1       I2C Bus time-out is happened
	   *
	   *    @details      When I2C bus occurs time-out event, the time-out flag will be set.
	   *    \hideinitializer
	   */
#define I2C_GET_TIMEOUT_FLAG(i2c)   ( ((i2c)->TOCTL & I2C_TOCTL_TOIF_Msk) == I2C_TOCTL_TOIF_Msk ? 1:0 )

	   /**
		*    @brief        To get wake-up flag from I2C Bus
		*
		*    @param[in]    i2c     Specify I2C port
		*
		*    @retval       0       Chip is not woken-up from power-down mode
		*    @retval       1       Chip is woken-up from power-down mode
		*
		*    @details      I2C bus occurs wake-up event, wake-up flag will be set.
		*    \hideinitializer
		*/
#define I2C_GET_WAKEUP_FLAG(i2c) ( ((i2c)->WKSTS & I2C_WKSTS_WKIF_Msk) == I2C_WKSTS_WKIF_Msk ? 1:0  )

		/**
		 *    @brief        To clear wake-up flag
		 *
		 *    @param[in]    i2c     Specify I2C port
		 *
		 *    @return       None
		 *
		 *    @details      If wake-up flag is set, use this macro to clear it.
		 *    \hideinitializer
		 */
#define I2C_CLEAR_WAKEUP_FLAG(i2c)  ((i2c)->WKSTS = I2C_WKSTS_WKIF_Msk)

		 /**
		   * @brief      Enable RX PDMA function.
		   * @param[in]  i2c The pointer of the specified I2C module.
		   * @return     None.
		   * @details    Set RXPDMAEN bit of I2C_CTL1 register to enable RX PDMA transfer function.
		   * \hideinitializer
		   */
#define I2C_ENABLE_RX_PDMA(i2c)   ((i2c)->CTL1 |= I2C_CTL1_RXPDMAEN_Msk)

		   /**
			 * @brief      Enable TX PDMA function.
			 * @param[in]  i2c The pointer of the specified I2C module.
			 * @return     None.
			 * @details    Set TXPDMAEN bit of I2C_CTL1 register to enable TX PDMA transfer function.
			 * \hideinitializer
			 */
#define I2C_ENABLE_TX_PDMA(i2c)   ((i2c)->CTL1 |= I2C_CTL1_TXPDMAEN_Msk)

			 /**
			   * @brief      Disable RX PDMA transfer.
			   * @param[in]  i2c The pointer of the specified I2C module.
			   * @return     None.
			   * @details    Clear RXPDMAEN bit of I2C_CTL1 register to disable RX PDMA transfer function.
			   * \hideinitializer
			   */
#define I2C_DISABLE_RX_PDMA(i2c)   ((i2c)->CTL1 &= ~I2C_CTL1_RXPDMAEN_Msk)

			   /**
				 * @brief      Disable TX PDMA transfer.
				 * @param[in]  i2c The pointer of the specified I2C module.
				 * @return     None.
				 * @details    Clear TXPDMAEN bit of I2C_CTL1 register to disable TX PDMA transfer function.
				 * \hideinitializer
				 */
#define I2C_DISABLE_TX_PDMA(i2c)   ((i2c)->CTL1 &= ~I2C_CTL1_TXPDMAEN_Msk)

				 /**
				   * @brief      Enable PDMA stretch function.
				   * @param[in]  i2c The pointer of the specified I2C module.
				   * @return     None.
				   * @details    Enable this function is to stretch bus by hardware after PDMA transfer is done if SI is not cleared.
				   * \hideinitializer
				   */
#define I2C_ENABLE_PDMA_STRETCH(i2c)   ((i2c)->CTL1 |= I2C_CTL1_PDMASTR_Msk)

				   /**
					 * @brief      Disable PDMA stretch function.
					 * @param[in]  i2c The pointer of the specified I2C module.
					 * @return     None.
					 * @details    I2C will send STOP after PDMA transfers done automatically.
					 * \hideinitializer
					 */
#define I2C_DISABLE_PDMA_STRETCH(i2c)   ((i2c)->CTL1 &= ~I2C_CTL1_PDMASTR_Msk)

					 /**
					   * @brief      Reset PDMA function.
					   * @param[in]  i2c The pointer of the specified I2C module.
					   * @return     None.
					   * @details    I2C PDMA engine will be reset after this function is called.
					   * \hideinitializer
					   */
#define I2C_DISABLE_RST_PDMA(i2c)   ((i2c)->CTL1 |= I2C_CTL1_PDMARST_Msk)

/* nuc980 i2c Status */
// Master
#define  M_START                 0x08  //Start
#define  M_REPEAT_START          0x10  //Master Repeat Start
#define  M_TRAN_ADDR_ACK         0x18  //Master Transmit Address ACK
#define  M_TRAN_ADDR_NACK        0x20  //Master Transmit Address NACK
#define  M_TRAN_DATA_ACK         0x28  //Master Transmit Data ACK
#define  M_TRAN_DATA_NACK        0x30  //Master Transmit Data NACK
#define  M_ARB_LOST              0x38  //Master Arbitration Los
#define  M_RECE_ADDR_ACK         0x40  //Master Receive Address ACK
#define  M_RECE_ADDR_NACK        0x48  //Master Receive Address NACK
#define  M_RECE_DATA_ACK         0x50  //Master Receive Data ACK
#define  M_RECE_DATA_NACK        0x58  //Master Receive Data NACK
#define  BUS_ERROR               0x00  //Bus error

// Slave
#define  S_REPEAT_START_STOP     0xA0  //Slave Transmit Repeat Start or Stop
#define  S_TRAN_ADDR_ACK         0xA8  //Slave Transmit Address ACK
#define  S_TRAN_DATA_ACK         0xB8  //Slave Transmit Data ACK
#define  S_TRAN_DATA_NACK        0xC0  //Slave Transmit Data NACK
#define  S_TRAN_LAST_DATA_ACK    0xC8  //Slave Transmit Last Data ACK
#define  S_RECE_ADDR_ACK         0x60  //Slave Receive Address ACK
#define  S_RECE_ARB_LOST         0x68  //Slave Receive Arbitration Lost
#define  S_RECE_DATA_ACK         0x80  //Slave Receive Data ACK
#define  S_RECE_DATA_NACK        0x88  //Slave Receive Data NACK

//GC Mode
#define  GC_ADDR_ACK             0x70  //GC mode Address ACK
#define  GC_ARB_LOST             0x78  //GC mode Arbitration Lost
#define  GC_DATA_ACK             0x90  //GC mode Data ACK
#define  GC_DATA_NACK            0x98  //GC mode Data NACK

//Other
#define  ADDR_TRAN_ARB_LOST      0xB0  //Address Transmit Arbitration Lost
#define  BUS_RELEASED            0xF8  //Bus Released

#endif /* NUC980_I2C_H */
