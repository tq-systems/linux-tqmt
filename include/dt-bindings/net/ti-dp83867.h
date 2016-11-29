/*
 * Device Tree constants for the Texas Instruments DP83867 PHY
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * Copyright:   (C) 2015 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef _DT_BINDINGS_TI_DP83867_H
#define _DT_BINDINGS_TI_DP83867_H

/* PHY CTRL bits */
#define DP83867_PHYCR_FIFO_DEPTH_3_B_NIB	0x00
#define DP83867_PHYCR_FIFO_DEPTH_4_B_NIB	0x01
#define DP83867_PHYCR_FIFO_DEPTH_6_B_NIB	0x02
#define DP83867_PHYCR_FIFO_DEPTH_8_B_NIB	0x03

/* RGMIIDCTL internal delay for rx and tx */
#define	DP83867_RGMIIDCTL_250_PS	0x0
#define	DP83867_RGMIIDCTL_500_PS	0x1
#define	DP83867_RGMIIDCTL_750_PS	0x2
#define	DP83867_RGMIIDCTL_1_NS		0x3
#define	DP83867_RGMIIDCTL_1_25_NS	0x4
#define	DP83867_RGMIIDCTL_1_50_NS	0x5
#define	DP83867_RGMIIDCTL_1_75_NS	0x6
#define	DP83867_RGMIIDCTL_2_00_NS	0x7
#define	DP83867_RGMIIDCTL_2_25_NS	0x8
#define	DP83867_RGMIIDCTL_2_50_NS	0x9
#define	DP83867_RGMIIDCTL_2_75_NS	0xa
#define	DP83867_RGMIIDCTL_3_00_NS	0xb
#define	DP83867_RGMIIDCTL_3_25_NS	0xc
#define	DP83867_RGMIIDCTL_3_50_NS	0xd
#define	DP83867_RGMIIDCTL_3_75_NS	0xe
#define	DP83867_RGMIIDCTL_4_00_NS	0xf

#define DP83867_IOMUXCFG_CLK_O_SEL_REF_CLK      0x0C
#define DP83867_IOMUXCFG_CLK_O_SEL_CHD_TX       0x0B
#define DP83867_IOMUXCFG_CLK_O_SEL_CHC_TX       0x0A
#define DP83867_IOMUXCFG_CLK_O_SEL_CHB_TX       0x09
#define DP83867_IOMUXCFG_CLK_O_SEL_CHA_TX       0x08
#define DP83867_IOMUXCFG_CLK_O_SEL_CHD_RX_DIV5  0x07
#define DP83867_IOMUXCFG_CLK_O_SEL_CHC_RX_DIV5  0x06
#define DP83867_IOMUXCFG_CLK_O_SEL_CHB_RX_DIV5  0x05
#define DP83867_IOMUXCFG_CLK_O_SEL_CHA_RX_DIV5  0x04
#define DP83867_IOMUXCFG_CLK_O_SEL_CHD_RX       0x03
#define DP83867_IOMUXCFG_CLK_O_SEL_CHC_RX       0x02
#define DP83867_IOMUXCFG_CLK_O_SEL_CHB_RX       0x01
#define DP83867_IOMUXCFG_CLK_O_SEL_CHA_RX       0x00

#endif
