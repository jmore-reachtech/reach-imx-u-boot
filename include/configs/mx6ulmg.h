/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6UL 14x14 EVK board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __MX6UL_14X14_EVK_CONFIG_H
#define __MX6UL_14X14_EVK_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/imx-common/gpio.h>

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN	(16 * SZ_1M)

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE	UART1_BASE

#define PHYS_SDRAM_SIZE			SZ_256M
#define CONFIG_LDO_BYPASS_CHECK
#define CONFIG_MODULE_FUSE
#define CONFIG_BOOTARGS_CMA_SIZE   " cma=96M "

/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x8000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			    1000

#define CONFIG_STACKSIZE		    SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* I2C configs */
#define CONFIG_CMD_I2C
#ifdef CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_SPEED		100000

/* PMIC on modular gateway i.mx6ul */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE3001
#define CONFIG_POWER_PFUZE3001_I2C_ADDR  0x08
#endif

#ifdef CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define CONFIG_FEC_ENET_DEV		0
 
#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE				ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR      0x2
#define CONFIG_FEC_XCV_TYPE         RMII
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE				ENET2_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR		0x1
#define CONFIG_FEC_XCV_TYPE			RMII
#endif
#define CONFIG_ETHPRIME				"FEC"
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#endif

/* MMC Configs */
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC1_BASE_ADDR
#define CONFIG_SYS_FSL_USDHC_NUM	2

#define CONFIG_SYS_MMC_IMG_LOAD_PART   1
#define CONFIG_SYS_MMC_ENV_DEV		0   /* USDHC2 */
#define CONFIG_SYS_MMC_ENV_PART		0	/* user area */
#define CONFIG_MMCROOT			    "/dev/mmcblk0p"  /* USDHC2 */

/* Environment */
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_SIZE			SZ_8K
#define CONFIG_ENV_OFFSET		(12 * SZ_64K)

#define ENV_MEM_LAYOUT_SETTINGS \
	"fdt_high=0xffffffff\0" \
	"initrd_high=ffffffff\0" \
	"fdt_addr_r=0x83000000\0" \
	"kernel_addr_r=0x80800000\0"

/*
 * UpdateHub configuration
 */

#define CONFIG_BOOTCOUNT_ENV

/* Environment */
#define UPDATEHUB_LOAD_OS_A     "load mmc 0:1 ${kernel_addr_r} /boot/zImage; " \
                                "load mmc 0:1 ${fdt_addr_r} /boot/${fdt_file}; "
#define UPDATEHUB_FIND_ROOT_A   "part uuid mmc 0:1 uuid"

#define UPDATEHUB_LOAD_OS_B     "load mmc 0:2 ${kernel_addr_r} /boot/zImage; " \
                                "load mmc 0:2 ${fdt_addr_r} /boot/${fdt_file}; "
#define UPDATEHUB_FIND_ROOT_B   "part uuid mmc 0:2 uuid"

#define UPDATEHUB_BOOTARGS      "root=PARTUUID=${uuid} rootwait rw " \
                                "console=ttymxc0,115200"
#define UPDATEHUB_BOOTCMD       "bootz ${kernel_addr_r} - ${fdt_addr_r}"

#include <configs/updatehub-common.h>

#define CONFIG_EXTRA_ENV_SETTINGS \
    ENV_MEM_LAYOUT_SETTINGS \
	"ip_dyn=yes\0" \
    "fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
	"mmcroot=" CONFIG_MMCROOT "\0" \
    UPDATEHUB_ENV

#endif
