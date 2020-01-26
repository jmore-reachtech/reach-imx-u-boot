/*
 * Copyright (C) 2020 Reach Technology.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"
#include <asm/arch/imx-regs.h>
#include <asm/mach-imx/gpio.h>
#include <linux/sizes.h>

#define MACH_TYPE_HAWTHORNE		4901
#define CONFIG_MACH_TYPE		MACH_TYPE_HAWTHORNE

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_LATE_INIT

#define CONFIG_MXC_UART_BASE		UART1_BASE

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + SZ_1G + SZ_512M)
#define CONFIG_LOADADDR			0x12000000

/* MMC Configuration */
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC4_BASE_ADDR

/* Ethernet Configuration */
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1

/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_SPEED		  100000

#define CONFIG_CONSOLE_DEV		"ttymxc0"

/* Framebuffer */
#define CONFIG_VIDEO_SHUTDOWN_LCD
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_IPUV3_CLK 198000000
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

/* NAND support */
#define CONFIG_SYS_MAX_NAND_DEVICE     1
#define CONFIG_SYS_NAND_BASE           0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdt_addr_r=0x18000000\0" \
	"fdt_high=0xffffffff\0" \
	"kernel_addr_r=0x12000000\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"bootargs=rw rootfstype=ext4 console=${console},${baudrate} quiet " \
		"consoleblank=0 vt.global_cursor_default=0\0" \
	"splash=splash.bmp\0" \
	"splashpos=m,m\0" \
	"splashpart=4\0" \
	"splashen=y\0" \
	"mender_pre_setup_commands=setenv panel ${board_rev}; " \
		"if test $splashen = y; then " \
			"load mmc ${mender_uboot_dev}:${splashpart} ${kernel_addr_r} ${splash}; " \
			"bmp display ${kernel_addr_r}; " \
		"else " \
			"echo Boot splash disabled; " \
		"fi; \0" \

#define CONFIG_BOOTCOMMAND	""

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

/* Physical Memory Map */
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE			(SZ_2G)

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* environment organization */

#define CONFIG_ENV_SIZE			(8 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0

#endif			       /* __CONFIG_H * */
