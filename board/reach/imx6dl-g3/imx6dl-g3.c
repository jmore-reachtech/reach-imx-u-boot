/*
 * Copyright (C) 2020 Reach Technology.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <asm/io.h>
#include <linux/fb.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <malloc.h>
#include <mmc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>

/* Special MXCFB sync flags are here. */
#include "../drivers/video/mxcfb.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL_37 (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_37ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

#define USDHC3_CD_GPIO		IMX_GPIO_NR(1, 2)
#define ETH_PHY_RESET		IMX_GPIO_NR(1, 25)

#define BACKLIGHT_EN		IMX_GPIO_NR(2, 0)
#define DISPLAY_EN		IMX_GPIO_NR(2, 3)
#define BACKLIGHT_PWM		IMX_GPIO_NR(1, 18)

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_GPIO_2__GPIO1_IO02      | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK    | MUX_PAD_CTRL(USDHC_PAD_CTRL_37),
	MX6_PAD_SD4_CMD__SD4_CMD    | MUX_PAD_CTRL(USDHC_PAD_CTRL_37),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL_37),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL_37),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL_37),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL_37),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL_37),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL_37),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL_37),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL_37),
};

static iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* AR8031 PHY Reset */
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const display_enable_pads[] = {
	MX6_PAD_NANDF_D0__GPIO2_IO00  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D3__GPIO2_IO03  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD1_CMD__GPIO1_IO18  | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO5_IO27 | I2C_PAD,
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO5_IO26 | I2C_PAD,
		.gp = IMX_GPIO_NR(5, 26)
	},
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* Reset AR8031 PHY */
	gpio_direction_output(ETH_PHY_RESET, 0);
	/* the datasheet states a 10ms delay */
	udelay(1000 * 10);
	gpio_set_value(ETH_PHY_RESET, 1);
}

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	/*
	 * Following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    MicroSD
	 * mmc1                    eMMC
	 */
	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			usdhc_cfg[0].max_bus_width = 4;
			gpio_direction_input(USDHC3_CD_GPIO);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			usdhc_cfg[1].max_bus_width = 8;
			break;
		default:
			printf("Invalid SDHC port");
			return -EINVAL;
		}

		status = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		if (status)
			return status;
	}

	return 0;
}

static int mx6_rgmii_rework(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

/* Ethernet PHY can be located at address 0x1 */
#define ETH_PHY_MASK	(1 << 0x1)

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	setup_iomux_enet();

	bus = fec_get_miibus(base, -1);
	if (!bus)
		return 0;
	/* our phy is at addr 1, just look there */
	phydev = phy_find_by_mask(bus, ETH_PHY_MASK, PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		ret = -EINVAL;
		goto free_bus;
	}

	debug("using phy at address %d\n", phydev->addr);
	ret = fec_probe(bis, -1, IMX_FEC_BASE, bus, phydev);
	if (ret)
		goto free_phydev;

	return 0;

free_phydev:
	free(phydev);
free_bus:
	free(bus);
	return ret;
}

#if defined(CONFIG_VIDEO_IPUV3)
static iomux_v3_cfg_t const rgb_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15,
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,
	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00,
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01,
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02,
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03,
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04,
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05,
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06,
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07,
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08,
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09,
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10,
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11,
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12,
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13,
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14,
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15,
	MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16,
	MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17,
};

static int detect_7_panel_lcd(struct display_info_t const *dev)
{
	if (strstr(env_get("mender_dtb_name"), "g3-7-wvga-lcd"))
		return 1;
	else
		return 0;
}

static int detect_7_panel_ldb(struct display_info_t const *dev)
{
	/* FIXME: return 0 until ldb clock tree is fixed for
	   pixel clocks != 65 Mhz */
	return 0;

	if (strstr(env_get("mender_dtb_name"), "g3-7-wvga-ldb"))
		return 1;
	else
		return 0;
}

static int detect_5_7_panel_lcd(struct display_info_t const *dev)
{
	if (strstr(env_get("mender_dtb_name"), "g3-5p7-vga-lcd"))
		return 1;
	else
		return 0;
}

static int detect_5_7_panel_ldb(struct display_info_t const *dev)
{
	/* FIXME: return 0 until ldb clock tree is fixed for
	   pixel clocks != 65 Mhz */
	return 0;

	if (strstr(env_get("mender_dtb_name"), "g3-5p7-vga-ldb"))
		return 1;
	else
		return 0;
}

static int detect_10_1_panel(struct display_info_t const *dev)
{
	if (strstr(env_get("mender_dtb_name"), "g3-10p1"))
		return 1;
	else
		return 0;
}

static int detect_10_4_panel(struct display_info_t const *dev)
{
	if (strstr(env_get("mender_dtb_name"), "g3-10p4"))
		return 1;
	else
		return 0;
}

static int detect_12_1_panel(struct display_info_t const *dev)
{
	if (strstr(env_get("mender_dtb_name"), "g3-12p1"))
		return 1;
	else
		return 0;
}

static void enable_rgb(struct display_info_t const *dev)
{	
	imx_iomux_v3_setup_multiple_pads(rgb_pads, ARRAY_SIZE(rgb_pads));
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int reg;

	/* U-boot's IPU clock tree support is broken   */
	/* this setup supports only 65Mhz pixel clocks */

	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &ccm->cs2cdr);

	reg = readl(&ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &ccm->cscmr2);

	reg = readl(&ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	      | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
	      | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_HIGH
	      | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	      | IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT
	      | IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	      | IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~IOMUXC_GPR3_LVDS0_MUX_CTL_MASK)
	       | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
		  << IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}

struct display_info_t const displays[] = {{
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= detect_7_panel_lcd,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "7-wvga-lcd",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = KHZ2PICOS(35714),
		.left_margin    = 100,
		.right_margin   = 100,
		.upper_margin   = 56,
		.lower_margin   = 15,
		.hsync_len      = 5,
		.vsync_len      = 10,
		.sync           = FB_SYNC_CLK_LAT_FALL,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_7_panel_ldb,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "7-wvga-ldb",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = KHZ2PICOS(35000),
		.left_margin    = 118,
		.right_margin   = 10,
		.upper_margin   = 35,
		.lower_margin   = 8,
		.hsync_len      = 128,
		.vsync_len      = 45,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= detect_5_7_panel_lcd,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "5p7-vga-lcd",
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.pixclock       = KHZ2PICOS(25175),
		.left_margin    = 48,
		.right_margin   = 16,
		.upper_margin   = 31,
		.lower_margin   = 11,
		.hsync_len      = 96,
		.vsync_len      = 2,
		.sync           = FB_SYNC_CLK_LAT_FALL,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_5_7_panel_ldb,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "5p7-vga-ldb",
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.pixclock       = KHZ2PICOS(25175),
		.left_margin    = 48,
		.right_margin   = 16,
		.upper_margin   = 31,
		.lower_margin   = 11,
		.hsync_len      = 96,
		.vsync_len      = 2,
		.sync           = FB_SYNC_CLK_LAT_FALL,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_10_1_panel,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "10p1-wxga-ldb",
		.refresh        = 60,
		.xres           = 1280,
		.yres           = 800,
		.pixclock       = KHZ2PICOS(65000),
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_10_4_panel,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "10p4-xga-ldb",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = KHZ2PICOS(60000),
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_12_1_panel,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "12p1-wxga-ldb",
		.refresh        = 60,
		.xres           = 1280,
		.yres           = 800,
		.pixclock       = KHZ2PICOS(65000),
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} } };

size_t display_count = ARRAY_SIZE(displays);

int board_video_skip(void)
{
	int i;
	int ret;
	char const *panel;

	for (i = 0; i < display_count; i++) {
		struct display_info_t const *dev = displays+i;
		if (dev->detect && dev->detect(dev)) {
			panel = dev->mode.name;
			break;
		}
	}

	if (i < display_count) {
		ret = ipuv3_fb_init(&displays[i].mode, displays[i].di ? 1 : 0,
				    displays[i].pixfmt);
		if (!ret) {
			if (displays[i].enable)
				displays[i].enable(displays + i);

			printf("Display: %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else
			printf("Display %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
	} else {
		printf("Could not find supported display, disabling splash\n");
		env_set("splashen", "n");
		return -EINVAL;
	}

	return 0;
}

static void setup_display()
{
	enable_ipu_clock();
}

#endif

int board_early_init_f(void)
{
	setup_iomux_uart();

	imx_iomux_v3_setup_multiple_pads(display_enable_pads,
					 ARRAY_SIZE(display_enable_pads));

	gpio_direction_output(DISPLAY_EN, 0);
	gpio_direction_output(BACKLIGHT_EN, 0);
	gpio_direction_output(BACKLIGHT_PWM, 0);

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_late_init(void)
{
	gpio_set_value(DISPLAY_EN, 1);

#if defined(CONFIG_VIDEO_IPUV3)

	/* turn on backlight if splash is enabled */

	if (strstr(env_get("splashen"), "y")) {
		/* check for inverted backlight pwm */
		if (strstr(env_get("mender_dtb_name"), "-inv.dtb"))
			gpio_set_value(BACKLIGHT_PWM, 0);
		else
			gpio_set_value(BACKLIGHT_PWM, 1);

		if (strstr(env_get("mender_dtb_name"), "g3-7"))
			mdelay(400);
		else if (strstr(env_get("mender_dtb_name"), "g3-10p1"))
			mdelay(300);
		else
			mdelay(150);

		gpio_set_value(BACKLIGHT_EN, 1);
	}
#else
	/* check for inverted backlight pwm and ensure it is off */
	if (strstr(env_get("mender_dtb_name"), "-inv.dtb"))
		gpio_set_value(BACKLIGHT_PWM, 1);
	else
		gpio_set_value(BACKLIGHT_PWM, 0);

#endif

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);

	return 0;
}

int checkboard(void)
{
	puts("Board: imx6dl-g3\n");

	return 0;
}

void shutdown_lcd(void)
{
#if defined(CONFIG_VIDEO_IPUV3)
	/* turn off backlight enable and pwm off but leave display enabled */
	gpio_set_value(BACKLIGHT_EN, 0);

	/* check for inverted backlight pwm */
	if (strstr(env_get("mender_dtb_name"), "-inv.dtb"))
		gpio_set_value(BACKLIGHT_PWM, 1);
	else
		gpio_set_value(BACKLIGHT_PWM, 0);
#endif
}
