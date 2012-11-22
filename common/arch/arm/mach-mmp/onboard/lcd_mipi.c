#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/machine.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp3.h>
#include <mach/tc35876x.h>
#include <mach/pxa168fb.h>
#include <plat/i2c.h>

#include "../common.h"

#define FB_XRES         1280
#define FB_YRES         720
static struct fb_videomode video_modes[] = {
	/* innolux WVGA mode info */
	[0] = {
	       .pixclock = 62500,
	       .refresh = 60,
	       .xres = FB_XRES,
	       .yres = FB_YRES,
	       .hsync_len = 2,
	       .left_margin = 10,
	       .right_margin = 116,
	       .vsync_len = 2,
	       .upper_margin = 10,
	       .lower_margin = 4,
	       .sync = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
	       },
};

static struct dsi_info abilene_dsi = {
	.id = 1,
	.lanes = 2,
	.bpp = 24,
	.rgb_mode = DSI_LCD_INPUT_DATA_RGB_MODE_565,
	.burst_mode = DSI_BURST_MODE_SYNC_EVENT,
	.hbp_en = 1,
	.hfp_en = 1,
};

static int tc358765_reset(struct pxa168fb_info *fbi)
{
	int gpio;

	/*
	 * FIXME: It is board related, baceuse zx will be replaced soon,
	 * it is temproary distinguished by cpu
	 */
	if (cpu_is_mmp3_z1() || cpu_is_mmp3_z0())
		gpio = mfp_to_gpio(GPIO135_LCD_RST);
	else
		gpio = mfp_to_gpio(GPIO128_LCD_RST);

	if (gpio_request(gpio, "lcd reset gpio")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return -1;
	}

	gpio_direction_output(gpio, 0);
	mdelay(100);
	gpio_direction_output(gpio, 1);
	mdelay(100);

	gpio_free(gpio);
	return 0;
}
static int dsi_dump_tc358765(void)
{
	u32 val_32, reg;
	int ret = 0;
	/*dump register value*/
	ret = tc35876x_read32(PPI_TX_RX_TA, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - PPI_TX_RX_TA = 0x%x\n", val_32);
	else {
		reg = PPI_TX_RX_TA;
		goto out_dump;
	}

	mdelay(10);
	ret = tc35876x_read32(PPI_LPTXTIMECNT, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - PPI_LPTXTIMECNT = 0x%x\n",
		 val_32);
	else  {
		reg = PPI_LPTXTIMECNT;
		goto out_dump;
	}
	mdelay(10);
	ret  = tc35876x_read32(PPI_D0S_CLRSIPOCOUNT, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - PPI_D0S_CLRSIPOCOUNT = 0x%x\n",
	       val_32);
	else {
		reg = PPI_D0S_CLRSIPOCOUNT;
		goto out_dump;
	}
	mdelay(10);
	ret  = tc35876x_read32(PPI_D1S_CLRSIPOCOUNT, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - PPI_D1S_CLRSIPOCOUNT = 0x%x\n",
	       val_32);
	else {
		reg = PPI_D1S_CLRSIPOCOUNT;
		goto out_dump;
	}
	mdelay(10);
	ret = tc35876x_read32(PPI_LANEENABLE, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - PPI_LANEENABLE = 0x%x\n", val_32);
	else {
		reg = PPI_LANEENABLE;
		goto out_dump;
	}
	mdelay(10);
	ret = tc35876x_read32(DSI_LANEENABLE, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - DSI_LANEENABLE = 0x%x\n", val_32);
	else {
		reg = DSI_LANEENABLE;
		goto out_dump;
	}
	mdelay(10);
	ret = tc35876x_read32(PPI_STARTPPI, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - PPI_STARTPPI = 0x%x\n", val_32);
	else {
		reg = PPI_STARTPPI;
		goto out_dump;
	}
	ret = tc35876x_read32(DSI_STARTDSI, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - DSI_STARTDSI = 0x%x\n", val_32);
	else {
		reg = DSI_STARTDSI;
		goto out_dump;
	}
	mdelay(10);
	ret = tc35876x_read32(VPCTRL, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - VPCTRL = 0x%x\n", val_32);
	else {
		reg = VPCTRL;
		goto out_dump;
	}
	mdelay(10);
	ret = tc35876x_read32(HTIM1, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - HTIM1 = 0x%x\n", val_32);
	else {
		reg = HTIM1;
		goto out_dump;
	}
	mdelay(10);
	ret = tc35876x_read32(HTIM2, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - HTIM2 = 0x%x\n", val_32);
	else {
		reg = HTIM2;
		goto out_dump;
	}
	mdelay(10);
	ret = tc35876x_read32(VTIM1, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - VTIM1 = 0x%x\n", val_32);
	else {
		reg = VTIM1;
		goto out_dump;
	}
	mdelay(10);
	ret = tc35876x_read32(VTIM2, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - VTIM2 = 0x%x\n", val_32);
	else {
		reg = VTIM2;
		goto out_dump;
	}
	mdelay(10);
	ret = tc35876x_read32(VFUEN, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - VFUEN = 0x%x\n", val_32);
	else {
		reg = VFUEN;
		goto out_dump;
	}
	mdelay(10);
	ret = tc35876x_read32(LVCFG, &val_32);
	if (!ret)
		printk(KERN_DEBUG "tc35876x - LVCFG = 0x%x\n", val_32);
	else {
		reg = LVCFG;
		goto out_dump;
	}
	mdelay(10);


	ret = tc35876x_read32(DSI_INTSTAUS, &val_32);
	if (!ret)
		printk(KERN_DEBUG "!!tc35876x - DSI_INTSTAUS= 0x%x BEFORE\n",
	       val_32);
	else {
		reg = DSI_INTSTAUS;
		goto out_dump;
	}

	mdelay(10);
	ret = tc35876x_write32(DSI_INTCLR, 0xFFFFFFFF);
	if (ret < 0)
		printk(KERN_DEBUG "tc35876x - write DSI_INTCLR(0x%x) error.\n",
	       DSI_INTCLR);
	mdelay(10);
	ret = tc35876x_read32(DSI_INTSTAUS, &val_32);
	if (!ret)
		printk(KERN_DEBUG "!!tc35876x - DSI_INTSTAUS= 0x%x AFTER\n",
	       val_32);
	else{
		reg = DSI_INTSTAUS;
		goto out_dump;
	}
	mdelay(10);

out_dump:
	if (ret < 0)
		printk(KERN_DEBUG "tc35876x - read reg(0x%x) failed.\n", reg);
	return ret;
}

static int dsi_set_tc358765(struct pxa168fb_info *fbi)
{
	u16 chip_id = 0;
	int status, reg;

	status = tc35876x_read16(TC358765_CHIPID_REG, &chip_id);
	if ((status < 0) || (chip_id != TC358765_CHIPID)) {
		printk(KERN_WARNING "tc35876x unavailable! chip_id %x\n",
		       chip_id);
		return -EIO;
	} else {
		printk(KERN_INFO "tc35876x(chip id:0x%02x) detected.\n",
		       chip_id);
	}
	/* REG 0x13C,DAT 0x000C000F*/
	status = tc35876x_write32(PPI_TX_RX_TA, 0x000C000F);
	if (status < 0) {
		reg = PPI_TX_RX_TA;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x114,DAT 0x0000000A*/
	status = tc35876x_write32(PPI_LPTXTIMECNT, 0x0000000A);
	if (status < 0) {
		reg = PPI_LPTXTIMECNT;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x164,DAT 0x00000005*/
	status = tc35876x_write32(PPI_D0S_CLRSIPOCOUNT, 0x00000005);
	if (status < 0) {
		reg = PPI_D0S_CLRSIPOCOUNT;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x168,DAT 0x00000005*/
	status = tc35876x_write32(PPI_D1S_CLRSIPOCOUNT, 0x00000005);
	if (status < 0) {
		reg = PPI_D1S_CLRSIPOCOUNT;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x134,DAT 0x00000007*/
	status = tc35876x_write32(PPI_LANEENABLE, 0x0000007);
	if (status < 0) {
		reg = PPI_LANEENABLE;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x210,DAT 0x00000007*/
	status = tc35876x_write32(DSI_LANEENABLE, 0x0000007);
	if (status < 0) {
		reg = DSI_LANEENABLE;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x104,DAT 0x00000001*/
	status = tc35876x_write32(PPI_STARTPPI, 0x0000001);
	if (status < 0) {
		reg = PPI_STARTPPI;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x204,DAT 0x00000001*/
	status = tc35876x_write32(DSI_STARTDSI, 0x0000001);
	if (status < 0) {
		reg = DSI_STARTDSI;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x450,DAT 0x00012020, bit 4 is VTGEN on*/
	status = tc35876x_write32(VPCTRL, 0x00012020);
	if (status < 0) {
		reg = VPCTRL;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x454,DAT 0x000A0002*/
	status = tc35876x_write32(HTIM1, 0x000A0002);
	if (status < 0) {
		reg = HTIM1;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x458,DAT 0x00740500*/
	status = tc35876x_write32(HTIM2, 0x00740500);
	if (status < 0) {
		reg = HTIM2;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x45C,DAT 0x000A0004*/
	status = tc35876x_write32(VTIM1, 0x000A0004);
	if (status < 0) {
		reg = VTIM1;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x460,DAT 0x000402D0*/
	status = tc35876x_write32(VTIM2, 0x000402D0);
	if (status < 0) {
		reg = VTIM2;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x464,DAT 0x00000001*/
	status = tc35876x_write32(VFUEN, 0x00000001);
	if (status < 0) {
		reg = VFUEN;
		goto out_set;
	}
	mdelay(10);
	/*REG 0x49C,DAT 0x00000201*/
	status = tc35876x_write32(LVCFG, 0x00000201);
	if (status < 0) {
		reg = LVCFG;
		goto out_set;
	}
	mdelay(10);

	status = dsi_dump_tc358765();
	return status;
out_set:
	if (status < 0)
		printk(KERN_WARNING "tc35876x write reg(0x%x) failed.\n", reg);
	return status;
}

static int abilene_lcd_power(struct pxa168fb_info *fbi,
			     unsigned int spi_gpio_cs,
			     unsigned int spi_gpio_reset, int on)
{
	struct regulator *v_ldo = NULL;
	int lcd_rst_n;
	/*
	 * FIXME: It is board related, baceuse zx will be replaced soon,
	 * it is temproary distinguished by cpu
	 */
	if (cpu_is_mmp3_z1() || cpu_is_mmp3_z0())
		lcd_rst_n = mfp_to_gpio(GPIO135_LCD_RST);
	else
		lcd_rst_n = mfp_to_gpio(GPIO128_LCD_RST);

	/* set LDOs 17 and 03 to 1.2V for MIPI Bridge */
	if (on) {
		/* v_ldo17 1.2v */
		v_ldo = regulator_get(NULL, "v_ldo17");
		if (IS_ERR(v_ldo))
			v_ldo = NULL;
		else {
			regulator_enable(v_ldo);
			regulator_set_voltage(v_ldo, 1200000, 1200000);
			/* regulator_put(v_ldo); */
		}
		/* v_ldo3 1.2v */
		v_ldo = regulator_get(NULL, "v_ldo3");
		if (IS_ERR(v_ldo))
			v_ldo = NULL;
		else {
			regulator_enable(v_ldo);
			regulator_set_voltage(v_ldo, 1200000, 1200000);
			/* regulator_put(v_ldo); */
		}
	} else {
		/* disable v_ldo03 1.2v */
		v_ldo = regulator_get(NULL, "v_ldo3");
		if (IS_ERR(v_ldo))
			v_ldo = NULL;
		else
			regulator_disable(v_ldo);
		/* disable v_ldo17 1.2v */
		v_ldo = regulator_get(NULL, "v_ldo17");
		if (IS_ERR(v_ldo))
			v_ldo = NULL;
		else
			regulator_disable(v_ldo);
	}

	/* set panel reset */
	if (gpio_request(lcd_rst_n, "lcd reset gpio")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_rst_n);
		return -1;
	}
	if (on)
		gpio_direction_output(lcd_rst_n, 1);
	else
		gpio_direction_output(lcd_rst_n, 0);
	gpio_free(lcd_rst_n);

	printk(KERN_DEBUG "%s on %d\n", __func__, on);
	return 0;
}

static int dsi_init(struct pxa168fb_info *fbi)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	int sclk, ret = 0;

	/* reset DSI controller */
	dsi_reset(fbi, 1);
	mdelay(1);

	/* disable continuous clock */
	dsi_cclk_set(fbi, 0);

	/*  reset the bridge */
	if (mi->xcvr_reset)
		mi->xcvr_reset(fbi);

	/* dsi out of reset */
	dsi_reset(fbi, 0);

	/* set dphy */
	dsi_set_dphy(fbi);

	/* set dsi controller */
	dsi_set_controller(fbi);

	/* turn on DSI continuous clock */
	dsi_cclk_set(fbi, 1);

	/* XM: save original sclk divider & set to 6
	 * divider 6 is a magic number to help adjust phase.
	 * ESC clk = core clk /12 = 800Mhz/12 = 66Mhz. */
	sclk = sclk_div_get(fbi);
	sclk_div_set(fbi, 6);

	/* set dsi to dpi conversion chip */
	if (mi->phy_type == DSI2DPI) {
		ret = mi->dsi2dpi_set(fbi);
		if (ret < 0)
			printk(KERN_WARNING "dsi2dpi_set error!\n");
	}

	/* restore sclk div */
	sclk_div_set(fbi, sclk);
	msleep(1);

	return 0;
}

#define		LCD_ISR_CLEAR_MASK		0xffff00cc

static struct pxa168fb_mach_info abilene_mipi_lcd_info __initdata = {
	.id = "GFX Layer",
	.num_modes = ARRAY_SIZE(video_modes),
	.modes = video_modes,
	.sclk_div = 0xE0001210,
	.pix_fmt = PIX_FMT_RGB565,
	.burst_len = 16,
	.isr_clear_mask	= LCD_ISR_CLEAR_MASK,
	/* don't care about io_pin_allocation_mode and dumb_mode
	 * since the panel is hard connected with lcd panel path and
	 * dsi1 output
	 */
	.panel_rgb_reverse_lanes = 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena = 0,
	.invert_pixclock = 0,
	.panel_rbswap = 0,
	.active = 1,
	.enable_lcd = 1,
	.spi_gpio_cs = -1,
	.spi_gpio_reset = -1,
	.max_fb_size = FB_XRES * FB_YRES * 8 + 4096,
	.phy_type = DSI2DPI,
	.phy_init = dsi_init,
#ifdef CONFIG_TC35876X
	.dsi2dpi_set = dsi_set_tc358765,
	.xcvr_reset = tc358765_reset,
#else
#error Please select CONFIG_TC35876X in menuconfig to enable DSI bridge
#endif
	.dsi = &abilene_dsi,
	.pxa168fb_lcd_power = &abilene_lcd_power,
	.sclk_src = 500000000,
};

static struct pxa168fb_mach_info abilene_mipi_lcd_ovly_info __initdata = {
	.id = "Video Layer",
	.num_modes = ARRAY_SIZE(video_modes),
	.modes = video_modes,
	.pix_fmt = PIX_FMT_RGB565,
	.panel_rgb_reverse_lanes = 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena = 0,
	.invert_pixclock = 0,
	.panel_rbswap = 0,
	.active = 1,
	.enable_lcd = 1,
	.spi_gpio_cs = -1,
	.spi_gpio_reset = -1,
	.max_fb_size = FB_XRES * FB_YRES * 8 + 4096,
	.vdma_enable = 1,
};

void __init abilene_add_lcd_mipi(void)
{
	struct pxa168fb_mach_info *fb = &abilene_mipi_lcd_info, *ovly =
	    &abilene_mipi_lcd_ovly_info;

	/* add frame buffer drivers */
	mmp3_add_fb(fb);
#ifdef CONFIG_PXA168_V4L2_OVERLAY
	mmp3_add_v4l2_ovly(ovly);
#else
	mmp3_add_fb_ovly(ovly);
#endif
}
