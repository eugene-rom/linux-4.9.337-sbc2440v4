/* linux/arch/arm/mach-s3c2440/mach-sbc2440v4.c
 *
 * Copyright (c) 2008 Ramax Lo <ramaxlo@gmail.com>
 *      Based on mach-anubis.c by Ben Dooks <ben@simtec.co.uk>
 *      and modifications by SBZ <sbz@spgui.org> and
 *      Weibing <http://weibing.blogbus.com> and
 *      Michel Pollet <buserror@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/serial_s3c.h>
#include <linux/dm9000.h>
#include <linux/mmc/host.h>
#include <linux/spi/s3c24xx.h>
#if defined(WITH_ADS7846)
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#endif

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/fb.h>
#include <asm/mach-types.h>

#include <mach/regs-gpio.h>
#include <linux/platform_data/leds-s3c24xx.h>
#include <mach/regs-lcd.h>
#include <mach/gpio-samsung.h>
#include <linux/platform_data/mtd-nand-s3c2410.h>
#include <linux/platform_data/i2c-s3c2410.h>
#include <linux/platform_data/mmc-s3cmci.h>
#include <linux/platform_data/usb-s3c2410_udc.h>
#include <linux/platform_data/touchscreen-s3c2410.h>
#include <linux/mtd/partitions.h>

#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/samsung-time.h>

#include <sound/s3c24xx_uda134x.h>

#include "regs-mem.h"
#include "common.h"

#define MACH_SBC2440V4_DM9K_BASE (S3C2410_CS4 + 0x300)

static struct map_desc sbc2440v4_iodesc[] __initdata = {
	/* nothing to declare, move along */
};

#define UCON S3C2410_UCON_DEFAULT
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE


static struct s3c2410_uartcfg sbc2440v4_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
};

/* USB device UDC support */
static struct s3c2410_udc_mach_info sbc2440v4_udc_cfg __initdata = {
	.pullup_pin = S3C2410_GPG(12),
};


/* LCD timing and setup */

/*
 * This macro simplifies the table bellow
 */
#define _LCD_DECLARE(_clock,_xres,margin_left,margin_right,hsync, \
			_yres,margin_top,margin_bottom,vsync, refresh) \
	.width = _xres, \
	.xres = _xres, \
	.height = _yres, \
	.yres = _yres, \
	.left_margin	= margin_left,	\
	.right_margin	= margin_right,	\
	.upper_margin	= margin_top,	\
	.lower_margin	= margin_bottom,	\
	.hsync_len	= hsync,	\
	.vsync_len	= vsync,	\
	.pixclock	= ((_clock*100000000000LL) /	\
			   ((refresh) * \
			   (hsync + margin_left + _xres + margin_right) * \
			   (vsync + margin_top + _yres + margin_bottom))), \
	.bpp		= 16,\
	.type		= (S3C2410_LCDCON1_TFT16BPP |\
			   S3C2410_LCDCON1_TFT)

static struct s3c2410fb_display sbc2440v4_lcd_cfg[] __initdata = {
	[0] = {	/* sbc2440v4 + 3.5" TFT + touchscreen */
		_LCD_DECLARE(
			7,			/* The 3.5 is quite fast */
			240, 21, 38, 6, 	/* x timing */
			320, 4, 4, 2,		/* y timing */
			60),			/* refresh rate */
		.lcdcon5	= (S3C2410_LCDCON5_FRM565 |
				   S3C2410_LCDCON5_INVVLINE |
				   S3C2410_LCDCON5_INVVFRAME |
				   S3C2410_LCDCON5_INVVDEN |
				   S3C2410_LCDCON5_PWREN),
	},
	[1] = { /* sbc2440v4 + 7" TFT + touchscreen */
		_LCD_DECLARE(
			10,			/* the 7" runs slower */
			800, 40, 40, 48, 	/* x timing */
			480, 29, 3, 3,		/* y timing */
			50),			/* refresh rate */
		.lcdcon5	= (S3C2410_LCDCON5_FRM565 |
				   S3C2410_LCDCON5_INVVLINE |
				   S3C2410_LCDCON5_INVVFRAME |
				   S3C2410_LCDCON5_PWREN),
	},
	/* The VGA shield can outout at several resolutions. All share 
	 * the same timings, however, anything smaller than 1024x768
	 * will only be displayed in the top left corner of a 1024x768
	 * XGA output unless you add optional dip switches to the shield.
	 * Therefore timings for other resolutions have been omitted here.
	 */
	[2] = {
		_LCD_DECLARE(
			10,
			1024, 1, 2, 2,		/* y timing */
			768, 200, 16, 16, 	/* x timing */
			24),	/* refresh rate, maximum stable,
				 tested with the FPGA shield */
		.lcdcon5	= (S3C2410_LCDCON5_FRM565 |
				   S3C2410_LCDCON5_HWSWP),
	},
	/* sbc2440v4 + 3.5" TFT (LCD-W35i, LQ035Q1DG06 type) + touchscreen*/
	[3] = {
		_LCD_DECLARE(
			/* clock */
			7,
			/* xres, margin_right, margin_left, hsync */
			320, 68, 66, 4,
			/* yres, margin_top, margin_bottom, vsync */
			240, 4, 4, 9,
			/* refresh rate */
			60),
		.lcdcon5	= (S3C2410_LCDCON5_FRM565 |
				   S3C2410_LCDCON5_INVVDEN |
				   S3C2410_LCDCON5_INVVFRAME |
				   S3C2410_LCDCON5_INVVLINE |
				   S3C2410_LCDCON5_INVVCLK |
				   S3C2410_LCDCON5_HWSWP),
	},
	[4] = { /* sbc2440v4 + 8" TFT L80 + touchscreen */
		_LCD_DECLARE(
			10,
			640, 200, 16, 16, 	/* x timing */
			480, 2, 2, 2,		/* y timing */
			50),			/* refresh rate */
		.lcdcon5	= (S3C2410_LCDCON5_FRM565 |
				   S3C2410_LCDCON5_HWSWP),
	},
};

/* todo - put into gpio header */

#define S3C2410_GPCCON_MASK(x)	(3 << ((x) * 2))
#define S3C2410_GPDCON_MASK(x)	(3 << ((x) * 2))

static struct s3c2410fb_mach_info sbc2440v4_fb_info __initdata = {
	.displays	 = &sbc2440v4_lcd_cfg[0], /* not constant! see init */
	.num_displays	 = 1,
	.default_display = 0,

	/* Enable VD[2..7], VD[10..15], VD[18..23] and VCLK, syncs, VDEN
	 * and disable the pull down resistors on pins we are using for LCD
	 * data. */

	.gpcup		= (0xf << 1) | (0x3f << 10),

	.gpccon		= (S3C2410_GPC1_VCLK   | S3C2410_GPC2_VLINE |
			   S3C2410_GPC3_VFRAME | S3C2410_GPC4_VM |
			   S3C2410_GPC10_VD2   | S3C2410_GPC11_VD3 |
			   S3C2410_GPC12_VD4   | S3C2410_GPC13_VD5 |
			   S3C2410_GPC14_VD6   | S3C2410_GPC15_VD7),

	.gpccon_mask	= (S3C2410_GPCCON_MASK(1)  | S3C2410_GPCCON_MASK(2)  |
			   S3C2410_GPCCON_MASK(3)  | S3C2410_GPCCON_MASK(4)  |
			   S3C2410_GPCCON_MASK(10) | S3C2410_GPCCON_MASK(11) |
			   S3C2410_GPCCON_MASK(12) | S3C2410_GPCCON_MASK(13) |
			   S3C2410_GPCCON_MASK(14) | S3C2410_GPCCON_MASK(15)),

	.gpdup		= (0x3f << 2) | (0x3f << 10),

	.gpdcon		= (S3C2410_GPD2_VD10  | S3C2410_GPD3_VD11 |
			   S3C2410_GPD4_VD12  | S3C2410_GPD5_VD13 |
			   S3C2410_GPD6_VD14  | S3C2410_GPD7_VD15 |
			   S3C2410_GPD10_VD18 | S3C2410_GPD11_VD19 |
			   S3C2410_GPD12_VD20 | S3C2410_GPD13_VD21 |
			   S3C2410_GPD14_VD22 | S3C2410_GPD15_VD23),

	.gpdcon_mask	= (S3C2410_GPDCON_MASK(2)  | S3C2410_GPDCON_MASK(3) |
			   S3C2410_GPDCON_MASK(4)  | S3C2410_GPDCON_MASK(5) |
			   S3C2410_GPDCON_MASK(6)  | S3C2410_GPDCON_MASK(7) |
			   S3C2410_GPDCON_MASK(10) | S3C2410_GPDCON_MASK(11)|
			   S3C2410_GPDCON_MASK(12) | S3C2410_GPDCON_MASK(13)|
			   S3C2410_GPDCON_MASK(14) | S3C2410_GPDCON_MASK(15)),
};

/* MMC/SD  */

static struct s3c24xx_mci_pdata sbc2440v4_mmc_cfg __initdata = {
   .gpio_detect   = S3C2410_GPG(8),
   .gpio_wprotect = S3C2410_GPH(8),
   .set_power     = NULL,
   .ocr_avail     = MMC_VDD_32_33|MMC_VDD_33_34,
};

/* NAND Flash on SBC2440V4 board */

static struct mtd_partition sbc2440v4_default_nand_part[] __initdata = {
	[0] = {
		.name	= "bootloader",
		.size	= 0x00030000,
		.offset	= 0,
	},
	[1] = {
		.name	= "kernel",
		.offset = 0x00050000,
		.size	= 0x00200000,
	},
	[2] = {
		.name	= "root",
		.offset = 0x00250000,
		.size	= 0x03bac000,
	},
	[3] = {
		.name	= "varlib",
		.offset = 0x03dfc000,
		.size	= 0x001f4000, // leave room for flash_bbt
	}
};

static struct s3c2410_nand_set sbc2440v4_nand_sets[] __initdata = {
	[0] = {
		.name		= "nand",
		.nr_chips	= 1,
		.nr_partitions	= ARRAY_SIZE(sbc2440v4_default_nand_part),
		.partitions	= sbc2440v4_default_nand_part,
		.flash_bbt 	= 1,
	},
};

static struct s3c2410_platform_nand sbc2440v4_nand_info __initdata = {
	.tacls		= 1,
	.twrph0		= 60,
	.twrph1		= 30,
	.nr_sets	= ARRAY_SIZE(sbc2440v4_nand_sets),
	.sets		= sbc2440v4_nand_sets,
	.ignore_unset_ecc = 1,
};

/* DM9000AEP 10/100 ethernet controller */

static struct resource sbc2440v4_dm9k_resource[] = {
	[0] = DEFINE_RES_MEM(MACH_SBC2440V4_DM9K_BASE, 4),
	[1] = DEFINE_RES_MEM(MACH_SBC2440V4_DM9K_BASE + 4, 4),
	[2] = DEFINE_RES_NAMED(IRQ_EINT7, 1, NULL, IORESOURCE_IRQ \
						| IORESOURCE_IRQ_HIGHEDGE),
};

/*
 * The DM9000 has no eeprom, and it's MAC address is set by
 * the bootloader before starting the kernel.
 */
static struct dm9000_plat_data sbc2440v4_dm9k_pdata = {
	.flags		= (DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM),
};

static struct platform_device sbc2440v4_device_eth = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sbc2440v4_dm9k_resource),
	.resource	= sbc2440v4_dm9k_resource,
	.dev		= {
		.platform_data	= &sbc2440v4_dm9k_pdata,
	},
};

/* LEDS */

static struct s3c24xx_led_platdata sbc2440v4_led1_pdata = {
	.name		= "led1",
	.gpio		= S3C2410_GPB(5),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.def_trigger	= "heartbeat",
};

static struct s3c24xx_led_platdata sbc2440v4_led2_pdata = {
	.name		= "led2",
	.gpio		= S3C2410_GPB(6),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.def_trigger	= "nand-disk",
};

static struct s3c24xx_led_platdata sbc2440v4_led3_pdata = {
	.name		= "led3",
	.gpio		= S3C2410_GPB(7),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.def_trigger	= "mmc0",
};

static struct s3c24xx_led_platdata sbc2440v4_led4_pdata = {
	.name		= "led4",
	.gpio		= S3C2410_GPB(8),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.def_trigger	= "",
};

static struct platform_device sbc2440v4_led1 = {
	.name		= "s3c24xx_led",
	.id		= 1,
	.dev		= {
		.platform_data	= &sbc2440v4_led1_pdata,
	},
};

static struct platform_device sbc2440v4_led2 = {
	.name		= "s3c24xx_led",
	.id		= 2,
	.dev		= {
		.platform_data	= &sbc2440v4_led2_pdata,
	},
};

static struct platform_device sbc2440v4_led3 = {
	.name		= "s3c24xx_led",
	.id		= 3,
	.dev		= {
		.platform_data	= &sbc2440v4_led3_pdata,
	},
};

static struct platform_device sbc2440v4_led4 = {
	.name		= "s3c24xx_led",
	.id		= 4,
	.dev		= {
		.platform_data	= &sbc2440v4_led4_pdata,
	},
};

/* AUDIO */

static struct s3c24xx_uda134x_platform_data sbc2440v4_audio_pins = {
	.l3_clk = S3C2410_GPB(4),
	.l3_mode = S3C2410_GPB(2),
	.l3_data = S3C2410_GPB(3),
	.model = UDA134X_UDA1341
};

static struct platform_device sbc2440v4_audio = {
	.name		= "s3c24xx_uda134x",
	.id		= 0,
	.dev		= {
		.platform_data	= &sbc2440v4_audio_pins,
	},
};

static struct uda134x_platform_data s3c24xx_uda134x = {
	.l3 = {
		.gpio_clk = S3C2410_GPB(4),
		.gpio_data = S3C2410_GPB(3),
		.gpio_mode = S3C2410_GPB(2),
		.use_gpios = 1,
		.data_hold = 1,
		.data_setup = 1,
		.clock_high = 1,
		.mode_hold = 1,
		.mode = 1,
		.mode_setup = 1,
	},
	.model = UDA134X_UDA1341,
};

static struct platform_device uda1340_codec = {
		.name = "uda134x-codec",
		.id = -1,
		.dev = {
			.platform_data	= &s3c24xx_uda134x,
		},
};

static struct s3c2410_ts_mach_info sbc2440v4_ts_cfg __initdata = {
	.delay			= 10000,
	.presc			= 49,
	.oversampling_shift	= 2,
	/* S3C2440 uses internal TS FETs; do NOT set cfg_gpio (GPG12 is USB_EN) */
};

static struct platform_device *sbc2440v4_devices[] __initdata = {
	&s3c_device_ohci,
	&s3c_device_wdt,
	&s3c_device_i2c0,
	&s3c_device_rtc,
	&s3c_device_adc,
	&s3c_device_spi0,
	&s3c_device_usbgadget,
	&sbc2440v4_device_eth,
	&sbc2440v4_led1,
	&sbc2440v4_led2,
	&sbc2440v4_led3,
	&sbc2440v4_led4,
	&s3c_device_nand,
	&s3c_device_sdi,
	&s3c2440_device_dma,
	&s3c_device_iis,
	&uda1340_codec,
	&sbc2440v4_audio,
};

static void __init sbc2440v4_map_io(void)
{
	s3c24xx_init_io(sbc2440v4_iodesc, ARRAY_SIZE(sbc2440v4_iodesc));
	s3c24xx_init_uarts(sbc2440v4_uartcfgs, ARRAY_SIZE(sbc2440v4_uartcfgs));
	samsung_set_timer_source(SAMSUNG_PWM3, SAMSUNG_PWM4);
}

static void __init sbc2440v4_init_time(void)
{
	s3c2440_init_clocks(12000000);
	samsung_timer_init();
}

/*
 * sbc2440v4_features string
 *
 * t = Touchscreen present
 * b = backlight / LCD power enable (GPG4)
 * 0-9 LCD configuration
 *
 */
static char sbc2440v4_features_str[12] __initdata = "0tb";

static int __init sbc2440v4_features_setup(char *str)
{
	if (str)
		strlcpy(sbc2440v4_features_str, str, sizeof(sbc2440v4_features_str));
	return 1;
}

__setup("sbc2440v4=", sbc2440v4_features_setup);

#define FEATURE_SCREEN		(1 << 0)
#define FEATURE_BACKLIGHT	(1 << 1)
#define FEATURE_TOUCH		(1 << 2)

struct sbc2440v4_features_t {
	int count;
	int done;
	int lcd_index;
	struct platform_device *optional[8];
};

static void __init sbc2440v4_parse_features(
		struct sbc2440v4_features_t * features,
		const char * features_str )
{
	const char * fp = features_str;

	features->count = 0;
	features->done = 0;
	features->lcd_index = -1;

	while (*fp) {
		char f = *fp++;

		switch (f) {
		case '0'...'9':	/* tft screen */
			if (features->done & FEATURE_SCREEN) {
				printk(KERN_INFO "SBC2440V4: '%c' ignored, "
					"screen type already set\n", f);
			} else {
				int li = f - '0';
				if (li >= ARRAY_SIZE(sbc2440v4_lcd_cfg))
					printk(KERN_INFO "SBC2440V4: "
						"'%c' out of range LCD mode\n", f);
				else {
					features->optional[features->count++] =
							&s3c_device_lcd;
					features->lcd_index = li;
				}
			}
			features->done |= FEATURE_SCREEN;
			break;
		case 'b':
			if (features->done & FEATURE_BACKLIGHT)
				printk(KERN_INFO "SBC2440V4: '%c' ignored, "
					"backlight already set\n", f);
			features->done |= FEATURE_BACKLIGHT;
			break;
		case 't':
			if (!IS_ENABLED(CONFIG_TOUCHSCREEN_S3C2410)) {
				printk(KERN_INFO "SBC2440V4: '%c' ignored, "
					"touchscreen not compiled in\n", f);
			} else if (features->done & FEATURE_TOUCH) {
				printk(KERN_INFO "SBC2440V4: '%c' ignored, "
					"touchscreen already set\n", f);
			} else {
				features->optional[features->count++] =
					&s3c_device_ts;
				features->done |= FEATURE_TOUCH;
			}
			break;
		}
	}
}

#define S3C2410_BWSCON_DW4_16	(1 << 16)
#define S3C2410_BWSCON_WS4	(S3C2410_BWSCON_WS << 16)

/*
 * setup for DM9000.
 */
static void __init sbc2440v4_dm9000_bus_timing_init(void)
{
	u32 bwsc_old, bwsc_new, bank4_val;

	bwsc_old = readl(S3C2410_BWSCON);

	bwsc_new = (bwsc_old & ~(0xf << 16)) |
		   S3C2410_BWSCON_DW4_16 |
		   S3C2410_BWSCON_WS4 |
		   S3C2410_BWSCON_ST4;

	writel(bwsc_new, S3C2410_BWSCON);

	bank4_val = (0 << S3C2410_BANKCON_Tacs_SHIFT) |
		    (3 << S3C2410_BANKCON_Tcos_SHIFT) |
		    (7 << S3C2410_BANKCON_Tacc_SHIFT) |
		    (1 << S3C2410_BANKCON_Tcoh_SHIFT) |
		    (3 << S3C2410_BANKCON_Tcah_SHIFT) |
		    (3 << S3C2410_BANKCON_Tacp_SHIFT);

	writel(bank4_val, S3C2410_BANKCON4);
	bank4_val = readl(S3C2410_BANKCON4);

	pr_info("sbc2440v4: BWSCON old=0x%08x new=0x%08x BANKCON4=0x%08x\n",
		bwsc_old, bwsc_new, bank4_val);
}

#if defined(WITH_ADS7846)
/*
 * XPT2046 / ADS7846 SPI touchscreen on SPI0 CS1.
 * Set TS_CS_GPIO to any free output GPIO and TS_IRQ_GPIO to a free EINT pin
 * (e.g. GPF1, GPF4, GPG5, GPG6, GPG7 on CON2).
 */
#define SBC2440V4_TS_CS_GPIO	S3C2410_GPJ(0)
#define SBC2440V4_TS_IRQ_GPIO	S3C2410_GPF(1)

static const int sbc2440v4_spi0_cs_pins[] = {
	S3C2410_GPG(2),		/* CS0: ST7789V */
	SBC2440V4_TS_CS_GPIO,	/* CS1: XPT2046 / ADS7846 */
};

static void sbc2440v4_spi0_set_cs(struct s3c2410_spi_info *spi, int cs, int pol)
{
	if (cs >= 0 && cs < ARRAY_SIZE(sbc2440v4_spi0_cs_pins))
		gpio_set_value(sbc2440v4_spi0_cs_pins[cs], pol);
}

static struct ads7846_platform_data sbc2440v4_ts_pdata = {
	.model			= 7846,
	.vref_delay_usecs	= 100,
	.x_plate_ohms		= 400,
	.y_plate_ohms		= 400,
	.pressure_max		= 1024,
	.debounce_max		= 10,
	.debounce_tol		= 3,
	.debounce_rep		= 1,
	.gpio_pendown		= SBC2440V4_TS_IRQ_GPIO,
};

static struct spi_board_info sbc2440v4_spi_board_info[] __initdata = {
	{
		.modalias	= "ads7846",
		.max_speed_hz	= 2000000,
		.bus_num	= 0,
		.chip_select	= 1,
		.mode		= SPI_MODE_0,
		.platform_data	= &sbc2440v4_ts_pdata,
	},
};
#endif

static void sbc2440v4_spi0_gpio_setup(struct s3c2410_spi_info *spi, int enable)
{
	if (enable)
		s3c_gpio_cfgall_range(S3C2410_GPE(11), 3,
				      S3C_GPIO_SFN(2), S3C_GPIO_PULL_NONE);
}

static struct s3c2410_spi_info sbc2440v4_spi0_cfg = {
#if defined(WITH_ADS7846)
	.pin_cs		= -1,
	.num_cs		= ARRAY_SIZE(sbc2440v4_spi0_cs_pins),
	.set_cs		= sbc2440v4_spi0_set_cs,
#else
	.pin_cs		= S3C2410_GPG(2),
	.num_cs		= 1,
#endif
	.bus_num	= 0,
	.use_fiq	= 1,
	.gpio_setup	= sbc2440v4_spi0_gpio_setup,
};

static void __init sbc2440v4_init(void)
{
	struct sbc2440v4_features_t features = { 0 };

	sbc2440v4_dm9000_bus_timing_init();

	printk(KERN_INFO "SBC2440V4: Option string sbc2440v4=%s\n", sbc2440v4_features_str);

	/* Parse the feature string */
	sbc2440v4_parse_features(&features, sbc2440v4_features_str);

	if (features.lcd_index != -1 || (features.done & FEATURE_BACKLIGHT)) {
		/* Enable LCD_PWR (GPG4 -> LCD41P pin 30) */
		WARN_ON(gpio_request_one(S3C2410_GPG(4),
					 GPIOF_OUT_INIT_HIGH, "lcd-pwr"));
		gpio_free(S3C2410_GPG(4));
	}

	if (features.lcd_index != -1) {
		sbc2440v4_fb_info.displays =
			&sbc2440v4_lcd_cfg[features.lcd_index];

		pr_info("SBC2440V4: LCD [%d:%dx%d]\n", features.lcd_index,
			sbc2440v4_lcd_cfg[features.lcd_index].width,
			sbc2440v4_lcd_cfg[features.lcd_index].height);

		s3c24xx_fb_set_platdata(&sbc2440v4_fb_info);
	}


	/* setup spi */
	s3c_device_spi0.dev.platform_data = &sbc2440v4_spi0_cfg;
#if defined(WITH_ADS7846)
	{
		int i, irq;

		for (i = 0; i < ARRAY_SIZE(sbc2440v4_spi0_cs_pins); i++)
			WARN_ON(gpio_request_one(sbc2440v4_spi0_cs_pins[i],
						 GPIOF_OUT_INIT_HIGH, "spi0-cs"));

		irq = gpio_to_irq(SBC2440V4_TS_IRQ_GPIO);
		if (WARN_ON(irq < 0)) {
			pr_err("SBC2440V4: invalid TS IRQ GPIO %d\n",
			       SBC2440V4_TS_IRQ_GPIO);
		} else {
			sbc2440v4_spi_board_info[0].irq = irq;
			spi_register_board_info(sbc2440v4_spi_board_info,
						ARRAY_SIZE(sbc2440v4_spi_board_info));
		}
	}
#endif

	if (features.done & FEATURE_TOUCH)
		s3c24xx_ts_set_platdata(&sbc2440v4_ts_cfg);

	s3c24xx_udc_set_platdata(&sbc2440v4_udc_cfg);
	s3c24xx_mci_set_platdata(&sbc2440v4_mmc_cfg);
	s3c_nand_set_platdata(&sbc2440v4_nand_info);
	s3c_i2c0_set_platdata(NULL);

	platform_add_devices(sbc2440v4_devices, ARRAY_SIZE(sbc2440v4_devices));

	if (features.count)	/* the optional features */
		platform_add_devices(features.optional, features.count);

}


MACHINE_START(SBC2440V4, "SBC2440V4")
	.atag_offset	= 0x100,
	.map_io		= sbc2440v4_map_io,
	.init_machine	= sbc2440v4_init,
	.init_irq	= s3c2440_init_irq,
	.init_time	= sbc2440v4_init_time,
MACHINE_END
