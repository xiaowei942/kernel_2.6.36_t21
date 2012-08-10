/* linux/arch/arm/mach-s3c64xx/mach-mini6410.c
 *
 * Copyright 2010 FriendlyARM (www.arm9.net)
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/gfp.h>
#include <linux/dm9000.h>
#include <linux/gpio_keys.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/proc_fs.h>
#include <linux/usb/android_composite.h>

#include <video/platform_lcd.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/regs-fb.h>
#include <mach/map.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <mach/regs-modem.h>
#include <mach/regs-gpio.h>
#include <mach/regs-sys.h>
#include <mach/regs-srom.h>
#include <plat/iic.h>
#include <plat/fb.h>
#include <plat/gpio-cfg.h>
#include <plat/nand.h>

#include <mach/s3c6410.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/adc.h>
#include <mach/ts.h>
#include <plat/regs-usb-hsotg-phy.h>
#include <plat/udc-hs.h>
#include <plat/audio.h>
#include <plat/fimc.h>

#include <linux/mmc/host.h>
#include <plat/sdhci.h>

#include <plat/keypad.h>
#include <linux/input.h>

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

extern void s3c64xx_reserve_bootmem(void);
extern int s3c_media_read_proc(char *buf, char **start, off_t offset,
		int count, int *eof, void *data);

static struct s3c2410_uartcfg mini6410_uartcfgs[] __initdata = {
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
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
};

/* framebuffer and LCD setup. */
static void mini6410_lcd_power_set(struct plat_lcd_data *pd,
		unsigned int power)
{
	/* Add you power up/down control here */
}

static struct plat_lcd_data mini6410_lcd_power_data = {
	.set_power	= mini6410_lcd_power_set,
};

static struct platform_device mini6410_lcd_powerdev = {
	.name			= "platform-lcd",
	.dev.parent		= &s3c_device_fb.dev,
	.dev.platform_data	= &mini6410_lcd_power_data,
};

static struct s3c_fb_pd_win mini6410_fb_win0 = {
	/* this is to ensure we use win0 */
	.win_mode	= {
#if 0
		.pixclock	= 115440,
#endif
		.left_margin	= 0x03,
		.right_margin	= 0x02,
		.upper_margin	= 0x01,
		.lower_margin	= 0x01,
		.hsync_len	= 0x28,
		.vsync_len	= 0x01,
		.xres		= 480,
		.yres		= 272,
	},
	.max_bpp	= 32,
	.default_bpp	= 16,
};

/* 405566 clocks per frame => 60Hz refresh requires 24333960Hz clock */
static struct s3c_fb_platdata mini6410_lcd_pdata __initdata = {
	.setup_gpio	= s3c64xx_fb_gpio_setup_24bpp,
	.win[0]		= &mini6410_fb_win0,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
};

/* MMC/SD config */
static struct s3c_sdhci_platdata mini6410_hsmmc0_pdata = {
    .max_width      = 4,
    .cd_type        = S3C_SDHCI_CD_INTERNAL,
};

static struct s3c_sdhci_platdata mini6410_hsmmc1_pdata = {
    .max_width      = 4,
    .cd_type        = S3C_SDHCI_CD_PERMANENT,
};

/* Nand flash */
struct mtd_partition mini6410_nand_part[] = {
	{
		.name		= "Bootloader",
		.offset		= 0,
		.size		= (4 * 128 *SZ_1K),
		.mask_flags	= MTD_CAP_NANDFLASH,
	},
	{
		.name		= "Kernel",
		.offset		= (4 * 128 *SZ_1K),
		.size		= (5*SZ_1M) ,
		.mask_flags	= MTD_CAP_NANDFLASH,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	}
};

static struct s3c2410_nand_set mini6410_nand_sets[] = {
	[0] = {
		.name       = "nand",
		.nr_chips   = 1,
		.nr_partitions  = ARRAY_SIZE(mini6410_nand_part),
		.partitions = mini6410_nand_part,
	},
};

static struct s3c2410_platform_nand mini6410_nand_info = {
	.tacls      = 25,
	.twrph0     = 55,
	.twrph1     = 40,
	.nr_sets    = ARRAY_SIZE(mini6410_nand_sets),
	.sets       = mini6410_nand_sets,
};

#ifdef CONFIG_USB_SUPPORT
/* Initializes OTG Phy. to output 48M clock */
void s3c_otg_phy_config(int enable) {
	u32 val;

	if (enable) {
		__raw_writel(0x0, S3C_PHYPWR);	/* Power up */

		val = __raw_readl(S3C_PHYCLK);
		val &= ~S3C_PHYCLK_CLKSEL_MASK;
		__raw_writel(val, S3C_PHYCLK);

		__raw_writel(0x1, S3C_RSTCON);
		udelay(5);
		__raw_writel(0x0, S3C_RSTCON);	/* Finish the reset */
		udelay(5);
	} else {
		__raw_writel(0x19, S3C_PHYPWR);	/* Power down */
	}
}
EXPORT_SYMBOL(s3c_otg_phy_config);
#endif

static struct s3c_hsotg_plat mini6410_hsotg_data __initdata = {
	.dma			= S3C_HSOTG_DMA_DRV,
	.is_osc			= 0,
};

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_all[] = {
	"usb_mass_storage",
	"adb",
};

static struct android_usb_product usb_products[] = {
	{
		.product_id		= 0x4e11,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions		= usb_functions_ums,
	},
	{
		.product_id		= 0x4e12,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions		= usb_functions_ums_adb,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns      = 1,
	.vendor     = "FriendlyARM",
	.product    = "Mini6410",
	.release    = 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name   = "usb_mass_storage",
	.id = -1,
	.dev    = {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id		= 0x18d1,
	.product_id		= 0x4e11,
	.version		= 0x0100,
	.product_name	= "Mini6410",
	.manufacturer_name	= "FriendlyARM",
	.num_products	= ARRAY_SIZE(usb_products),
	.products		= usb_products,
	.num_functions	= ARRAY_SIZE(usb_functions_all),
	.functions		= usb_functions_all,
};

static struct platform_device s3c_device_android_usb = {
	.name			= "android_usb",
	.id				= -1,
	.dev			= {
		.platform_data = &android_usb_pdata,
	},
};

/* Ethernet */
#ifdef CONFIG_DM9000
#define S3C64XX_PA_DM9000	(0x18000000)
#define S3C64XX_SZ_DM9000	SZ_1M
#define S3C64XX_VA_DM9000	S3C_ADDR(0x03b00300)

static struct resource dm9000_resources[] = {
	[0] = {
		.start		= S3C64XX_PA_DM9000,
		.end		= S3C64XX_PA_DM9000 + 3,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= S3C64XX_PA_DM9000 + 4,
		.end		= S3C64XX_PA_DM9000 + S3C64XX_SZ_DM9000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[2] = {
		.start		= IRQ_EINT(7),
		.end		= IRQ_EINT(7),
		.flags		= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	},
};

static struct dm9000_plat_data dm9000_setup = {
	.flags			= DM9000_PLATF_16BITONLY,
	.dev_addr		= { 0x08, 0x90, 0x00, 0xa0, 0x90, 0x90 },
};

static struct platform_device s3c_device_dm9000 = {
	.name			= "dm9000",
	.id				= 0,
	.num_resources	= ARRAY_SIZE(dm9000_resources),
	.resource		= dm9000_resources,
	.dev			= {
		.platform_data = &dm9000_setup,
	}
};

static int __init dm9000_set_mac(char *str) {
	unsigned char addr[6];
	unsigned int val;
	int idx = 0;
	char *p = str, *end;

	while (*p && idx < 6) {
		val = simple_strtoul(p, &end, 16);
		if (end <= p) {
			/* convert failed */
			break;
		} else {
			addr[idx++] = val;
			p = end;
			if (*p == ':'|| *p == '-') {
				p++;
			} else {
				break;
			}
		}
	}

	if (idx == 6) {
		printk("Setup ethernet address to %pM\n", addr);
		memcpy(dm9000_setup.param_addr, addr, 6);
	}

	return 1;
}

__setup("ethmac=", dm9000_set_mac);
#endif

static struct gpio_keys_button gpio_buttons[] = {
	{
		.gpio		= S3C64XX_GPN(1),
		.code		= 116,
		.desc		= "POWER",
		.active_low	= 0,
		.wakeup		= 0,
	}, {
		.gpio		= S3C64XX_GPN(2),
		.code		= 102,
		.desc		= "HOME",
		.active_low	= 1,
		.wakeup		= 0,
	}, {
		.gpio		= S3C64XX_GPN(3),
		.code		= 42,
		.desc		= "LEFT-SHIFT",
		.active_low	= 1,
		.wakeup		= 0,
	}
	
/*	
	{
		.gpio		= S3C64XX_GPN(0),
		.code		= 158,
		.desc		= "HOME",
		.active_low	= 1,
		.wakeup		= 0,
	}, {
		.gpio		= S3C64XX_GPL(11),
		.code		= 102,
		.desc		= "BACK",
		.active_low	= 1,
		.wakeup		= 0,
	}, {
		.gpio		= S3C64XX_GPN(1),
		.code		= 139,
		.desc		= "MENU",
		.active_low	= 1,
		.wakeup		= 0,
	}, {
		.gpio		= S3C64XX_GPL(12),
		.code		= 232,
		.desc		= "DPAD_CENTER",
		.active_low	= 1,
		.wakeup		= 0,
	}, {
		.gpio		= S3C64XX_GPN(3),
		.code		= 105,
		.desc		= "DPAD_LEFT",
		.active_low	= 1,
		.wakeup		= 0,
	}, {
		.gpio		= S3C64XX_GPN(4),
		.code		= 108,
		.desc		= "DPAD_DOWN",
		.active_low	= 1,
		.wakeup		= 0,
	}, {
		.gpio		= S3C64XX_GPN(5),
		.code		= 103,
		.desc		= "DPAD_UP",
		.active_low	= 1,
		.wakeup		= 0,
	}, {
		.gpio		= S3C64XX_GPN(2),
		.code		= 106,
		.desc		= "DPAD_RIGHT",
		.active_low	= 1,
		.wakeup		= 0,
	}
*/



};

static struct gpio_keys_platform_data gpio_button_data = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device s3c_device_gpio_btns = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &gpio_button_data,
	}
};

static struct map_desc mini6410_iodesc[] = {
	{
		/* LCD support */
		.virtual    = (unsigned long)S3C_VA_LCD,
		.pfn        = __phys_to_pfn(S3C_PA_FB),
		.length     = SZ_16K,
		.type       = MT_DEVICE,
	},
#ifdef CONFIG_DM9000
	{
		.virtual	= (u32)S3C64XX_VA_DM9000,
		.pfn		= __phys_to_pfn(S3C64XX_PA_DM9000),
		.length		= S3C64XX_SZ_DM9000,
		.type		= MT_DEVICE,
	},
#endif
};

static struct platform_device *mini6410_devices[] __initdata = {
#ifdef CONFIG_MINI6410_SD_CH0
	&s3c_device_hsmmc0,
#endif
#ifdef CONFIG_MINI6410_SD_CH1
	&s3c_device_hsmmc1,
#endif
	&s3c_device_i2c0,
#ifdef CONFIG_S3C_DEV_I2C1
	&s3c_device_i2c1,
#endif
	&s3c_device_nand,
	&s3c_device_fb,
	&s3c_device_ohci,
	&s3c_device_usb_hsotg,

//////////////  WEI  //////////////
#if 0
#ifdef CONFIG_SND_S3C_SOC_AC97
	&s3c64xx_device_ac97,
#else
	&s3c64xx_device_iisv4,
#endif
#endif

	&s3c64xx_device_pcm1,
	&s3c64xx_device_iis0,
//////////////  WEI ///////////////

	&mini6410_lcd_powerdev,
	&usb_mass_storage_device,

#ifdef CONFIG_DM9000
	&s3c_device_dm9000,
#endif
#ifdef CONFIG_S3C_ADC
	&s3c_device_adc,
#endif
#if defined(CONFIG_TOUCHSCREEN_MINI6410) || defined(CONFIG_SAMSUNG_DEV_TS)
	&s3c_device_ts,
#endif

#if defined(CONFIG_SAMSUNG_DEV_KEYPAD)
	&samsung_device_keypad,
#endif
	&s3c_device_wdt,
#ifdef CONFIG_S3C_DEV_RTC
	&s3c_device_rtc,
#endif
	&s3c_device_gpio_btns,
	&s3c_device_android_usb,

	/* Multimedia support */
#ifdef CONFIG_VIDEO_SAMSUNG
	&s3c_device_vpp,
	&s3c_device_mfc,
	&s3c_device_tvenc,
	&s3c_device_tvscaler,
	&s3c_device_rotator,
	&s3c_device_jpeg,
	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_g2d,
	&s3c_device_g3d,
#endif
};

static struct i2c_board_info i2c_devs0[] __initdata = {
	{ I2C_BOARD_INFO("wm8993", 0x1a), },
};

static struct i2c_board_info i2c_devs1[] __initdata = {
	/* Add your i2c device here */
};

#ifdef CONFIG_SAMSUNG_DEV_TS
static struct s3c2410_ts_mach_info s3c_ts_platform __initdata = {
	.delay			= 10000,
	.presc			= 49,
	.oversampling_shift	= 2,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_MINI6410
static struct s3c_ts_mach_info s3c_ts_platform __initdata = {
	.delay			= 0xFFFF,
	.presc			= 0xFF,
	.oversampling_shift	= 2,
	.resol_bit		= 12,
	.s3c_adc_con	= ADC_TYPE_2,
};
#endif

#ifdef CONFIG_SAMSUNG_DEV_KEYPAD
static uint32_t mini6410_keymap[] __initdata = {
	/* KEY(row, col, keycode) */
/*	KEY(0, 3, KEY_1), KEY(0, 4, KEY_2), KEY(0, 5, KEY_3),
	KEY(0, 6, KEY_4), KEY(0, 7, KEY_5),
	KEY(1, 3, KEY_A), KEY(1, 4, KEY_B), KEY(1, 5, KEY_C),
	KEY(1, 6, KEY_D), KEY(1, 7, KEY_E)
*/
	/* Number Matrix */
	KEY(1,1,KEY_0),
	KEY(4,2,KEY_1),
	KEY(4,1,KEY_2),
	KEY(4,0,KEY_3),

	KEY(3,2,KEY_4),
	KEY(3,1,KEY_5),
	KEY(3,0,KEY_6),
	
	KEY(2,2,KEY_7),
	KEY(2,1,KEY_8),
	KEY(2,0,KEY_9),
	
	KEY(7,1,KEY_UP),
	KEY(7,2,KEY_DOWN),
	KEY(6,2,KEY_LEFT),
	KEY(7,0,KEY_RIGHT),


	KEY(0,1,KEY_BACK),//右上角按键
	KEY(0,2,KEY_MENU), //左上角按键
/*	KEY(1,2,KEY_), 	//For "*"
	KEY(,,KEY_), 	//For "#"
*/


	KEY(6,0,KEY_ESC),
	KEY(6,1,KEY_OK),
	KEY(5,2,KEY_CANCEL),
	KEY(5,1,KEY_BACKSPACE),
	KEY(5,0,KEY_REPLY),
};

static struct matrix_keymap_data mini6410_keymap_data __initdata = {
	.keymap		= mini6410_keymap,
	.keymap_size	= ARRAY_SIZE(mini6410_keymap),
};


static struct samsung_keypad_platdata mini6410_keypad_platform __initdata = {
	.keymap_data 		= &mini6410_keymap_data,
	.rows 			= 8,
	.cols 			= 8,

};
#endif
static void __init mini6410_map_io(void)
{
	u32 tmp;

	s3c64xx_init_io(mini6410_iodesc, ARRAY_SIZE(mini6410_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(mini6410_uartcfgs, ARRAY_SIZE(mini6410_uartcfgs));

	/* set the LCD type */

	tmp = __raw_readl(S3C64XX_SPCON);
	tmp &= ~S3C64XX_SPCON_LCD_SEL_MASK;
	tmp |= S3C64XX_SPCON_LCD_SEL_RGB;
	__raw_writel(tmp, S3C64XX_SPCON);

	/* remove the lcd bypass */
	tmp = __raw_readl(S3C64XX_MODEM_MIFPCON);
	tmp &= ~MIFPCON_LCD_BYPASS;
	__raw_writel(tmp, S3C64XX_MODEM_MIFPCON);

#ifdef CONFIG_VIDEO_SAMSUNG
	s3c64xx_reserve_bootmem();
#endif
}

static void __init mini6410_machine_init(void)
{
	u32 cs1;

	s3c_i2c0_set_platdata(NULL);
#ifdef CONFIG_S3C_DEV_I2C1
	s3c_i2c1_set_platdata(NULL);
#endif

	s3c_fb_set_platdata(&mini6410_lcd_pdata);

#ifdef CONFIG_SAMSUNG_DEV_TS
	s3c24xx_ts_set_platdata(&s3c_ts_platform);
#endif
#ifdef CONFIG_TOUCHSCREEN_MINI6410
	s3c_ts_set_platdata(&s3c_ts_platform);
#endif

	s3c_sdhci0_set_platdata(&mini6410_hsmmc0_pdata);
	s3c_sdhci1_set_platdata(&mini6410_hsmmc1_pdata);
	s3c_hsotg_set_platdata(&mini6410_hsotg_data);
#ifdef CONFIG_SAMSUNG_DEV_KEYPAD
	samsung_keypad_set_platdata(&mini6410_keypad_platform);
#endif
#ifdef CONFIG_MTD_NAND_S3C
	s3c_device_nand.name = "s3c6410-nand";
#endif
	s3c_nand_set_platdata(&mini6410_nand_info);

///////////////  WEI  //////////////	
	//s3c64xx_ac97_setup_gpio(0);
///////////////  WEI  //////////////

	/* configure nCS1 width to 16 bits */

	cs1 = __raw_readl(S3C64XX_SROM_BW) &
		    ~(S3C64XX_SROM_BW__CS_MASK << S3C64XX_SROM_BW__NCS1__SHIFT);
	cs1 |= ((1 << S3C64XX_SROM_BW__DATAWIDTH__SHIFT) |
		(1 << S3C64XX_SROM_BW__WAITENABLE__SHIFT) |
		(1 << S3C64XX_SROM_BW__BYTEENABLE__SHIFT)) <<
						   S3C64XX_SROM_BW__NCS1__SHIFT;
	__raw_writel(cs1, S3C64XX_SROM_BW);

	/* set timing for nCS1 suitable for ethernet chip */

	__raw_writel((0 << S3C64XX_SROM_BCX__PMC__SHIFT) |
		     (6 << S3C64XX_SROM_BCX__TACP__SHIFT) |
		     (4 << S3C64XX_SROM_BCX__TCAH__SHIFT) |
		     (1 << S3C64XX_SROM_BCX__TCOH__SHIFT) |
		     (0xe << S3C64XX_SROM_BCX__TACC__SHIFT) |
		     (4 << S3C64XX_SROM_BCX__TCOS__SHIFT) |
		     (0 << S3C64XX_SROM_BCX__TACS__SHIFT), S3C64XX_SROM_BC1);

	/* Request GPIOs (gpio_request) here if needed */

	if (ARRAY_SIZE(i2c_devs0)) {
		i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));
	}
	if (ARRAY_SIZE(i2c_devs1)) {
		i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));
	}

#ifdef CONFIG_S3C64XX_DEV_FIMC0
	s3c_fimc0_set_platdata(NULL);
#endif
#ifdef CONFIG_S3C64XX_DEV_FIMC1
	s3c_fimc1_set_platdata(NULL);
#endif

	platform_add_devices(mini6410_devices, ARRAY_SIZE(mini6410_devices));

#ifdef CONFIG_VIDEO_SAMSUNG
	create_proc_read_entry("videomem", 0, NULL, s3c_media_read_proc, NULL);
#endif
}

MACHINE_START(MINI6410, "MINI6410")
	/* Maintainer: Ben Dooks <ben-linux@fluff.org> */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C64XX_PA_SDRAM + 0x100,

	.init_irq	= s3c6410_init_irq,
	.map_io		= mini6410_map_io,
	.init_machine	= mini6410_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
