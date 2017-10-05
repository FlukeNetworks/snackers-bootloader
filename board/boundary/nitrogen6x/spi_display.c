/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/mx6-pins.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/spi.h>
#include <asm/imx-common/video.h>
#include <spi.h>
#include "spi_display.h"
#include <i2c.h> // KLL_MOD

#define DI0_PAD_CTRL	PAD_CTL_DSE_120ohm

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#if defined(CONFIG_MX6QDL)
#define IOMUX_PAD_CTRL(name, ctrl)	NEW_PAD_CTRL(MX6Q_PAD_##name, ctrl), \
					NEW_PAD_CTRL(MX6DL_PAD_##name, ctrl)
#else
#define IOMUX_PAD_CTRL(name, ctrl)	NEW_PAD_CTRL(MX6_PAD_##name, ctrl)
#endif


static iomux_v3_cfg_t const spi_display_pads[] = {
#ifdef SNACKERS_BOARD
	/* ECSPI5 */
	IOMUX_PAD_CTRL(SD1_CLK__ECSPI5_SCLK, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_CMD__ECSPI5_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(SD1_DAT0__ECSPI5_MISO, SPI_PAD_CTRL),
#define GP_ECSPI5_CS		IMX_GPIO_NR(1, 19)
	IOMUX_PAD_CTRL(SD1_DAT2__GPIO1_IO19, SPI_PAD_CTRL),
#define GP_SPI_DISPLAY_RESET	IMX_GPIO_NR(1, 15)
	IOMUX_PAD_CTRL(SD2_DAT0__GPIO1_IO15, SPI_PAD_CTRL),
#define GP_BACKLIGHT		IMX_GPIO_NR(1, 11)		/* PWM1 */
	IOMUX_PAD_CTRL(SD2_CMD__GPIO1_IO11, SPI_PAD_CTRL),

#else    
	/* ECSPI2 */
	IOMUX_PAD_CTRL(CSI0_DAT8__ECSPI2_SCLK, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT9__ECSPI2_MOSI, SPI_PAD_CTRL),
	IOMUX_PAD_CTRL(CSI0_DAT10__ECSPI2_MISO, SPI_PAD_CTRL),
#define GP_ECSPI2_CS		IMX_GPIO_NR(5, 29)
	IOMUX_PAD_CTRL(CSI0_DAT11__GPIO5_IO29, SPI_PAD_CTRL),
#define GP_SPI_DISPLAY_RESET	IMX_GPIO_NR(4, 20)
	IOMUX_PAD_CTRL(DI0_PIN4__GPIO4_IO20, SPI_PAD_CTRL),
#define GP_BACKLIGHT		IMX_GPIO_NR(1, 21)		/* PWM1 */
	IOMUX_PAD_CTRL(SD1_DAT3__GPIO1_IO21, SPI_PAD_CTRL),

#endif /* SNACKERS_BOARD */
	/* DI0 */
	IOMUX_PAD_CTRL(DI0_DISP_CLK__IPU1_DI0_DISP_CLK, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN15__IPU1_DI0_PIN15, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN2__IPU1_DI0_PIN02, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DI0_PIN3__IPU1_DI0_PIN03, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT0__IPU1_DISP0_DATA00, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT1__IPU1_DISP0_DATA01, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT2__IPU1_DISP0_DATA02, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT3__IPU1_DISP0_DATA03, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT4__IPU1_DISP0_DATA04, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT5__IPU1_DISP0_DATA05, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT6__IPU1_DISP0_DATA06, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT7__IPU1_DISP0_DATA07, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT8__IPU1_DISP0_DATA08, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT9__IPU1_DISP0_DATA09, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT10__IPU1_DISP0_DATA10, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT11__IPU1_DISP0_DATA11, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT12__IPU1_DISP0_DATA12, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT13__IPU1_DISP0_DATA13, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT14__IPU1_DISP0_DATA14, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT15__IPU1_DISP0_DATA15, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT16__IPU1_DISP0_DATA16, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT17__IPU1_DISP0_DATA17, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT18__IPU1_DISP0_DATA18, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT19__IPU1_DISP0_DATA19, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT20__IPU1_DISP0_DATA20, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT21__IPU1_DISP0_DATA21, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT22__IPU1_DISP0_DATA22, DI0_PAD_CTRL),
	IOMUX_PAD_CTRL(DISP0_DAT23__IPU1_DISP0_DATA23, DI0_PAD_CTRL),
};

#if 1 // KLL_MOD
/* NVD ILI9806E uses 9-bit SPI */
#define LCM_SpiWriteCmd(x)  0x00, (x)
#define LCM_SpiWriteData(x) 0x01, (x)
#define LCM_SpiWriteEnd     0x02
#endif

int gNewDisplay = 0;

#if 1 // KLL_MOD

int auo_detect_spi(struct display_info_t const *dev)
{
	/* NVD touch controller responds immediately (i2c bus 2, addr 0x38) */
	/* AUO touch controller requires more delay (i2c bus 2 addr 0x3a) */
	int i;
	i2c_set_bus_num(2);
	for (i=0; i<20; ++i) {
		if (i2c_probe(0x38) == 0) {
			printf("KLL_DEBUG> %s: nvd detected %dms\n", __func__, i);
			debug("%s: nvd detected %dms\n", __func__, i);
			gNewDisplay = 1;
			return 0;
		}
		if (i2c_probe(0x3a) == 0) {
			printf("KLL_DEBUG> %s: auo detected %dms\n", __func__, i);
			debug("%s: auo detected %dms\n", __func__, i);
			return detect_spi(dev);
		}
		mdelay(1);
	}
	printf("KLL_DEBUG> %s: timeout %dms\n", __func__, i);
	debug("%s: timeout %dms\n", __func__, i);

#if 1 // read the hardware id from eeprom...
	{
		ulong dev_addr = CONFIG_SYS_DEF_EEPROM_ADDR;
		ulong addr = 0x00100000; // temporary memory address for results
		ulong off = 0x7fe0; // offset to last 32 bytes in the 32Kbyte EEPROM
		ulong cnt = 0x20; // read 32 bytes
		int rcode = 0;

		rcode = eeprom_read (dev_addr, off, (uchar *) addr, cnt);
		//printf("KLL_DEBUG> %s: eeprom_read() reads hwId into memory at 0x00100000\n", __func__);

		uchar *pHwId = (uchar*)(addr + 3); //0x00100003;
		char szHwId[4+1] = "";

		memcpy(szHwId, pHwId, 4);
		szHwId[4] = 0;
		printf("KLL_DEBUG> %s: hwId= %s\n", __func__, szHwId);
		int hwId = (int)simple_strtoul(szHwId, NULL, 10);
		//if ( strcmp(szHwId, "0004") == 0 )
		if ( hwId >= 4 )
		{
			gNewDisplay = 1;
		}
	}
#endif
	return 0;
}

static int spi_display_cmds(struct spi_slave *spi, u8 *cmds)
{
        int ret = 0;

//printf("KLL_DEBUG> new display 2017-10-02: %s\n", __func__); // KLL_MOD

        debug("%s\n", __func__);
        for (/*null*/; *cmds < LCM_SpiWriteEnd; cmds += 2) {
                ret = spi_xfer(spi, 9, cmds, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
                if (ret) {
                        debug("%s: write failed %#02x,%#02x, %d\n", __func__, cmds[0], cmds[1], ret);
                        return ret;
                }
                udelay(2);
        }
        return ret;
}

static int auo_spi_display_cmds(struct spi_slave *spi, u8 *cmds)
{
	u8 buf[4];
	int ret = 0;

	debug("%s\n", __func__);
	while (1) {
		uint reg = (cmds[0] << 8) | cmds[1];
		uint len = cmds[2];

		if (!len && !reg)
			break;
		cmds += 3;
		do {
			buf[0] = 0x20;
			buf[1] = reg >> 8;
			ret = spi_xfer(spi, 2 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				debug("%s: Failed to select reg1 0x%x, %d\n", __func__, reg, ret);
				return ret;
			}
			udelay(2);
			buf[0] = 0;
			buf[1] = reg;
			ret = spi_xfer(spi, 2 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				debug("%s: Failed to select reg2 0x%x, %d\n", __func__, reg, ret);
				return ret;
			}
			udelay(2);
			if (!len) {
				debug("spi: reg:%04x\n", reg);
				break;
			}
			buf[0] = 0x40;
			buf[1] = *cmds++;
			ret = spi_xfer(spi, 2 * 8, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret) {
				debug("%s: Failed to select reg3 0x%x, %d\n", __func__, reg, ret);
				return ret;
			}
			debug("spi: reg:%04x %02x\n", reg, buf[1]);
			udelay(2);
			reg++;
		} while (--len);
	}
	return ret;
}
#endif

#if 1 // KLL_MOD
static u8 display_init_cmds[] = {
	LCM_SpiWriteCmd(0xFF),
	LCM_SpiWriteData(0xFF),
	LCM_SpiWriteData(0x98),
	LCM_SpiWriteData(0x06),
	LCM_SpiWriteData(0x04),
	LCM_SpiWriteData(0x01),  /* Page 1 Command Set */

	LCM_SpiWriteCmd(0x08),
	LCM_SpiWriteData(0x18),

	LCM_SpiWriteCmd(0x21),
	LCM_SpiWriteData(0x01),

	LCM_SpiWriteCmd(0x30),
	LCM_SpiWriteData(0x01),

	LCM_SpiWriteCmd(0x31),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x40),
	LCM_SpiWriteData(0x10),

	LCM_SpiWriteCmd(0x41),
	LCM_SpiWriteData(0x77),

	LCM_SpiWriteCmd(0x42),
	LCM_SpiWriteData(0x03),

	LCM_SpiWriteCmd(0x43),
	LCM_SpiWriteData(0x89),

	LCM_SpiWriteCmd(0x44),
	LCM_SpiWriteData(0x86),

	LCM_SpiWriteCmd(0x50),
	LCM_SpiWriteData(0x78),

	LCM_SpiWriteCmd(0x51),
	LCM_SpiWriteData(0x78),

	LCM_SpiWriteCmd(0x52),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x53),
	LCM_SpiWriteData(0x3a),

	LCM_SpiWriteCmd(0x57),
	LCM_SpiWriteData(0x50),

	LCM_SpiWriteCmd(0x60),
	LCM_SpiWriteData(0x07),

	LCM_SpiWriteCmd(0x61),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x62),
	LCM_SpiWriteData(0x08),

	LCM_SpiWriteCmd(0x63),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0xA0),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0xA1),
	LCM_SpiWriteData(0x07),

	LCM_SpiWriteCmd(0xA2),
	LCM_SpiWriteData(0x0a),

	LCM_SpiWriteCmd(0xA3),
	LCM_SpiWriteData(0x0b),

	LCM_SpiWriteCmd(0xA4),
	LCM_SpiWriteData(0x04),

	LCM_SpiWriteCmd(0xA5),
	LCM_SpiWriteData(0x08),

	LCM_SpiWriteCmd(0xA6),
	LCM_SpiWriteData(0x06),

	LCM_SpiWriteCmd(0xA7),
	LCM_SpiWriteData(0x04),

	LCM_SpiWriteCmd(0xA8),
	LCM_SpiWriteData(0x07),

	LCM_SpiWriteCmd(0xA9),
	LCM_SpiWriteData(0x0a),

	LCM_SpiWriteCmd(0xAA),
	LCM_SpiWriteData(0x12),

	LCM_SpiWriteCmd(0xAB),
	LCM_SpiWriteData(0x07),

	LCM_SpiWriteCmd(0xAC),
	LCM_SpiWriteData(0x0d),

	LCM_SpiWriteCmd(0xAD),
	LCM_SpiWriteData(0x1d),

	LCM_SpiWriteCmd(0xAE),
	LCM_SpiWriteData(0x16),

	LCM_SpiWriteCmd(0xAF),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0xC0),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0xC1),
	LCM_SpiWriteData(0x07),

	LCM_SpiWriteCmd(0xC2),
	LCM_SpiWriteData(0x0a),

	LCM_SpiWriteCmd(0xC3),
	LCM_SpiWriteData(0x0b),

	LCM_SpiWriteCmd(0xC4),
	LCM_SpiWriteData(0x04),

	LCM_SpiWriteCmd(0xC5),
	LCM_SpiWriteData(0x08),

	LCM_SpiWriteCmd(0xC6),
	LCM_SpiWriteData(0x06),

	LCM_SpiWriteCmd(0xC7),
	LCM_SpiWriteData(0x04),

	LCM_SpiWriteCmd(0xC8),
	LCM_SpiWriteData(0x07),

	LCM_SpiWriteCmd(0xC9),
	LCM_SpiWriteData(0x0a),

	LCM_SpiWriteCmd(0xCA),
	LCM_SpiWriteData(0x12),

	LCM_SpiWriteCmd(0xCB),
	LCM_SpiWriteData(0x07),

	LCM_SpiWriteCmd(0xCC),
	LCM_SpiWriteData(0x0d),

	LCM_SpiWriteCmd(0xCD),
	LCM_SpiWriteData(0x1d),

	LCM_SpiWriteCmd(0xCE),
	LCM_SpiWriteData(0x16),

	LCM_SpiWriteCmd(0xCF),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0xFF),
	LCM_SpiWriteData(0xFF),
	LCM_SpiWriteData(0x98),
	LCM_SpiWriteData(0x06),
	LCM_SpiWriteData(0x04),
	LCM_SpiWriteData(0x06),  /* Page 6 Command Set */

	LCM_SpiWriteCmd(0x00),
	LCM_SpiWriteData(0x21),

	LCM_SpiWriteCmd(0x01),
	LCM_SpiWriteData(0x09),

	LCM_SpiWriteCmd(0x02),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x03),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x04),
	LCM_SpiWriteData(0x01),

	LCM_SpiWriteCmd(0x05),
	LCM_SpiWriteData(0x01),

	LCM_SpiWriteCmd(0x06),
	LCM_SpiWriteData(0x80),

	LCM_SpiWriteCmd(0x07),
	LCM_SpiWriteData(0x05),

	LCM_SpiWriteCmd(0x08),
	LCM_SpiWriteData(0x02),

	LCM_SpiWriteCmd(0x09),
	LCM_SpiWriteData(0x80),

	LCM_SpiWriteCmd(0x0A),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x0B),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x0C),
	LCM_SpiWriteData(0x0a),

	LCM_SpiWriteCmd(0x0D),
	LCM_SpiWriteData(0x0a),

	LCM_SpiWriteCmd(0x0E),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x0F),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x10),
	LCM_SpiWriteData(0xE0),

	LCM_SpiWriteCmd(0x11),
	LCM_SpiWriteData(0xE4),

	LCM_SpiWriteCmd(0x12),
	LCM_SpiWriteData(0x04),

	LCM_SpiWriteCmd(0x13),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x14),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x15),
	LCM_SpiWriteData(0xC0),

	LCM_SpiWriteCmd(0x16),
	LCM_SpiWriteData(0x08),

	LCM_SpiWriteCmd(0x17),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x18),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x19),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x1A),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x1B),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x1C),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x1D),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x20),
	LCM_SpiWriteData(0x01),

	LCM_SpiWriteCmd(0x21),
	LCM_SpiWriteData(0x23),

	LCM_SpiWriteCmd(0x22),
	LCM_SpiWriteData(0x45),

	LCM_SpiWriteCmd(0x23),
	LCM_SpiWriteData(0x67),

	LCM_SpiWriteCmd(0x24),
	LCM_SpiWriteData(0x01),

	LCM_SpiWriteCmd(0x25),
	LCM_SpiWriteData(0x23),

	LCM_SpiWriteCmd(0x26),
	LCM_SpiWriteData(0x45),

	LCM_SpiWriteCmd(0x27),
	LCM_SpiWriteData(0x67),

	LCM_SpiWriteCmd(0x30),
	LCM_SpiWriteData(0x01),

	LCM_SpiWriteCmd(0x31),
	LCM_SpiWriteData(0x11),

	LCM_SpiWriteCmd(0x32),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x33),
	LCM_SpiWriteData(0xee),

	LCM_SpiWriteCmd(0x34),
	LCM_SpiWriteData(0xff),

	LCM_SpiWriteCmd(0x35),
	LCM_SpiWriteData(0xBB),

	LCM_SpiWriteCmd(0x36),
	LCM_SpiWriteData(0xcA),

	LCM_SpiWriteCmd(0x37),
	LCM_SpiWriteData(0xDD),

	LCM_SpiWriteCmd(0x38),
	LCM_SpiWriteData(0xaC),

	LCM_SpiWriteCmd(0x39),
	LCM_SpiWriteData(0x76),

	LCM_SpiWriteCmd(0x3A),
	LCM_SpiWriteData(0x67),

	LCM_SpiWriteCmd(0x3B),
	LCM_SpiWriteData(0x22),

	LCM_SpiWriteCmd(0x3C),
	LCM_SpiWriteData(0x22),

	LCM_SpiWriteCmd(0x3D),
	LCM_SpiWriteData(0x22),

	LCM_SpiWriteCmd(0x3E),
	LCM_SpiWriteData(0x22),

	LCM_SpiWriteCmd(0x3F),
	LCM_SpiWriteData(0x22),

	LCM_SpiWriteCmd(0x40),
	LCM_SpiWriteData(0x22),

	LCM_SpiWriteCmd(0x52),
	LCM_SpiWriteData(0x10),

	LCM_SpiWriteCmd(0x53),
	LCM_SpiWriteData(0x10),

	LCM_SpiWriteCmd(0xFF),
	LCM_SpiWriteData(0xFF),
	LCM_SpiWriteData(0x98),
	LCM_SpiWriteData(0x06),
	LCM_SpiWriteData(0x04),
	LCM_SpiWriteData(0x07), /* Page 7 Command Set */

	LCM_SpiWriteCmd(0x17),
	LCM_SpiWriteData(0x22),

	LCM_SpiWriteCmd(0x02),
	LCM_SpiWriteData(0x77),

	LCM_SpiWriteCmd(0xe1),
	LCM_SpiWriteData(0x79),

	LCM_SpiWriteCmd(0x26),
	LCM_SpiWriteData(0xb2),

	LCM_SpiWriteCmd(0xFF),
	LCM_SpiWriteData(0xFF),
	LCM_SpiWriteData(0x98),
	LCM_SpiWriteData(0x06),
	LCM_SpiWriteData(0x04),
	LCM_SpiWriteData(0x00), /* Page 0 Command Set */

	LCM_SpiWriteCmd(0x3A),
	LCM_SpiWriteData(0x70),

	LCM_SpiWriteCmd(0x36),
	LCM_SpiWriteData(0x00),

	LCM_SpiWriteCmd(0x11),
	LCM_SpiWriteEnd
};

#define A(reg, cnt) (reg >> 8), (reg & 0xff), cnt

static u8 auo_display_init_cmds[] = {
/* Display Mode Setting */
	A(0xf000, 5), 0x55, 0xaa, 0x52, 0x08, 0x00,
	A(0xb100, 2), 0x0c, 0x00,
	A(0xbc00, 3), 0x05, 0x05, 0x05,
	A(0xb700, 2), 0x22, 0x22,
	A(0xb800, 4), 0x01, 0x03, 0x03, 0x03,
	A(0xc803, 1), 0x96,
	A(0xc805, 1), 0x96,
	A(0xc807, 1), 0x96,
	A(0xc809, 1), 0x96,
	A(0xc80b, 1), 0x2a,
	A(0xc80c, 1), 0x2a,
	A(0xc80f, 1), 0x2a,
	A(0xc810, 1), 0x2a,
	A(0xf000, 5), 0x55, 0xaa, 0x52, 0x08, 0x01,
	A(0xb900, 3), 0x34, 0x34, 0x34,
	A(0xba00, 3), 0x14, 0x14, 0x14,
	A(0xbe00, 2), 0x00, 0x8c,
	A(0xb000, 3), 0x00, 0x00, 0x00,
	A(0xb800, 3), 0x24, 0x24, 0x24,
	A(0xbc00, 3), 0x00, 0x88, 0x01,
	A(0xbd00, 3), 0x00, 0x88, 0x01,
	A(0xd100, 52),  0x00, 0x00, 0x00, 0x10, 0x00, 0x31, 0x00, 0x5a, 0x00, 0x78, 0x00, 0x9b, 0x00, 0xbe, 0x00, 0xe6, 0x01, 0x04,
			0x01, 0x36, 0x01, 0x59, 0x01, 0x90, 0x01, 0xbd, 0x01, 0xbe, 0x01, 0xe5, 0x02, 0x0d, 0x02, 0x29, 0x02, 0x44,
			0x02, 0x5d, 0x02, 0xbc, 0x02, 0xe9, 0x03, 0x16, 0x03, 0x48, 0x03, 0xac, 0x03, 0xe8, 0x03, 0xff,
	A(0xd200, 52),  0x00, 0x00, 0x00, 0x10, 0x00, 0x31, 0x00, 0x5a, 0x00, 0x78, 0x00, 0x9b, 0x00, 0xbe, 0x00, 0xe6, 0x01, 0x04,
			0x01, 0x36, 0x01, 0x59, 0x01, 0x90, 0x01, 0xbd, 0x01, 0xbe, 0x01, 0xe5, 0x02, 0x0d, 0x02, 0x29, 0x02, 0x44,
			0x02, 0x5d, 0x02, 0xbc, 0x02, 0xe9, 0x03, 0x16, 0x03, 0x48, 0x03, 0xac, 0x03, 0xe8, 0x03, 0xff,
	A(0xd300, 52),  0x00, 0x00, 0x00, 0x10, 0x00, 0x31, 0x00, 0x5a, 0x00, 0x78, 0x00, 0x9b, 0x00, 0xbe, 0x00, 0xe6, 0x01, 0x04,
			0x01, 0x36, 0x01, 0x59, 0x01, 0x90, 0x01, 0xbd, 0x01, 0xbe, 0x01, 0xe5, 0x02, 0x0d, 0x02, 0x29, 0x02, 0x44,
			0x02, 0x5d, 0x02, 0xbc, 0x02, 0xe9, 0x03, 0x16, 0x03, 0x48, 0x03, 0xac, 0x03, 0xe8, 0x03, 0xff,
	A(0xd400, 52),  0x00, 0x00, 0x00, 0x10, 0x00, 0x31, 0x00, 0x5a, 0x00, 0x78, 0x00, 0x9b, 0x00, 0xbe, 0x00, 0xe6, 0x01, 0x04,
			0x01, 0x36, 0x01, 0x59, 0x01, 0x90, 0x01, 0xbd, 0x01, 0xbe, 0x01, 0xe5, 0x02, 0x0d, 0x02, 0x29, 0x02, 0x44,
			0x02, 0x5d, 0x02, 0xbc, 0x02, 0xe9, 0x03, 0x16, 0x03, 0x48, 0x03, 0xac, 0x03, 0xe8, 0x03, 0xff,
	A(0xd500, 52),  0x00, 0x00, 0x00, 0x10, 0x00, 0x31, 0x00, 0x5a, 0x00, 0x78, 0x00, 0x9b, 0x00, 0xbe, 0x00, 0xe6, 0x01, 0x04,
			0x01, 0x36, 0x01, 0x59, 0x01, 0x90, 0x01, 0xbd, 0x01, 0xbe, 0x01, 0xe5, 0x02, 0x0d, 0x02, 0x29, 0x02, 0x44,
			0x02, 0x5d, 0x02, 0xbc, 0x02, 0xe9, 0x03, 0x16, 0x03, 0x48, 0x03, 0xac, 0x03, 0xe8, 0x03, 0xff,
	A(0xd600, 52),  0x00, 0x00, 0x00, 0x10, 0x00, 0x31, 0x00, 0x5a, 0x00, 0x78, 0x00, 0x9b, 0x00, 0xbe, 0x00, 0xe6, 0x01, 0x04,
			0x01, 0x36, 0x01, 0x59, 0x01, 0x90, 0x01, 0xbd, 0x01, 0xbe, 0x01, 0xe5, 0x02, 0x0d, 0x02, 0x29, 0x02, 0x44,
			0x02, 0x5d, 0x02, 0xbc, 0x02, 0xe9, 0x03, 0x16, 0x03, 0x48, 0x03, 0xac, 0x03, 0xe8, 0x03, 0xff,
	A(0x1100, 0),	/* exit sleep mode, wait 120 ms */
	A(0, 0)
};
#endif

#if 1 // KLL_MOD
static u8 display_on_cmds[] = {
        LCM_SpiWriteCmd(0x29),
        LCM_SpiWriteEnd
};

static u8 auo_display_on_cmds[] = {
	A(0x2900, 0),
	A(0, 0)
};
#endif


int modifyVideoArgs(char *szVideoargs)
{
  char args_mod[200] = "";

  //printf("args_orig: %s\n", szVideoargs);
  //printf("len videoargs= %d\n", strlen(szVideoargs));

  strcpy(args_mod, szVideoargs);
  char *pDisplay = strstr(args_mod, "dev=lcd,");
  if ( !pDisplay )
    return 1;

  pDisplay += strlen("dev=lcd,");
  *pDisplay = 0; // truncate remainder of string
  //printf("args_mod: %s\n", args_mod);

  char *pSuffix = strstr(szVideoargs, ",if=RGB24");
  if ( !pSuffix )
    return 1;

  if ( gNewDisplay )
  {
    strcat(args_mod, "NVD_HSD050"); // 480x854
  }
  else
  {
    strcat(args_mod, "AUO_G050");  // 480x800
  }

  strcat(args_mod, pSuffix); 
  //printf("args_mod: %s\n", args_mod);
  strcpy(szVideoargs, args_mod);

  return 0;
}

void enable_spi_rgb(struct display_info_t const *dev)
{
#ifdef SNACKERS_BOARD
	unsigned cs_gpio = GP_ECSPI5_CS;
#else
	unsigned cs_gpio = GP_ECSPI2_CS;
#endif

	struct spi_slave *spi;
	int ret;

#if 1 // KLL_MOD
        /* save resolution for bootargs (see common/image_android.c) */
        char res_str[16];
        sprintf(res_str, "%dx%d", dev->mode.xres, dev->mode.yres);
        int err = setenv("res", res_str);
        //printf("KLL_DEBUG> %s: gNewDisplay= %d\n", __func__, gNewDisplay);
        //printf("KLL_DEBUG> %s: setenv res %s\n", __func__, res_str);
#endif

#if 1 // KLL_MOD 
        char commandline[200] = "";
        char *videoargs = getenv("videoargs");
        strcpy(commandline, videoargs);

        //char *res_env = getenv("res");
        //if (res_env && strcmp(res_env, "480x854")==0) gNewDisplay = 1;

	modifyVideoArgs(commandline);
        err = setenv("videoargs", commandline);
        //printf("KLL_DEBUG> %s: setenv videoargs \"%s\"\n", __func__, commandline);
#endif

	gpio_direction_output(GP_BACKLIGHT, 1);
	gpio_direction_output(cs_gpio, 1);

	enable_spi_clk(1, dev->bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(dev->bus, cs_gpio << 8, 1000000, SPI_MODE_0);
	if (!spi) {
		printf("%s: Failed to set up slave\n", __func__);
		return;
	}

	/* Claim spi bus */
	ret = spi_claim_bus(spi);
	if (ret) {
		debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
		goto free_bus;
	}

	/*
	 * Initialization sequence
	 * 1. Display Mode Settings
	 * 2. Power Settings
	 * 3. Gamma Settings
	 * 4. Sleep Out
	 * 5. Wait >= 7 frame
	 * 6. Display on
	 */
	ret = spi_display_cmds(spi, display_init_cmds);
	if (ret) {
		printf("%s: Failed to display_init_cmds %d\n", __func__, ret);
		goto release_bus;
	}
	mdelay(200);
	ret = spi_display_cmds(spi, display_on_cmds);
	if (ret) {
		printf("%s: Failed to display_on_cmds %d\n", __func__, ret);
		goto release_bus;
	}
	ret = 1;

	/* Release spi bus */
release_bus:
	spi_release_bus(spi);
free_bus:
	spi_free_slave(spi);
	enable_spi_clk(0, dev->bus);
	return;
}

/*
 * Return 1 for successful detection of display
 */
int detect_spi(struct display_info_t const *dev)
{
#ifdef SNACKERS_BOARD
	unsigned cs_gpio = GP_ECSPI5_CS;
#else
	unsigned cs_gpio = GP_ECSPI2_CS;
#endif

	unsigned reset_gpio = GP_SPI_DISPLAY_RESET;

	debug("%s\n", __func__);
	gpio_direction_output(cs_gpio, 1);
	gpio_direction_output(reset_gpio, 1);
	SETUP_IOMUX_PADS(spi_display_pads);
	gpio_direction_output(reset_gpio, 0);
	udelay(200);
	gpio_direction_output(reset_gpio, 1);
	mdelay(200);
	return 1;
}

#if 1 // KLL_MOD
void auo_enable_spi_rgb(struct display_info_t const *dev)
{
	unsigned cs_gpio = GP_ECSPI5_CS;
	struct spi_slave *spi;
	int ret;

	gpio_direction_output(GP_BACKLIGHT, 1);
	gpio_direction_output(cs_gpio, 1);

	enable_spi_clk(1, dev->bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(dev->bus, cs_gpio << 8, 1000000, SPI_MODE_0);
	if (!spi) {
		printf("%s: Failed to set up slave\n", __func__);
		return;
	}

	/* Claim spi bus */
	ret = spi_claim_bus(spi);
	if (ret) {
		debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
		goto free_bus;
	}

	/*
	 * Initialization sequence
	 * 1. Display Mode Settings
	 * 2. Power Settings
	 * 3. Gamma Settings
	 * 4. Sleep Out
	 * 5. Wait >= 7 frame
	 * 6. Display on
	 */
	ret = auo_spi_display_cmds(spi, auo_display_init_cmds);
	if (ret) {
		printf("%s: Failed to display_init_cmds %d\n", __func__, ret);
		goto release_bus;
	}
	mdelay(200);
	ret = auo_spi_display_cmds(spi, auo_display_on_cmds);
	if (ret) {
		printf("%s: Failed to display_on_cmds %d\n", __func__, ret);
		goto release_bus;
	}
	ret = 1;

	/* Release spi bus */
release_bus:
	spi_release_bus(spi);
free_bus:
	spi_free_slave(spi);
	enable_spi_clk(0, dev->bus);
	return;
}
#endif

static int do_spid(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
#ifdef SNACKERS_BOARD
	unsigned cs_gpio = GP_ECSPI5_CS;
#else
	unsigned cs_gpio = GP_ECSPI2_CS;
#endif
	struct spi_slave *spi;
	int ret = 0;
	int arg = 2;
#ifdef SNACKERS_BOARD
	int bus = 4;
#else
	int bus = 1;
#endif
	uint reg;
	u8 buf[80];

	if (argc < 2)
		return 1;
	gpio_direction_output(GP_BACKLIGHT, 1);
	gpio_direction_output(cs_gpio, 1);

	enable_spi_clk(1, bus);

	/* Setup spi_slave */
	spi = spi_setup_slave(bus, cs_gpio << 8, 1000000, SPI_MODE_0);
	if (!spi) {
		printf("%s: Failed to set up slave\n", __func__);
		return 1;
	}

	/* Claim spi bus */
	ret = spi_claim_bus(spi);
	if (ret) {
		debug("%s: Failed to claim SPI bus: %d\n", __func__, ret);
		goto free_bus;
	}

	if (argc > ARRAY_SIZE(buf) - 3)
		argc = ARRAY_SIZE(buf) - 3;

	reg = simple_strtoul(argv[1], NULL, 16);
	buf[0] = reg >> 8;
	buf[1] = reg;
	buf[2] = argc - arg;
	while (arg < argc) {
		buf[arg + 1] = simple_strtoul(argv[arg], NULL, 16);
		arg++;
	}
	arg++;
	buf[arg++] = 0;
	buf[arg++] = 0;
	buf[arg++] = 0;
	spi_display_cmds(spi, buf);
	spi_release_bus(spi);
free_bus:
	spi_free_slave(spi);
	enable_spi_clk(0, bus);
	return ret ? 1 : 0;
}

U_BOOT_CMD(
	spid, 70, 0, do_spid,
	"write cmd, data to spi display",
	"reg16 [byte]"
);

