/*
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#undef MX6PAD
#undef MX6NAME

#ifdef FOR_DL_SOLO
#define MX6PAD(a) MX6DL_PAD_##a
#define MX6NAME(a) mx6dl_solo_##a
#else
#define MX6PAD(a) MX6Q_PAD_##a
#define MX6NAME(a) mx6q_##a
#endif


#define AUD_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define CSI_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define BUTTON_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_SRE_SLOW)

#define WEAK_PULLDN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define OUTPUT_40OHM (PAD_CTL_SPEED_MED|PAD_CTL_DSE_40ohm)

static const iomux_v3_cfg_t MX6NAME(bt_pads)[] = {
	/* ECSPI1 pads */
	NEW_PAD_CTRL(MX6PAD(EIM_D17__ECSPI1_MISO), SPI_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(EIM_D18__ECSPI1_MOSI), SPI_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(EIM_D16__ECSPI1_SCLK), SPI_PAD_CTRL),
#define GP_ECSPI1_SS1	IMX_GPIO_NR(3, 19)
	NEW_PAD_CTRL(MX6PAD(EIM_D19__GPIO3_IO19), SPI_PAD_CTRL), /* SS1 */

	/* ENET pads that don't change for PHY reset */
	NEW_PAD_CTRL(MX6PAD(ENET_MDIO__ENET_MDIO), ENET_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(ENET_MDC__ENET_MDC), ENET_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(RGMII_TXC__RGMII_TXC), ENET_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(RGMII_TD0__RGMII_TD0), ENET_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(RGMII_TD1__RGMII_TD1), ENET_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(RGMII_TD2__RGMII_TD2), ENET_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(RGMII_TD3__RGMII_TD3), ENET_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(RGMII_TX_CTL__RGMII_TX_CTL), ENET_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(ENET_REF_CLK__ENET_TX_CLK), ENET_PAD_CTRL),
	/* pin 42 PHY nRST */
#define GP_ENET_PHY_RESET	IMX_GPIO_NR(1, 27)
	NEW_PAD_CTRL(MX6PAD(ENET_RXD0__GPIO1_IO27), OUTPUT_40OHM),
#define GP_ENET_PHY_INT		IMX_GPIO_NR(1, 28)
	NEW_PAD_CTRL(MX6PAD(ENET_TX_EN__GPIO1_IO28), WEAK_PULLUP),	/* Micrel RGMII Phy Interrupt */

	/* gpios J91  */
	/* grounds, 1,2,11,12,29,46,47,48,49,50 */
#define GP_BT_GPIO1	IMX_GPIO_NR(2, 15)
	NEW_PAD_CTRL(MX6PAD(SD4_DAT7__GPIO2_IO15), WEAK_PULLUP),	/* bt_gpio1, pin 3 */
#define GP_BT_GPIO2	IMX_GPIO_NR(2, 14)
	NEW_PAD_CTRL(MX6PAD(SD4_DAT6__GPIO2_IO14), WEAK_PULLUP),	/* bt_gpio2, pin 4 */
#define GP_BT_GPIO3	IMX_GPIO_NR(2, 13)
	NEW_PAD_CTRL(MX6PAD(SD4_DAT5__GPIO2_IO13), WEAK_PULLUP),	/* bt_gpio3, pin 5 */
#define GP_BT_GPIO4	IMX_GPIO_NR(2, 12)
	NEW_PAD_CTRL(MX6PAD(SD4_DAT4__GPIO2_IO12), WEAK_PULLUP),	/* bt_gpio4, pin 6 */
#define GP_BT_GPIO5	IMX_GPIO_NR(2, 11)
	NEW_PAD_CTRL(MX6PAD(SD4_DAT3__GPIO2_IO11), WEAK_PULLUP),	/* bt_gpio5, pin 7 */
#define GP_BT_GPIO6	IMX_GPIO_NR(2, 10)
	NEW_PAD_CTRL(MX6PAD(SD4_DAT2__GPIO2_IO10), WEAK_PULLUP),	/* bt_gpio6, pin 8 */
#define GP_BT_GPIO7	IMX_GPIO_NR(2, 9)
	NEW_PAD_CTRL(MX6PAD(SD4_DAT1__GPIO2_IO09), WEAK_PULLUP),	/* bt_gpio7, pin 9 */
#define GP_BT_GPIO8	IMX_GPIO_NR(2, 8)
	NEW_PAD_CTRL(MX6PAD(SD4_DAT0__GPIO2_IO08), WEAK_PULLUP),	/* bt_gpio8, pin 10 */
#define GP_BT_GPIO9	IMX_GPIO_NR(7, 9)
	NEW_PAD_CTRL(MX6PAD(SD4_CMD__GPIO7_IO09), WEAK_PULLUP),		/* bt_gpio9, pin 13 */
#define GP_BT_GPIO10	IMX_GPIO_NR(7, 10)
	NEW_PAD_CTRL(MX6PAD(SD4_CLK__GPIO7_IO10), WEAK_PULLUP),		/* bt_gpio10, pin 14 */
#define GP_BT_GPIO11	IMX_GPIO_NR(2, 7)
	NEW_PAD_CTRL(MX6PAD(NANDF_D7__GPIO2_IO07), WEAK_PULLUP),	/* bt_gpio11, pin 15 */
#define GP_BT_GPIO12	IMX_GPIO_NR(2, 6)
	NEW_PAD_CTRL(MX6PAD(NANDF_D6__GPIO2_IO06), WEAK_PULLUP),	/* bt_gpio12, pin 16 */
#define GP_BT_GPIO13	IMX_GPIO_NR(2, 5)
	NEW_PAD_CTRL(MX6PAD(NANDF_D5__GPIO2_IO05), WEAK_PULLUP),	/* bt_gpio13, pin 17 */
#define GP_BT_GPIO14	IMX_GPIO_NR(2, 4)
	NEW_PAD_CTRL(MX6PAD(NANDF_D4__GPIO2_IO04), WEAK_PULLUP),	/* bt_gpio14, pin 18 */
#define GP_BT_GPIO15	IMX_GPIO_NR(2, 3)
	NEW_PAD_CTRL(MX6PAD(NANDF_D3__GPIO2_IO03), WEAK_PULLUP),	/* bt_gpio15, pin 19 */
#define GP_BT_GPIO16	IMX_GPIO_NR(2, 2)
	NEW_PAD_CTRL(MX6PAD(NANDF_D2__GPIO2_IO02), WEAK_PULLUP),	/* bt_gpio16, pin 20 */
#define GP_BT_GPIO17	IMX_GPIO_NR(2, 1)
	NEW_PAD_CTRL(MX6PAD(NANDF_D1__GPIO2_IO01), WEAK_PULLUP),	/* bt_gpio17, pin 21 */
#define GP_BT_GPIO18	IMX_GPIO_NR(2, 0)
	NEW_PAD_CTRL(MX6PAD(NANDF_D0__GPIO2_IO00), WEAK_PULLUP),	/* bt_gpio18, pin 22 */
#define GP_BT_GPIO19	IMX_GPIO_NR(6, 10)
	NEW_PAD_CTRL(MX6PAD(NANDF_RB0__GPIO6_IO10), WEAK_PULLUP),	/* bt_gpio19, pin 23 */
#define GP_BT_GPIO20	IMX_GPIO_NR(6, 9)
	NEW_PAD_CTRL(MX6PAD(NANDF_WP_B__GPIO6_IO09), WEAK_PULLUP),	/* bt_gpio20, pin 24 */
#define GP_BT_GPIO21	IMX_GPIO_NR(6, 7)
	NEW_PAD_CTRL(MX6PAD(NANDF_CLE__GPIO6_IO07), WEAK_PULLUP),	/* bt_gpio21, pin 25 */
#define GP_BT_GPIO22	IMX_GPIO_NR(6, 8)
	NEW_PAD_CTRL(MX6PAD(NANDF_ALE__GPIO6_IO08), WEAK_PULLUP),	/* bt_gpio22, pin 26 */
#define GP_BT_GPIO23	IMX_GPIO_NR(6, 16)
	NEW_PAD_CTRL(MX6PAD(NANDF_CS3__GPIO6_IO16), WEAK_PULLUP),	/* bt_gpio23, pin 27 */
#define GP_BT_GPIO24	IMX_GPIO_NR(6, 15)
	NEW_PAD_CTRL(MX6PAD(NANDF_CS2__GPIO6_IO15), WEAK_PULLUP),	/* bt_gpio24, pin 28 */
#define GP_BT_GPIO25	IMX_GPIO_NR(6, 14)
	NEW_PAD_CTRL(MX6PAD(NANDF_CS1__GPIO6_IO14), WEAK_PULLUP),	/* bt_gpio25, pin 30 */
#define GP_BT_GPIO26	IMX_GPIO_NR(6, 11)
	NEW_PAD_CTRL(MX6PAD(NANDF_CS0__GPIO6_IO11), WEAK_PULLUP),	/* bt_gpio26, pin 31 */
#define GP_BT_GPIO27	IMX_GPIO_NR(4, 16)
	NEW_PAD_CTRL(MX6PAD(DI0_DISP_CLK__GPIO4_IO16), WEAK_PULLUP),	/* bt_gpio27, pin 32 */
#define GP_BT_GPIO28	IMX_GPIO_NR(4, 18)
	NEW_PAD_CTRL(MX6PAD(DI0_PIN2__GPIO4_IO18), WEAK_PULLUP),	/* bt_gpio28, pin 33 */
#define GP_BT_GPIO29	IMX_GPIO_NR(4, 19)
	NEW_PAD_CTRL(MX6PAD(DI0_PIN3__GPIO4_IO19), WEAK_PULLUP),	/* bt_gpio29, pin 34 */
#define GP_BT_GPIO30	IMX_GPIO_NR(4, 20)
	NEW_PAD_CTRL(MX6PAD(DI0_PIN4__GPIO4_IO20), WEAK_PULLUP),	/* bt_gpio30, pin 35 */
#define GP_BT_GPIO31	IMX_GPIO_NR(4, 17)
	NEW_PAD_CTRL(MX6PAD(DI0_PIN15__GPIO4_IO17), WEAK_PULLUP),	/* bt_gpio31, pin 36 */
#define GP_BT_GPIO32	IMX_GPIO_NR(4, 25)
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT4__GPIO4_IO25), WEAK_PULLUP),	/* bt_gpio32, pin 37 */
#define GP_BT_GPIO33	IMX_GPIO_NR(5, 10)
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT16__GPIO5_IO10), WEAK_PULLUP),	/* bt_gpio33, pin 38 */
#define GP_BT_GPIO34	IMX_GPIO_NR(5, 11)
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT17__GPIO5_IO11), WEAK_PULLUP),	/* bt_gpio34, pin 39 */
#define GP_BT_GPIO35	IMX_GPIO_NR(5, 2)
	NEW_PAD_CTRL(MX6PAD(EIM_A25__GPIO5_IO02), WEAK_PULLUP),		/* bt_gpio35, pin 40 */
#define GP_BT_GPIO36	IMX_GPIO_NR(3, 29)
	NEW_PAD_CTRL(MX6PAD(EIM_D29__GPIO3_IO29), WEAK_PULLUP),		/* bt_gpio36, pin 41 */
#define GP_BT_GPIO37	IMX_GPIO_NR(2, 30)
	NEW_PAD_CTRL(MX6PAD(EIM_EB2__GPIO2_IO30), WEAK_PULLUP),		/* bt_gpio37, pin 42 */
#define GP_BT_GPIO38	IMX_GPIO_NR(2, 31)
	NEW_PAD_CTRL(MX6PAD(EIM_EB3__GPIO2_IO31), WEAK_PULLUP),		/* bt_gpio38, pin 43 */
#define GP_BT_GPIO39	IMX_GPIO_NR(5, 26)
	NEW_PAD_CTRL(MX6PAD(CSI0_DAT8__GPIO5_IO26), WEAK_PULLUP),	/* bt_gpio39, pin 44 */
#define GP_BT_GPIO40	IMX_GPIO_NR(5, 27)
	NEW_PAD_CTRL(MX6PAD(CSI0_DAT9__GPIO5_IO27), WEAK_PULLUP),	/* bt_gpio40, pin 45 */

	/* Power control, active high */
#define GP_PWR_J1	IMX_GPIO_NR(5, 15)
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT21__GPIO5_IO15), OUTPUT_40OHM),	/* J1 Power enable */
#define GP_PWR_J2	IMX_GPIO_NR(5, 14)
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT20__GPIO5_IO14), OUTPUT_40OHM),	/* J2 */
#define GP_PWR_J3	IMX_GPIO_NR(2, 23)
	NEW_PAD_CTRL(MX6PAD(EIM_CS0__GPIO2_IO23), OUTPUT_40OHM),	/* J3 */
#define GP_PWR_J4	IMX_GPIO_NR(2, 25)
	NEW_PAD_CTRL(MX6PAD(EIM_OE__GPIO2_IO25), OUTPUT_40OHM),		/* J4 */
#define GP_PWR_J6	IMX_GPIO_NR(2, 26)
	NEW_PAD_CTRL(MX6PAD(EIM_RW__GPIO2_IO26), OUTPUT_40OHM),		/* J6 */
#define GP_PWR_J7	IMX_GPIO_NR(2, 27)
	NEW_PAD_CTRL(MX6PAD(EIM_LBA__GPIO2_IO27), OUTPUT_40OHM),	/* J7 */

	/* Dry Contact */
#define GP_DRY_CONTACT	IMX_GPIO_NR(3, 31)
	NEW_PAD_CTRL(MX6PAD(EIM_D31__GPIO3_IO31), WEAK_PULLDN),

	/*
	 * PCIe - tw6869 dedicated,
	 * VIN1-4 used,
	 * AIN1-2 amplified, AIN3-4 not amplified
	 */
#define GP_PCIE_RESET		IMX_GPIO_NR(4, 9)	/* tw6869 reset */
	NEW_PAD_CTRL(MX6PAD(KEY_ROW1__GPIO4_IO09), OUTPUT_40OHM),

	/* rtc - i2c1 */
#define GP_RTC_RV4162_IRQ	IMX_GPIO_NR(4, 11)
	NEW_PAD_CTRL(MX6PAD(KEY_ROW2__GPIO4_IO11), WEAK_PULLUP),

	/* SDI - gs2971 on CSI1 */
#ifdef FOR_DL_SOLO
	/* Dualite/Solo doesn't have IPU2 */
	NEW_PAD_CTRL(MX6PAD(EIM_A24__IPU1_CSI1_DATA19), CSI_PAD_CTRL),	/* GPIO2[30] */
	NEW_PAD_CTRL(MX6PAD(EIM_A23__IPU1_CSI1_DATA18), CSI_PAD_CTRL),	/* GPIO6[6] */
	NEW_PAD_CTRL(MX6PAD(EIM_A22__IPU1_CSI1_DATA17), CSI_PAD_CTRL),	/* GPIO2[16] */
	NEW_PAD_CTRL(MX6PAD(EIM_A21__IPU1_CSI1_DATA16), CSI_PAD_CTRL),	/* GPIO2[17] */
	NEW_PAD_CTRL(MX6PAD(EIM_A20__IPU1_CSI1_DATA15), CSI_PAD_CTRL),	/* GPIO2[18] */
	NEW_PAD_CTRL(MX6PAD(EIM_A19__IPU1_CSI1_DATA14), CSI_PAD_CTRL),	/* GPIO2[19] */
	NEW_PAD_CTRL(MX6PAD(EIM_A18__IPU1_CSI1_DATA13), CSI_PAD_CTRL),	/* GPIO2[20] */
	NEW_PAD_CTRL(MX6PAD(EIM_A17__IPU1_CSI1_DATA12), CSI_PAD_CTRL),	/* GPIO2[21] */
	NEW_PAD_CTRL(MX6PAD(EIM_EB0__IPU1_CSI1_DATA11), CSI_PAD_CTRL),	/* GPIO2[28] */
	NEW_PAD_CTRL(MX6PAD(EIM_EB1__IPU1_CSI1_DATA10), CSI_PAD_CTRL),	/* GPIO2[29] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA0__IPU1_CSI1_DATA09), CSI_PAD_CTRL),	/* GPIO3[0] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA1__IPU1_CSI1_DATA08), CSI_PAD_CTRL),	/* GPIO3[1] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA2__IPU1_CSI1_DATA07), CSI_PAD_CTRL),	/* GPIO3[2] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA3__IPU1_CSI1_DATA06), CSI_PAD_CTRL),	/* GPIO3[3] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA4__IPU1_CSI1_DATA05), CSI_PAD_CTRL),	/* GPIO3[4] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA5__IPU1_CSI1_DATA04), CSI_PAD_CTRL),	/* GPIO3[5] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA6__IPU1_CSI1_DATA03), CSI_PAD_CTRL),	/* GPIO3[6] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA7__IPU1_CSI1_DATA02), CSI_PAD_CTRL),	/* GPIO3[7] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA8__IPU1_CSI1_DATA01), CSI_PAD_CTRL),	/* GPIO3[8] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA9__IPU1_CSI1_DATA00), CSI_PAD_CTRL),	/* GPIO3[9] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA10__IPU1_CSI1_DATA_EN), CSI_PAD_CTRL),	/* GPIO3[10] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA11__IPU1_CSI1_HSYNC), CSI_PAD_CTRL),	/* GPIO3[11] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA12__IPU1_CSI1_VSYNC), CSI_PAD_CTRL),	/* GPIO3[12] */
	NEW_PAD_CTRL(MX6PAD(EIM_A16__IPU1_CSI1_PIXCLK), CSI_PAD_CTRL),	/* GPIO2[22] */
#else
	NEW_PAD_CTRL(MX6PAD(EIM_A24__IPU2_CSI1_DATA19), CSI_PAD_CTRL),	/* GPIO2[30] */
	NEW_PAD_CTRL(MX6PAD(EIM_A23__IPU2_CSI1_DATA18), CSI_PAD_CTRL),	/* GPIO6[6] */
	NEW_PAD_CTRL(MX6PAD(EIM_A22__IPU2_CSI1_DATA17), CSI_PAD_CTRL),	/* GPIO2[16] */
	NEW_PAD_CTRL(MX6PAD(EIM_A21__IPU2_CSI1_DATA16), CSI_PAD_CTRL),	/* GPIO2[17] */
	NEW_PAD_CTRL(MX6PAD(EIM_A20__IPU2_CSI1_DATA15), CSI_PAD_CTRL),	/* GPIO2[18] */
	NEW_PAD_CTRL(MX6PAD(EIM_A19__IPU2_CSI1_DATA14), CSI_PAD_CTRL),	/* GPIO2[19] */
	NEW_PAD_CTRL(MX6PAD(EIM_A18__IPU2_CSI1_DATA13), CSI_PAD_CTRL),	/* GPIO2[20] */
	NEW_PAD_CTRL(MX6PAD(EIM_A17__IPU2_CSI1_DATA12), CSI_PAD_CTRL),	/* GPIO2[21] */
	NEW_PAD_CTRL(MX6PAD(EIM_EB0__IPU2_CSI1_DATA11), CSI_PAD_CTRL),	/* GPIO2[28] */
	NEW_PAD_CTRL(MX6PAD(EIM_EB1__IPU2_CSI1_DATA10), CSI_PAD_CTRL),	/* GPIO2[29] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA0__IPU2_CSI1_DATA09), CSI_PAD_CTRL),	/* GPIO3[0] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA1__IPU2_CSI1_DATA08), CSI_PAD_CTRL),	/* GPIO3[1] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA2__IPU2_CSI1_DATA07), CSI_PAD_CTRL),	/* GPIO3[2] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA3__IPU2_CSI1_DATA06), CSI_PAD_CTRL),	/* GPIO3[3] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA4__IPU2_CSI1_DATA05), CSI_PAD_CTRL),	/* GPIO3[4] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA5__IPU2_CSI1_DATA04), CSI_PAD_CTRL),	/* GPIO3[5] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA6__IPU2_CSI1_DATA03), CSI_PAD_CTRL),	/* GPIO3[6] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA7__IPU2_CSI1_DATA02), CSI_PAD_CTRL),	/* GPIO3[7] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA8__IPU2_CSI1_DATA01), CSI_PAD_CTRL),	/* GPIO3[8] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA9__IPU2_CSI1_DATA00), CSI_PAD_CTRL),	/* GPIO3[9] */
	NEW_PAD_CTRL(MX6PAD(EIM_DA10__IPU2_CSI1_DATA_EN), CSI_PAD_CTRL),	/* GPIO3[10] - pin B5 stat2 */
	NEW_PAD_CTRL(MX6PAD(EIM_DA11__IPU2_CSI1_HSYNC), CSI_PAD_CTRL),	/* GPIO3[11] - pin A5 stat0 */
	NEW_PAD_CTRL(MX6PAD(EIM_DA12__IPU2_CSI1_VSYNC), CSI_PAD_CTRL),	/* GPIO3[12] - pin A6 stat1 */
	NEW_PAD_CTRL(MX6PAD(EIM_A16__IPU2_CSI1_PIXCLK), CSI_PAD_CTRL),	/* GPIO2[22] - pin A8 */
#endif
#define GP_GS2971_SMPTE_BYPASS	IMX_GPIO_NR(2, 24)
	NEW_PAD_CTRL(MX6PAD(EIM_CS1__GPIO2_IO24), WEAK_PULLUP),		/* pin G7 - i/o SMPTE bypass */
#define GP_GS2971_RESET		IMX_GPIO_NR(3, 13)
	NEW_PAD_CTRL(MX6PAD(EIM_DA13__GPIO3_IO13), OUTPUT_40OHM),	/* 0 - pin C7 - reset */
#define GP_GS2971_DVI_LOCK	IMX_GPIO_NR(3, 14)
	NEW_PAD_CTRL(MX6PAD(EIM_DA14__GPIO3_IO14), WEAK_PULLUP),	/* pin B6 - stat3 - DVI_LOCK */
#define GP_GS2971_DATA_ERR	IMX_GPIO_NR(3, 15)
	NEW_PAD_CTRL(MX6PAD(EIM_DA15__GPIO3_IO15), WEAK_PULLUP),	/* pin C6 - stat5 - DATA error */
#define GP_GS2971_LB_CONT	IMX_GPIO_NR(3, 20)
	NEW_PAD_CTRL(MX6PAD(EIM_D20__GPIO3_IO20), WEAK_PULLUP),		/* pin A3 - LB control - float, analog input */
#define GP_GS2971_Y_1ANC	IMX_GPIO_NR(4, 26)
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT5__GPIO4_IO26), WEAK_PULLUP),	/* pin C5 - stat4 - 1ANC - Y signal detect */
#define GP_GS2971_RC_BYPASS	IMX_GPIO_NR(4, 27)
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT6__GPIO4_IO27), OUTPUT_40OHM),	/* 0 - pin G3 - RC bypass - output is buffered(low) */
#define GP_GS2971_IOPROC_EN	IMX_GPIO_NR(4, 28)
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT7__GPIO4_IO28), OUTPUT_40OHM),	/* 0 - pin H8 - io(A/V) processor enable */
#define GP_GS2971_AUDIO_EN	IMX_GPIO_NR(4, 29)
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT8__GPIO4_IO29), OUTPUT_40OHM),	/* 0 - pin H3 - Audio Enable */
#define GP_GS2971_TIM_861	IMX_GPIO_NR(4, 30)
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT9__GPIO4_IO30), OUTPUT_40OHM),	/* 0 - pin H5 - TIM861 timing format, 0-use HSYNC/VSYNC */
#define GP_GS2971_SW_EN		IMX_GPIO_NR(4, 31)
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT10__GPIO4_IO31), OUTPUT_40OHM),	/* 0 - pin D7 - SW_EN - line lock enable */
#define GP_GS2971_STANDBY	IMX_GPIO_NR(5, 0)
	NEW_PAD_CTRL(MX6PAD(EIM_WAIT__GPIO5_IO00), OUTPUT_40OHM),	/* 1 - pin K2 - Standby */
#define GP_GS2971_DVB_ASI	IMX_GPIO_NR(5, 5)
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT11__GPIO5_IO05), WEAK_PULLUP),	/* pin G8 i/o DVB_ASI */

	NEW_PAD_CTRL(MX6PAD(DISP0_DAT23__AUD4_RXD), AUD_PAD_CTRL),	/* pin J3 - AOUT1/2 */
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT19__AUD4_RXC), AUD_PAD_CTRL),	/* pin J4 - ACLK*/
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT18__AUD4_RXFS), AUD_PAD_CTRL),	/* pin H4 - WCLK*/
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT2__ECSPI3_MISO), AUD_PAD_CTRL),	/* pin E7 - SDOUT */
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT1__ECSPI3_MOSI), AUD_PAD_CTRL),	/* pin E8 - SDIN */
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT0__ECSPI3_SCLK), AUD_PAD_CTRL),	/* pin F8 - SCLK */
#define GP_ECSPI3_CS0		IMX_GPIO_NR(4, 24)
	NEW_PAD_CTRL(MX6PAD(DISP0_DAT3__GPIO4_IO24), OUTPUT_40OHM),	/* 0 - pin F7 - CS0 */

	/* UART1  */
	NEW_PAD_CTRL(MX6PAD(CSI0_DAT10__UART1_TX_DATA), UART_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(CSI0_DAT11__UART1_RX_DATA), UART_PAD_CTRL),

	/* UART2 for debug */
	NEW_PAD_CTRL(MX6PAD(EIM_D26__UART2_TX_DATA), UART_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(EIM_D27__UART2_RX_DATA), UART_PAD_CTRL),

	/* UART3 */
	NEW_PAD_CTRL(MX6PAD(EIM_D24__UART3_TX_DATA), UART_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(EIM_D25__UART3_RX_DATA), UART_PAD_CTRL),

	/* UART4 */
	NEW_PAD_CTRL(MX6PAD(KEY_COL0__UART4_TX_DATA), UART_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(KEY_ROW0__UART4_RX_DATA), UART_PAD_CTRL),

	/* UART5 */
	NEW_PAD_CTRL(MX6PAD(CSI0_DAT14__UART5_TX_DATA), UART_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(CSI0_DAT15__UART5_RX_DATA), UART_PAD_CTRL),

	/* UART6/7 on sc16is752 on i2c2 */
#define GP_SC16IS752_IRQ		IMX_GPIO_NR(4, 10)
	NEW_PAD_CTRL(MX6PAD(KEY_COL2__GPIO4_IO10), WEAK_PULLUP),		/* irq */

	/* USBH1 */
	NEW_PAD_CTRL(MX6PAD(EIM_D30__USB_H1_OC), WEAK_PULLUP),
#define GP_USB_HUB_RESET	IMX_GPIO_NR(7, 12)
	NEW_PAD_CTRL(MX6PAD(GPIO_17__GPIO7_IO12), OUTPUT_40OHM),	/* USB Hub Reset for USB2512 4 port hub */
	/*
	 * port1 - 10/100 ethernet using AX88772A on J90
	 * port2 - usb connector on J26
	 * port3 - usb connector on J25
	 * port4 - usb connector on J96
	 */
#define GP_AX88772A_RESET	IMX_GPIO_NR(5, 20)
	NEW_PAD_CTRL(MX6PAD(CSI0_DATA_EN__GPIO5_IO20), OUTPUT_40OHM),

	/* USBOTG - J80 */
	NEW_PAD_CTRL(MX6PAD(GPIO_1__USB_OTG_ID)	, WEAK_PULLUP),
	NEW_PAD_CTRL(MX6PAD(KEY_COL4__USB_OTG_OC), WEAK_PULLUP),
#define GP_USB_OTG_PWR		IMX_GPIO_NR(3, 22)
	NEW_PAD_CTRL(MX6PAD(EIM_D22__GPIO3_IO22), OUTPUT_40OHM),

	/* USDHC1: Full size SD card holder - J88 */
	NEW_PAD_CTRL(MX6PAD(SD1_CLK__SD1_CLK), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD1_CMD__SD1_CMD), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD1_DAT0__SD1_DATA0), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD1_DAT1__SD1_DATA1), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD1_DAT2__SD1_DATA2), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD1_DAT3__SD1_DATA3), USDHC_PAD_CTRL),
#define GP_SD1_CD		IMX_GPIO_NR(1, 4)
	NEW_PAD_CTRL(MX6PAD(GPIO_4__GPIO1_IO04), WEAK_PULLUP),
#define GP_SD1_WP		IMX_GPIO_NR(1, 2)
	NEW_PAD_CTRL(MX6PAD(GPIO_2__GPIO1_IO02), WEAK_PULLUP),
	/* Needs to invert and use key_col1 */
#define GP_SD1_POWER_SEL	IMX_GPIO_NR(7, 13)		/* low 1.8V, high 3.3V */
	NEW_PAD_CTRL(MX6PAD(GPIO_18__GPIO7_IO13), OUTPUT_40OHM),

	/* USDHC2:  micro sd - J87 */
	NEW_PAD_CTRL(MX6PAD(SD2_CLK__SD2_CLK), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD2_CMD__SD2_CMD), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD2_DAT0__SD2_DATA0), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD2_DAT1__SD2_DATA1), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD2_DAT2__SD2_DATA2), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD2_DAT3__SD2_DATA3), USDHC_PAD_CTRL),
#define GP_SD2_CD		IMX_GPIO_NR(3, 23)
	NEW_PAD_CTRL(MX6PAD(EIM_D23__GPIO3_IO23), WEAK_PULLUP),

	/* USDHC3 - eMMC */
	NEW_PAD_CTRL(MX6PAD(SD3_CLK__SD3_CLK), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD3_CMD__SD3_CMD), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD3_DAT0__SD3_DATA0), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD3_DAT1__SD3_DATA1), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD3_DAT2__SD3_DATA2), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD3_DAT3__SD3_DATA3), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD3_DAT4__SD3_DATA4), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD3_DAT5__SD3_DATA5), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD3_DAT6__SD3_DATA6), USDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(SD3_DAT7__SD3_DATA7), USDHC_PAD_CTRL),
#define GP_EMMC_RESET	IMX_GPIO_NR(7, 8)
	NEW_PAD_CTRL(MX6PAD(SD3_RST__GPIO7_IO08), OUTPUT_40OHM),		/* eMMC reset */
};

static const iomux_v3_cfg_t MX6NAME(enet_pads1)[] = {
	/* pin 35 - 1 (PHY_AD2) on reset */
	NEW_PAD_CTRL(MX6PAD(RGMII_RXC__GPIO6_IO30), OUTPUT_40OHM),
	/* pin 32 - 1 - (MODE0) all */
	NEW_PAD_CTRL(MX6PAD(RGMII_RD0__GPIO6_IO25), OUTPUT_40OHM),
	/* pin 31 - 1 - (MODE1) all */
	NEW_PAD_CTRL(MX6PAD(RGMII_RD1__GPIO6_IO27), OUTPUT_40OHM),
	/* pin 28 - 1 - (MODE2) all */
	NEW_PAD_CTRL(MX6PAD(RGMII_RD2__GPIO6_IO28), OUTPUT_40OHM),
	/* pin 27 - 1 - (MODE3) all */
	NEW_PAD_CTRL(MX6PAD(RGMII_RD3__GPIO6_IO29), OUTPUT_40OHM),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	NEW_PAD_CTRL(MX6PAD(RGMII_RX_CTL__GPIO6_IO24), OUTPUT_40OHM),
};

static const iomux_v3_cfg_t MX6NAME(enet_pads2)[] = {
	NEW_PAD_CTRL(MX6PAD(RGMII_RXC__RGMII_RXC), ENET_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(RGMII_RD0__RGMII_RD0), ENET_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(RGMII_RD1__RGMII_RD1), ENET_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(RGMII_RD2__RGMII_RD2), ENET_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(RGMII_RD3__RGMII_RD3), ENET_PAD_CTRL),
	NEW_PAD_CTRL(MX6PAD(RGMII_RX_CTL__RGMII_RX_CTL), ENET_PAD_CTRL),
};

/*
 *
 */
#define PC I2C_PAD_CTRL

struct i2c_pads_info MX6NAME(i2c_pad_info)[] = {
{
	/* I2C1, SGTL5000 */
	.scl = {
		.i2c_mode = NEW_PAD_CTRL(MX6PAD(EIM_D21__I2C1_SCL), PC),
		.gpio_mode = NEW_PAD_CTRL(MX6PAD(EIM_D21__GPIO3_IO21), PC),
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = NEW_PAD_CTRL(MX6PAD(EIM_D28__I2C1_SDA), PC),
		.gpio_mode = NEW_PAD_CTRL(MX6PAD(EIM_D28__GPIO3_IO28), PC),
		.gp = IMX_GPIO_NR(3, 28)
	}
}, {
	/* I2C2 Camera, MIPI */
	.scl = {
		.i2c_mode = NEW_PAD_CTRL(MX6PAD(KEY_COL3__I2C2_SCL), PC),
		.gpio_mode = NEW_PAD_CTRL(MX6PAD(KEY_COL3__GPIO4_IO12), PC),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = NEW_PAD_CTRL(MX6PAD(KEY_ROW3__I2C2_SDA), PC),
		.gpio_mode = NEW_PAD_CTRL(MX6PAD(KEY_ROW3__GPIO4_IO13), PC),
		.gp = IMX_GPIO_NR(4, 13)
	}
}, {
	/* I2C3, J15 - RGB connector */
	.scl = {
		.i2c_mode = NEW_PAD_CTRL(MX6PAD(GPIO_5__I2C3_SCL), PC),
		.gpio_mode = NEW_PAD_CTRL(MX6PAD(GPIO_5__GPIO1_IO05), PC),
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = NEW_PAD_CTRL(MX6PAD(GPIO_16__I2C3_SDA), PC),
		.gpio_mode = NEW_PAD_CTRL(MX6PAD(GPIO_16__GPIO7_IO11), PC),
		.gp = IMX_GPIO_NR(7, 11)
	}
}
};