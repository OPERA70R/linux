// SPDX-License-Identifier: GPL-2.0-only
/*
 * A V4L2 driver for Sony IMX766 cameras.
 * Copyright (c) 2024, Danila Tikhonov <danila@jiaxyga.com>
 *
 * Based on Sony imx412 camera driver
 * Copyright (C) 2021 Intel Corporation
 */
#include <linux/unaligned.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Streaming Mode */
#define IMX766_REG_MODE_SELECT	0x0100
#define IMX766_MODE_STANDBY	0x00
#define IMX766_MODE_STREAMING	0x01

/* Lines per frame */
#define IMX766_REG_LPFR		0x0340

/* Chip ID */
#define IMX766_REG_ID		0x0016
#define IMX766_CHIP_ID		0x766

/* Exposure control */
#define IMX766_REG_EXPOSURE_CIT	0x0202
#define IMX766_EXPOSURE_MIN	8
#define IMX766_EXPOSURE_OFFSET	22
#define IMX766_EXPOSURE_STEP	1
#define IMX766_EXPOSURE_DEFAULT	0x0648

/* Analog gain control */
#define IMX766_REG_AGAIN	0x0204
#define IMX766_AGAIN_MIN	0
#define IMX766_AGAIN_MAX	978
#define IMX766_AGAIN_STEP	1
#define IMX766_AGAIN_DEFAULT	0

/* Group hold register */
#define IMX766_REG_HOLD		0x0104

/* Input clock rate */
#define IMX766_INCLK_RATE	19200000

/* CSI2 HW configuration */
#define IMX766_LINK_FREQ_999MHZ	999000000
#define IMX766_LINK_FREQ_436MHZ	436000000 /* 872/2 */
#define IMX766_LINK_FREQ_560MHZ	300000000 /* 1120/2 */
#define IMX766_LINK_FREQ	600000000
#define IMX766_NUM_DATA_LANES	4 // 3?

#define IMX766_REG_MIN		0x00
#define IMX766_REG_MAX		0xffff

/**
 * struct imx766_reg - imx766 sensor register
 * @address: Register address
 * @val: Register value
 */
struct imx766_reg {
	u16 address;
	u8 val;
};

/**
 * struct imx766_reg_list - imx766 sensor register list
 * @num_of_regs: Number of registers in the list
 * @regs: Pointer to register list
 */
struct imx766_reg_list {
	u32 num_of_regs;
	const struct imx766_reg *regs;
};

/**
 * struct imx766_mode - imx766 sensor mode structure
 * @width: Frame width
 * @height: Frame height
 * @code: Format code
 * @hblank: Horizontal blanking in lines
 * @vblank: Vertical blanking in lines
 * @vblank_min: Minimum vertical blanking in lines
 * @vblank_max: Maximum vertical blanking in lines
 * @pclk: Sensor pixel clock
 * @link_freq_idx: Link frequency index
 * @reg_list: Register list for sensor mode
 */
struct imx766_mode {
	u32 width;
	u32 height;
	u32 code;
	u32 hblank;
	u32 vblank;
	u32 vblank_min;
	u32 vblank_max;
	u64 pclk;
	u32 link_freq_idx;
	struct imx766_reg_list reg_list;
};

static const char * const imx766_supply_names[] = {
	"vana",		/* 2.8V Analog Power */
	"vif",		/* 1.2V or 1.8V Interface Power */
	"vdig",		/* 1.1V Digital Power */
};

/**
 * struct imx766 - imx766 sensor device structure
 * @dev: Pointer to generic device
 * @client: Pointer to i2c client
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @reset_gpio: Sensor reset gpio
 * @pwdn_gpio: Sensor pwdn gpio
 * @power_gpio: Sensor power gpio
 * @inclk: Sensor input clock
 * @supplies: Regulator supplies
 * @ctrl_handler: V4L2 control handler
 * @link_freq_ctrl: Pointer to link frequency control
 * @pclk_ctrl: Pointer to pixel clock control
 * @hblank_ctrl: Pointer to horizontal blanking control
 * @vblank_ctrl: Pointer to vertical blanking control
 * @exp_ctrl: Pointer to exposure control
 * @again_ctrl: Pointer to analog gain control
 * @vblank: Vertical blanking in lines
 * @cur_mode: Pointer to current selected sensor mode
 * @mutex: Mutex for serializing sensor controls
 */
struct imx766 {
	struct device *dev;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	struct gpio_desc *power_gpio;
	struct clk *inclk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx766_supply_names)];
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq_ctrl;
	struct v4l2_ctrl *pclk_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct {
		struct v4l2_ctrl *exp_ctrl;
		struct v4l2_ctrl *again_ctrl;
	};
	u32 vblank;
	const struct imx766_mode *cur_mode;
	struct mutex mutex;
};

static const s64 link_freq[] = {
	//IMX766_LINK_FREQ_999MHZ,
	//IMX766_LINK_FREQ_436MHZ,
	//IMX766_LINK_FREQ_560MHZ,
	IMX766_LINK_FREQ,
};

/* Sensor mode registers */
static const struct imx766_reg mode_4096x3072_regs[] = {
	//Power ON
	//Input EXTCLK
	//XCLR OFF
	//External Clock Setting
	{ 0x0136, 0x18 },
	{ 0x0137, 0x00 },
	//Register version
	{ 0x33F0, 0x03 },
	{ 0x33F1, 0x08 },
	//Signaling mode setting
	{ 0x0111, 0x03 },
	//Global Setting
	{ 0x33D3, 0x01 },
	{ 0x3892, 0x01 },
	{ 0x4C14, 0x00 },
	{ 0x4C15, 0x07 },
	{ 0x4C16, 0x00 },
	{ 0x4C17, 0x1B },
	{ 0x4C1A, 0x00 },
	{ 0x4C1B, 0x03 },
	{ 0x4C1C, 0x00 },
	{ 0x4C1D, 0x00 },
	{ 0x4C1E, 0x00 },
	{ 0x4C1F, 0x02 },
	{ 0x4C20, 0x00 },
	{ 0x4C21, 0x5F },
	{ 0x4C26, 0x00 },
	{ 0x4C27, 0x43 },
	{ 0x4C28, 0x00 },
	{ 0x4C29, 0x09 },
	{ 0x4C2A, 0x00 },
	{ 0x4C2B, 0x4A },
	{ 0x4C2C, 0x00 },
	{ 0x4C2D, 0x00 },
	{ 0x4C2E, 0x00 },
	{ 0x4C2F, 0x02 },
	{ 0x4C30, 0x00 },
	{ 0x4C31, 0xC6 },
	{ 0x4C3E, 0x00 },
	{ 0x4C3F, 0x55 },
	{ 0x4C52, 0x00 },
	{ 0x4C53, 0x97 },
	{ 0x4CB4, 0x00 },
	{ 0x4CB5, 0x55 },
	{ 0x4CC8, 0x00 },
	{ 0x4CC9, 0x97 },
	{ 0x4D04, 0x00 },
	{ 0x4D05, 0x4F },
	{ 0x4D74, 0x00 },
	{ 0x4D75, 0x55 },
	{ 0x4F06, 0x00 },
	{ 0x4F07, 0x5F },
	{ 0x4F48, 0x00 },
	{ 0x4F49, 0xC6 },
	{ 0x544A, 0xFF },
	{ 0x544B, 0xFF },
	{ 0x544E, 0x01 },
	{ 0x544F, 0xBD },
	{ 0x5452, 0xFF },
	{ 0x5453, 0xFF },
	{ 0x5456, 0x00 },
	{ 0x5457, 0xA5 },
	{ 0x545A, 0xFF },
	{ 0x545B, 0xFF },
	{ 0x545E, 0x00 },
	{ 0x545F, 0xA5 },
	{ 0x5496, 0x00 },
	{ 0x5497, 0xA2 },
	{ 0x54F6, 0x01 },
	{ 0x54F7, 0x55 },
	{ 0x54F8, 0x01 },
	{ 0x54F9, 0x61 },
	{ 0x5670, 0x00 },
	{ 0x5671, 0x85 },
	{ 0x5672, 0x01 },
	{ 0x5673, 0x77 },
	{ 0x5674, 0x01 },
	{ 0x5675, 0x2F },
	{ 0x5676, 0x02 },
	{ 0x5677, 0x55 },
	{ 0x5678, 0x00 },
	{ 0x5679, 0x85 },
	{ 0x567A, 0x01 },
	{ 0x567B, 0x77 },
	{ 0x567C, 0x01 },
	{ 0x567D, 0x2F },
	{ 0x567E, 0x02 },
	{ 0x567F, 0x55 },
	{ 0x5680, 0x00 },
	{ 0x5681, 0x85 },
	{ 0x5682, 0x01 },
	{ 0x5683, 0x77 },
	{ 0x5684, 0x01 },
	{ 0x5685, 0x2F },
	{ 0x5686, 0x02 },
	{ 0x5687, 0x55 },
	{ 0x5688, 0x00 },
	{ 0x5689, 0x85 },
	{ 0x568A, 0x01 },
	{ 0x568B, 0x77 },
	{ 0x568C, 0x01 },
	{ 0x568D, 0x2F },
	{ 0x568E, 0x02 },
	{ 0x568F, 0x55 },
	{ 0x5690, 0x01 },
	{ 0x5691, 0x7A },
	{ 0x5692, 0x02 },
	{ 0x5693, 0x6C },
	{ 0x5694, 0x01 },
	{ 0x5695, 0x35 },
	{ 0x5696, 0x02 },
	{ 0x5697, 0x5B },
	{ 0x5698, 0x01 },
	{ 0x5699, 0x7A },
	{ 0x569A, 0x02 },
	{ 0x569B, 0x6C },
	{ 0x569C, 0x01 },
	{ 0x569D, 0x35 },
	{ 0x569E, 0x02 },
	{ 0x569F, 0x5B },
	{ 0x56A0, 0x01 },
	{ 0x56A1, 0x7A },
	{ 0x56A2, 0x02 },
	{ 0x56A3, 0x6C },
	{ 0x56A4, 0x01 },
	{ 0x56A5, 0x35 },
	{ 0x56A6, 0x02 },
	{ 0x56A7, 0x5B },
	{ 0x56A8, 0x01 },
	{ 0x56A9, 0x80 },
	{ 0x56AA, 0x02 },
	{ 0x56AB, 0x72 },
	{ 0x56AC, 0x01 },
	{ 0x56AD, 0x2F },
	{ 0x56AE, 0x02 },
	{ 0x56AF, 0x55 },
	{ 0x5902, 0x0E },
	{ 0x5A50, 0x04 },
	{ 0x5A51, 0x04 },
	{ 0x5A69, 0x01 },
	{ 0x5C49, 0x0D },
	{ 0x5D60, 0x08 },
	{ 0x5D61, 0x08 },
	{ 0x5D62, 0x08 },
	{ 0x5D63, 0x08 },
	{ 0x5D64, 0x08 },
	{ 0x5D67, 0x08 },
	{ 0x5D6C, 0x08 },
	{ 0x5D6E, 0x08 },
	{ 0x5D71, 0x08 },
	{ 0x5D8E, 0x14 },
	{ 0x5D90, 0x03 },
	{ 0x5D91, 0x0A },
	{ 0x5D92, 0x1F },
	{ 0x5D93, 0x05 },
	{ 0x5D97, 0x1F },
	{ 0x5D9A, 0x06 },
	{ 0x5D9C, 0x1F },
	{ 0x5DA1, 0x1F },
	{ 0x5DA6, 0x1F },
	{ 0x5DA8, 0x1F },
	{ 0x5DAB, 0x1F },
	{ 0x5DC0, 0x06 },
	{ 0x5DC1, 0x06 },
	{ 0x5DC2, 0x07 },
	{ 0x5DC3, 0x06 },
	{ 0x5DC4, 0x07 },
	{ 0x5DC7, 0x07 },
	{ 0x5DCC, 0x07 },
	{ 0x5DCE, 0x07 },
	{ 0x5DD1, 0x07 },
	{ 0x5E3E, 0x00 },
	{ 0x5E3F, 0x00 },
	{ 0x5E41, 0x00 },
	{ 0x5E48, 0x00 },
	{ 0x5E49, 0x00 },
	{ 0x5E4A, 0x00 },
	{ 0x5E4C, 0x00 },
	{ 0x5E4D, 0x00 },
	{ 0x5E4E, 0x00 },
	{ 0x6026, 0x03 },
	{ 0x6028, 0x03 },
	{ 0x602A, 0x03 },
	{ 0x602C, 0x03 },
	{ 0x602F, 0x03 },
	{ 0x6036, 0x03 },
	{ 0x6038, 0x03 },
	{ 0x603A, 0x03 },
	{ 0x603C, 0x03 },
	{ 0x603F, 0x03 },
	{ 0x6074, 0x19 },
	{ 0x6076, 0x19 },
	{ 0x6078, 0x19 },
	{ 0x607A, 0x19 },
	{ 0x607D, 0x19 },
	{ 0x6084, 0x32 },
	{ 0x6086, 0x32 },
	{ 0x6088, 0x32 },
	{ 0x608A, 0x32 },
	{ 0x608D, 0x32 },
	{ 0x60C2, 0x4A },
	{ 0x60C4, 0x4A },
	{ 0x60CB, 0x4A },
	{ 0x60D2, 0x4A },
	{ 0x60D4, 0x4A },
	{ 0x60DB, 0x4A },
	{ 0x62F9, 0x14 },
	{ 0x6305, 0x13 },
	{ 0x6307, 0x13 },
	{ 0x630A, 0x13 },
	{ 0x630D, 0x0D },
	{ 0x6317, 0x0D },
	{ 0x632F, 0x2E },
	{ 0x6333, 0x2E },
	{ 0x6339, 0x2E },
	{ 0x6343, 0x2E },
	{ 0x6347, 0x2E },
	{ 0x634D, 0x2E },
	{ 0x6352, 0x00 },
	{ 0x6353, 0x5F },
	{ 0x6366, 0x00 },
	{ 0x6367, 0x5F },
	{ 0x638F, 0x95 },
	{ 0x6393, 0x95 },
	{ 0x6399, 0x95 },
	{ 0x63A3, 0x95 },
	{ 0x63A7, 0x95 },
	{ 0x63AD, 0x95 },
	{ 0x63B2, 0x00 },
	{ 0x63B3, 0xC6 },
	{ 0x63C6, 0x00 },
	{ 0x63C7, 0xC6 },
	{ 0x8BDB, 0x02 },
	{ 0x8BDE, 0x02 },
	{ 0x8BE1, 0x2D },
	{ 0x8BE4, 0x00 },
	{ 0x8BE5, 0x00 },
	{ 0x8BE6, 0x01 },
	{ 0x9002, 0x14 },
	{ 0x9200, 0xB5 },
	{ 0x9201, 0x9E },
	{ 0x9202, 0xB5 },
	{ 0x9203, 0x42 },
	{ 0x9204, 0xB5 },
	{ 0x9205, 0x43 },
	{ 0x9206, 0xBD },
	{ 0x9207, 0x20 },
	{ 0x9208, 0xBD },
	{ 0x9209, 0x22 },
	{ 0x920A, 0xBD },
	{ 0x920B, 0x23 },
	{ 0xB5D7, 0x10 },
	{ 0xBD24, 0x00 },
	{ 0xBD25, 0x00 },
	{ 0xBD26, 0x00 },
	{ 0xBD27, 0x00 },
	{ 0xBD28, 0x00 },
	{ 0xBD29, 0x00 },
	{ 0xBD2A, 0x00 },
	{ 0xBD2B, 0x00 },
	{ 0xBD2C, 0x32 },
	{ 0xBD2D, 0x70 },
	{ 0xBD2E, 0x25 },
	{ 0xBD2F, 0x30 },
	{ 0xBD30, 0x3B },
	{ 0xBD31, 0xE0 },
	{ 0xBD32, 0x69 },
	{ 0xBD33, 0x40 },
	{ 0xBD34, 0x25 },
	{ 0xBD35, 0x90 },
	{ 0xBD36, 0x58 },
	{ 0xBD37, 0x00 },
	{ 0xBD38, 0x00 },
	{ 0xBD39, 0x00 },
	{ 0xBD3A, 0x00 },
	{ 0xBD3B, 0x00 },
	{ 0xBD3C, 0x32 },
	{ 0xBD3D, 0x70 },
	{ 0xBD3E, 0x25 },
	{ 0xBD3F, 0x90 },
	{ 0xBD40, 0x58 },
	{ 0xBD41, 0x00 },
	//Global Setting 2
	{ 0x793B, 0x01 },
	{ 0xACC6, 0x00 },
	{ 0xACF5, 0x00 },
	{ 0x793B, 0x00 },
	//Global Setting for 12
	{ 0x1F04, 0xB3 },
	{ 0x1F05, 0x01 },
	{ 0x1F06, 0x07 },
	{ 0x1F07, 0x66 },
	{ 0x1F08, 0x01 },
	{ 0x4D18, 0x00 },
	{ 0x4D19, 0x9D },
	{ 0x4D88, 0x00 },
	{ 0x4D89, 0x97 },
	{ 0x5C57, 0x0A },
	{ 0x5D94, 0x1F },
	{ 0x5D9E, 0x1F },
	{ 0x5E50, 0x23 },
	{ 0x5E51, 0x20 },
	{ 0x5E52, 0x07 },
	{ 0x5E53, 0x20 },
	{ 0x5E54, 0x07 },
	{ 0x5E55, 0x27 },
	{ 0x5E56, 0x0B },
	{ 0x5E57, 0x24 },
	{ 0x5E58, 0x0B },
	{ 0x5E60, 0x24 },
	{ 0x5E61, 0x24 },
	{ 0x5E62, 0x1B },
	{ 0x5E63, 0x23 },
	{ 0x5E64, 0x1B },
	{ 0x5E65, 0x28 },
	{ 0x5E66, 0x22 },
	{ 0x5E67, 0x28 },
	{ 0x5E68, 0x23 },
	{ 0x5E70, 0x25 },
	{ 0x5E71, 0x24 },
	{ 0x5E72, 0x20 },
	{ 0x5E73, 0x24 },
	{ 0x5E74, 0x20 },
	{ 0x5E75, 0x28 },
	{ 0x5E76, 0x27 },
	{ 0x5E77, 0x29 },
	{ 0x5E78, 0x24 },
	{ 0x5E80, 0x25 },
	{ 0x5E81, 0x25 },
	{ 0x5E82, 0x24 },
	{ 0x5E83, 0x25 },
	{ 0x5E84, 0x23 },
	{ 0x5E85, 0x2A },
	{ 0x5E86, 0x28 },
	{ 0x5E87, 0x2A },
	{ 0x5E88, 0x28 },
	{ 0x5E90, 0x24 },
	{ 0x5E91, 0x24 },
	{ 0x5E92, 0x28 },
	{ 0x5E93, 0x29 },
	{ 0x5E97, 0x25 },
	{ 0x5E98, 0x25 },
	{ 0x5E99, 0x2A },
	{ 0x5E9A, 0x2A },
	{ 0x5E9E, 0x3A },
	{ 0x5E9F, 0x3F },
	{ 0x5EA0, 0x17 },
	{ 0x5EA1, 0x3F },
	{ 0x5EA2, 0x17 },
	{ 0x5EA3, 0x32 },
	{ 0x5EA4, 0x10 },
	{ 0x5EA5, 0x33 },
	{ 0x5EA6, 0x10 },
	{ 0x5EAE, 0x3D },
	{ 0x5EAF, 0x48 },
	{ 0x5EB0, 0x3B },
	{ 0x5EB1, 0x45 },
	{ 0x5EB2, 0x37 },
	{ 0x5EB3, 0x3A },
	{ 0x5EB4, 0x31 },
	{ 0x5EB5, 0x3A },
	{ 0x5EB6, 0x31 },
	{ 0x5EBE, 0x40 },
	{ 0x5EBF, 0x48 },
	{ 0x5EC0, 0x3F },
	{ 0x5EC1, 0x45 },
	{ 0x5EC2, 0x3F },
	{ 0x5EC3, 0x3A },
	{ 0x5EC4, 0x32 },
	{ 0x5EC5, 0x3A },
	{ 0x5EC6, 0x33 },
	{ 0x5ECE, 0x4B },
	{ 0x5ECF, 0x4A },
	{ 0x5ED0, 0x48 },
	{ 0x5ED1, 0x4C },
	{ 0x5ED2, 0x45 },
	{ 0x5ED3, 0x3F },
	{ 0x5ED4, 0x3A },
	{ 0x5ED5, 0x3F },
	{ 0x5ED6, 0x3A },
	{ 0x5EDE, 0x48 },
	{ 0x5EDF, 0x45 },
	{ 0x5EE0, 0x3A },
	{ 0x5EE1, 0x3A },
	{ 0x5EE5, 0x4A },
	{ 0x5EE6, 0x4C },
	{ 0x5EE7, 0x3F },
	{ 0x5EE8, 0x3F },
	{ 0x5EEC, 0x06 },
	{ 0x5EED, 0x06 },
	{ 0x5EEE, 0x02 },
	{ 0x5EEF, 0x06 },
	{ 0x5EF0, 0x01 },
	{ 0x5EF1, 0x09 },
	{ 0x5EF2, 0x05 },
	{ 0x5EF3, 0x06 },
	{ 0x5EF4, 0x04 },
	{ 0x5EFC, 0x07 },
	{ 0x5EFD, 0x09 },
	{ 0x5EFE, 0x05 },
	{ 0x5EFF, 0x08 },
	{ 0x5F00, 0x04 },
	{ 0x5F01, 0x09 },
	{ 0x5F02, 0x05 },
	{ 0x5F03, 0x09 },
	{ 0x5F04, 0x04 },
	{ 0x5F0C, 0x08 },
	{ 0x5F0D, 0x09 },
	{ 0x5F0E, 0x06 },
	{ 0x5F0F, 0x09 },
	{ 0x5F10, 0x06 },
	{ 0x5F11, 0x09 },
	{ 0x5F12, 0x09 },
	{ 0x5F13, 0x09 },
	{ 0x5F14, 0x06 },
	{ 0x5F1C, 0x09 },
	{ 0x5F1D, 0x09 },
	{ 0x5F1E, 0x09 },
	{ 0x5F1F, 0x09 },
	{ 0x5F20, 0x08 },
	{ 0x5F21, 0x09 },
	{ 0x5F22, 0x09 },
	{ 0x5F23, 0x09 },
	{ 0x5F24, 0x09 },
	{ 0x5F2C, 0x09 },
	{ 0x5F2D, 0x09 },
	{ 0x5F2E, 0x09 },
	{ 0x5F2F, 0x09 },
	{ 0x5F33, 0x09 },
	{ 0x5F34, 0x09 },
	{ 0x5F35, 0x09 },
	{ 0x5F36, 0x09 },
	{ 0x5F3A, 0x01 },
	{ 0x5F3D, 0x07 },
	{ 0x5F3F, 0x01 },
	{ 0x5F4B, 0x01 },
	{ 0x5F4D, 0x04 },
	{ 0x5F4F, 0x02 },
	{ 0x5F51, 0x02 },
	{ 0x5F5A, 0x02 },
	{ 0x5F5B, 0x01 },
	{ 0x5F5D, 0x03 },
	{ 0x5F5E, 0x07 },
	{ 0x5F5F, 0x01 },
	{ 0x5F60, 0x01 },
	{ 0x5F61, 0x01 },
	{ 0x5F6A, 0x01 },
	{ 0x5F6C, 0x01 },
	{ 0x5F6D, 0x01 },
	{ 0x5F6E, 0x04 },
	{ 0x5F70, 0x02 },
	{ 0x5F72, 0x02 },
	{ 0x5F7A, 0x01 },
	{ 0x5F7B, 0x03 },
	{ 0x5F7C, 0x01 },
	{ 0x5F7D, 0x01 },
	{ 0x5F82, 0x01 },
	{ 0x60C6, 0x4A },
	{ 0x60C8, 0x4A },
	{ 0x60D6, 0x4A },
	{ 0x60D8, 0x4A },
	{ 0x62E4, 0x33 },
	{ 0x62E9, 0x33 },
	{ 0x62EE, 0x1C },
	{ 0x62EF, 0x33 },
	{ 0x62F3, 0x33 },
	{ 0x62F6, 0x1C },
	{ 0x33F2, 0x01 },
	{ 0x1F04, 0xA3 },
	{ 0x1F05, 0x01 },
	{ 0x406E, 0x00 },
	{ 0x406F, 0x08 },
	{ 0x4D08, 0x00 },
	{ 0x4D09, 0x2C },
	{ 0x4D0E, 0x00 },
	{ 0x4D0F, 0x64 },
	{ 0x4D18, 0x00 },
	{ 0x4D19, 0xB1 },
	{ 0x4D1E, 0x00 },
	{ 0x4D1F, 0xCB },
	{ 0x4D3A, 0x00 },
	{ 0x4D3B, 0x91 },
	{ 0x4D40, 0x00 },
	{ 0x4D41, 0x64 },
	{ 0x4D4C, 0x00 },
	{ 0x4D4D, 0xE8 },
	{ 0x4D52, 0x00 },
	{ 0x4D53, 0xCB },
	{ 0x4D78, 0x00 },
	{ 0x4D79, 0x2C },
	{ 0x4D7E, 0x00 },
	{ 0x4D7F, 0x64 },
	{ 0x4D88, 0x00 },
	{ 0x4D89, 0xAB },
	{ 0x4D8E, 0x00 },
	{ 0x4D8F, 0xCB },
	{ 0x4DA6, 0x00 },
	{ 0x4DA7, 0xE7 },
	{ 0x4DAC, 0x00 },
	{ 0x4DAD, 0xCB },
	{ 0x5B98, 0x00 },
	{ 0x5C52, 0x05 },
	{ 0x5C57, 0x09 },
	{ 0x5D94, 0x0A },
	{ 0x5D9E, 0x0A },
	{ 0x5E50, 0x22 },
	{ 0x5E51, 0x22 },
	{ 0x5E52, 0x07 },
	{ 0x5E53, 0x20 },
	{ 0x5E54, 0x06 },
	{ 0x5E55, 0x23 },
	{ 0x5E56, 0x0A },
	{ 0x5E57, 0x23 },
	{ 0x5E58, 0x0A },
	{ 0x5E60, 0x25 },
	{ 0x5E61, 0x29 },
	{ 0x5E62, 0x1C },
	{ 0x5E63, 0x26 },
	{ 0x5E64, 0x1C },
	{ 0x5E65, 0x2D },
	{ 0x5E66, 0x1E },
	{ 0x5E67, 0x2A },
	{ 0x5E68, 0x1E },
	{ 0x5E70, 0x26 },
	{ 0x5E71, 0x26 },
	{ 0x5E72, 0x22 },
	{ 0x5E73, 0x23 },
	{ 0x5E74, 0x20 },
	{ 0x5E75, 0x28 },
	{ 0x5E76, 0x23 },
	{ 0x5E77, 0x28 },
	{ 0x5E78, 0x23 },
	{ 0x5E80, 0x28 },
	{ 0x5E81, 0x28 },
	{ 0x5E82, 0x29 },
	{ 0x5E83, 0x27 },
	{ 0x5E84, 0x26 },
	{ 0x5E85, 0x2A },
	{ 0x5E86, 0x2D },
	{ 0x5E87, 0x2A },
	{ 0x5E88, 0x2A },
	{ 0x5E90, 0x26 },
	{ 0x5E91, 0x23 },
	{ 0x5E92, 0x28 },
	{ 0x5E93, 0x28 },
	{ 0x5E97, 0x2F },
	{ 0x5E98, 0x2E },
	{ 0x5E99, 0x32 },
	{ 0x5E9A, 0x32 },
	{ 0x5E9E, 0x50 },
	{ 0x5E9F, 0x50 },
	{ 0x5EA0, 0x1E },
	{ 0x5EA1, 0x50 },
	{ 0x5EA2, 0x1D },
	{ 0x5EA3, 0x3E },
	{ 0x5EA4, 0x14 },
	{ 0x5EA5, 0x3E },
	{ 0x5EA6, 0x14 },
	{ 0x5EAE, 0x58 },
	{ 0x5EAF, 0x5E },
	{ 0x5EB0, 0x4B },
	{ 0x5EB1, 0x5A },
	{ 0x5EB2, 0x4B },
	{ 0x5EB3, 0x4C },
	{ 0x5EB4, 0x3A },
	{ 0x5EB5, 0x4C },
	{ 0x5EB6, 0x38 },
	{ 0x5EBE, 0x56 },
	{ 0x5EBF, 0x57 },
	{ 0x5EC0, 0x50 },
	{ 0x5EC1, 0x55 },
	{ 0x5EC2, 0x50 },
	{ 0x5EC3, 0x46 },
	{ 0x5EC4, 0x3E },
	{ 0x5EC5, 0x46 },
	{ 0x5EC6, 0x3E },
	{ 0x5ECE, 0x5A },
	{ 0x5ECF, 0x5F },
	{ 0x5ED0, 0x5E },
	{ 0x5ED1, 0x5A },
	{ 0x5ED2, 0x5A },
	{ 0x5ED3, 0x50 },
	{ 0x5ED4, 0x4C },
	{ 0x5ED5, 0x50 },
	{ 0x5ED6, 0x4C },
	{ 0x5EDE, 0x57 },
	{ 0x5EDF, 0x55 },
	{ 0x5EE0, 0x46 },
	{ 0x5EE1, 0x46 },
	{ 0x5EE5, 0x73 },
	{ 0x5EE6, 0x6E },
	{ 0x5EE7, 0x5F },
	{ 0x5EE8, 0x5A },
	{ 0x5EEC, 0x0A },
	{ 0x5EED, 0x0A },
	{ 0x5EEE, 0x0F },
	{ 0x5EEF, 0x0A },
	{ 0x5EF0, 0x0E },
	{ 0x5EF1, 0x08 },
	{ 0x5EF2, 0x0C },
	{ 0x5EF3, 0x0C },
	{ 0x5EF4, 0x0F },
	{ 0x5EFC, 0x0A },
	{ 0x5EFD, 0x0A },
	{ 0x5EFE, 0x14 },
	{ 0x5EFF, 0x0A },
	{ 0x5F00, 0x14 },
	{ 0x5F01, 0x0A },
	{ 0x5F02, 0x14 },
	{ 0x5F03, 0x0A },
	{ 0x5F04, 0x19 },
	{ 0x5F0C, 0x0A },
	{ 0x5F0D, 0x0A },
	{ 0x5F0E, 0x0A },
	{ 0x5F0F, 0x05 },
	{ 0x5F10, 0x0A },
	{ 0x5F11, 0x06 },
	{ 0x5F12, 0x08 },
	{ 0x5F13, 0x0A },
	{ 0x5F14, 0x0C },
	{ 0x5F1C, 0x0A },
	{ 0x5F1D, 0x0A },
	{ 0x5F1E, 0x0A },
	{ 0x5F1F, 0x0A },
	{ 0x5F20, 0x0A },
	{ 0x5F21, 0x0A },
	{ 0x5F22, 0x0A },
	{ 0x5F23, 0x0A },
	{ 0x5F24, 0x0A },
	{ 0x5F2C, 0x0A },
	{ 0x5F2D, 0x05 },
	{ 0x5F2E, 0x06 },
	{ 0x5F2F, 0x0A },
	{ 0x5F33, 0x0A },
	{ 0x5F34, 0x0A },
	{ 0x5F35, 0x0A },
	{ 0x5F36, 0x0A },
	{ 0x5F3A, 0x00 },
	{ 0x5F3D, 0x02 },
	{ 0x5F3F, 0x0A },
	{ 0x5F4A, 0x0A },
	{ 0x5F4B, 0x0A },
	{ 0x5F4D, 0x0F },
	{ 0x5F4F, 0x00 },
	{ 0x5F51, 0x00 },
	{ 0x5F5A, 0x00 },
	{ 0x5F5B, 0x00 },
	{ 0x5F5D, 0x0A },
	{ 0x5F5E, 0x02 },
	{ 0x5F5F, 0x0A },
	{ 0x5F60, 0x0A },
	{ 0x5F61, 0x00 },
	{ 0x5F6A, 0x00 },
	{ 0x5F6C, 0x0A },
	{ 0x5F6D, 0x06 },
	{ 0x5F6E, 0x0F },
	{ 0x5F70, 0x00 },
	{ 0x5F72, 0x00 },
	{ 0x5F7A, 0x00 },
	{ 0x5F7B, 0x0A },
	{ 0x5F7C, 0x0A },
	{ 0x5F7D, 0x00 },
	{ 0x5F82, 0x06 },
	{ 0x60C6, 0x36 },
	{ 0x60C8, 0x36 },
	{ 0x60D6, 0x36 },
	{ 0x60D8, 0x36 },
	{ 0x62DF, 0x56 },
	{ 0x62E0, 0x52 },
	{ 0x62E4, 0x38 },
	{ 0x62E5, 0x51 },
	{ 0x62E9, 0x35 },
	{ 0x62EA, 0x54 },
	{ 0x62EE, 0x1D },
	{ 0x62EF, 0x38 },
	{ 0x62F3, 0x33 },
	{ 0x62F6, 0x26 },
	{ 0x6412, 0x1E },
	{ 0x6413, 0x1E },
	{ 0x6414, 0x1E },
	{ 0x6415, 0x1E },
	{ 0x6416, 0x1E },
	{ 0x6417, 0x1E },
	{ 0x6418, 0x1E },
	{ 0x641A, 0x1E },
	{ 0x641B, 0x1E },
	{ 0x641C, 0x1E },
	{ 0x641D, 0x1E },
	{ 0x641E, 0x1E },
	{ 0x641F, 0x1E },
	{ 0x6420, 0x1E },
	{ 0x6421, 0x1E },
	{ 0x6422, 0x1E },
	{ 0x6424, 0x1E },
	{ 0x6425, 0x1E },
	{ 0x6426, 0x1E },
	{ 0x6427, 0x1E },
	{ 0x6428, 0x1E },
	{ 0x6429, 0x1E },
	{ 0x642A, 0x1E },
	{ 0x642B, 0x1E },
	{ 0x642C, 0x1E },
	{ 0x642E, 0x1E },
	{ 0x642F, 0x1E },
	{ 0x6430, 0x1E },
	{ 0x6431, 0x1E },
	{ 0x6432, 0x1E },
	{ 0x6433, 0x1E },
	{ 0x6434, 0x1E },
	{ 0x6435, 0x1E },
	{ 0x6436, 0x1E },
	{ 0x6438, 0x1E },
	{ 0x6439, 0x1E },
	{ 0x643A, 0x1E },
	{ 0x643B, 0x1E },
	{ 0x643D, 0x1E },
	{ 0x643E, 0x1E },
	{ 0x643F, 0x1E },
	{ 0x6441, 0x1E },
	{ 0x33F2, 0x02 },
	{ 0x1F08, 0x00 },
	{ 0xA307, 0x30 },
	{ 0xA309, 0x30 },
	{ 0xA30B, 0x30 },
	{ 0xA406, 0x03 },
	{ 0xA407, 0x48 },
	{ 0xA408, 0x03 },
	{ 0xA409, 0x48 },
	{ 0xA40A, 0x03 },
	{ 0xA40B, 0x48 },
	// preview
	//QBIN(VBIN)_4096x3072 @30FPS
	//MIPI output setting
	{ 0x0112, 0x0A },
	{ 0x0113, 0x0A },
	{ 0x0114, 0x02 },
	//Line Length PCK Setting
	{ 0x0342, 0x3D },
	{ 0x0343, 0x00 },
	//Frame Length Lines Setting
	{ 0x0340, 0x10 },
	{ 0x0341, 0x02 },
	//ROI Setting
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x00 },
	{ 0x0347, 0x00 },
	{ 0x0348, 0x1F },
	{ 0x0349, 0xFF },
	{ 0x034A, 0x17 },
	{ 0x034B, 0xFF },
	//Mode Setting
	{ 0x0900, 0x01 },
	{ 0x0901, 0x22 },
	{ 0x0902, 0x08 },
	{ 0x3005, 0x02 },
	{ 0x3120, 0x04 },
	{ 0x3121, 0x01 },
	{ 0x3200, 0x41 },
	{ 0x3201, 0x41 },
	{ 0x32D6, 0x00 },
	//Digital Crop & Scaling
	{ 0x0408, 0x00 },
	{ 0x0409, 0x00 },
	{ 0x040A, 0x00 },
	{ 0x040B, 0x00 },
	{ 0x040C, 0x10 },
	{ 0x040D, 0x00 },
	{ 0x040E, 0x0C },
	{ 0x040F, 0x00 },
	//Output Size Setting
	{ 0x034C, 0x10 },
	{ 0x034D, 0x00 },
	{ 0x034E, 0x0C },
	{ 0x034F, 0x00 },
	//Clock Setting
	{ 0x0301, 0x05 },
	{ 0x0303, 0x02 },
	{ 0x0305, 0x04 },
	{ 0x0306, 0x00 },
	{ 0x0307, 0xC8 },
	{ 0x030B, 0x02 },
	{ 0x030D, 0x03 },
	{ 0x030E, 0x02 },
	{ 0x030F, 0x0D },
	//Other Setting
	{ 0x30CB, 0x00 },
	{ 0x30CC, 0x10 },
	{ 0x30CD, 0x00 },
	{ 0x30CE, 0x03 },
	{ 0x30CF, 0x00 },
	{ 0x319C, 0x01 },
	{ 0x3800, 0x01 },
	{ 0x3801, 0x01 },
	{ 0x3802, 0x02 },
	{ 0x3847, 0x03 },
	{ 0x38B0, 0x00 },
	{ 0x38B1, 0x00 },
	{ 0x38B2, 0x00 },
	{ 0x38B3, 0x00 },
	{ 0x38C4, 0x01 },
	{ 0x38C5, 0x2C },
	{ 0x4C3A, 0x02 },
	{ 0x4C3B, 0xD2 },
	{ 0x4C68, 0x04 },
	{ 0x4C69, 0x7E },
	{ 0x4CF8, 0x07 },
	{ 0x4CF9, 0x9E },
	{ 0x4DB8, 0x08 },
	{ 0x4DB9, 0x98 },
	//Integration Setting
	{ 0x0202, 0x0F },
	{ 0x0203, 0xD2 },
	{ 0x0224, 0x01 },
	{ 0x0225, 0xF4 },
	{ 0x313A, 0x01 },
	{ 0x313B, 0xF4 },
	{ 0x3803, 0x00 },
	{ 0x3804, 0x17 },
	{ 0x3805, 0xC0 },
	//Gain Setting
	{ 0x0204, 0x00 },
	{ 0x0205, 0x00 },
	{ 0x020E, 0x01 },
	{ 0x020F, 0x00 },
	{ 0x0216, 0x00 },
	{ 0x0217, 0x00 },
	{ 0x0218, 0x01 },
	{ 0x0219, 0x00 },
	{ 0x313C, 0x00 },
	{ 0x313D, 0x00 },
	{ 0x313E, 0x01 },
	{ 0x313F, 0x00 },
	//EPD Setting
	{ 0x0860, 0x01 },
	{ 0x0861, 0x2D },
	{ 0x0862, 0x01 },
	{ 0x0863, 0x2D },
	//PHASE PIX Setting
	{ 0x30B4, 0x01 },
	//PHASE PIX data type Setting
	{ 0x3066, 0x03 },
	{ 0x3067, 0x2B },
	{ 0x3068, 0x06 },
	{ 0x3069, 0x2B },
	//DOL Setting
	{ 0x33D0, 0x00 },
	{ 0x33D1, 0x00 },
	{ 0x33D4, 0x01 },
	{ 0x33DC, 0x0A },
	{ 0x33DD, 0x0A },
	{ 0x33DE, 0x0A },
	{ 0x33DF, 0x0A },
	//DOL data type Setting
	{ 0x3070, 0x01 },
	{ 0x3077, 0x04 },
	{ 0x3078, 0x2B },
	{ 0x3079, 0x07 },
	{ 0x307A, 0x2B },
	{ 0x307B, 0x01 },
	{ 0x3080, 0x02 },
	{ 0x3087, 0x05 },
	{ 0x3088, 0x2B },
	{ 0x3089, 0x08 },
	{ 0x308A, 0x2B },
	{ 0x308B, 0x02 },
	{ 0x3901, 0x2B },
	{ 0x3902, 0x00 },
	{ 0x3903, 0x12 },
	{ 0x3905, 0x2B },
	{ 0x3906, 0x01 },
	{ 0x3907, 0x12 },
	{ 0x3909, 0x2B },
	{ 0x390A, 0x02 },
	{ 0x390B, 0x12 },
	{ 0x3911, 0x00 },
};

/* Supported sensor mode configurations */
static const struct imx766_mode supported_mode = {
	.width = 4096,
	.height = 3072,
	.hblank = 11520,
	.vblank = 1026,
	.vblank_min = 1026,
	.vblank_max = 62463,// 29695
	.pclk = 1920000000,
	.link_freq_idx = 0,
	.code = MEDIA_BUS_FMT_SRGGB10_1X10,
	.reg_list = {
		.num_of_regs = ARRAY_SIZE(mode_4096x3072_regs),
		.regs = mode_4096x3072_regs,
	},
};

/**
 * to_imx766() - imx766 V4L2 sub-device to imx766 device.
 * @subdev: pointer to imx766 V4L2 sub-device
 *
 * Return: pointer to imx766 device
 */
static inline struct imx766 *to_imx766(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct imx766, sd);
}

/**
 * imx766_read_reg() - Read registers.
 * @imx766: pointer to imx766 device
 * @reg: register address
 * @len: length of bytes to read. Max supported bytes is 4
 * @val: pointer to register value to be filled.
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_read_reg(struct imx766 *imx766, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx766->sd);
	struct i2c_msg msgs[2] = {0};
	u8 addr_buf[2] = {0};
	u8 data_buf[4] = {0};
	int ret;

	if (WARN_ON(len > 4))
		return -EINVAL;

	put_unaligned_be16(reg, addr_buf);

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/**
 * imx766_write_reg() - Write register
 * @imx766: pointer to imx766 device
 * @reg: register address
 * @len: length of bytes. Max supported bytes is 4
 * @val: register value
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_write_reg(struct imx766 *imx766, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx766->sd);
	u8 buf[6] = {0};

	if (WARN_ON(len > 4))
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/**
 * imx766_write_regs() - Write a list of registers
 * @imx766: pointer to imx766 device
 * @regs: list of registers to be written
 * @len: length of registers array
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_write_regs(struct imx766 *imx766,
			     const struct imx766_reg *regs, u32 len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx766_write_reg(imx766, regs[i].address, 1, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * imx766_update_controls() - Update control ranges based on streaming mode
 * @imx766: pointer to imx766 device
 * @mode: pointer to imx766_mode sensor mode
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_update_controls(struct imx766 *imx766,
				  const struct imx766_mode *mode)
{
	int ret;

	ret = __v4l2_ctrl_s_ctrl(imx766->link_freq_ctrl, mode->link_freq_idx);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(imx766->hblank_ctrl, mode->hblank);
	if (ret)
		return ret;

	return __v4l2_ctrl_modify_range(imx766->vblank_ctrl, mode->vblank_min,
					mode->vblank_max, 1, mode->vblank);
}

/**
 * imx766_update_exp_gain() - Set updated exposure and gain
 * @imx766: pointer to imx766 device
 * @exposure: updated exposure value
 * @gain: updated analog gain value
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_update_exp_gain(struct imx766 *imx766, u32 exposure, u32 gain)
{
	u32 lpfr;
	int ret;

	lpfr = imx766->vblank + imx766->cur_mode->height;

	dev_dbg(imx766->dev, "Set exp %u, analog gain %u, lpfr %u\n",
		exposure, gain, lpfr);

	ret = imx766_write_reg(imx766, IMX766_REG_HOLD, 1, 1);
	if (ret)
		return ret;

	ret = imx766_write_reg(imx766, IMX766_REG_LPFR, 2, lpfr);
	if (ret)
		goto error_release_group_hold;

	ret = imx766_write_reg(imx766, IMX766_REG_EXPOSURE_CIT, 2, exposure);
	if (ret)
		goto error_release_group_hold;

	ret = imx766_write_reg(imx766, IMX766_REG_AGAIN, 2, gain);

error_release_group_hold:
	imx766_write_reg(imx766, IMX766_REG_HOLD, 1, 0);

	return ret;
}

/**
 * imx766_set_ctrl() - Set subdevice control
 * @ctrl: pointer to v4l2_ctrl structure
 *
 * Supported controls:
 * - V4L2_CID_VBLANK
 * - cluster controls:
 *   - V4L2_CID_ANALOGUE_GAIN
 *   - V4L2_CID_EXPOSURE
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx766 *imx766 =
		container_of(ctrl->handler, struct imx766, ctrl_handler);
	u32 analog_gain;
	u32 exposure;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		imx766->vblank = imx766->vblank_ctrl->val;

		dev_dbg(imx766->dev, "Received vblank %u, new lpfr %u\n",
			imx766->vblank,
			imx766->vblank + imx766->cur_mode->height);

		ret = __v4l2_ctrl_modify_range(imx766->exp_ctrl,
					       IMX766_EXPOSURE_MIN,
					       imx766->vblank +
					       imx766->cur_mode->height -
					       IMX766_EXPOSURE_OFFSET,
					       1, IMX766_EXPOSURE_DEFAULT);
		break;
	case V4L2_CID_EXPOSURE:
		/* Set controls only if sensor is in power on state */
		if (!pm_runtime_get_if_in_use(imx766->dev))
			return 0;

		exposure = ctrl->val;
		analog_gain = imx766->again_ctrl->val;

		dev_dbg(imx766->dev, "Received exp %u, analog gain %u\n",
			exposure, analog_gain);

		ret = imx766_update_exp_gain(imx766, exposure, analog_gain);

		pm_runtime_put(imx766->dev);

		break;
	default:
		dev_err(imx766->dev, "Invalid control %d\n", ctrl->id);
		ret = -EINVAL;
	}

	return ret;
}

/* V4l2 subdevice control ops*/
static const struct v4l2_ctrl_ops imx766_ctrl_ops = {
	.s_ctrl = imx766_set_ctrl,
};

/**
 * imx766_enum_mbus_code() - Enumerate V4L2 sub-device mbus codes
 * @sd: pointer to imx766 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @code: V4L2 sub-device code enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = supported_mode.code;

	return 0;
}

/**
 * imx766_enum_frame_size() - Enumerate V4L2 sub-device frame sizes
 * @sd: pointer to imx766 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fsize)
{
	if (fsize->index > 0)
		return -EINVAL;

	if (fsize->code != supported_mode.code)
		return -EINVAL;

	fsize->min_width = supported_mode.width;
	fsize->max_width = fsize->min_width;
	fsize->min_height = supported_mode.height;
	fsize->max_height = fsize->min_height;

	return 0;
}

/**
 * imx766_fill_pad_format() - Fill subdevice pad format
 *                            from selected sensor mode
 * @imx766: pointer to imx766 device
 * @mode: pointer to imx766_mode sensor mode
 * @fmt: V4L2 sub-device format need to be filled
 */
static void imx766_fill_pad_format(struct imx766 *imx766,
				   const struct imx766_mode *mode,
				   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = mode->code;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->format.quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;
}

/**
 * imx766_get_pad_format() - Get subdevice pad format
 * @sd: pointer to imx766 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx766 *imx766 = to_imx766(sd);

	mutex_lock(&imx766->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		fmt->format = *framefmt;
	} else {
		imx766_fill_pad_format(imx766, imx766->cur_mode, fmt);
	}

	mutex_unlock(&imx766->mutex);

	return 0;
}

/**
 * imx766_set_pad_format() - Set subdevice pad format
 * @sd: pointer to imx766 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx766 *imx766 = to_imx766(sd);
	const struct imx766_mode *mode;
	int ret = 0;

	mutex_lock(&imx766->mutex);

	mode = &supported_mode;
	imx766_fill_pad_format(imx766, mode, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else {
		ret = imx766_update_controls(imx766, mode);
		if (!ret)
			imx766->cur_mode = mode;
	}

	mutex_unlock(&imx766->mutex);

	return ret;
}

/**
 * imx766_init_state() - Initialize sub-device state
 * @sd: pointer to imx766 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state)
{
	struct imx766 *imx766 = to_imx766(sd);
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	imx766_fill_pad_format(imx766, &supported_mode, &fmt);

	return imx766_set_pad_format(sd, sd_state, &fmt);
}

/**
 * imx766_start_streaming() - Start sensor stream
 * @imx766: pointer to imx766 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_start_streaming(struct imx766 *imx766)
{
	const struct imx766_reg_list *reg_list;
	int ret;

	/* Write sensor mode registers */
	reg_list = &imx766->cur_mode->reg_list;
	ret = imx766_write_regs(imx766, reg_list->regs,
				reg_list->num_of_regs);
	if (ret) {
		dev_err(imx766->dev, "fail to write initial registers\n");
		return ret;
	}

	/* Setup handler will write actual exposure and gain */
	ret =  __v4l2_ctrl_handler_setup(imx766->sd.ctrl_handler);
	if (ret) {
		dev_err(imx766->dev, "fail to setup handler\n");
		return ret;
	}

	/* Delay is required before streaming*/
	usleep_range(7400, 8000);

	/* Start streaming */
	ret = imx766_write_reg(imx766, IMX766_REG_MODE_SELECT,
			       1, IMX766_MODE_STREAMING);
	if (ret) {
		dev_err(imx766->dev, "fail to start streaming\n");
		return ret;
	}

	return 0;
}

/**
 * imx766_stop_streaming() - Stop sensor stream
 * @imx766: pointer to imx766 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_stop_streaming(struct imx766 *imx766)
{
	return imx766_write_reg(imx766, IMX766_REG_MODE_SELECT,
				1, IMX766_MODE_STANDBY);
}

/**
 * imx766_set_stream() - Enable sensor streaming
 * @sd: pointer to imx766 subdevice
 * @enable: set to enable sensor streaming
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx766 *imx766 = to_imx766(sd);
	int ret;

	mutex_lock(&imx766->mutex);

	if (enable) {
		ret = pm_runtime_resume_and_get(imx766->dev);
		if (ret)
			goto error_unlock;

		ret = imx766_start_streaming(imx766);
		if (ret)
			goto error_power_off;
	} else {
		imx766_stop_streaming(imx766);
		pm_runtime_put(imx766->dev);
	}

	mutex_unlock(&imx766->mutex);

	return 0;

error_power_off:
	pm_runtime_put(imx766->dev);
error_unlock:
	mutex_unlock(&imx766->mutex);

	return ret;
}

/**
 * imx766_detect() - Detect imx766 sensor
 * @imx766: pointer to imx766 device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int imx766_detect(struct imx766 *imx766)
{
	int ret;
	u32 val;

	ret = imx766_read_reg(imx766, IMX766_REG_ID, 2, &val);
	if (ret)
		return ret;

	if (val != IMX766_CHIP_ID) {
		dev_err(imx766->dev, "chip id mismatch: %x!=%x\n",
			IMX766_CHIP_ID, val);
		return -ENXIO;
	}

	return 0;
}

/**
 * imx766_parse_hw_config() - Parse HW configuration and check if supported
 * @imx766: pointer to imx766 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_parse_hw_config(struct imx766 *imx766)
{
	struct fwnode_handle *fwnode = dev_fwnode(imx766->dev);
	struct v4l2_fwnode_endpoint bus_cfg = {};
	struct fwnode_handle *ep;
	unsigned long rate;
	unsigned int i;
	int ret;

	if (!fwnode)
		return -ENXIO;

	/* Request reset pin */
	imx766->reset_gpio = devm_gpiod_get_optional(imx766->dev, "reset",
						     GPIOD_OUT_LOW);
	if (IS_ERR(imx766->reset_gpio)) {
		dev_err(imx766->dev, "failed to get reset gpio %ld\n",
			PTR_ERR(imx766->reset_gpio));
		return PTR_ERR(imx766->reset_gpio);
	}

	/* Request pwdn pin */
	imx766->pwdn_gpio = devm_gpiod_get_optional(imx766->dev, "pwdn",
						     GPIOD_OUT_LOW);
	if (IS_ERR(imx766->pwdn_gpio)) {
		dev_err(imx766->dev, "failed to get reset gpio %ld\n",
			PTR_ERR(imx766->pwdn_gpio));
		return PTR_ERR(imx766->pwdn_gpio);
	}

	/* Request power pin */
	imx766->power_gpio = devm_gpiod_get_optional(imx766->dev, "power",
						     GPIOD_OUT_LOW);
	if (IS_ERR(imx766->power_gpio)) {
		dev_err(imx766->dev, "failed to get reset gpio %ld\n",
			PTR_ERR(imx766->power_gpio));
		return PTR_ERR(imx766->power_gpio);
	}

	/* Get sensor input clock */
	imx766->inclk = devm_clk_get(imx766->dev, NULL);
	if (IS_ERR(imx766->inclk)) {
		dev_err(imx766->dev, "could not get inclk\n");
		return PTR_ERR(imx766->inclk);
	}

	rate = clk_get_rate(imx766->inclk);
	if (rate != IMX766_INCLK_RATE) {
		dev_err(imx766->dev, "inclk frequency mismatch\n");
		return -EINVAL;
	}

	/* Get optional DT defined regulators */
	for (i = 0; i < ARRAY_SIZE(imx766_supply_names); i++)
		imx766->supplies[i].supply = imx766_supply_names[i];

	ret = devm_regulator_bulk_get(imx766->dev,
				      ARRAY_SIZE(imx766_supply_names),
				      imx766->supplies);
	if (ret)
		return ret;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(imx766->dev, "selected bus-type is not supported\n");
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX766_NUM_DATA_LANES) {
		dev_err(imx766->dev,
			"number of CSI2 data lanes %d is not supported\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(imx766->dev, "no link frequencies defined\n");
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++)
		if (bus_cfg.link_frequencies[i] == IMX766_LINK_FREQ)
			goto done_endpoint_free;

	ret = -EINVAL;

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/* V4l2 subdevice ops */
static const struct v4l2_subdev_video_ops imx766_video_ops = {
	.s_stream = imx766_set_stream,
};

static const struct v4l2_subdev_pad_ops imx766_pad_ops = {
	.enum_mbus_code = imx766_enum_mbus_code,
	.enum_frame_size = imx766_enum_frame_size,
	.get_fmt = imx766_get_pad_format,
	.set_fmt = imx766_set_pad_format,
};

static const struct v4l2_subdev_ops imx766_subdev_ops = {
	.video = &imx766_video_ops,
	.pad = &imx766_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx766_internal_ops = {
	.init_state = imx766_init_state,
};

/**
 * imx766_power_on() - Sensor power on sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx766 *imx766 = to_imx766(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(imx766_supply_names),
				    imx766->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to enable regulators\n");
		return ret;
	}

	gpiod_set_value_cansleep(imx766->reset_gpio, 0);
	gpiod_set_value_cansleep(imx766->pwdn_gpio, 0);
	gpiod_set_value_cansleep(imx766->power_gpio, 0);

	ret = clk_prepare_enable(imx766->inclk);
	if (ret) {
		dev_err(imx766->dev, "fail to enable inclk\n");
		goto error_reset;
	}

	usleep_range(1000, 1200);

	return 0;

error_reset:
	gpiod_set_value_cansleep(imx766->reset_gpio, 1);
	gpiod_set_value_cansleep(imx766->pwdn_gpio, 1);
	gpiod_set_value_cansleep(imx766->power_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(imx766_supply_names),
			       imx766->supplies);

	return ret;
}

/**
 * imx766_power_off() - Sensor power off sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx766 *imx766 = to_imx766(sd);

	clk_disable_unprepare(imx766->inclk);

	gpiod_set_value_cansleep(imx766->reset_gpio, 1);
	gpiod_set_value_cansleep(imx766->pwdn_gpio, 1);
	gpiod_set_value_cansleep(imx766->power_gpio, 1);

	regulator_bulk_disable(ARRAY_SIZE(imx766_supply_names),
			       imx766->supplies);

	return 0;
}

/**
 * imx766_init_controls() - Initialize sensor subdevice controls
 * @imx766: pointer to imx766 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_init_controls(struct imx766 *imx766)
{
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr = &imx766->ctrl_handler;
	const struct imx766_mode *mode = imx766->cur_mode;
	u32 lpfr;
	int ret;

	/* set properties from fwnode (e.g. rotation, orientation) */
	ret = v4l2_fwnode_device_parse(imx766->dev, &props);
	if (ret)
		return ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	/* Serialize controls with sensor device */
	ctrl_hdlr->lock = &imx766->mutex;

	/* Initialize exposure and gain */
	lpfr = mode->vblank + mode->height;
	imx766->exp_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
					     &imx766_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX766_EXPOSURE_MIN,
					     lpfr - IMX766_EXPOSURE_OFFSET,
					     IMX766_EXPOSURE_STEP,
					     IMX766_EXPOSURE_DEFAULT);

	imx766->again_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
					       &imx766_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN,
					       IMX766_AGAIN_MIN,
					       IMX766_AGAIN_MAX,
					       IMX766_AGAIN_STEP,
					       IMX766_AGAIN_DEFAULT);

	v4l2_ctrl_cluster(2, &imx766->exp_ctrl);

	imx766->vblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						&imx766_ctrl_ops,
						V4L2_CID_VBLANK,
						mode->vblank_min,
						mode->vblank_max,
						1, mode->vblank);

	/* Read only controls */
	imx766->pclk_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
					      &imx766_ctrl_ops,
					      V4L2_CID_PIXEL_RATE,
					      mode->pclk, mode->pclk,
					      1, mode->pclk);

	imx766->link_freq_ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr,
							&imx766_ctrl_ops,
							V4L2_CID_LINK_FREQ,
							ARRAY_SIZE(link_freq) -
							1,
							mode->link_freq_idx,
							link_freq);
	if (imx766->link_freq_ctrl)
		imx766->link_freq_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx766->hblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						&imx766_ctrl_ops,
						V4L2_CID_HBLANK,
						IMX766_REG_MIN,
						IMX766_REG_MAX,
						1, mode->hblank);
	if (imx766->hblank_ctrl)
		imx766->hblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx766_ctrl_ops, &props);

	if (ctrl_hdlr->error) {
		dev_err(imx766->dev, "control init failed: %d\n",
			ctrl_hdlr->error);
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ctrl_hdlr->error;
	}

	imx766->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

/**
 * imx766_probe() - I2C client device binding
 * @client: pointer to i2c client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx766_probe(struct i2c_client *client)
{
	struct imx766 *imx766;
	int ret;

	imx766 = devm_kzalloc(&client->dev, sizeof(*imx766), GFP_KERNEL);
	if (!imx766)
		return -ENOMEM;

	imx766->dev = &client->dev;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&imx766->sd, client, &imx766_subdev_ops);
	imx766->sd.internal_ops = &imx766_internal_ops;

	ret = imx766_parse_hw_config(imx766);
	if (ret)
		return dev_err_probe(imx766->dev, ret,
				     "HW configuration is not supported\n");

	mutex_init(&imx766->mutex);

	ret = imx766_power_on(imx766->dev);
	if (ret) {
		dev_err(imx766->dev, "failed to power-on the sensor\n");
		goto error_mutex_destroy;
	}

	/* Check module identity */
	ret = imx766_detect(imx766);
	if (ret) {
		dev_err(imx766->dev, "failed to find sensor: %d\n", ret);
		goto error_power_off;
	}

	/* Set default mode to max resolution */
	imx766->cur_mode = &supported_mode;
	imx766->vblank = imx766->cur_mode->vblank;

	ret = imx766_init_controls(imx766);
	if (ret) {
		dev_err(imx766->dev, "failed to init controls: %d\n", ret);
		goto error_power_off;
	}

	/* Initialize subdev */
	imx766->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx766->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	imx766->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx766->sd.entity, 1, &imx766->pad);
	if (ret) {
		dev_err(imx766->dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx766->sd);
	if (ret < 0) {
		dev_err(imx766->dev,
			"failed to register async subdev: %d\n", ret);
		goto error_media_entity;
	}

	pm_runtime_set_active(imx766->dev);
	pm_runtime_enable(imx766->dev);
	pm_runtime_idle(imx766->dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&imx766->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(imx766->sd.ctrl_handler);
error_power_off:
	imx766_power_off(imx766->dev);
error_mutex_destroy:
	mutex_destroy(&imx766->mutex);

	return ret;
}

static void imx766_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx766 *imx766 = to_imx766(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx766_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&imx766->mutex);
}

static const struct dev_pm_ops imx766_pm_ops = {
	SET_RUNTIME_PM_OPS(imx766_power_off, imx766_power_on, NULL)
};

static const struct of_device_id imx766_of_match[] = {
	{ .compatible = "sony,imx766" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx766_of_match);

static struct i2c_driver imx766_driver = {
	.probe = imx766_probe,
	.remove = imx766_remove,
	.driver = {
		.name = "imx766",
		.pm = &imx766_pm_ops,
		.of_match_table = imx766_of_match,
	},
};

module_i2c_driver(imx766_driver);

MODULE_DESCRIPTION("Sony IMX766 sensor driver");
MODULE_LICENSE("GPL");
