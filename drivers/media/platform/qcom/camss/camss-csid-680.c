// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM Camera Subsystem - CSID (CSI Decoder) Module
 *
 * Copyright (C) 2020-2023 Linaro Ltd.
 */
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>

#include "camss-csid-gen2.h"
#include "camss-csid.h"
#include "camss.h"

#define CSID_HW_VERSION				0x0
#define  HW_VERSION_STEPPING			0
#define  HW_VERSION_REVISION			16
#define  HW_VERSION_GENERATION			28

#define CSID_RESET_CFG				0xc
#define  RESET_CFG_MODE_IMMEDIATE		BIT(0)
#define  RESET_CFG_LOCATION_COMPLETE		BIT(4)
#define CSID_RESET_CMD				0x10
#define  RESET_CMD_HW_RESET			BIT(0)
#define  RESET_CMD_SW_RESET			BIT(1)
#define  RESET_CMD_IRQ_CTRL			BIT(2)
#define CSID_IRQ_CMD				0x14

#define CSID_CSI2_RUP_AUP_CMD			0x18
#define  CSI2_RUP_AUP_MASK(rdi)			(0x100010 << (rdi))

#define CSID_TOP_IRQ_STATUS			0x7c
#define CSID_TOP_IRQ_MASK			0x80
#define CSID_TOP_IRQ_CLEAR			0x84
#define  TOP_IRQ_RESET				BIT(0)
#define  TOP_IRQ_RX				BIT(2)
#define  TOP_IRQ_LONG_PKT(rdi)			(BIT(8) << (rdi))
#define  TOP_IRQ_BUF_DONE			BIT(13)

#define CSID_BUF_DONE_IRQ_STATUS		0x8c
#define CSID_BUF_DONE_IRQ_MASK			0x90
#define CSID_BUF_DONE_IRQ_CLEAR			0x94

#define CSID_CSI2_RX_IRQ_STATUS			0x9c
#define CSID_CSI2_RX_IRQ_MASK			0xa0
#define CSID_CSI2_RX_IRQ_CLEAR			0xa4

#define CSID_CSI2_RDI_IRQ_STATUS(rdi)		(0xec + 0x10 * (rdi))
#define CSID_CSI2_RDI_IRQ_MASK(rdi)		(0xf0 + 0x10 * (rdi))
#define CSID_CSI2_RDI_IRQ_CLEAR(rdi)		(0xf4 + 0x10 * (rdi))

#define CSID_CSI2_RX_CFG0			0x200
#define  CSI2_RX_CFG0_LANE_NUM			0
#define  CSI2_RX_CFG0_DL0_INPUT			4
#define  CSI2_RX_CFG0_DL1_INPUT			8
#define  CSI2_RX_CFG0_DL2_INPUT			12
#define  CSI2_RX_CFG0_DL3_INPUT			16
#define  CSI2_RX_CFG0_PHY_NUM			20
#define  CSI2_RX_CFG0_PHY_TYPE			24

#define CSID_CSI2_RX_CFG1			0x204
#define  CSI2_RX_CFG1_PACKET_ECC_CORRECTION_EN	BIT(0)
#define  CSI2_RX_CFG1_VC_MODE			BIT(1)
#define  CSI2_RX_CFG1_MISR_EN			BIT(6)
#define  CSI2_RX_CFG1_DYN_SENSOR_EN		BIT(10)

#define CSID_CSI2_RX_CAPTURE_CTRL		0x208
#define  CSI2_RX_CAPTURE_LONG_PKT_EN		BIT(0)
#define  CSI2_RX_CAPTURE_SHORT_PKT_EN		BIT(1)
#define  CSI2_RX_CAPTURE_CPHY_PKT_EN		BIT(2)
#define  CSI2_RX_CAPTURE_LONG_PKT_DT		4
#define  CSI2_RX_CAPTURE_LONG_PKT_VC		10
#define  CSI2_RX_CAPTURE_SHORT_PKT_VC		15
#define  CSI2_RX_CAPTURE_CPHY_PKT_DT		20
#define  CSI2_RX_CAPTURE_CPHY_PKT_VC		26

#define CSID_RDI_CFG0(rdi)			(0x500 + 0x100 * (rdi))
#define  RDI_CFG0_DECODE_FORMAT			12
#define  RDI_CFG0_DATA_TYPE			16
#define  RDI_CFG0_VIRTUAL_CHANNEL		22
#define  RDI_CFG0_DT_ID				27
#define  RDI_CFG0_PATH_EN			BIT(31)

#define CSID_RDI_CTRL				0x504

#define CSID_RDI_CFG1(rdi)			(0x510 + 0x100 * (rdi))
#define  RDI_CFG1_TIMESTAMP_STB_SEL		0
#define  RDI_CFG1_TIMESTAMP_EN			BIT(4)
#define  RDI_CFG1_DROP_H_EN			BIT(5)
#define  RDI_CFG1_DROP_V_EN			BIT(6)
#define  RDI_CFG1_CROP_H_EN			BIT(7)
#define  RDI_CFG1_CROP_V_EN			BIT(8)
#define  RDI_CFG1_PACKING_FMT			15

#define CSID_RDI_ERR_RECOVERY_CFG0(rdi)		(0x514 + 0x100 * (rdi))
#define CSID_RDI_EPOCH_IRQ_CFG(rdi)		(0x52c + 0x100 * (rdi))
#define CSID_RDI_FRM_DROP_PATTERN(rdi)		(0x540 + 0x100 * (rdi))
#define CSID_RDI_FRM_DROP_PERIOD(rdi)		(0x544 + 0x100 * (rdi))
#define CSID_RDI_IRQ_SUBSAMPLE_PATTERN(rdi)	(0x548 + 0x100 * (rdi))
#define CSID_RDI_IRQ_SUBSAMPLE_PERIOD(rdi)	(0x54c + 0x100 * (rdi))
#define CSID_RDI_PIX_DROP_PATTERN(rdi)		(0x558 + 0x100 * (rdi))
#define CSID_RDI_PIX_DROP_PERIOD(rdi)		(0x55c + 0x100 * (rdi))
#define CSID_RDI_LINE_DROP_PATTERN(rdi)		(0x560 + 0x100 * (rdi))
#define CSID_RDI_LINE_DROP_PERIOD(rdi)		(0x564 + 0x100 * (rdi))

static const struct csid_format csid_formats[] = {
	{
		MEDIA_BUS_FMT_UYVY8_2X8,
		DATA_TYPE_YUV422_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		2,
	},
	{
		MEDIA_BUS_FMT_VYUY8_2X8,
		DATA_TYPE_YUV422_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		2,
	},
	{
		MEDIA_BUS_FMT_YUYV8_2X8,
		DATA_TYPE_YUV422_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		2,
	},
	{
		MEDIA_BUS_FMT_YVYU8_2X8,
		DATA_TYPE_YUV422_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		2,
	},
	{
		MEDIA_BUS_FMT_SBGGR8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		1,
	},
	{
		MEDIA_BUS_FMT_SGBRG8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		1,
	},
	{
		MEDIA_BUS_FMT_SGRBG8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		1,
	},
	{
		MEDIA_BUS_FMT_SRGGB8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		1,
	},
	{
		MEDIA_BUS_FMT_SBGGR10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10,
		1,
	},
	{
		MEDIA_BUS_FMT_SGBRG10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10,
		1,
	},
	{
		MEDIA_BUS_FMT_SGRBG10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10,
		1,
	},
	{
		MEDIA_BUS_FMT_SRGGB10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10,
		1,
	},
	{
		MEDIA_BUS_FMT_Y8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		1,
	},
	{
		MEDIA_BUS_FMT_Y10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10,
		1,
	},
	{
		MEDIA_BUS_FMT_SBGGR12_1X12,
		DATA_TYPE_RAW_12BIT,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12,
		1,
	},
	{
		MEDIA_BUS_FMT_SGBRG12_1X12,
		DATA_TYPE_RAW_12BIT,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12,
		1,
	},
	{
		MEDIA_BUS_FMT_SGRBG12_1X12,
		DATA_TYPE_RAW_12BIT,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12,
		1,
	},
	{
		MEDIA_BUS_FMT_SRGGB12_1X12,
		DATA_TYPE_RAW_12BIT,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12,
		1,
	},
	{
		MEDIA_BUS_FMT_SBGGR14_1X14,
		DATA_TYPE_RAW_14BIT,
		DECODE_FORMAT_UNCOMPRESSED_14_BIT,
		14,
		1,
	},
	{
		MEDIA_BUS_FMT_SGBRG14_1X14,
		DATA_TYPE_RAW_14BIT,
		DECODE_FORMAT_UNCOMPRESSED_14_BIT,
		14,
		1,
	},
	{
		MEDIA_BUS_FMT_SGRBG14_1X14,
		DATA_TYPE_RAW_14BIT,
		DECODE_FORMAT_UNCOMPRESSED_14_BIT,
		14,
		1,
	},
	{
		MEDIA_BUS_FMT_SRGGB14_1X14,
		DATA_TYPE_RAW_14BIT,
		DECODE_FORMAT_UNCOMPRESSED_14_BIT,
		14,
		1,
	},
};

static void csid_configure_stream(struct csid_device *csid, u8 enable)
{
	struct v4l2_mbus_framefmt *input_format = &csid->fmt[MSM_CSID_PAD_SRC];
	const struct csid_format *format = csid_get_fmt_entry(csid->formats,
							      csid->nformats,
							      input_format->code);
	u32 val, phy_sel = csid->phy.csiphy_id;
	u8 lane_cnt = csid->phy.lane_cnt;
	u8 vc = 0, dt_id = vc * 4;

	if (!enable)
		return;

	if (!lane_cnt)
		lane_cnt = 4;

	/* RDI configuration */
	val = format->decode_format << RDI_CFG0_DECODE_FORMAT;
	val |= format->data_type << RDI_CFG0_DATA_TYPE;
	val |= vc << RDI_CFG0_VIRTUAL_CHANNEL;
	val |= dt_id << RDI_CFG0_DT_ID;
	writel_relaxed(val, csid->base + CSID_RDI_CFG0(0));

	val = (0x2 << RDI_CFG1_TIMESTAMP_STB_SEL) | RDI_CFG1_TIMESTAMP_EN;
	val |= RDI_CFG1_DROP_H_EN | RDI_CFG1_DROP_V_EN |
	      RDI_CFG1_CROP_H_EN | RDI_CFG1_CROP_V_EN;
	val |= 0x1 << RDI_CFG1_PACKING_FMT;
	/*val = 0x2 << RDI_CFG1_TIMESTAMP_STB_SEL;*/
	writel_relaxed(val, csid->base + CSID_RDI_CFG1(0));

	writel_relaxed((u32)1, csid->base + CSID_RDI_FRM_DROP_PERIOD(0));
	writel_relaxed((u32)0, csid->base + CSID_RDI_FRM_DROP_PATTERN(0));
	writel_relaxed((u32)0, csid->base + CSID_RDI_PIX_DROP_PATTERN(0));
	writel_relaxed((u32)1, csid->base + CSID_RDI_PIX_DROP_PERIOD(0));
	writel_relaxed((u32)0, csid->base + CSID_RDI_LINE_DROP_PATTERN(0));
	writel_relaxed((u32)1, csid->base + CSID_RDI_LINE_DROP_PERIOD(0));

	/* Enable RDI */
	val = readl_relaxed(csid->base + CSID_RDI_CFG0(0));
	val |= RDI_CFG0_PATH_EN;
	writel_relaxed(val, csid->base + CSID_RDI_CFG0(0));

	writel_relaxed(BIT(3) | BIT(0), csid->base + CSID_RDI_ERR_RECOVERY_CFG0(0));
	writel_relaxed((u32)0x0, csid->base + CSID_RDI_EPOCH_IRQ_CFG(0));

	/* Enable CSI2 */
	writel_relaxed(CSI2_RUP_AUP_MASK(0), csid->base + CSID_CSI2_RUP_AUP_CMD);

	val = (lane_cnt - 1) << CSI2_RX_CFG0_LANE_NUM;
	val |= csid->phy.lane_assign << CSI2_RX_CFG0_DL0_INPUT;
	val |= (phy_sel + 1) << CSI2_RX_CFG0_PHY_NUM;		/* See CAM_ISP_IFE_IN_RES_* */
	writel_relaxed(val, csid->base + CSID_CSI2_RX_CFG0);

	val = CSI2_RX_CFG1_PACKET_ECC_CORRECTION_EN | CSI2_RX_CFG1_MISR_EN;
	writel_relaxed(val, csid->base + CSID_CSI2_RX_CFG1);

	/* Enable interrupts */
	val = TOP_IRQ_RESET | TOP_IRQ_LONG_PKT(0) | TOP_IRQ_BUF_DONE;
	writel_relaxed(val, csid->base + CSID_TOP_IRQ_MASK);

	val = 0x1bf9800;
	writel_relaxed(val, csid->base + CSID_CSI2_RX_IRQ_MASK);

	val = 0x10881004;
	writel_relaxed(val, csid->base + CSID_CSI2_RDI_IRQ_MASK(0));

	/* Enable RX */
	val = dt_id << CSI2_RX_CAPTURE_LONG_PKT_DT;
	val = 0x2b0;
	val |= vc << CSI2_RX_CAPTURE_LONG_PKT_VC;
	val |= CSI2_RX_CAPTURE_LONG_PKT_EN;
	writel_relaxed(val, csid->base + CSID_CSI2_RX_CAPTURE_CTRL);

	writel_relaxed(BIT(0), csid->base + CSID_RDI_CTRL);
}

/*
 * csid_hw_version - CSID hardware version query
 * @csid: CSID device
 *
 * Return HW version or error
 */
static u32 csid_hw_version(struct csid_device *csid)
{
	u32 hw_version, hw_gen, hw_rev, hw_step;

	hw_version = readl_relaxed(csid->base + CSID_HW_VERSION);
	hw_gen = (hw_version >> HW_VERSION_GENERATION) & 0xF;
	hw_rev = (hw_version >> HW_VERSION_REVISION) & 0xFFF;
	hw_step = (hw_version >> HW_VERSION_STEPPING) & 0xFFFF;
	dev_dbg(csid->camss->dev, "CSID HW Version = %u.%u.%u\n",
		hw_gen, hw_rev, hw_step);

	return hw_version;
}

/*
 * csid_reset - Trigger reset on CSID module and wait to complete
 * @csid: CSID device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_reset(struct csid_device *csid)
{
	unsigned long time;
	u32 val;

	reinit_completion(&csid->reset_complete);

	val = TOP_IRQ_RESET;
	writel_relaxed(val, csid->base + CSID_TOP_IRQ_MASK);

	val = RESET_CFG_MODE_IMMEDIATE | RESET_CFG_LOCATION_COMPLETE;
	writel_relaxed(val, csid->base + CSID_RESET_CFG);

	val = RESET_CMD_SW_RESET;
	writel_relaxed(val, csid->base + CSID_RESET_CMD);

        time = wait_for_completion_timeout(&csid->reset_complete,
					   msecs_to_jiffies(CSID_RESET_TIMEOUT_MS));
	if (!time) {
		dev_err(csid->camss->dev, "CSID reset timeout\n");
		return -EIO;
	}

	return 0;
}

/*
 * csid_isr - CSID module interrupt service routine
 * @irq: Interrupt line
 * @dev: CSID device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t csid_isr(int irq, void *dev)
{
	struct csid_device *csid = dev;
	bool reset_done, err = false;
	u32 top_val, val;

	top_val = readl_relaxed(csid->base + CSID_TOP_IRQ_STATUS);
	writel_relaxed(top_val, csid->base + CSID_TOP_IRQ_CLEAR);
	reset_done = top_val & TOP_IRQ_RESET;

	if (top_val & TOP_IRQ_RX) {
		val = readl_relaxed(csid->base + CSID_CSI2_RX_IRQ_STATUS);
		writel_relaxed(val, csid->base + CSID_CSI2_RX_IRQ_CLEAR);
		err = val & 0x1fff800;
	}

	if (top_val & TOP_IRQ_LONG_PKT(0)) {
		val = readl_relaxed(csid->base + CSID_CSI2_RDI_IRQ_STATUS(0));
		writel_relaxed(val, csid->base + CSID_CSI2_RDI_IRQ_CLEAR(0));

		writel_relaxed(CSI2_RUP_AUP_MASK(0), csid->base + CSID_CSI2_RUP_AUP_CMD);
	}

	if (top_val & TOP_IRQ_BUF_DONE) {
		val = readl_relaxed(csid->base + CSID_BUF_DONE_IRQ_STATUS);
		writel_relaxed(val, csid->base + CSID_BUF_DONE_IRQ_CLEAR);
	}

	writel_relaxed(BIT(0), csid->base + CSID_IRQ_CMD);

	if (reset_done)
		complete(&csid->reset_complete);

	return (err ? IRQ_WAKE_THREAD : IRQ_HANDLED);
}

/*
 * csid_isr_thread - CSID module threaded interrupt service routine
 * @irq: Interrupt line
 * @dev: CSID device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t csid_isr_thread(int irq, void *dev)
{
	struct csid_device *csid = dev;

	dev_err(csid->camss->dev, "RX error, CSID reset\n");

	csid_reset(csid);

	return IRQ_HANDLED;
}

static u32 csid_src_pad_code(struct csid_device *csid, u32 sink_code,
			     unsigned int match_format_idx, u32 match_code)
{
	switch (sink_code) {
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	{
		u32 src_code[] = {
			MEDIA_BUS_FMT_SBGGR10_1X10,
			MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_LE,
		};

		return csid_find_code(src_code, ARRAY_SIZE(src_code),
				      match_format_idx, match_code);
	}
	case MEDIA_BUS_FMT_Y10_1X10:
	{
		u32 src_code[] = {
			MEDIA_BUS_FMT_Y10_1X10,
			MEDIA_BUS_FMT_Y10_2X8_PADHI_LE,
		};

		return csid_find_code(src_code, ARRAY_SIZE(src_code),
				      match_format_idx, match_code);
	}
	default:
		if (match_format_idx > 0)
			return 0;

		return sink_code;
	}
}

static void csid_subdev_init(struct csid_device *csid)
{
	csid->formats = csid_formats;
	csid->nformats = ARRAY_SIZE(csid_formats);
	csid->testgen.modes = csid_testgen_modes;
	csid->testgen.nmodes = CSID_PAYLOAD_MODE_NUM_SUPPORTED_GEN2;
}

const struct csid_hw_ops csid_ops_680 = {
	.configure_stream = csid_configure_stream,
	.configure_testgen_pattern = csid_configure_testgen_pattern,
	.hw_version = csid_hw_version,
	.isr = csid_isr,
	.isr_thread = csid_isr_thread,
	.reset = csid_reset,
	.src_pad_code = csid_src_pad_code,
	.subdev_init = csid_subdev_init,
};
