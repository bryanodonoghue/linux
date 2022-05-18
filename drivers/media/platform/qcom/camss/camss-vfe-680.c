// SPDX-License-Identifier: GPL-2.0
/*
 * camss-vfe-680.c
 *
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module v680
 *
 * Copyright (C) 2020-2023 Linaro Ltd.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "camss.h"
#include "camss-vfe.h"

#define VFE_HW_VERSION				0x00

#define VFE_IRQ0_STATUS(vfe)			(vfe_is_lite(vfe) ? 0x1c : 0x44)
#define VFE_IRQ1_STATUS(vfe)			(vfe_is_lite(vfe) ? 0x20 : 0x48)
#define VFE_IRQ0_MASK(vfe)			(vfe_is_lite(vfe) ? 0x24 : 0x34)
#define VFE_IRQ1_MASK(vfe)			(vfe_is_lite(vfe) ? 0x28 : 0x38)
#define VFE_IRQ0_CLEAR(vfe)			(vfe_is_lite(vfe) ? 0x2c : 0x3c)
#define VFE_IRQ1_CLEAR(vfe)			(vfe_is_lite(vfe) ? 0x30 : 0x40)
#define  VFE_IRQ1_SOF(vfe, rdi)			((vfe_is_lite(vfe) ? BIT(2) : BIT(8)) << (rdi * 2))
#define  VFE_IRQ1_EOF(vfe, rdi)			((vfe_is_lite(vfe) ? BIT(3) : BIT(9)) << (rdi * 2))
#define VFE_IRQ_GLOBAL_CLEAR(vfe)		(vfe_is_lite(vfe) ? 0x38 : 0x30)
#define VFE_DIAG_CONFIG				(vfe_is_lite(vfe) ? 0x40 : 0x50)

#define VFE_BUS_IRQ0_MASK(vfe)			(vfe_is_lite(vfe) ? 0x218 : 0xc18)
#define VFE_BUS_IRQ1_MASK(vfe)			(vfe_is_lite(vfe) ? 0x21c : 0xc1c)
#define VFE_BUS_IRQ0_CLEAR(vfe)			(vfe_is_lite(vfe) ? 0x220 : 0xc20)
#define VFE_BUS_IRQ1_CLEAR(vfe)			(vfe_is_lite(vfe) ? 0x224 : 0xc24)
#define VFE_BUS_IRQ0_STATUS(vfe)		(vfe_is_lite(vfe) ? 0x228 : 0xc20)
#define VFE_BUS_IRQ1_STATUS(vfe)		(vfe_is_lite(vfe) ? 0x22c : 0xc24)
#define VFE_BUS_IRQ_GLOBAL_CLEAR(vfe)		(vfe_is_lite(vfe) ? 0x230 : 0xc30)

#define VFE_BUS_CFG(vfe, c)			((vfe_is_lite(vfe) ? 0x400 : 0xe00) + (c) * 0x100)
#define VFE_BUS_IMAGE_ADDR(vfe, c)		((vfe_is_lite(vfe) ? 0x404 : 0xe04) + (c) * 0x100)
#define VFE_BUS_FRAME_INCR(vfe, c)		((vfe_is_lite(vfe) ? 0x408 : 0xe08) + (c) * 0x100)
#define VFE_BUS_IMAGE_CFG0(vfe, c)		((vfe_is_lite(vfe) ? 0x40c : 0xe0c) + (c) * 0x100)
#define VFE_BUS_IMAGE_CFG1(vfe, c)		((vfe_is_lite(vfe) ? 0x410 : 0xe10) + (c) * 0x100)
#define VFE_BUS_IMAGE_CFG2(vfe, c)		((vfe_is_lite(vfe) ? 0x414 : 0xe14) + (c) * 0x100)
#define VFE_BUS_PACKER_CFG(vfe, c)		((vfe_is_lite(vfe) ? 0x418 : 0xe18) + (c) * 0x100)
#define VFE_BUS_IRQ_SUBSAMPLE_PERIOD(vfe, c)	((vfe_is_lite(vfe) ? 0x430 : 0xe30) + (c) * 0x100)
#define VFE_BUS_IRQ_SUBSAMPLE_PATTERN(vfe, c)	((vfe_is_lite(vfe) ? 0x434 : 0xe34) + (c) * 0x100)
#define VFE_BUS_FRAMEDROP_PERIOD(vfe, c)	((vfe_is_lite(vfe) ? 0x438 : 0xe38) + (c) * 0x100)
#define VFE_BUS_FRAMEDROP_PATTERN(vfe, c)	((vfe_is_lite(vfe) ? 0x43c : 0xe3c) + (c) * 0x100)
#define VFE_BUS_MMU_PREFETCH_CFG(vfe, c)	((vfe_is_lite(vfe) ? 0x460 : 0xe60) + (c) * 0x100)
#define VFE_BUS_MMU_PREFETCH_MAX_OFFSET(vfe, c)	((vfe_is_lite(vfe) ? 0x464 : 0xe64) + (c) * 0x100)
#define VFE_BUS_ADDR_STATUS0(vfe, c)		((vfe_is_lite(vfe) ? 0x470 : 0xe70) + (c) * 0x100)

static u32 vfe_hw_version(struct vfe_device *vfe)
{
	u32 hw_version = readl_relaxed(vfe->base + VFE_HW_VERSION);
	u32 gen = (hw_version >> 28) & 0xF;
	u32 rev = (hw_version >> 16) & 0xFFF;
	u32 step = hw_version & 0xFFFF;

	dev_dbg(vfe->camss->dev, "VFE%d HW Version = %u.%u.%u\n", vfe->id, gen, rev, step);

	return hw_version;
}

static void vfe_global_reset(struct vfe_device *vfe)
{
	/* VFE680 has no global reset, simply report a completion */
	complete(&vfe->reset_complete);
}

static void vfe_isr_wm_done(struct vfe_device *vfe, u8 wm);

/*
 * vfe_isr - VFE module interrupt handler
 * @irq: Interrupt line
 * @dev: VFE device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t vfe_isr(int irq, void *dev)
{
	struct vfe_device *vfe = dev;
	u32 status;

	status = readl_relaxed(vfe->base + VFE_IRQ0_STATUS(vfe));
	writel_relaxed(status, vfe->base + VFE_IRQ0_CLEAR(vfe));

	status = readl_relaxed(vfe->base + VFE_IRQ1_STATUS(vfe));
	writel_relaxed(status, vfe->base + VFE_IRQ1_CLEAR(vfe));

	writel_relaxed(BIT(0), vfe->base + VFE_IRQ_GLOBAL_CLEAR(vfe));

	if (status & VFE_IRQ1_EOF(vfe, 0))
		vfe_isr_wm_done(vfe, 0);

	if (status & VFE_IRQ1_EOF(vfe, 1))
		vfe_isr_wm_done(vfe, 1);

	if (status & VFE_IRQ1_EOF(vfe, 2))
		vfe_isr_wm_done(vfe, 2);

	return IRQ_HANDLED;
}

/*
 * vfe_halt - Trigger halt on VFE module and wait to complete
 * @vfe: VFE device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int vfe_halt(struct vfe_device *vfe)
{
	/* rely on vfe_disable_output() to stop the VFE */
	return 0;
}

static void vfe_enable_irq(struct vfe_device *vfe)
{
	writel_relaxed(BIT(0), vfe->base + VFE_IRQ0_MASK(vfe));
	writel_relaxed((u32)0xc, vfe->base + VFE_IRQ1_MASK(vfe));

	writel_relaxed((u32)0xd0000000, vfe->base + VFE_BUS_IRQ0_MASK(vfe));
	writel_relaxed((u32)0x0, vfe->base + VFE_BUS_IRQ1_MASK(vfe));
}

static void vfe_disable_irq(struct vfe_device *vfe)
{
	writel_relaxed((u32)0x0, vfe->base + VFE_IRQ0_MASK(vfe));
	writel_relaxed((u32)0x0, vfe->base + VFE_IRQ1_MASK(vfe));

	writel_relaxed((u32)0x0, vfe->base + VFE_BUS_IRQ0_MASK(vfe));
	writel_relaxed((u32)0x0, vfe->base + VFE_BUS_IRQ1_MASK(vfe));
}

static int vfe_get_output(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_output *output;
	unsigned long flags;
	int wm_idx;

	spin_lock_irqsave(&vfe->output_lock, flags);

	output = &line->output;
	if (output->state != VFE_OUTPUT_OFF) {
		dev_err(vfe->camss->dev, "Output is running\n");
		goto error;
	}

	output->wm_num = 1;

	wm_idx = vfe_reserve_wm(vfe, line->id);
	if (wm_idx < 0) {
		dev_err(vfe->camss->dev, "Can not reserve wm\n");
		goto error_get_wm;
	}
	output->wm_idx[0] = wm_idx;

	output->drop_update_idx = 0;

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;

error_get_wm:
	vfe_release_wm(vfe, output->wm_idx[0]);
	output->state = VFE_OUTPUT_OFF;
error:
	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return -EINVAL;
}

static void vfe_wm_update(struct vfe_device *vfe, u8 wm, u32 addr,
			  struct vfe_line *line)
{
	//dev_err(vfe->camss->dev, "Update WM: addr = 0x%x, wm = %d\n", addr, wm);
	writel_relaxed(addr, vfe->base + VFE_BUS_IMAGE_ADDR(vfe, wm));
}

static void vfe_update_hfr(struct vfe_device *vfe, u8 wm)
{
	writel_relaxed((u32)0x1, vfe->base + VFE_BUS_FRAMEDROP_PATTERN(vfe, wm));
	writel_relaxed((u32)0x0, vfe->base + VFE_BUS_FRAMEDROP_PERIOD(vfe, wm));
	writel_relaxed((u32)0x0, vfe->base + VFE_BUS_IRQ_SUBSAMPLE_PERIOD(vfe, wm));
	writel_relaxed((u32)0x1, vfe->base + VFE_BUS_IRQ_SUBSAMPLE_PATTERN(vfe, wm));
}

/*
static void vfe_helper(struct vfe_device *vfe, u8 wm, u32 addr, struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix =
		&line->video_out.active_fmt.fmt.pix_mp;
	u32 stride = pix->plane_fmt[0].bytesperline;
	u32 cfg = (pix->height << 16) | (stride >> 4);

	writel_relaxed(BIT(0), vfe->base + VFE_BUS_CFG(vfe, wm));
	writel_relaxed(cfg, vfe->base + VFE_BUS_IMAGE_CFG0(vfe, wm));
	writel_relaxed(stride, vfe->base + VFE_BUS_IMAGE_CFG2(vfe, wm));
	writel_relaxed((u32)0x0, vfe->base + VFE_BUS_IMAGE_CFG1(vfe, wm));
	vfe_wm_update(vfe, wm, addr, line);
	writel_relaxed(stride * pix->height, vfe->base + VFE_BUS_FRAME_INCR(vfe, wm));
	writel_relaxed(BIT(0), vfe->base + VFE_BUS_CFG(vfe, wm));
}
*/

static void vfe_wm_start(struct vfe_device *vfe, u8 wm, struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix =
		&line->video_out.active_fmt.fmt.pix_mp;
	u32 stride = pix->plane_fmt[0].bytesperline;
	u32 cfg = (pix->height << 16) | (stride >> 4);

	writel_relaxed((u32)pix->plane_fmt[0].bytesperline * pix->height, vfe->base + VFE_BUS_FRAME_INCR(vfe, wm));

	writel_relaxed(cfg, vfe->base + VFE_BUS_IMAGE_CFG0(vfe, wm));
	writel_relaxed(stride, vfe->base + VFE_BUS_IMAGE_CFG2(vfe, wm));
	writel_relaxed((u32)0x0, vfe->base + VFE_BUS_IMAGE_CFG1(vfe, wm));
	writel_relaxed((u32)0x0, vfe->base + VFE_BUS_PACKER_CFG(vfe, wm));

	/* MMU */
	writel_relaxed((u32)0x1, vfe->base + VFE_BUS_MMU_PREFETCH_CFG(vfe, wm));
	writel_relaxed((u32)0xffffffff, vfe->base + VFE_BUS_MMU_PREFETCH_MAX_OFFSET(vfe, wm));

	/* Enable WM */
	//writel_relaxed(BIT(0), vfe->base + VFE_BUS_CFG(vfe, wm));
	writel_relaxed(BIT(0) | 0x10000, vfe->base + VFE_BUS_CFG(vfe, wm));

	//writel_relaxed((u32)0xb00, vfe->base + VFE_BUS_CFG(vfe, wm) + 0x80); // .debug_status_cfg
	//writel_relaxed((u32)0x3, vfe->base + 0x74); // .top_debug_cfg
}

static void vfe_wm_stop(struct vfe_device *vfe, u8 wm)
{
	writel_relaxed((u32)0x0, vfe->base + VFE_BUS_CFG(vfe, wm));
}

static inline void vfe_reg_update_clear(struct vfe_device *vfe,
					enum vfe_line_id line_id)
{
	//printk(KERN_ERR "%s: %u", __func__, line_id);
	//vfe->reg_update &= ~REG_UPDATE_RDI(vfe, line_id);
}

static void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	//printk(KERN_ERR "%s: %u", __func__, line_id);
	//vfe->reg_update |= REG_UPDATE_RDI(vfe, line_id);
	//writel_relaxed(vfe->reg_update, vfe->base + VFE_REG_UPDATE_CMD);
}

static void vfe_isr_wm_done(struct vfe_device *vfe, u8 wm)
{
	struct vfe_line *line = &vfe->line[vfe->wm_output_map[wm]];
	struct camss_buffer *ready_buf;
	struct vfe_output *output;
	unsigned long flags;
	u32 index;
	u64 ts = ktime_get_ns();

	spin_lock_irqsave(&vfe->output_lock, flags);

	if (vfe->wm_output_map[wm] == VFE_LINE_NONE) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Received wm done for unmapped index\n");
		goto out_unlock;
	}
	output = &vfe->line[vfe->wm_output_map[wm]].output;

	ready_buf = output->buf[0];
	if (!ready_buf) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Missing ready buf %d!\n", output->state);
		goto out_unlock;
	}

	ready_buf->vb.vb2_buf.timestamp = ts;
	ready_buf->vb.sequence = output->sequence++;

	index = 0;
	output->buf[0] = output->buf[1];
	if (output->buf[0])
		index = 1;

	output->buf[index] = vfe_buf_get_pending(output);

	if (output->buf[index]) {
		//vfe_helper(vfe, wm, output->buf[index]->addr[0], line);
		vfe_wm_update(vfe, output->wm_idx[0], output->buf[index]->addr[0], line);
	} else
		output->gen2.active_num--;

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	vb2_buffer_done(&ready_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

	return;

out_unlock:
        spin_unlock_irqrestore(&vfe->output_lock, flags);
}

static int vfe_enable_output(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_output *output = &line->output;
	unsigned long flags;
	unsigned int i;

	vfe_update_hfr(vfe, output->wm_idx[0]);

	spin_lock_irqsave(&vfe->output_lock, flags);

	vfe_reg_update_clear(vfe, line->id);

	if (output->state != VFE_OUTPUT_OFF) {
		dev_err(vfe->camss->dev, "Output is not in reserved state %d\n",
			output->state);
		spin_unlock_irqrestore(&vfe->output_lock, flags);
		return -EINVAL;
	}

	WARN_ON(output->gen2.active_num);

	output->state = VFE_OUTPUT_ON;

	output->sequence = 0;
	output->wait_reg_update = 0;
	reinit_completion(&output->reg_update);

	vfe_wm_start(vfe, output->wm_idx[0], line);

	for (i = 0; i < 2; i++) {
		output->buf[i] = vfe_buf_get_pending(output);
		if (!output->buf[i])
			break;
		output->gen2.active_num++;
		vfe_wm_update(vfe, output->wm_idx[0], output->buf[i]->addr[0], line);
	}

	vfe_reg_update(vfe, line->id);

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
}

static int vfe_queue_buffer(struct camss_video *vid,
			    struct camss_buffer *buf)
{
	struct vfe_line *line = container_of(vid, struct vfe_line, video_out);
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_output *output;
	unsigned long flags;

	output = &line->output;

	spin_lock_irqsave(&vfe->output_lock, flags);

	if (output->state == VFE_OUTPUT_ON && output->gen2.active_num < 2) {
		output->buf[output->gen2.active_num++] = buf;
		vfe_wm_update(vfe, output->wm_idx[0], buf->addr[0], line);
	} else {
		vfe_buf_add_pending(output, buf);
	}

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
}

static const struct camss_video_ops vfe_video_ops_680 = {
	.queue_buffer = vfe_queue_buffer,
	.flush_buffers = vfe_flush_buffers,
};

static void vfe_subdev_init(struct device *dev, struct vfe_device *vfe)
{
	vfe->video_ops = vfe_video_ops_680;
	vfe->line_num = 1;
}

static int vfe_enable(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	int ret;

	mutex_lock(&vfe->stream_lock);

	if (!vfe->stream_count)
		vfe_enable_irq(vfe);

	vfe->stream_count++;

	mutex_unlock(&vfe->stream_lock);

	ret = vfe_get_output(line);
	if (ret < 0)
		goto error_get_output;

	ret = vfe_enable_output(line);
	if (ret < 0)
		goto error_enable_output;

	vfe->was_streaming = 1;

	return 0;

error_enable_output:
	vfe_put_output(line);

error_get_output:
	mutex_lock(&vfe->stream_lock);

	vfe->stream_count--;
	vfe_disable_irq(vfe);

	mutex_unlock(&vfe->stream_lock);

	return ret;
}

const struct vfe_hw_ops vfe_ops_680 = {
	.global_reset = vfe_global_reset,
	.hw_version = vfe_hw_version,
	.isr = vfe_isr,
	.pm_domain_off = vfe_pm_domain_off,
	.pm_domain_on = vfe_pm_domain_on,
	.subdev_init = vfe_subdev_init,
	.vfe_disable = vfe_disable,
	.vfe_enable = vfe_enable,
	.vfe_halt = vfe_halt,
};
