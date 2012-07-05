/*
 * Support eMMa-PrP through mem2mem framework.
 *
 * eMMa-PrP is a piece of HW that allows fetching buffers
 * from one memory location and do several operations on
 * them such as scaling or format conversion giving, as a result
 * a new processed buffer in another memory location.
 *
 * Based on mem2mem_testdev.c by Pawel Osciak.
 *
 * Copyright (c) 2011 Vista Silicon S.L.
 * Javier Martin <javier.martin@vista-silicon.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version
 */
#define DEBUG

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>

#include <linux/platform_device.h>
#include <linux/sched.h>

#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#define MEM2MEM_TEST_MODULE_NAME "mem2mem-deinterlace"

MODULE_DESCRIPTION("mem2mem device which supports eMMa-PrP present in mx2 SoCs");
MODULE_AUTHOR("Javier Martin <javier.martin@vista-silicon.com");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.1");

static bool debug = true;
module_param(debug, bool, 0644);

#define MIN_W 32
#define MIN_H 32
#define MAX_W 2040
#define MAX_H 2046

#define W_ALIGN_MASK_YUV420	0x07 /* multiple of 8 */
#define W_ALIGN_MASK_OTHERS	0x03 /* multiple of 4 */
#define H_ALIGN_MASK		0x01 /* multiple of 2 */

/* Flags that indicate a format can be used for capture/output */
#define MEM2MEM_CAPTURE	(1 << 0)
#define MEM2MEM_OUTPUT	(1 << 1)

#define MEM2MEM_NAME		"m2m-deinterlace"

/* In bytes, per queue */
#define MEM2MEM_VID_MEM_LIMIT	(16 * 1024 * 1024)

#define dprintk(dev, fmt, arg...) \
	v4l2_dbg(1, debug, &dev->v4l2_dev, "%s: " fmt, __func__, ## arg)

#define DST_QUEUE_OFF_BASE	(1 << 30)

/* EMMA PrP */
#define PRP_CNTL                        0x00
#define PRP_INTR_CNTL                   0x04
#define PRP_INTRSTATUS                  0x08
#define PRP_SOURCE_Y_PTR                0x0c
#define PRP_SOURCE_CB_PTR               0x10
#define PRP_SOURCE_CR_PTR               0x14
#define PRP_DEST_RGB1_PTR               0x18
#define PRP_DEST_RGB2_PTR               0x1c
#define PRP_DEST_Y_PTR                  0x20
#define PRP_DEST_CB_PTR                 0x24
#define PRP_DEST_CR_PTR                 0x28
#define PRP_SRC_FRAME_SIZE              0x2c
#define PRP_DEST_CH1_LINE_STRIDE        0x30
#define PRP_SRC_PIXEL_FORMAT_CNTL       0x34
#define PRP_CH1_PIXEL_FORMAT_CNTL       0x38
#define PRP_CH1_OUT_IMAGE_SIZE          0x3c
#define PRP_CH2_OUT_IMAGE_SIZE          0x40
#define PRP_SRC_LINE_STRIDE             0x44
#define PRP_CSC_COEF_012                0x48
#define PRP_CSC_COEF_345                0x4c
#define PRP_CSC_COEF_678                0x50
#define PRP_CH1_RZ_HORI_COEF1           0x54
#define PRP_CH1_RZ_HORI_COEF2           0x58
#define PRP_CH1_RZ_HORI_VALID           0x5c
#define PRP_CH1_RZ_VERT_COEF1           0x60
#define PRP_CH1_RZ_VERT_COEF2           0x64
#define PRP_CH1_RZ_VERT_VALID           0x68
#define PRP_CH2_RZ_HORI_COEF1           0x6c
#define PRP_CH2_RZ_HORI_COEF2           0x70
#define PRP_CH2_RZ_HORI_VALID           0x74
#define PRP_CH2_RZ_VERT_COEF1           0x78
#define PRP_CH2_RZ_VERT_COEF2           0x7c
#define PRP_CH2_RZ_VERT_VALID           0x80

#define PRP_CNTL_CH1EN          (1 << 0)
#define PRP_CNTL_CH2EN          (1 << 1)
#define PRP_CNTL_CSIEN          (1 << 2)
#define PRP_CNTL_DATA_IN_YUV420 (0 << 3)
#define PRP_CNTL_DATA_IN_YUV422 (1 << 3)
#define PRP_CNTL_DATA_IN_RGB16  (2 << 3)
#define PRP_CNTL_DATA_IN_RGB32  (3 << 3)
#define PRP_CNTL_CH1_OUT_RGB8   (0 << 5)
#define PRP_CNTL_CH1_OUT_RGB16  (1 << 5)
#define PRP_CNTL_CH1_OUT_RGB32  (2 << 5)
#define PRP_CNTL_CH1_OUT_YUV422 (3 << 5)
#define PRP_CNTL_CH2_OUT_YUV420 (0 << 7)
#define PRP_CNTL_CH2_OUT_YUV422 (1 << 7)
#define PRP_CNTL_CH2_OUT_YUV444 (2 << 7)
#define PRP_CNTL_CH1_LEN        (1 << 9)
#define PRP_CNTL_CH2_LEN        (1 << 10)
#define PRP_CNTL_SKIP_FRAME     (1 << 11)
#define PRP_CNTL_SWRST          (1 << 12)
#define PRP_CNTL_CLKEN          (1 << 13)
#define PRP_CNTL_WEN            (1 << 14)
#define PRP_CNTL_CH1BYP         (1 << 15)
#define PRP_CNTL_IN_TSKIP(x)    ((x) << 16)
#define PRP_CNTL_CH1_TSKIP(x)   ((x) << 19)
#define PRP_CNTL_CH2_TSKIP(x)   ((x) << 22)
#define PRP_CNTL_INPUT_FIFO_LEVEL(x)    ((x) << 25)
#define PRP_CNTL_RZ_FIFO_LEVEL(x)       ((x) << 27)
#define PRP_CNTL_CH2B1EN        (1 << 29)
#define PRP_CNTL_CH2B2EN        (1 << 30)
#define PRP_CNTL_CH2FEN         (1 << 31)

#define PRP_SIZE_HEIGHT(x)	(x)
#define PRP_SIZE_WIDTH(x)	((x) << 16)

/* IRQ Enable and status register */
#define PRP_INTR_RDERR          (1 << 0)
#define PRP_INTR_CH1WERR        (1 << 1)
#define PRP_INTR_CH2WERR        (1 << 2)
#define PRP_INTR_CH1FC          (1 << 3)
#define PRP_INTR_CH2FC          (1 << 5)
#define PRP_INTR_LBOVF          (1 << 7)
#define PRP_INTR_CH2OVF         (1 << 8)

#define PRP_INTR_ST_RDERR	(1 << 0)
#define PRP_INTR_ST_CH1WERR	(1 << 1)
#define PRP_INTR_ST_CH2WERR	(1 << 2)
#define PRP_INTR_ST_CH2B2CI	(1 << 3)
#define PRP_INTR_ST_CH2B1CI	(1 << 4)
#define PRP_INTR_ST_CH1B2CI	(1 << 5)
#define PRP_INTR_ST_CH1B1CI	(1 << 6)
#define PRP_INTR_ST_LBOVF	(1 << 7)
#define PRP_INTR_ST_CH2OVF	(1 << 8)

struct emmaprp_fmt {
	char	*name;
	u32	fourcc;
	enum v4l2_field field;
	/* Types the format can be used for */
	u32	types;
};

static struct emmaprp_fmt formats[] = {
	{
		.name	= "YUV 4:2:0 Planar",
		.fourcc	= V4L2_PIX_FMT_YUV420,
		.field	= V4L2_FIELD_NONE,
		.types	= MEM2MEM_CAPTURE,
	},
	{
		.name	= "YUV 4:2:0 Planar",
		.fourcc	= V4L2_PIX_FMT_YUV420,
		.field	= V4L2_FIELD_SEQ_TB,
		.types	= MEM2MEM_OUTPUT,
	},
};

/* Per-queue, driver-specific private data */
struct emmaprp_q_data {
	unsigned int		width;
	unsigned int		height;
	unsigned int		sizeimage;
	struct emmaprp_fmt	*fmt;
};

enum {
	V4L2_M2M_SRC = 0,
	V4L2_M2M_DST = 1,
};

enum {
	YUV420_DMA_Y_ODD,
	YUV420_DMA_Y_EVEN,
	YUV420_DMA_U_ODD,
	YUV420_DMA_U_EVEN,
	YUV420_DMA_V_ODD,
	YUV420_DMA_V_EVEN,
};
// static void emmaprp_dma_task(unsigned long data);
//
// static unsigned long my_tasklet_data;
//
// DECLARE_TASKLET( my_tasklet, emmaprp_dma_task,
// 		 (unsigned long) &my_tasklet_data );

/* Source and destination queue data */
static struct emmaprp_q_data q_data[2];

static struct emmaprp_q_data *get_q_data(enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &q_data[V4L2_M2M_SRC];
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &q_data[V4L2_M2M_DST];
	default:
		BUG();
	}
	return NULL;
}

#define NUM_FORMATS ARRAY_SIZE(formats)

static struct emmaprp_fmt *find_format(struct v4l2_format *f)
{
	struct emmaprp_fmt *fmt;
	unsigned int k;

	for (k = 0; k < NUM_FORMATS; k++) {
		fmt = &formats[k];
		if ((fmt->types == f->type) &&
			(fmt->fourcc == f->fmt.pix.pixelformat))
			break;
	}

	if (k == NUM_FORMATS)
		return NULL;

	return &formats[k];
}

struct emmaprp_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	*vfd;

	atomic_t		busy;
	struct mutex		dev_mutex;
	spinlock_t		irqlock;

	struct dma_chan		*dma_chan;

	struct v4l2_m2m_dev	*m2m_dev;
	struct vb2_alloc_ctx	*alloc_ctx;
};

struct emmaprp_ctx {
	struct emmaprp_dev	*dev;

	/* Abort requested by m2m */
	int			aborting;
	dma_cookie_t		cookie;
	struct v4l2_m2m_ctx	*m2m_ctx;
};

/*
 * mem2mem callbacks
 */
static int emmaprp_job_ready(void *priv)
{
	struct emmaprp_ctx *ctx = priv;
	struct emmaprp_dev *pcdev = ctx->dev;

//printk("%s\n", __func__);
	if ((v4l2_m2m_num_src_bufs_ready(ctx->m2m_ctx) > 0)
	    && (v4l2_m2m_num_dst_bufs_ready(ctx->m2m_ctx) > 0)
	    && (atomic_read(&ctx->dev->busy) == 0)) {
//printk("%s: task ready to run\n", __func__);
		return 1;
	}

	dprintk(pcdev, "Task not ready to run\n");

	return 0;
}

static void emmaprp_job_abort(void *priv)
{
	struct emmaprp_ctx *ctx = priv;
	struct emmaprp_dev *pcdev = ctx->dev;

	ctx->aborting = 1;

	dprintk(pcdev, "Aborting task\n");

	v4l2_m2m_job_finish(pcdev->m2m_dev, ctx->m2m_ctx);
}

static void emmaprp_lock(void *priv)
{
	struct emmaprp_ctx *ctx = priv;
	struct emmaprp_dev *pcdev = ctx->dev;
	mutex_lock(&pcdev->dev_mutex);
}

static void emmaprp_unlock(void *priv)
{
	struct emmaprp_ctx *ctx = priv;
	struct emmaprp_dev *pcdev = ctx->dev;
	mutex_unlock(&pcdev->dev_mutex);
}

static void dma_callback(void *data)
{
	struct emmaprp_ctx *curr_ctx = data;
	struct emmaprp_dev *pcdev = curr_ctx->dev;
	struct vb2_buffer *src_vb, *dst_vb;
	printk("%s\n", __func__);
// 	printk("%s: all transfers ended!!\n", __func__);
	atomic_set(&pcdev->busy, 0);

	src_vb = v4l2_m2m_src_buf_remove(curr_ctx->m2m_ctx);
	dst_vb = v4l2_m2m_dst_buf_remove(curr_ctx->m2m_ctx);
	dst_vb->v4l2_buf.sequence = src_vb->v4l2_buf.sequence;
	printk("%s: dst_sequence: %d, src_sequence: %d\n", __func__, dst_vb->v4l2_buf.sequence, src_vb->v4l2_buf.sequence);
	v4l2_m2m_buf_done(src_vb, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst_vb, VB2_BUF_STATE_DONE);

// irq_ok:
	//printk("%s: job_finish\n", __func__);
	v4l2_m2m_job_finish(pcdev->m2m_dev, curr_ctx->m2m_ctx);

}

static void emmaprp_issue_dma(struct emmaprp_ctx *ctx, int op)
{
	struct emmaprp_q_data *s_q_data, *d_q_data;
	struct vb2_buffer *src_buf, *dst_buf;
	struct emmaprp_dev *pcdev = ctx->dev;
	struct dma_chan *chan = pcdev->dma_chan;
	struct dma_device *dmadev = chan->device;
	struct dma_async_tx_descriptor *tx;
	struct dma_interleaved_template *xt;
	unsigned int s_width, s_height;
	unsigned int d_width, d_height;
	unsigned int d_size, s_size;
	dma_addr_t p_in, p_out;
	enum dma_ctrl_flags flags;

	xt = kzalloc(sizeof(struct dma_async_tx_descriptor)
			+ sizeof(struct data_chunk), GFP_KERNEL);
	if (!xt)
		printk("MALLOC ERROR!!!!!\n");

	src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);

	s_q_data = get_q_data(V4L2_BUF_TYPE_VIDEO_OUTPUT);
	s_width	= s_q_data->width;
	s_height = s_q_data->height;
	s_size = s_width * s_height;

	d_q_data = get_q_data(V4L2_BUF_TYPE_VIDEO_CAPTURE);
	d_width = d_q_data->width;
	d_height = d_q_data->height;
	d_size = d_width * d_height;

	/* FIXME: try/set_fmt must adjust this properly */
	BUG_ON(d_size != s_size);
	//printk("%s 6\n", __func__);
	p_in = (dma_addr_t)vb2_dma_contig_plane_dma_addr(src_buf, 0);
	p_out = (dma_addr_t)vb2_dma_contig_plane_dma_addr(dst_buf, 0);
	if (!p_in || !p_out) {
		v4l2_err(&pcdev->v4l2_dev,
			 "Acquiring kernel pointers to buffers failed\n");
		return;
	}

	//printk("%s p_in = %p, p_out = %p\n", __func__, p_in, p_out);
	switch (op) {
		case YUV420_DMA_Y_ODD:
			//printk("%s: YUV420_DMA_Y_ODD\n", __func__);
// 			printk("%s: s_width = %d\n", __func__, s_width);
			xt->numf = s_height / 2;
			xt->sgl[0].size = s_width;
			xt->sgl[0].icg = s_width;
			xt->src_start = p_in;
			xt->dst_start = p_out;
			break;
		case YUV420_DMA_Y_EVEN:
			//printk("%s: YUV420_DMA_Y_EVEN\n", __func__);
			xt->numf = s_height / 2;
			xt->sgl[0].size = s_width;
			xt->sgl[0].icg = s_width;
			xt->src_start = p_in + s_size / 2;
			xt->dst_start = p_out + s_width;
			break;
		case YUV420_DMA_U_ODD:
			//printk("%s: YUV420_DMA_U_ODD\n", __func__);
			xt->numf = s_height / 4;
			xt->sgl[0].size = s_width / 2;
			xt->sgl[0].icg = s_width / 2;
			xt->src_start = p_in + s_size;
			xt->dst_start = p_out + s_size;
			break;
		case YUV420_DMA_U_EVEN:
			//printk("%s: YUV420_DMA_U_EVEN\n", __func__);
			xt->numf = s_height / 4;
			xt->sgl[0].size = s_width / 2;
			xt->sgl[0].icg = s_width / 2;
			xt->src_start = p_in + (9 * s_size) / 8;
			xt->dst_start = p_out + s_size + s_width / 2;
			break;
		case YUV420_DMA_V_ODD:
			//printk("%s: YUV420_DMA_V_ODD\n", __func__);
			xt->numf = s_height / 4;
			xt->sgl[0].size = s_width / 2;
			xt->sgl[0].icg = s_width / 2;
			xt->src_start = p_in + (5 * s_size) / 4;
			xt->dst_start = p_out + (5 * s_size) / 4;
			break;
		case YUV420_DMA_V_EVEN:
		default:
			//printk("%s: YUV420_DMA_V_EVEN\n", __func__);
			xt->numf = s_height / 4;
			xt->sgl[0].size = s_width / 2;
			xt->sgl[0].icg = s_width / 2;
			xt->src_start = p_in + (11 * s_size) / 8;
			xt->dst_start = p_out + (5 * s_size) / 4 + s_width / 2;
			break;
	}
	//printk("%s 8\n", __func__);
	/* Common parameters for al transfers */
// 	xt.sgl[0].size = chunk.size;
// 	xt.sgl[0].icg = chunk.icg;
	xt->frame_size = 1;
	xt->dir = DMA_MEM_TO_MEM;
	xt->src_sgl = false;
	xt->dst_sgl = true;
	flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT |
		DMA_COMPL_SKIP_DEST_UNMAP | DMA_COMPL_SKIP_SRC_UNMAP;
// 	printk("%s: 3 ctx = %p \n", __func__, ctx);
	tx = dmadev->device_prep_interleaved_dma(chan, xt, flags);
// 	tx = dmadev->device_prep_interleaved_dma(chan, NULL, flags);
	if (tx == NULL) {
		v4l2_warn(&pcdev->v4l2_dev, "DMA interleaved prep error\n");
		printk("%s: lolz ctx = %p \n", __func__, ctx);
		return;
	}
//        tx = dmadev->device_prep_dma_memcpy(chan, p_out, p_in, s_size * 3/2, flags);
//        if (tx == NULL) {
//                v4l2_warn(&pcdev->v4l2_dev,
//                          "DMA prep error with src=0x%x dst=0x%x len=%d\n",
//                          p_in, p_out, s_size * 3/2);
//                return;
//        }
// 	//printk("%s 9\n", __func__);
// printk("%s: 4 ctx = %p\n", __func__, ctx);
	if (op == YUV420_DMA_V_EVEN) {
		tx->callback = dma_callback;
		tx->callback_param = ctx;
	}
// printk("%s: 5 ctx = %p\n", __func__, ctx);
	ctx->cookie = dmaengine_submit(tx);
// printk("%s: 6 ctx = %p\n", __func__, ctx);
	//printk("%s 10\n", __func__);
	if (dma_submit_error(ctx->cookie)) {
		v4l2_warn(&pcdev->v4l2_dev,
			  "DMA submit error %d with src=0x%x dst=0x%x len=0x%x\n",
			  ctx->cookie, p_in, p_out, s_size * 3/2);
		return;
	}
// printk("%s: 7 ctx = %p\n", __func__, ctx);
	//printk("%s 11\n", __func__);
	dma_async_issue_pending(chan);
// // 	while (1) {
// // 		if (dma_async_memcpy_complete(chan, ctx->cookie, NULL, NULL))
// // 			break;
// // 		else
// // 			schedule();
// // 	}
// 		if (irqs_disabled())
// 			printk("%s: IRQS DISABLED!!\n", __func__);
// 	dma_sync_wait(chan, ctx->cookie);
// 	printk("%s 12 ctx = %p\n", __func__, ctx);
	kfree(xt);
}

// static void emmaprp_dma_task(unsigned long data)
// {
// 	struct emmaprp_dev *pcdev = (struct emmaprp_dev *)my_tasklet_data;
// 	struct dma_chan *chan = pcdev->dma_chan;
// 	struct vb2_buffer *src_vb, *dst_vb;
// 	struct emmaprp_ctx *curr_ctx;
//
// 	//printk("%s: pcdev: %p\n", __func__, pcdev);
// 	//printk("%s: pcdev->m2m_dev: %p\n", __func__, pcdev->m2m_dev);
// 	//printk("%s: my_tasklet_data = %p\n", __func__, my_tasklet_data);
// // 	curr_ctx = v4l2_m2m_get_curr_priv(pcdev->m2m_dev);
// // 	if (NULL == curr_ctx) {
// // 		printk(KERN_ERR
// // 			"Instance released before the end of transaction\n");
// // 		return;
// // 	}
//
// 	if (curr_ctx->aborting)
// 		return; /*FIXME*/
// printk("%s: 1\n", __func__);
// 	emmaprp_issue_dma(curr_ctx, YUV420_DMA_Y_ODD);
// printk("%s: 2\n", __func__);
// 	emmaprp_issue_dma(curr_ctx, YUV420_DMA_Y_EVEN);
// printk("%s: 3\n", __func__);
// 	emmaprp_issue_dma(curr_ctx, YUV420_DMA_U_ODD);
// printk("%s: 4\n", __func__);
// 	emmaprp_issue_dma(curr_ctx, YUV420_DMA_U_EVEN);
// printk("%s: 5\n", __func__);
// 	emmaprp_issue_dma(curr_ctx, YUV420_DMA_V_ODD);
// printk("%s: 6\n", __func__);
// 	emmaprp_issue_dma(curr_ctx, YUV420_DMA_V_EVEN);
// printk("%s: 7\n", __func__);
// 	dma_async_issue_pending(chan);
// printk("%s: 8\n", __func__);
// }

static void emmaprp_device_run(void *priv)
{
	struct emmaprp_ctx *ctx = priv;
	struct emmaprp_dev *pcdev = ctx->dev;
	struct dma_chan *chan = pcdev->dma_chan;

	atomic_set(&ctx->dev->busy, 1);

	//printk("%s: pcdev = %p\n", __func__, pcdev);
	//printk("%s: pcdev->m2m_dev = %p\n", __func__, pcdev->m2m_dev);

// 	my_tasklet_data = (unsigned long)pcdev;
	//printk("%s: my_tasklet_data = %p\n", __func__, my_tasklet_data);
// 	tasklet_schedule(&my_tasklet);
// 	emmaprp_dma_task((unsigned long) pcdev);
// printk("%s: 1 ctx = %p\n", __func__, ctx);
	dprintk(ctx->dev, "%s: init\n", __func__);

	emmaprp_issue_dma(ctx, YUV420_DMA_Y_ODD);
// printk("%s: 2 ctx = %p\n", __func__, ctx);
	emmaprp_issue_dma(ctx, YUV420_DMA_Y_EVEN);
// printk("%s: 3 ctx = %p\n", __func__, ctx);
	emmaprp_issue_dma(ctx, YUV420_DMA_U_ODD);
// printk("%s: 4 ctx = %p\n", __func__, ctx);
	emmaprp_issue_dma(ctx, YUV420_DMA_U_EVEN);
// printk("%s: 5 ctx = %p\n", __func__, ctx);
	emmaprp_issue_dma(ctx, YUV420_DMA_V_ODD);
// printk("%s: 6 ctx = %p\n", __func__, ctx);
	emmaprp_issue_dma(ctx, YUV420_DMA_V_EVEN);
// printk("%s: 7 ctx = %p\n", __func__, ctx);
	dma_async_issue_pending(chan);
// printk("%s: 8 ctx = %p\n", __func__, ctx);
	dprintk(ctx->dev, "%s: exit\n", __func__);
}

/*
 * video ioctls
 */
static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strncpy(cap->driver, MEM2MEM_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, MEM2MEM_NAME, sizeof(cap->card) - 1);
	cap->bus_info[0] = 0;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT
			  | V4L2_CAP_STREAMING;

	return 0;
}

static int enum_fmt(struct v4l2_fmtdesc *f, u32 type)
{
	int i, num;
	struct emmaprp_fmt *fmt;

	num = 0;

	for (i = 0; i < NUM_FORMATS; ++i) {
		if (formats[i].types & type) {
			/* index-th format of type type found ? */
			if (num == f->index)
				break;
			/* Correct type but haven't reached our index yet,
			 * just increment per-type index */
			++num;
		}
	}

	if (i < NUM_FORMATS) {
		/* Format found */
		fmt = &formats[i];
		strncpy(f->description, fmt->name, sizeof(f->description) - 1);
		f->pixelformat = fmt->fourcc;
		return 0;
	}

	/* Format not found */
	return -EINVAL;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, MEM2MEM_CAPTURE);
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, MEM2MEM_OUTPUT);
}

static int vidioc_g_fmt(struct emmaprp_ctx *ctx, struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct emmaprp_q_data *q_data;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(f->type);

	f->fmt.pix.width	= q_data->width;
	f->fmt.pix.height	= q_data->height;
	f->fmt.pix.field	= q_data->fmt->field;
	f->fmt.pix.pixelformat	= q_data->fmt->fourcc;
	f->fmt.pix.bytesperline = q_data->width * 3 / 2;
	f->fmt.pix.sizeimage	= q_data->sizeimage;

	return 0;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return vidioc_g_fmt(priv, f);
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return vidioc_g_fmt(priv, f);
}

static int vidioc_try_fmt(struct v4l2_format *f, struct emmaprp_fmt *fmt)
{
	struct emmaprp_q_data *q_data = get_q_data(f->type);
	enum v4l2_field field = q_data->fmt->field;

	/* V4L2 specification suggests the driver corrects the format struct
	 * if any of the dimensions is unsupported */
	f->fmt.pix.field = field;

	if (f->fmt.pix.height < MIN_H)
		f->fmt.pix.height = MIN_H;
	else if (f->fmt.pix.height > MAX_H)
		f->fmt.pix.height = MAX_H;

	if (f->fmt.pix.width < MIN_W)
		f->fmt.pix.width = MIN_W;
	else if (f->fmt.pix.width > MAX_W)
		f->fmt.pix.width = MAX_W;

	f->fmt.pix.height &= ~H_ALIGN_MASK;
	f->fmt.pix.width &= ~W_ALIGN_MASK_YUV420;
	f->fmt.pix.bytesperline = f->fmt.pix.width * 3 / 2;

	f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.bytesperline;

	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct emmaprp_fmt *fmt;
	struct emmaprp_ctx *ctx = priv;

	fmt = find_format(f);
	if (!fmt || !(fmt->types & MEM2MEM_CAPTURE)) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}
	return vidioc_try_fmt(f, fmt);
}

static int vidioc_try_fmt_vid_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct emmaprp_fmt *fmt;
	struct emmaprp_ctx *ctx = priv;

	fmt = find_format(f);
	if (!fmt || !(fmt->types & MEM2MEM_OUTPUT)) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_s_fmt(struct emmaprp_ctx *ctx, struct v4l2_format *f)
{
	struct emmaprp_q_data *q_data;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(f->type);
	if (!q_data)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		v4l2_err(&ctx->dev->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	q_data->fmt		= find_format(f);
	q_data->width		= f->fmt.pix.width;
	q_data->height		= f->fmt.pix.height;
	q_data->sizeimage = q_data->width * q_data->height * 3 / 2;

	dprintk(ctx->dev,
		"Setting format for type %d, wxh: %dx%d, fmt: %d\n",
		f->type, q_data->width, q_data->height, q_data->fmt->fourcc);

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;

	ret = vidioc_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;
	return vidioc_s_fmt(priv, f);
}

static int vidioc_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;

	ret = vidioc_try_fmt_vid_out(file, priv, f);
	if (ret)
		return ret;

	return vidioc_s_fmt(priv, f);
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *reqbufs)
{
	struct emmaprp_ctx *ctx = priv;

	return v4l2_m2m_reqbufs(file, ctx->m2m_ctx, reqbufs);
}

// int v4l2_m2m_querybuf_custom(struct file *file, struct v4l2_m2m_ctx *m2m_ctx,
// 		      struct v4l2_buffer *buf)
// {
// 	struct vb2_queue *vq;
// 	int ret = 0;
// 	unsigned int i;
// 	struct vb2_buffer *vb;
// 
// 	vq = v4l2_m2m_get_vq(m2m_ctx, buf->type);
// 	ret = vb2_querybuf(vq, buf);
// 
// 	vb = vq->bufs[buf->index];
// 	buf->m.offset = vb2_dma_contig_plane_dma_addr(vb, 0);
// 	if (!buf->m.offset)
// 		return -ENOMEM;
// 	vb->v4l2_planes[0].m.mem_offset = buf->m.offset;
// 
// 	/* Adjust MMAP memory offsets for the CAPTURE queue */
// 	if (buf->memory == V4L2_MEMORY_MMAP && !V4L2_TYPE_IS_OUTPUT(vq->type)) {
// 		if (V4L2_TYPE_IS_MULTIPLANAR(vq->type)) {
// 			for (i = 0; i < buf->length; ++i)
// 				buf->m.planes[i].m.mem_offset
// 					|= DST_QUEUE_OFF_BASE;
// 		} else {
// 			buf->m.offset |= DST_QUEUE_OFF_BASE;
// 		}
// 	}
// 
// 	return ret;
// }

static int vidioc_querybuf(struct file *file, void *priv,
			   struct v4l2_buffer *buf)
{
	struct emmaprp_ctx *ctx = priv;

	return v4l2_m2m_querybuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct emmaprp_ctx *ctx = priv;

	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct emmaprp_ctx *ctx = priv;

	dprintk(ctx->dev, "dqbuf\n");
	return v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	struct emmaprp_ctx *ctx = priv;

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int vidioc_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct emmaprp_ctx *ctx = priv;

	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}

static const struct v4l2_ioctl_ops emmaprp_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,

	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	= vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	= vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	= vidioc_s_fmt_vid_cap,

	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out	= vidioc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out	= vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out	= vidioc_s_fmt_vid_out,

	.vidioc_reqbufs		= vidioc_reqbufs,
	.vidioc_querybuf	= vidioc_querybuf,

	.vidioc_qbuf		= vidioc_qbuf,
	.vidioc_dqbuf		= vidioc_dqbuf,

	.vidioc_streamon	= vidioc_streamon,
	.vidioc_streamoff	= vidioc_streamoff,
};


/*
 * Queue operations
 */
struct vb2_dc_conf {
	struct device           *dev;
};

static int emmaprp_queue_setup(struct vb2_queue *vq,
				const struct v4l2_format *fmt,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct emmaprp_ctx *ctx = vb2_get_drv_priv(vq);
	struct emmaprp_q_data *q_data;
	unsigned int size, count = *nbuffers;

	q_data = get_q_data(vq->type);

	size = q_data->width * q_data->height * 3 / 2;

	while (size * count > MEM2MEM_VID_MEM_LIMIT)
		(count)--;

	*nplanes = 1;
	*nbuffers = count;
	sizes[0] = size;

	alloc_ctxs[0] = ctx->dev->alloc_ctx;

	dprintk(ctx->dev, "get %d buffer(s) of size %d each.\n", count, size);

	return 0;
}

static int emmaprp_buf_prepare(struct vb2_buffer *vb)
{
	struct emmaprp_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct emmaprp_q_data *q_data;

	dprintk(ctx->dev, "type: %d\n", vb->vb2_queue->type);

	q_data = get_q_data(vb->vb2_queue->type);

	if (vb2_plane_size(vb, 0) < q_data->sizeimage) {
		dprintk(ctx->dev, "%s data will not fit into plane"
				  "(%lu < %lu)\n", __func__,
				  vb2_plane_size(vb, 0),
				  (long)q_data->sizeimage);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, q_data->sizeimage);

	return 0;
}

static void emmaprp_buf_queue(struct vb2_buffer *vb)
{
	struct emmaprp_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	v4l2_m2m_buf_queue(ctx->m2m_ctx, vb);
}

static struct vb2_ops emmaprp_qops = {
	.queue_setup	 = emmaprp_queue_setup,
	.buf_prepare	 = emmaprp_buf_prepare,
	.buf_queue	 = emmaprp_buf_queue,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct emmaprp_ctx *ctx = priv;
	int ret;

	memset(src_vq, 0, sizeof(*src_vq));
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &emmaprp_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	memset(dst_vq, 0, sizeof(*dst_vq));
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &emmaprp_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;

	return vb2_queue_init(dst_vq);
}

/*
 * File operations
 */
static int emmaprp_open(struct file *file)
{
	struct emmaprp_dev *pcdev = video_drvdata(file);
	struct emmaprp_ctx *ctx = NULL;

	ctx = kzalloc(sizeof *ctx, GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	file->private_data = ctx;
	ctx->dev = pcdev;

	ctx->m2m_ctx = v4l2_m2m_ctx_init(pcdev->m2m_dev, ctx, &queue_init);

	if (IS_ERR(ctx->m2m_ctx)) {
		int ret = PTR_ERR(ctx->m2m_ctx);

		kfree(ctx);
		return ret;
	}

	dprintk(pcdev, "Created instance %p, m2m_ctx: %p\n", ctx, ctx->m2m_ctx);

	return 0;
}

static int emmaprp_release(struct file *file)
{
	struct emmaprp_dev *pcdev = video_drvdata(file);
	struct emmaprp_ctx *ctx = file->private_data;

	dprintk(pcdev, "Releasing instance %p\n", ctx);

	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	kfree(ctx);

	return 0;
}

static unsigned int emmaprp_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct emmaprp_ctx *ctx = file->private_data;

	return v4l2_m2m_poll(file, ctx->m2m_ctx, wait);
}

// int v4l2_m2m_mmap_custom(struct file *file, struct v4l2_m2m_ctx *m2m_ctx,
// 			 struct vm_area_struct *vma)
// {
// 	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
// 	struct vb2_queue *vq;
// 
// 	if (!(offset & DST_QUEUE_OFF_BASE)) {
// 		vq = v4l2_m2m_get_src_vq(m2m_ctx);
// 	} else {
// 		vq = v4l2_m2m_get_dst_vq(m2m_ctx);
// 		vma->vm_pgoff &= ~(DST_QUEUE_OFF_BASE >> PAGE_SHIFT);
// 	}
// 
// 	return vb2_mmap(vq, vma);
// }

static int emmaprp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct emmaprp_ctx *ctx = file->private_data;

	return v4l2_m2m_mmap(file, ctx->m2m_ctx, vma);
}

static const struct v4l2_file_operations emmaprp_fops = {
	.owner		= THIS_MODULE,
	.open		= emmaprp_open,
	.release	= emmaprp_release,
	.poll		= emmaprp_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= emmaprp_mmap,
};

static struct video_device emmaprp_videodev = {
	.name		= MEM2MEM_NAME,
	.fops		= &emmaprp_fops,
	.ioctl_ops	= &emmaprp_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
};

static struct v4l2_m2m_ops m2m_ops = {
	.device_run	= emmaprp_device_run,
	.job_ready	= emmaprp_job_ready,
	.job_abort	= emmaprp_job_abort,
	.lock		= emmaprp_lock,
	.unlock		= emmaprp_unlock,
};

static int emmaprp_probe(struct platform_device *pdev)
{
	struct emmaprp_dev *pcdev;
	struct video_device *vfd;
	dma_cap_mask_t mask;
	int ret = 0;

	pcdev = kzalloc(sizeof *pcdev, GFP_KERNEL);
	if (!pcdev)
		return -ENOMEM;

	spin_lock_init(&pcdev->irqlock);

	dma_cap_zero(mask);
	dma_cap_set(DMA_INTERLEAVE, mask);
	pcdev->dma_chan = dma_request_channel(mask, NULL, pcdev);
	if (!pcdev->dma_chan)
		goto free_dev;
	//printk("%s: dma channel: %d\n", __func__, pcdev->dma_chan->chan_id);
	if (!dma_has_cap(DMA_INTERLEAVE, pcdev->dma_chan->device->cap_mask)) {
		v4l2_err(&pcdev->v4l2_dev, "DMA does not support INTERLEAVE\n");
		goto rel_dma;
	}

	ret = v4l2_device_register(&pdev->dev, &pcdev->v4l2_dev);
	if (ret)
		goto rel_dma;

	atomic_set(&pcdev->busy, 0);
	mutex_init(&pcdev->dev_mutex);

	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&pcdev->v4l2_dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto unreg_dev;
	}

	*vfd = emmaprp_videodev;
      /* Locking in file operations other than ioctl should be done
          by the driver, not the V4L2 core.
          This driver needs auditing so that this flag can be removed. */
       set_bit(V4L2_FL_LOCK_ALL_FOPS, &vfd->flags);

	vfd->lock = &pcdev->dev_mutex;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
	if (ret) {
		v4l2_err(&pcdev->v4l2_dev, "Failed to register video device\n");
		goto rel_vdev;
	}

	video_set_drvdata(vfd, pcdev);
	snprintf(vfd->name, sizeof(vfd->name), "%s", emmaprp_videodev.name);
	pcdev->vfd = vfd;
	v4l2_info(&pcdev->v4l2_dev, MEM2MEM_TEST_MODULE_NAME
			" Device registered as /dev/video%d\n", vfd->num);

	platform_set_drvdata(pdev, pcdev);

	pcdev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(pcdev->alloc_ctx)) {
		v4l2_err(&pcdev->v4l2_dev, "Failed to alloc vb2 context\n");
		ret = PTR_ERR(pcdev->alloc_ctx);
		goto err_ctx;
	}

	pcdev->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(pcdev->m2m_dev)) {
		v4l2_err(&pcdev->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(pcdev->m2m_dev);
		goto err_m2m;
	}

	q_data[V4L2_M2M_SRC].fmt = &formats[1];
	q_data[V4L2_M2M_DST].fmt = &formats[0];

	return 0;

	v4l2_m2m_release(pcdev->m2m_dev);
err_m2m:
	video_unregister_device(pcdev->vfd);
err_ctx:
	vb2_dma_contig_cleanup_ctx(pcdev->alloc_ctx);
rel_vdev:
	video_device_release(vfd);
unreg_dev:
	v4l2_device_unregister(&pcdev->v4l2_dev);
rel_dma:
	dma_release_channel(pcdev->dma_chan);
free_dev:
	kfree(pcdev);

	return ret;
}

static int emmaprp_remove(struct platform_device *pdev)
{
	struct emmaprp_dev *pcdev =
		(struct emmaprp_dev *)platform_get_drvdata(pdev);

	v4l2_info(&pcdev->v4l2_dev, "Removing " MEM2MEM_TEST_MODULE_NAME);
	v4l2_m2m_release(pcdev->m2m_dev);
	video_unregister_device(pcdev->vfd);
	v4l2_device_unregister(&pcdev->v4l2_dev);
	vb2_dma_contig_cleanup_ctx(pcdev->alloc_ctx);
	dma_release_channel(pcdev->dma_chan);
	kfree(pcdev);

	return 0;
}

static struct platform_driver emmaprp_pdrv = {
	.probe		= emmaprp_probe,
	.remove		= emmaprp_remove,
	.driver		= {
		.name	= MEM2MEM_NAME,
		.owner	= THIS_MODULE,
	},
};

static void __exit emmaprp_exit(void)
{
	platform_driver_unregister(&emmaprp_pdrv);
}

static int __init emmaprp_init(void)
{
	return platform_driver_register(&emmaprp_pdrv);
}

module_init(emmaprp_init);
module_exit(emmaprp_exit);

