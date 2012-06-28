/*
 * linux/drivers/media/video/coda/coda_common.h
 *
 * Copyright (C) 2012 Vista Silicon SL
 *    Javier Martin <javier.martin@vista-silicon.com>
 *    Xavier Duret
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _CODA_COMMON_H_
#define _CODA_COMMON_H_

#include <media/v4l2-device.h>
#include <linux/io.h>
#include "coda_regs.h"

extern int coda_debug;

enum {
	V4L2_M2M_SRC = 0,
	V4L2_M2M_DST = 1,
};

enum coda_fmt_type {
	CODA_FMT_ENC,
	CODA_FMT_RAW,
};

enum coda_inst_type {
	CODA_INST_INVALID,
	CODA_INST_ENCODER,
};

enum coda_node_type {
	CODA_NODE_INVALID = -1,
	CODA_NODE_ENCODER = 0,
};

struct coda_fmt {
	char *name;
	u32 fourcc;
	enum coda_fmt_type type;
};

/* Per-queue, driver-specific private data */
struct coda_q_data {
	unsigned int		width;
	unsigned int		height;
	unsigned int		sizeimage;
	struct coda_fmt	*fmt;
};

struct coda_aux_buf {
	void			*vaddr;
	dma_addr_t		paddr;
};

struct coda_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	*vfd_enc;
	struct platform_device	*plat_dev;

	void __iomem		*regs_base;
	struct clk		*clk;
	int			irq;

	struct coda_aux_buf	enc_codebuf;
	struct coda_aux_buf	enc_workbuf;
	struct coda_aux_buf	enc_parabuf;

	spinlock_t		irqlock;
	struct mutex		dev_mutex;
	struct v4l2_m2m_dev	*m2m_enc_dev;
	struct vb2_alloc_ctx	*alloc_enc_ctx;
};

struct coda_enc_params {
	u8			h264_intra_qp;
	u8			h264_inter_qp;
	u8			mpeg4_intra_qp;
	u8			mpeg4_inter_qp;
	u8			gop_size;
	int			codec_mode;
	enum v4l2_mpeg_video_multi_slice_mode slice_mode;
	u32			framerate;
	u16			bitrate;
	u32			slice_max_mb;
};

#define CODA_ENC_OUTPUT_BUFS	4
#define CODA_ENC_CAPTURE_BUFS	2

struct coda_ctx {
	struct coda_dev			*dev;
	int				aborting;
	int				rawstreamon;
	int				compstreamon;
	u32				isequence;
	struct coda_q_data		q_data[2];
	enum coda_inst_type		inst_type;
	struct coda_enc_params		enc_params;
	struct v4l2_m2m_ctx		*m2m_ctx;
	struct v4l2_ctrl_handler	ctrls;
	struct v4l2_fh			fh;
	struct vb2_buffer		*reference;
	int				gopcounter;
	char				vpu_header[3][64];
	int				vpu_header_size[3];
};

static inline void coda_write(struct coda_dev *dev, u32 data, u32 reg)
{
	v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
		 "%s: data=0x%x, reg=0x%x\n", __func__, data, reg);
	writel(data, dev->regs_base + reg);
}

static inline unsigned int coda_read(struct coda_dev *dev, u32 reg)
{
	u32 data;
	data = readl(dev->regs_base + reg);
	v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
		 "%s: data=0x%x, reg=0x%x\n", __func__, data, reg);
	return data;
}

static inline unsigned long coda_isbusy(struct coda_dev *dev)
{
	return coda_read(dev, CODA_REG_BIT_BUSY);
}

static inline int coda_is_initialized(struct coda_dev *dev)
{
	return (coda_read(dev, CODA_REG_BIT_CUR_PC) != 0);
}

static void coda_command_async(struct coda_dev *dev, int codec_mode,
				  int cmd)
{
	coda_write(dev, CODA_REG_BIT_BUSY_FLAG, CODA_REG_BIT_BUSY);
	/* TODO: 0 for the first instance of (encoder-decoder), 1 for the
	 * second one (except firmware which is always 0) */
	coda_write(dev, 0, CODA_REG_BIT_RUN_INDEX);
	coda_write(dev, codec_mode, CODA_REG_BIT_RUN_COD_STD);
	coda_write(dev, cmd, CODA_REG_BIT_RUN_COMMAND);
}

static int coda_command_sync(struct coda_dev *dev, int codec_mode,
				int cmd)
{
	unsigned int timeout = 100000;

	coda_command_async(dev, codec_mode, cmd);
	while (coda_isbusy(dev)) {
		if (timeout-- == 0)
			return -ETIMEDOUT;
	};
	return 0;
}

struct coda_q_data *get_q_data(struct coda_ctx *ctx,
					 enum v4l2_buf_type type);

#define fh_to_ctx(__fh) container_of(__fh, struct coda_ctx, fh)

#endif
