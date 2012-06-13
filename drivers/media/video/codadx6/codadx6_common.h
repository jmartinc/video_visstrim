/*
 * linux/drivers/media/video/codadx6/codadx6_common.h
 *
 * Copyright (C) 2012 Vista Silicon SL
 *    Javier Martin <javier.martin@vista-silicon.com>
 *    Xavier Duret <xavier@vista-silicon.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _CODADX6_COMMON_H_
#define _CODADX6_COMMON_H_

#include <media/v4l2-device.h>
#include <asm/io.h>
#include "codadx6_regs.h"

extern int codadx6_debug;

enum {
	V4L2_M2M_SRC = 0,
	V4L2_M2M_DST = 1,
};

enum codadx6_fmt_type {
	CODADX6_FMT_ENC,
	CODADX6_FMT_RAW,
};

enum codadx6_inst_type {
	CODADX6_INST_INVALID,
	CODADX6_INST_ENCODER,
};

enum codadx6_node_type {
	CODADX6_NODE_INVALID = -1,
	CODADX6_NODE_ENCODER = 0,
};

struct codadx6_fmt {
	char *name;
	u32 fourcc;
// 	u32 codec_mode;
	enum codadx6_fmt_type type;
};

/* Per-queue, driver-specific private data */
struct codadx6_q_data {
	unsigned int		width;
	unsigned int		height;
	unsigned int		sizeimage;
	struct codadx6_fmt	*fmt;
};

struct codadx6_aux_buf {
	void			*vaddr;
	dma_addr_t		paddr;
};

struct codadx6_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	*vfd_enc;
	struct platform_device	*plat_dev;

	void __iomem		*regs_base;
	struct clk		*clk;
	int			irq;

	struct codadx6_aux_buf	enc_codebuf;
	struct codadx6_aux_buf	enc_workbuf;
	struct codadx6_aux_buf	enc_parabuf;

	spinlock_t		irqlock;
	struct mutex		dev_mutex;
	struct v4l2_m2m_dev	*m2m_enc_dev;
	struct vb2_alloc_ctx	*alloc_enc_ctx;
};

struct codadx6_enc_params {
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

struct framebuffer {
	u32	y;
	u32	cb;
	u32	cr;
};

#define CODADX6_ENC_OUTPUT_BUFS	4
#define CODADX6_ENC_CAPTURE_BUFS	2

/* TODO: some data of this structure can be removed */
struct codadx6_enc_runtime {
	/* old EncOpenParam vpuParams */
	unsigned int	pic_width;
	unsigned int	pic_height;
	u32		bitstream_buf;	/* Seems to be pointer to compressed buffer */
	u32		bitstream_buf_size;
	u32		bitstream_format; /* This is probably redundant (q_data->fmt->fourcc) */
	int		initial_delay;	/* This is fixed to 0 */
	int		vbv_buffer_size; /* This is fixed to 0 */
	int		enable_autoskip; /* This is fixed to 1 */
	int		intra_refresh; /* This is fixed to 0 */
	int		gamma; /* This is fixed to 4096 */
	int		maxqp; /* This is fixed to 0 */
	/* old EncInfo structure inside dev->encInfo (pEncInfo->openParam = *pop) */
	u32		stream_rd_ptr; /* This can be safely removed (use bitstream_buf instead) */
	u32		stream_buf_start_addr; /* This can be removed (use bitstream_buf instead) */
	u32		stream_buf_size; /* This can be removed (use bitstream_buf_size) instead */
	u32		stream_buf_end_addr; /* This can be just dropped */
	struct framebuffer frame_buf_pool[CODADX6_ENC_OUTPUT_BUFS]; /* Can be removed if we write to parabuf directly */
	int		initial_info_obtained; /* This probably can be removed (framework protects) */
	int		num_frame_buffers; /* This can be removed */
	int		stride; /* This can be removed later */
	/* headers */
	char		vpu_header[3][64];
	int		vpu_header_size[3];
};

struct codadx6_ctx {
	struct codadx6_dev		*dev;
// 	int			aborting;
	int				rawstreamon;
	int				compstreamon;
	u32				isequence;
	struct codadx6_q_data		q_data[2];
	enum codadx6_inst_type		inst_type;
	struct codadx6_enc_params	enc_params;
	struct codadx6_enc_runtime	runtime;
	struct v4l2_m2m_ctx		*m2m_ctx;
	struct v4l2_ctrl_handler	ctrls;
	struct v4l2_fh			fh;
};

static inline void codadx6_write(struct codadx6_dev *dev, u32 data, u32 reg)
{
	v4l2_dbg(1, codadx6_debug, &dev->v4l2_dev,
		 "%s: data=0x%x, reg=0x%x\n", __func__, data, reg);
	writel(data, dev->regs_base + reg);
}

static inline unsigned int codadx6_read(struct codadx6_dev *dev, u32 reg)
{
	u32 data;
	data = readl(dev->regs_base + reg);
	v4l2_dbg(1, codadx6_debug, &dev->v4l2_dev,
		 "%s: data=0x%x, reg=0x%x\n", __func__, data, reg);
	return data;
}

static inline unsigned long codadx6_isbusy(struct codadx6_dev *dev) {
	return readl(dev->regs_base + CODADX6_REG_BIT_BUSY);
}

static void codadx6_command_async(struct codadx6_dev *dev, int codec_mode,
				  int cmd)
{
	codadx6_write(dev, CODADX6_REG_BIT_BUSY_FLAG, CODADX6_REG_BIT_BUSY);
	/* TODO: 0 for the first instance of (encoder-decoder), 1 for the second one
	 *(except firmware which is always 0) */
	codadx6_write(dev, 0, CODADX6_REG_BIT_RUN_INDEX);
	codadx6_write(dev, codec_mode, CODADX6_REG_BIT_RUN_COD_STD);
	codadx6_write(dev, cmd, CODADX6_REG_BIT_RUN_COMMAND);
}

static int codadx6_command_sync(struct codadx6_dev *dev, int codec_mode,
				int cmd)
{
	unsigned int timeout = 100000;

	codadx6_command_async(dev, codec_mode, cmd);
	while (codadx6_isbusy(dev)) {
	if (timeout-- == 0)
		return -ETIMEDOUT;
	};
	return 0;
}

struct codadx6_q_data *get_q_data(struct codadx6_ctx *ctx,
					 enum v4l2_buf_type type);

#define fh_to_ctx(__fh) container_of(__fh, struct codadx6_ctx, fh)

#endif
