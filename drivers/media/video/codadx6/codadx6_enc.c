/*
 * CodaDx6 multi-standard codec IP
 *
 * Copyright (C) 2012 Vista Silicon S.L.
 *   Javier Martin, <javier.martin@vista-silicon.com>
 *   Xavier Duret <xavier@vista-silicon.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>

#include "codadx6_common.h"
#include "codadx6_enc.h"

#define CODADX6_ENC_OUTPUT_BUFS		4
#define CODADX6_ENC_CAPTURE_BUFS	2

#define CODADX6_ENC_MAX_WIDTH		720
#define CODADX6_ENC_MAX_HEIGHT		576
#define CODADX6_ENC_MAX_FRAME_SIZE	0x90000

/*
 * V4L2 ioctl() operations.
 */
static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	/* TODO */
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	/* TODO */
	return 0;
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	/* TODO */
	return 0;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	/* TODO */
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	/* TODO */
	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	/* TODO */
	return 0;
}

static int vidioc_try_fmt_vid_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	/* TODO */
	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	/* TODO */
	return 0;
}

static int vidioc_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	/* TODO */
	return 0;
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *reqbufs)
{
	/* TODO */
	return 0;
}

static int vidioc_querybuf(struct file *file, void *priv,
			   struct v4l2_buffer *buf)
{
	/* TODO */
	return 0;
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	/* TODO */
	return 0;
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	/* TODO */
	return 0;
}

static int vidioc_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	/* TODO */
	return 0;
}

static int vidioc_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	/* TODO */
	return 0;
}

static const struct v4l2_ioctl_ops codadx6_enc_ioctl_ops = {
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

const struct v4l2_ioctl_ops *get_enc_v4l2_ioctl_ops(void)
{
	return &codadx6_enc_ioctl_ops;
}

/*
 * Mem-to-mem operations.
 */
static void codadx6_device_run(void *priv)
{
	/* TODO */
	return;
}

static void codadx6_job_abort(void *priv)
{
	/* TODO */
	return;
}

static void codadx6_lock(void *priv)
{
	/* TODO */
	return;
}

static void codadx6_unlock(void *priv)
{
	/* TODO */
	return;
}

static struct v4l2_m2m_ops codadx6_enc_m2m_ops = {
	.device_run	= codadx6_device_run,
	.job_abort	= codadx6_job_abort,
	.lock		= codadx6_lock,
	.unlock		= codadx6_unlock,
};

struct v4l2_m2m_ops *get_enc_m2m_ops(void)
{
	return &codadx6_enc_m2m_ops;
}

void set_enc_default_params(struct codadx6_ctx *ctx) {
	ctx->enc_params.h264_intra_qp = 1;
	ctx->enc_params.h264_inter_qp = 1;
	ctx->enc_params.mpeg4_intra_qp = 1;
	ctx->enc_params.mpeg4_inter_qp = 1;
	ctx->enc_params.codec_mode = CODADX6_MODE_INVALID;
	ctx->enc_params.slice_mode = 1;
	ctx->enc_params.slice_max_mb = 1;
}

/*
 * Queue operations
 */
static int codadx6_enc_queue_setup(struct vb2_queue *vq,
				const struct v4l2_format *fmt,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct codadx6_ctx *ctx = vb2_get_drv_priv(vq);
	unsigned int size;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		*nbuffers = CODADX6_ENC_OUTPUT_BUFS;
		if (fmt)
			size = fmt->fmt.pix.width *
				fmt->fmt.pix.height * 3 / 2;
		else
			size = CODADX6_ENC_MAX_WIDTH *
				CODADX6_ENC_MAX_HEIGHT * 3 / 2;
	} else {
		*nbuffers = CODADX6_ENC_CAPTURE_BUFS;
		size = CODADX6_ENC_MAX_FRAME_SIZE;
	}
	
	*nplanes = 1;
	sizes[0] = size;

	alloc_ctxs[0] = ctx->dev->alloc_enc_ctx;

	v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev,
		 "get %d buffer(s) of size %d each.\n", *nbuffers, size);

	return 0;
}

static int codadx6_enc_buf_prepare(struct vb2_buffer *vb)
{
	/* TODO */
	return 0;
}

static void codadx6_enc_buf_queue(struct vb2_buffer *vb)
{
	/* TODO */
}

static struct vb2_ops codadx6_enc_qops = {
	.queue_setup	 = codadx6_enc_queue_setup,
	.buf_prepare	 = codadx6_enc_buf_prepare,
	.buf_queue	 = codadx6_enc_buf_queue,
};

struct vb2_ops *get_enc_qops(void)
{
	return &codadx6_enc_qops;
}
