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

#define MIN_W 176
#define MIN_H 144
#define MAX_W 720
#define MAX_H 576

#define S_ALIGN		1 /* multiple of 2 */
#define W_ALIGN		1 /* multiple of 2 */
#define H_ALIGN		1 /* multiple of 2 */

static struct codadx6_fmt formats[] = {
        {
                .name = "YUV 4:2:0 Planar",
                .fourcc = V4L2_PIX_FMT_YUV420,
//                 .codec_mode = S5P_FIMV_CODEC_NONE,
                .type = CODADX6_FMT_RAW,
        },
        {
                .name = "H264 Encoded Stream",
                .fourcc = V4L2_PIX_FMT_H264,
//                 .codec_mode = S5P_FIMV_CODEC_H264_ENC,
                .type = CODADX6_FMT_ENC,
        },
        {
                .name = "MPEG4 Encoded Stream",
                .fourcc = V4L2_PIX_FMT_MPEG4,
//                 .codec_mode = S5P_FIMV_CODEC_MPEG4_ENC,
                .type = CODADX6_FMT_ENC,
        },
};

#define NUM_FORMATS ARRAY_SIZE(formats)

static struct codadx6_fmt *find_format(struct v4l2_format *f)
{
	struct codadx6_fmt *fmt;
	unsigned int k;

	for (k = 0; k < NUM_FORMATS; k++) {
		fmt = &formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			break;
	}

	if (k == NUM_FORMATS)
		return NULL;

	return &formats[k];
}

/*
 * V4L2 ioctl() operations.
 */
static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strncpy(cap->driver, CODADX6_ENC_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, CODADX6_ENC_NAME, sizeof(cap->card) - 1);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT
			  | V4L2_CAP_STREAMING;

	return 0;
}

static int enum_fmt(struct v4l2_fmtdesc *f, enum codadx6_fmt_type type)
{
	struct codadx6_fmt *fmt;
	int i, num = 0;
	
	for (i = 0; i < NUM_FORMATS; i++) {
		if (formats[i].type == type) {
			if (num == f->index)
				break;
			++num;
		}
	}

	if (i < NUM_FORMATS) {
		fmt = &formats[i];
		strlcpy(f->description, fmt->name, sizeof(f->description) - 1);
		f->pixelformat = fmt->fourcc;
		return 0;
	}

	/* Format not found */
	return -EINVAL;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, CODADX6_FMT_ENC);
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, CODADX6_FMT_RAW);
}

static int vidioc_g_fmt(struct codadx6_ctx *ctx, struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct codadx6_q_data *q_data;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);

	f->fmt.pix.field	= V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat	= q_data->fmt->fourcc;
	if (f->fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420) {
		f->fmt.pix.width	= q_data->width;
		f->fmt.pix.height	= q_data->height;
		f->fmt.pix.bytesperline = q_data->width * 3 / 2;
	} else { /* encoded formats h.264/mpeg4 */
		f->fmt.pix.width	= 0;
		f->fmt.pix.height	= 0;
		f->fmt.pix.bytesperline = q_data->sizeimage;
	}
	f->fmt.pix.sizeimage	= q_data->sizeimage;

	return 0;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return vidioc_g_fmt(fh_to_ctx(priv), f);
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return vidioc_g_fmt(fh_to_ctx(priv), f);
}

static int vidioc_try_fmt(struct v4l2_format *f)
{
	enum v4l2_field field;

	if (!find_format(f))
		return -EINVAL;

	field = f->fmt.pix.field;
	if (field == V4L2_FIELD_ANY)
		field = V4L2_FIELD_NONE;
	else if (V4L2_FIELD_NONE != field)
		return -EINVAL;

	/* V4L2 specification suggests the driver corrects the format struct
	 * if any of the dimensions is unsupported */
	f->fmt.pix.field = field;

	if (f->fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420) {
		v4l_bound_align_image(&f->fmt.pix.width, MIN_W, MAX_W,
				      W_ALIGN, &f->fmt.pix.height,
				      MIN_H, MAX_H, H_ALIGN, S_ALIGN);
		f->fmt.pix.bytesperline = f->fmt.pix.width * 3 / 2;
		f->fmt.pix.sizeimage = f->fmt.pix.height *
					f->fmt.pix.bytesperline;
	} else { /*encoded formats h.264/mpeg4 */
		f->fmt.pix.bytesperline = CODADX6_ENC_MAX_FRAME_SIZE;
		f->fmt.pix.sizeimage = CODADX6_ENC_MAX_FRAME_SIZE;
	}

	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct codadx6_fmt *fmt;
	struct codadx6_ctx *ctx = fh_to_ctx(priv);

	fmt = find_format(f);
	if (!fmt || !(fmt->type == CODADX6_FMT_ENC)) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	return vidioc_try_fmt(f);
}

static int vidioc_try_fmt_vid_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct codadx6_fmt *fmt;
	struct codadx6_ctx *ctx = fh_to_ctx(priv);

	fmt = find_format(f);
	if (!fmt || !(fmt->type == CODADX6_FMT_RAW)) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	return vidioc_try_fmt(f);
}

static int vidioc_s_fmt(struct codadx6_ctx *ctx, struct v4l2_format *f)
{
	struct codadx6_q_data *q_data;
	struct vb2_queue *vq;
	int ret;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);
	if (!q_data)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		v4l2_err(&ctx->dev->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	ret = vidioc_try_fmt(f);
	if (ret)
		return ret;

	q_data->fmt		= find_format(f);
	if (q_data->fmt->fourcc == V4L2_PIX_FMT_YUV420) {
		q_data->width		= f->fmt.pix.width;
		q_data->height		= f->fmt.pix.height;
		q_data->sizeimage = q_data->width * q_data->height * 3 / 2;
	} else { /* encoded format h.264/mpeg-4 */
		q_data->sizeimage = CODADX6_ENC_MAX_FRAME_SIZE;
	}

	v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev,
		"Setting format for type %d, wxh: %dx%d, fmt: %d\n",
		f->type, q_data->width, q_data->height, q_data->fmt->fourcc);

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;

	ret = vidioc_try_fmt_vid_cap(file, fh_to_ctx(priv), f);
	if (ret)
		return ret;

	return vidioc_s_fmt(fh_to_ctx(priv), f);
}

static int vidioc_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;

	ret = vidioc_try_fmt_vid_out(file, fh_to_ctx(priv), f);
	if (ret)
		return ret;

	return vidioc_s_fmt(fh_to_ctx(priv), f);
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *reqbufs)
{
	struct codadx6_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_reqbufs(file, ctx->m2m_ctx, reqbufs);
}

static int vidioc_querybuf(struct file *file, void *priv,
			   struct v4l2_buffer *buf)
{
	struct codadx6_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_querybuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct codadx6_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct codadx6_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	struct codadx6_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int vidioc_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct codadx6_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}

int vidioc_s_parm(struct file *file, void *priv, struct v4l2_streamparm *a)
{
	struct codadx6_ctx *ctx = fh_to_ctx(priv);

	if (a->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		if (a->parm.output.timeperframe.numerator != 1) {
			v4l2_err(&ctx->dev->v4l2_dev,
				 "FPS numerator must be 1\n");
			return -EINVAL;
		}
		ctx->enc_params.framerate =
					a->parm.output.timeperframe.denominator;
	} else {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Setting FPS is only possible for the output queue\n");
		return -EINVAL;
	}
	return 0;
}

int vidioc_g_parm(struct file *file, void *priv, struct v4l2_streamparm *a)
{
	struct codadx6_ctx *ctx = fh_to_ctx(priv);

	if (a->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		a->parm.output.timeperframe.denominator =
					ctx->enc_params.framerate;
		a->parm.output.timeperframe.numerator = 1;
	} else {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Getting FPS is only possible for the output queue\n");
		return -EINVAL;
	}
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

	.vidioc_s_parm		= vidioc_s_parm,
	.vidioc_g_parm		= vidioc_g_parm,
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
	struct codadx6_ctx *ctx = fh_to_ctx(priv);
	struct codadx6_dev *pcdev = ctx->dev;
	mutex_lock(&pcdev->dev_mutex);
}

static void codadx6_unlock(void *priv)
{
	struct codadx6_ctx *ctx = fh_to_ctx(priv);
	struct codadx6_dev *pcdev = ctx->dev;
	mutex_unlock(&pcdev->dev_mutex);
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
	ctx->enc_params.codec_mode = CODADX6_MODE_INVALID;

	/* Default formats for output and input queues */
	ctx->q_data[V4L2_M2M_SRC].fmt = &formats[0];
	ctx->q_data[V4L2_M2M_DST].fmt = &formats[1];
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
	struct codadx6_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct codadx6_q_data *q_data;

	v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev, "type: %d\n",
		 vb->vb2_queue->type);

	q_data = get_q_data(ctx, vb->vb2_queue->type);

	if (vb2_plane_size(vb, 0) < q_data->sizeimage) {
		v4l2_warn(&ctx->dev->v4l2_dev, "%s data will not fit into"
			"plane (%lu < %lu)\n", __func__, vb2_plane_size(vb, 0),
			(long)q_data->sizeimage);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, q_data->sizeimage);

	return 0;
}

static void codadx6_enc_buf_queue(struct vb2_buffer *vb)
{
	struct codadx6_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	v4l2_m2m_buf_queue(ctx->m2m_ctx, vb);
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

static int codadx6_enc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct codadx6_ctx *ctx =
			container_of(ctrl->handler, struct codadx6_ctx, ctrls);
	
	v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev,
		 "s_ctrl: id = %d, val = %d\n", ctrl->id, ctrl->val);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		ctx->bitrate = ctrl->val / 1000;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		ctx->enc_params.gop_size = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		ctx->enc_params.h264_intra_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
		ctx->enc_params.h264_inter_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_I_FRAME_QP:
		ctx->enc_params.mpeg4_intra_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_P_FRAME_QP:
		ctx->enc_params.mpeg4_inter_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE:
		ctx->enc_params.slice_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB:
		ctx->enc_params.slice_max_mb = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		break;
	default:
		v4l2_err(&ctx->dev->v4l2_dev,
			"Invalid control, id=%d, val=%d\n",
			ctrl->id, ctrl->val);
		return -EINVAL;
	}
	
	return 0;
}

static struct v4l2_ctrl_ops codadx6_enc_ctrl_ops = {
	.s_ctrl = codadx6_enc_s_ctrl,
};

int codadx6_enc_ctrls_setup(struct codadx6_ctx *ctx)
{
	v4l2_ctrl_handler_init(&ctx->ctrls, 9);

	v4l2_ctrl_new_std(&ctx->ctrls, &codadx6_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_BITRATE, 0, 32767000, 1, 0);
	v4l2_ctrl_new_std(&ctx->ctrls, &codadx6_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_GOP_SIZE, 1, 60, 1, 16);
	v4l2_ctrl_new_std(&ctx->ctrls, &codadx6_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP, 1, 51, 1, 25);
	v4l2_ctrl_new_std(&ctx->ctrls, &codadx6_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP, 1, 51, 1, 25);
	v4l2_ctrl_new_std(&ctx->ctrls, &codadx6_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_MPEG4_I_FRAME_QP, 1, 31, 1, 2);
	v4l2_ctrl_new_std(&ctx->ctrls, &codadx6_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_MPEG4_P_FRAME_QP, 1, 31, 1, 2);
	v4l2_ctrl_new_std_menu(&ctx->ctrls, &codadx6_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
		V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB, 0,
		V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB);
	v4l2_ctrl_new_std(&ctx->ctrls, &codadx6_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB, 1, 0x3fffffff, 1, 1);
	v4l2_ctrl_new_std_menu(&ctx->ctrls, &codadx6_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_HEADER_MODE,
		V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
		(1 << V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE),
		V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME);

	return v4l2_ctrl_handler_setup(&ctx->ctrls);
}
