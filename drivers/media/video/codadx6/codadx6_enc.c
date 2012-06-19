/*
 * CodaDx6 multi-standard codec IP
 *
 * Copyright (C) 2012 Vista Silicon S.L.
 *    Javier Martin, <javier.martin@vista-silicon.com>
 *    Xavier Duret
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/irq.h>

#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "codadx6_common.h"
#include "codadx6_enc.h"

#define CODADX6_ENC_MAX_WIDTH		720
#define CODADX6_ENC_MAX_HEIGHT		576
#define CODADX6_ENC_MAX_FRAME_SIZE	0x90000
#define FMO_SLICE_SAVE_BUF_SIZE         (32)

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
                .type = CODADX6_FMT_RAW,
        },
        {
                .name = "H264 Encoded Stream",
                .fourcc = V4L2_PIX_FMT_H264,
                .type = CODADX6_FMT_ENC,
        },
        {
                .name = "MPEG4 Encoded Stream",
                .fourcc = V4L2_PIX_FMT_MPEG4,
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
	int ret;
	struct codadx6_ctx *ctx = fh_to_ctx(priv);
	
	ret = v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
	return ret;
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

int codadx6_enc_isr(struct codadx6_dev *dev)
{
	struct codadx6_ctx *ctx;
	struct vb2_buffer *src_buf, *dst_buf, *tmp_buf;

	ctx = v4l2_m2m_get_curr_priv(dev->m2m_enc_dev);
	if (ctx == NULL) {
		v4l2_err(&dev->v4l2_dev, "Instance released before the end of transaction\n");
		return IRQ_HANDLED;
	}

	if (ctx->aborting) {
		v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev,
			 "task has been aborted\n");
		return IRQ_HANDLED;
	}

	if (codadx6_isbusy(ctx->dev)) {
		v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev,
			 "coda is still busy!!!!\n");
		return IRQ_NONE;
	}

	src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);


	/* codadx6_encoder_get_results */
	{
	u32 tmp1, tmp2;

	codadx6_read(dev, CODADX6_RET_ENC_PIC_TYPE);
	tmp1 = codadx6_read(dev, CODADX6_CMD_ENC_PIC_BB_START);
	tmp2 = codadx6_read(dev, CODADX6_REG_BIT_WR_PTR_0);
	/* Calculate bytesused field */
	if (dst_buf->v4l2_buf.sequence == 0) {
		dst_buf->v4l2_planes[0].bytesused = (tmp2 - tmp1) + ctx->runtime.vpu_header_size[0] +
							ctx->runtime.vpu_header_size[1] +
							ctx->runtime.vpu_header_size[2];
	} else {
		dst_buf->v4l2_planes[0].bytesused = (tmp2 - tmp1);
	}

	v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev, "frame size = %u\n", tmp2-tmp1);
	codadx6_read(dev, CODADX6_RET_ENC_PIC_SLICE_NUM);
	codadx6_read(dev, CODADX6_RET_ENC_PIC_FLAG);
	}

	if (src_buf->v4l2_buf.flags & V4L2_BUF_FLAG_KEYFRAME) {
		dst_buf->v4l2_buf.flags |= V4L2_BUF_FLAG_KEYFRAME;
		dst_buf->v4l2_buf.flags &= ~V4L2_BUF_FLAG_PFRAME;
	} else {
		dst_buf->v4l2_buf.flags |= V4L2_BUF_FLAG_PFRAME;
		dst_buf->v4l2_buf.flags &= ~V4L2_BUF_FLAG_KEYFRAME;
	}

	/* Free previous reference picture if available */
	if (ctx->reference) {
		v4l2_m2m_buf_done(ctx->reference, VB2_BUF_STATE_DONE);
		ctx->reference = NULL;
	}

	/* 
	 * For the last frame of the gop we don't need to save
	 * a reference picture.
	 */
	v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
	tmp_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	if (ctx->gopcounter == 0) {
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
	} else {
		ctx->reference = tmp_buf;
	}

	v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);

	ctx->gopcounter--;
	if (ctx->gopcounter < 0)
		ctx->gopcounter = ctx->enc_params.gop_size - 1;

	v4l2_dbg(1, codadx6_debug, &dev->v4l2_dev,
		"job finished: encoding frame (%d) (%s)\n",
		dst_buf->v4l2_buf.sequence,
		(dst_buf->v4l2_buf.flags & V4L2_BUF_FLAG_KEYFRAME) ?
		"KEYFRAME" : "PFRAME");

	v4l2_m2m_job_finish(ctx->dev->m2m_enc_dev, ctx->m2m_ctx);
	
	return IRQ_HANDLED;
}

static void codadx6_device_run(void *m2m_priv)
{
	struct codadx6_ctx *ctx = m2m_priv;
	struct codadx6_q_data *q_data_src, *q_data_dst;
	struct vb2_buffer *src_buf, *dst_buf;
	struct codadx6_dev *dev = ctx->dev;

	src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
	q_data_src = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	q_data_dst = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);


	src_buf->v4l2_buf.sequence = ctx->isequence;
	dst_buf->v4l2_buf.sequence = ctx->isequence;
	ctx->isequence++;

	/* 
	 * Workaround codadx6 firmware BUG that only marks the first
	 * frame as IDR. This is a problem for some decoders that can't
	 * recover when a frame is lost.
	 */
	if (src_buf->v4l2_buf.sequence % ctx->enc_params.gop_size) {
		src_buf->v4l2_buf.flags |= V4L2_BUF_FLAG_PFRAME;
		src_buf->v4l2_buf.flags &= ~V4L2_BUF_FLAG_KEYFRAME;
	} else {
		src_buf->v4l2_buf.flags |= V4L2_BUF_FLAG_KEYFRAME;
		src_buf->v4l2_buf.flags &= ~V4L2_BUF_FLAG_PFRAME;
	}

	ctx->runtime.source_frame.y = vb2_dma_contig_plane_dma_addr(src_buf, 0);
	ctx->runtime.source_frame.cb = ctx->runtime.source_frame.y +
				q_data_src->width * q_data_src->height;
	ctx->runtime.source_frame.cr = ctx->runtime.source_frame.cb +
				q_data_src->width / 2 * q_data_src->height / 2;

	if (src_buf->v4l2_buf.flags & V4L2_BUF_FLAG_KEYFRAME) {
		ctx->runtime.force_ipicture = 1;
		if (ctx->enc_params.codec_mode == CODADX6_MODE_ENCODE_H264) {
			ctx->runtime.quant_param = ctx->enc_params.h264_intra_qp;
		} else {
			ctx->runtime.quant_param = ctx->enc_params.mpeg4_intra_qp;
		}
	} else {
		ctx->runtime.force_ipicture = 0;
		if (ctx->enc_params.codec_mode == CODADX6_MODE_ENCODE_H264) {
			ctx->runtime.quant_param = ctx->enc_params.h264_inter_qp;
		} else {
			ctx->runtime.quant_param = ctx->enc_params.mpeg4_inter_qp;
		}
	}
	ctx->runtime.skip_picture = 0;
	ctx->runtime.all_inter_mb = 0;

	/*
	 * Copy headers at the beginning of the first frame for H.264 only.
	 * In MPEG4 they are already copied by the coda.
	 */
	if (src_buf->v4l2_buf.sequence == 0) {
		ctx->runtime.pic_stream_buffer_addr =
			vb2_dma_contig_plane_dma_addr(dst_buf, 0) +
			ctx->runtime.vpu_header_size[0] +
			ctx->runtime.vpu_header_size[1] +
			ctx->runtime.vpu_header_size[2];
		ctx->runtime.pic_stream_buffer_size = CODADX6_ENC_MAX_FRAME_SIZE -
			ctx->runtime.vpu_header_size[0] -
			ctx->runtime.vpu_header_size[1] -
			ctx->runtime.vpu_header_size[2];
		memcpy(vb2_plane_vaddr(dst_buf, 0),
		       &ctx->runtime.vpu_header[0][0], ctx->runtime.vpu_header_size[0]);
		memcpy(vb2_plane_vaddr(dst_buf, 0) + ctx->runtime.vpu_header_size[0],
		       &ctx->runtime.vpu_header[1][0], ctx->runtime.vpu_header_size[1]);
		memcpy(vb2_plane_vaddr(dst_buf, 0) + ctx->runtime.vpu_header_size[0] + ctx->runtime.vpu_header_size[1],
		       &ctx->runtime.vpu_header[2][0], ctx->runtime.vpu_header_size[2]);
	} else {
		ctx->runtime.pic_stream_buffer_addr = vb2_dma_contig_plane_dma_addr(dst_buf, 0);
		ctx->runtime.pic_stream_buffer_size = CODADX6_ENC_MAX_FRAME_SIZE;
	}
	
	/* codadx6_encoder_submit */
	{
		codadx6_write(dev, 0, CODADX6_CMD_ENC_PIC_ROT_MODE);
		codadx6_write(dev, ctx->runtime.quant_param, CODADX6_CMD_ENC_PIC_QS);
		
		if (ctx->runtime.skip_picture) {
			codadx6_write(dev, 1, CODADX6_CMD_ENC_PIC_OPTION);
		} else {
			codadx6_write(dev, ctx->runtime.source_frame.y, CODADX6_CMD_ENC_PIC_SRC_ADDR_Y);
			codadx6_write(dev, ctx->runtime.source_frame.cb, CODADX6_CMD_ENC_PIC_SRC_ADDR_CB);
			codadx6_write(dev, ctx->runtime.source_frame.cr, CODADX6_CMD_ENC_PIC_SRC_ADDR_CR);
			codadx6_write(dev, (ctx->runtime.all_inter_mb << 5) | (ctx->runtime.force_ipicture << 1 & 0x2), CODADX6_CMD_ENC_PIC_OPTION);
		}

		codadx6_write(dev, ctx->runtime.pic_stream_buffer_addr, CODADX6_CMD_ENC_PIC_BB_START);
		codadx6_write(dev, ctx->runtime.pic_stream_buffer_size / 1024, CODADX6_CMD_ENC_PIC_BB_SIZE);
		codadx6_command_async(dev, ctx->enc_params.codec_mode, CODADX6_COMMAND_PIC_RUN);
	}
}

static int codadx6_job_ready(void *m2m_priv)
{
	struct codadx6_ctx *ctx = m2m_priv;

	/* 
	 * For both 'P' and 'key' frame cases 1 picture
	 * and 1 frame are needed.
	 */
	if (!(v4l2_m2m_num_src_bufs_ready(ctx->m2m_ctx) >= 1) ||
		!(v4l2_m2m_num_dst_bufs_ready(ctx->m2m_ctx) >= 1)) {
		v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev,
			 "not ready: not enough video buffers.\n");
		return 0;
	}

	/* For P frames a reference picture is needed too */
	if ((ctx->gopcounter != (ctx->enc_params.gop_size - 1)) && (!ctx->reference)) {
		v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev,
			 "not ready: reference picture not available.\n");
		return 0;
	}

	if (codadx6_isbusy(ctx->dev)) {
		v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev,
			 "not ready: coda is still busy.\n");
		return 0;
	}

	v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev,
			"job ready\n");
	return 1;
}

static void codadx6_job_abort(void *priv)
{
	struct codadx6_ctx *ctx = priv;
	struct codadx6_dev *dev = ctx->dev;

	ctx->aborting = 1;

	v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev,
		 "Aborting task\n");
	
	v4l2_m2m_job_finish(dev->m2m_enc_dev, ctx->m2m_ctx);
}

static void codadx6_lock(void *m2m_priv)
{
	struct codadx6_ctx *ctx = m2m_priv;
	struct codadx6_dev *pcdev = ctx->dev;
	mutex_lock(&pcdev->dev_mutex);
}

static void codadx6_unlock(void *m2m_priv)
{
	struct codadx6_ctx *ctx = m2m_priv;
	struct codadx6_dev *pcdev = ctx->dev;
	mutex_unlock(&pcdev->dev_mutex);
}

static struct v4l2_m2m_ops codadx6_enc_m2m_ops = {
	.device_run	= codadx6_device_run,
	.job_ready	= codadx6_job_ready,
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
	ctx->enc_params.framerate = 30;
	ctx->reference = NULL;
	ctx->aborting = 0;

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

static void codadx6_wait_prepare(struct vb2_queue *q)
{
	struct codadx6_ctx *ctx = vb2_get_drv_priv(q);
	codadx6_unlock(ctx);
}

static void codadx6_wait_finish(struct vb2_queue *q)
{
	struct codadx6_ctx *ctx = vb2_get_drv_priv(q);
	codadx6_lock(ctx);
}

static int codadx6_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct codadx6_ctx *ctx = vb2_get_drv_priv(q);
	struct codadx6_dev *dev = ctx->dev;

	if (count < 1)
		return -EINVAL;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		ctx->rawstreamon = 1;
	} else {
		ctx->compstreamon = 1;
	}

	if (ctx->rawstreamon & ctx->compstreamon) {
		struct codadx6_q_data *q_data_src, *q_data_dst;
		struct vb2_buffer *buf;
		struct vb2_queue *vq;
		u32 value;
		int i = 0;

		ctx->gopcounter = ctx->enc_params.gop_size - 1;

		q_data_src = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);
		ctx->runtime.pic_width = q_data_src->width;
		ctx->runtime.pic_height = q_data_src->height;
		buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
		ctx->runtime.bitstream_buf = vb2_dma_contig_plane_dma_addr(buf, 0);
		q_data_dst = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);
		ctx->runtime.bitstream_buf_size = q_data_dst->sizeimage;
		ctx->runtime.bitstream_format = q_data_dst->fmt->fourcc;
		ctx->runtime.initial_delay = 0;
		ctx->runtime.vbv_buffer_size = 0;
		ctx->runtime.enable_autoskip = 1;
		ctx->runtime.intra_refresh = 0;
		ctx->runtime.gamma = 4096;
		ctx->runtime.maxqp = 0;

		if (!codadx6_is_initialized(dev)) {
			v4l2_err(&ctx->dev->v4l2_dev, "coda is not initialized.\n");
			return -EFAULT;
		}

		/* codadx6_encoder_init */
		{
		
		if (ctx->runtime.bitstream_format == V4L2_PIX_FMT_H264) {
			ctx->enc_params.codec_mode = CODADX6_MODE_ENCODE_H264;
		} else if (ctx->runtime.bitstream_format == V4L2_PIX_FMT_MPEG4) {
			ctx->enc_params.codec_mode = CODADX6_MODE_ENCODE_M4S2;
		}

		ctx->runtime.stream_rd_ptr = ctx->runtime.bitstream_buf;
		ctx->runtime.stream_buf_start_addr = ctx->runtime.bitstream_buf;
		ctx->runtime.stream_buf_size = ctx->runtime.bitstream_buf_size;
		ctx->runtime.stream_buf_end_addr = ctx->runtime.bitstream_buf +
						ctx->runtime.bitstream_buf_size;
		ctx->runtime.initial_info_obtained = 0;

		codadx6_write(dev, ctx->runtime.stream_rd_ptr, CODADX6_REG_BIT_RD_PTR_0);
		codadx6_write(dev, ctx->runtime.stream_buf_start_addr, CODADX6_REG_BIT_WR_PTR_0);
		value = codadx6_read(dev, CODADX6_REG_BIT_STREAM_CTRL);
		value &= 0xffe7;
		value |= 3 << 3;
		codadx6_write(dev, value, CODADX6_REG_BIT_STREAM_CTRL);
		}

		/* walk the src ready list and store buffer phys addresses  */
		vq = v4l2_m2m_get_vq(ctx->m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);
		for (i = 0; i < vq->num_buffers; i++) {
			buf = vq->bufs[i];
			ctx->runtime.frame_buf_pool[i].y = vb2_dma_contig_plane_dma_addr(buf, 0);
			ctx->runtime.frame_buf_pool[i].cb = ctx->runtime.frame_buf_pool[i].y +
				q_data_src->width * q_data_src->height;
			ctx->runtime.frame_buf_pool[i].cr = ctx->runtime.frame_buf_pool[i].cb +
				q_data_src->width / 2 * q_data_src->height / 2;
		}
		ctx->runtime.num_frame_buffers = vq->num_buffers;
		ctx->runtime.stride = q_data_src->width;
		
		/* codadx6_encoder_configure */
		{
		u32 data;

		codadx6_write(dev, 0xFFFF4C00, CODADX6_REG_BIT_SEARCH_RAM_BASE_ADDR);

		/* Could set rotation here if needed */
		data = (ctx->runtime.pic_width & CODADX6_PICWIDTH_MASK) << CODADX6_PICWIDTH_OFFSET;
		data |= (ctx->runtime.pic_height & CODADX6_PICHEIGHT_MASK) << CODADX6_PICHEIGHT_OFFSET;
		codadx6_write(dev, data, CODADX6_CMD_ENC_SEQ_SRC_SIZE);
		codadx6_write(dev, ctx->enc_params.framerate, CODADX6_CMD_ENC_SEQ_SRC_F_RATE);

		if (ctx->runtime.bitstream_format == V4L2_PIX_FMT_MPEG4) {
			codadx6_write(dev, CODADX6_ENCODE_MPEG4, CODADX6_CMD_ENC_SEQ_COD_STD);
			data  = (0 & CODADX6_MP4PARAM_VERID_MASK) << CODADX6_MP4PARAM_VERID_OFFSET;
			data |= (0 & CODADX6_MP4PARAM_INTRADCVLCTHR_MASK) << CODADX6_MP4PARAM_INTRADCVLCTHR_OFFSET;
			data |= (0 & CODADX6_MP4PARAM_REVERSIBLEVLCENABLE_MASK) << CODADX6_MP4PARAM_REVERSIBLEVLCENABLE_OFFSET;
			data |=  0 & CODADX6_MP4PARAM_DATAPARTITIONENABLE_MASK;
			codadx6_write(dev, data, CODADX6_CMD_ENC_SEQ_MP4_PARA);
		} else if (ctx->runtime.bitstream_format == V4L2_PIX_FMT_H264) {
			codadx6_write(dev, CODADX6_ENCODE_H264, CODADX6_CMD_ENC_SEQ_COD_STD);
			data  = (0 & CODADX6_264PARAM_DEBLKFILTEROFFSETBETA_MASK) << CODADX6_264PARAM_DEBLKFILTEROFFSETBETA_OFFSET;
			data |= (0 & CODADX6_264PARAM_DEBLKFILTEROFFSETALPHA_MASK) << CODADX6_264PARAM_DEBLKFILTEROFFSETALPHA_OFFSET;
			data |= (0 & CODADX6_264PARAM_DISABLEDEBLK_MASK) << CODADX6_264PARAM_DISABLEDEBLK_OFFSET;
			data |= (0 & CODADX6_264PARAM_CONSTRAINEDINTRAPREDFLAG_MASK) << CODADX6_264PARAM_CONSTRAINEDINTRAPREDFLAG_OFFSET;
			data |=  0 & CODADX6_264PARAM_CHROMAQPOFFSET_MASK;
			codadx6_write(dev, data, CODADX6_CMD_ENC_SEQ_264_PARA);
		}

		data  = (ctx->enc_params.slice_max_mb & CODADX6_SLICING_SIZE_MASK) << CODADX6_SLICING_SIZE_OFFSET;
		data |= (1 & CODADX6_SLICING_UNIT_MASK) << CODADX6_SLICING_UNIT_OFFSET;
		if (ctx->enc_params.slice_mode == V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB)
			data |=  1 & CODADX6_SLICING_MODE_MASK;
		codadx6_write(dev, data, CODADX6_CMD_ENC_SEQ_SLICE_MODE);
		data  =  ctx->enc_params.gop_size & CODADX6_GOP_SIZE_MASK;
		codadx6_write(dev, data, CODADX6_CMD_ENC_SEQ_GOP_SIZE);
		
		if (ctx->enc_params.bitrate) {
			/* Rate control enabled */
			data  = ((!ctx->runtime.enable_autoskip) & CODADX6_RATECONTROL_AUTOSKIP_MASK) << CODADX6_RATECONTROL_AUTOSKIP_OFFSET;
			data |= (ctx->runtime.initial_delay & CODADX6_RATECONTROL_INITIALDELAY_MASK) << CODADX6_RATECONTROL_INITIALDELAY_OFFSET;
			data |= (ctx->enc_params.bitrate & CODADX6_RATECONTROL_BITRATE_MASK) << CODADX6_RATECONTROL_BITRATE_OFFSET;
			data |=  1 & CODADX6_RATECONTROL_ENABLE_MASK;
		} else {
			data = 0;
		}
		codadx6_write(dev, data, CODADX6_CMD_ENC_SEQ_RC_PARA);

		codadx6_write(dev, ctx->runtime.vbv_buffer_size, CODADX6_CMD_ENC_SEQ_RC_BUF_SIZE);
		codadx6_write(dev, ctx->runtime.intra_refresh, CODADX6_CMD_ENC_SEQ_INTRA_REFRESH);

		codadx6_write(dev, ctx->runtime.stream_buf_start_addr, CODADX6_CMD_ENC_SEQ_BB_START);
		codadx6_write(dev, ctx->runtime.stream_buf_size / 1024, CODADX6_CMD_ENC_SEQ_BB_SIZE);

		if (ctx->runtime.maxqp) {
			/* adjust qp if they are above the maximum */
			if ((ctx->runtime.bitstream_format == V4L2_PIX_FMT_MPEG4) && (ctx->runtime.maxqp > 31)) ctx->runtime.maxqp = 31;  
			if ((ctx->runtime.bitstream_format == V4L2_PIX_FMT_H264) && (ctx->runtime.maxqp > 51)) ctx->runtime.maxqp = 51;
			data = (ctx->runtime.maxqp & CODADX6_QPMAX_MASK) << CODADX6_QPMAX_OFFSET;
			codadx6_write(dev, data, CODADX6_CMD_ENC_SEQ_RC_QP_MAX);
		}
    
		if (ctx->runtime.gamma) {
			/* set default gamma if not set */
			if (ctx->runtime.gamma > 32768) ctx->runtime.gamma = 32768;
			data = (ctx->runtime.gamma & CODADX6_GAMMA_MASK) << CODADX6_GAMMA_OFFSET;
			codadx6_write(dev, data, CODADX6_CMD_ENC_SEQ_RC_GAMMA);
		}

		data  = (ctx->runtime.gamma > 0) << CODADX6_OPTION_GAMMA_OFFSET;
		data |= (ctx->runtime.maxqp > 0) << CODADX6_OPTION_LIMITQP_OFFSET;
		data |= (0 & CODADX6_OPTION_SLICEREPORT_MASK) << CODADX6_OPTION_SLICEREPORT_OFFSET;
		codadx6_write(dev, data, CODADX6_CMD_ENC_SEQ_OPTION);

		if (ctx->enc_params.codec_mode == CODADX6_MODE_ENCODE_H264) {
			data  = (FMO_SLICE_SAVE_BUF_SIZE << 7);
			data |= (0 & CODADX6_FMOPARAM_TYPE_MASK) << CODADX6_FMOPARAM_TYPE_OFFSET;
			data |=  0 & CODADX6_FMOPARAM_SLICENUM_MASK;
			codadx6_write(dev, data, CODADX6_CMD_ENC_SEQ_FMO);
		}

		if (codadx6_command_sync(dev, ctx->enc_params.codec_mode, CODADX6_COMMAND_SEQ_INIT)) {
			v4l2_err(&ctx->dev->v4l2_dev, "CODADX6_COMMAND_SEQ_INIT timeout\n");
			return -ETIMEDOUT;
		}

		if (codadx6_read(dev, CODADX6_RET_ENC_SEQ_SUCCESS) == 0)
			return -EFAULT;

		/* Let the codec know the addresses of the frame buffers */
		for (i = 0; i < ctx->runtime.num_frame_buffers; i++) {
			u32 *p;

			p = ctx->dev->enc_parabuf.vaddr;
			p[i * 3] = ctx->runtime.frame_buf_pool[i].y;
			p[i * 3 + 1] = ctx->runtime.frame_buf_pool[i].cb;
			p[i * 3 + 2] = ctx->runtime.frame_buf_pool[i].cr;
		}
		codadx6_write(dev, ctx->runtime.num_frame_buffers, CODADX6_CMD_SET_FRAME_BUF_NUM);
		codadx6_write(dev, ctx->runtime.stride, CODADX6_CMD_SET_FRAME_BUF_STRIDE);
		if (codadx6_command_sync(dev, ctx->enc_params.codec_mode, CODADX6_COMMAND_SET_FRAME_BUF)) {
			v4l2_err(&ctx->dev->v4l2_dev, "CODADX6_COMMAND_SET_FRAME_BUF timeout\n");
			return -ETIMEDOUT;
		}

		ctx->runtime.initial_info_obtained = 1;
		}

		/* Save stream headers */
		buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
		if (ctx->runtime.bitstream_format == V4L2_PIX_FMT_H264) {
			/* Get SPS in the first frame and copy it to an intermediate buffer TODO: copy directly */
			codadx6_write(dev, vb2_dma_contig_plane_dma_addr(buf, 0), CODADX6_CMD_ENC_HEADER_BB_START);
			codadx6_write(dev, ctx->runtime.bitstream_buf_size, CODADX6_CMD_ENC_HEADER_BB_SIZE);
			codadx6_write(dev, CODADX6_HEADER_H264_SPS, CODADX6_CMD_ENC_HEADER_CODE);
			if (codadx6_command_sync(dev, ctx->enc_params.codec_mode, CODADX6_COMMAND_ENCODE_HEADER)) {
				v4l2_err(&ctx->dev->v4l2_dev, "CODADX6_COMMAND_ENCODE_HEADER timeout\n");
				return -ETIMEDOUT;
			}
			ctx->runtime.vpu_header_size[0] = codadx6_read(dev, CODADX6_REG_BIT_WR_PTR_0) - 
					codadx6_read(dev, CODADX6_CMD_ENC_HEADER_BB_START);
			memcpy(&ctx->runtime.vpu_header[0][0], vb2_plane_vaddr(buf, 0), ctx->runtime.vpu_header_size[0]);

			/* Get PPS in the first frame and copy it to an intermediate buffer TODO: copy directly*/
			codadx6_write(dev, vb2_dma_contig_plane_dma_addr(buf, 0), CODADX6_CMD_ENC_HEADER_BB_START);
			codadx6_write(dev, ctx->runtime.bitstream_buf_size, CODADX6_CMD_ENC_HEADER_BB_SIZE);
			codadx6_write(dev, CODADX6_HEADER_H264_PPS, CODADX6_CMD_ENC_HEADER_CODE);
			if (codadx6_command_sync(dev, ctx->enc_params.codec_mode, CODADX6_COMMAND_ENCODE_HEADER)) {
				v4l2_err(&ctx->dev->v4l2_dev, "CODADX6_COMMAND_ENCODE_HEADER timeout\n");
				return -ETIMEDOUT;
			}
			ctx->runtime.vpu_header_size[1] = codadx6_read(dev, CODADX6_REG_BIT_WR_PTR_0) - 
					codadx6_read(dev, CODADX6_CMD_ENC_HEADER_BB_START);
			memcpy(&ctx->runtime.vpu_header[1][0], vb2_plane_vaddr(buf, 0), ctx->runtime.vpu_header_size[1]);
			ctx->runtime.vpu_header_size[2] = 0;
		} else { /* MPEG4 */
			/* Get VOS in the first frame and copy it to an intermediate buffer TODO: copy directly */
			codadx6_write(dev, vb2_dma_contig_plane_dma_addr(buf, 0), CODADX6_CMD_ENC_HEADER_BB_START);
			codadx6_write(dev,  ctx->runtime.bitstream_buf_size, CODADX6_CMD_ENC_HEADER_BB_SIZE);
			codadx6_write(dev, CODADX6_HEADER_MP4V_VOS, CODADX6_CMD_ENC_HEADER_CODE);
			if (codadx6_command_sync(dev, ctx->enc_params.codec_mode, CODADX6_COMMAND_ENCODE_HEADER)) {
				v4l2_err(&ctx->dev->v4l2_dev, "CODADX6_COMMAND_ENCODE_HEADER timeout\n");
				return -ETIMEDOUT;
			}
			ctx->runtime.vpu_header_size[0] = codadx6_read(dev, CODADX6_REG_BIT_WR_PTR_0) - 
					codadx6_read(dev, CODADX6_CMD_ENC_HEADER_BB_START);
			memcpy(&ctx->runtime.vpu_header[0][0], vb2_plane_vaddr(buf, 0), ctx->runtime.vpu_header_size[0]);

			codadx6_write(dev, vb2_dma_contig_plane_dma_addr(buf, 0), CODADX6_CMD_ENC_HEADER_BB_START);
			codadx6_write(dev, ctx->runtime.bitstream_buf_size, CODADX6_CMD_ENC_HEADER_BB_SIZE);
			codadx6_write(dev, CODADX6_HEADER_MP4V_VIS, CODADX6_CMD_ENC_HEADER_CODE);
			if (codadx6_command_sync(dev, ctx->enc_params.codec_mode, CODADX6_COMMAND_ENCODE_HEADER)) {
				v4l2_err(&ctx->dev->v4l2_dev, "CODADX6_COMMAND_ENCODE_HEADER failed\n");
				return -ETIMEDOUT;
			}
			ctx->runtime.vpu_header_size[1] = codadx6_read(dev, CODADX6_REG_BIT_WR_PTR_0) -
					codadx6_read(dev, CODADX6_CMD_ENC_HEADER_BB_START);
			memcpy(&ctx->runtime.vpu_header[1][0], vb2_plane_vaddr(buf, 0), ctx->runtime.vpu_header_size[1]);

			codadx6_write(dev, vb2_dma_contig_plane_dma_addr(buf, 0), CODADX6_CMD_ENC_HEADER_BB_START);
			codadx6_write(dev, ctx->runtime.bitstream_buf_size, CODADX6_CMD_ENC_HEADER_BB_SIZE);
			codadx6_write(dev, CODADX6_HEADER_MP4V_VOL, CODADX6_CMD_ENC_HEADER_CODE);
			if (codadx6_command_sync(dev, ctx->enc_params.codec_mode, CODADX6_COMMAND_ENCODE_HEADER)) {
				v4l2_err(&ctx->dev->v4l2_dev, "CODADX6_COMMAND_ENCODE_HEADER failed\n");
				return -ETIMEDOUT;
			}
			ctx->runtime.vpu_header_size[2] = codadx6_read(dev, CODADX6_REG_BIT_WR_PTR_0) -
					codadx6_read(dev, CODADX6_CMD_ENC_HEADER_BB_START);
			memcpy(&ctx->runtime.vpu_header[2][0], vb2_plane_vaddr(buf, 0), ctx->runtime.vpu_header_size[2]);
		}
	}
	return 0;
}

static int codadx6_stop_streaming(struct vb2_queue *q)
{
	struct codadx6_ctx *ctx = vb2_get_drv_priv(q);
	struct codadx6_dev *dev = ctx->dev;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev, "%s: output\n", __func__);
		ctx->rawstreamon = 0;
	} else {
		v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev, "%s: capture\n", __func__);
		ctx->compstreamon = 0;
	}

	if (!ctx->rawstreamon & !ctx->compstreamon) {
		v4l2_dbg(1, codadx6_debug, &ctx->dev->v4l2_dev, "%s: sent command 'SEQ_END' to coda\n", __func__);
		if (codadx6_command_sync(dev, ctx->enc_params.codec_mode, CODADX6_COMMAND_SEQ_END)) {
			v4l2_err(&ctx->dev->v4l2_dev, "CODADX6_COMMAND_SEQ_END failed\n");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static struct vb2_ops codadx6_enc_qops = {
	.queue_setup		= codadx6_enc_queue_setup,
	.buf_prepare		= codadx6_enc_buf_prepare,
	.buf_queue		= codadx6_enc_buf_queue,
	.wait_prepare		= codadx6_wait_prepare,
	.wait_finish		= codadx6_wait_finish,
	.start_streaming	= codadx6_start_streaming,
	.stop_streaming		= codadx6_stop_streaming,
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
		ctx->enc_params.bitrate = ctrl->val / 1000;
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
