/*
 * linux/drivers/media/video/coda/coda_enc.h
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

#ifndef _CODA_ENC_H_
#define _CODA_ENC_H_

#define CODA_ENC_NAME	"coda-enc"

const struct v4l2_ioctl_ops *get_enc_v4l2_ioctl_ops(void);
struct v4l2_m2m_ops *get_enc_m2m_ops(void);
void set_enc_default_params(struct coda_ctx *ctx);
struct vb2_ops *get_enc_qops(void);
int coda_enc_ctrls_setup(struct coda_ctx *ctx);
int coda_enc_isr(struct coda_dev *dev);

#endif
