/*
 * linux/drivers/media/video/codadx6/codadx6_enc.h
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

#ifndef _CODADX6_ENC_H_
#define _CODADX6_ENC_H_

#define CODADX6_ENC_NAME	"codadx6-enc"

const struct v4l2_ioctl_ops *get_enc_v4l2_ioctl_ops(void);
struct v4l2_m2m_ops *get_enc_m2m_ops(void);
void set_enc_default_params(struct codadx6_ctx *ctx);
struct vb2_ops *get_enc_qops(void);
int codadx6_enc_ctrls_setup(struct codadx6_ctx *ctx);
int codadx6_enc_isr(struct codadx6_dev *dev);

#endif
