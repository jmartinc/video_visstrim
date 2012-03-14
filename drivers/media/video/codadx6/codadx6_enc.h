/*
 * linux/drivers/media/video/codadx6/codadx6_enc.h
 *
 * Copyright (C) 2012 Vista Silicon SL
 *		Javier Martin <javier.martin@vista-silicon.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _CODADX6_ENC_H_
#define _CODADX6_ENC_H_

const struct v4l2_ioctl_ops *get_enc_v4l2_ioctl_ops(void);
struct v4l2_m2m_ops *get_enc_m2m_ops(void);
void set_enc_default_params(struct codadx6_ctx *ctx);
struct vb2_ops *get_enc_qops(void);

#endif
