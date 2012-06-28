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

#include <linux/clk.h>
#include <linux/coda_codec.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <mach/hardware.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "coda_common.h"
#include "coda_regs.h"
#include "coda_enc.h"

#define CODA_NAME		"coda"

#define CODA_FMO_BUF_SIZE	32
#define CODA_CODE_BUF_SIZE	(64 * 1024)
#define CODA_WORK_BUF_SIZE	(288 * 1024 + CODA_FMO_BUF_SIZE * 8 * 1024)
#define CODA_PARA_BUF_SIZE	(10 * 1024)
#define CODA_ISRAM_SIZE	(2048 * 2)

#define CODA_SUPPORTED_PRODUCT_ID	0xf001
#define CODA_SUPPORTED_MAJOR		2
#define CODA_SUPPORTED_MINOR		2
#define CODA_SUPPORTED_RELEASE	5

int coda_debug = 3;
module_param(coda_debug, int, 0);
MODULE_PARM_DESC(coda_debug, "Debug level (0-1)");

struct coda_q_data *get_q_data(struct coda_ctx *ctx,
					 enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &(ctx->q_data[V4L2_M2M_SRC]);
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &(ctx->q_data[V4L2_M2M_DST]);
	default:
		BUG();
	}
	return NULL;
}

static enum coda_node_type coda_get_node_type(struct file *file)
{
	struct video_device *vfd = video_devdata(file);

	if (vfd->index == 0)
		return CODA_NODE_ENCODER;
	else /* decoder not supported */
		return CODA_NODE_INVALID;
}

static int coda_queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct coda_ctx *ctx = priv;
	int ret;

	memset(src_vq, 0, sizeof(*src_vq));
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	if (ctx->inst_type == CODA_INST_ENCODER) {
		src_vq->ops = get_enc_qops();
	} else {
		v4l2_err(&ctx->dev->v4l2_dev, "Instance not supported\n");
		return -EINVAL;
	}
	src_vq->mem_ops = &vb2_dma_contig_memops;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	memset(dst_vq, 0, sizeof(*dst_vq));
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	if (ctx->inst_type == CODA_INST_ENCODER) {
		dst_vq->ops = get_enc_qops();
	} else {
		v4l2_err(&ctx->dev->v4l2_dev, "Instance not supported\n");
		return -EINVAL;
	}
	dst_vq->mem_ops = &vb2_dma_contig_memops;

	return vb2_queue_init(dst_vq);
}

static int coda_open(struct file *file)
{
	struct coda_dev *dev = video_drvdata(file);
	struct coda_ctx *ctx = NULL;
	int ret = 0;

	ctx = kzalloc(sizeof *ctx, GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);
	ctx->dev = dev;

	if (coda_get_node_type(file) == CODA_NODE_ENCODER) {
		ctx->inst_type = CODA_INST_ENCODER;
		set_enc_default_params(ctx);
		ctx->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_enc_dev, ctx,
						 &coda_queue_init);
		if (IS_ERR(ctx->m2m_ctx)) {
			int ret = PTR_ERR(ctx->m2m_ctx);

			v4l2_err(&dev->v4l2_dev, "%s return error (%d)\n",
				 __func__, ret);
			goto err;
		}
		ret = coda_enc_ctrls_setup(ctx);
		if (ret) {
			v4l2_err(&dev->v4l2_dev, "failed to setup coda controls\n");

			goto err;
		}
	} else {
		v4l2_err(&dev->v4l2_dev, "node type not supported\n");
		ret = -EINVAL;
		goto err;
	}

	ctx->fh.ctrl_handler = &ctx->ctrls;

	clk_enable(dev->clk);

	v4l2_dbg(1, coda_debug, &dev->v4l2_dev, "Created instance %p\n",
		 ctx);

	return 0;

err:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int coda_release(struct file *file)
{
	struct coda_dev *dev = video_drvdata(file);
	struct coda_ctx *ctx = fh_to_ctx(file->private_data);

	v4l2_dbg(1, coda_debug, &dev->v4l2_dev, "Releasing instance %p\n",
		 ctx);

	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrls);
	clk_disable(dev->clk);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static unsigned int coda_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct coda_ctx *ctx = fh_to_ctx(file->private_data);

	return v4l2_m2m_poll(file, ctx->m2m_ctx, wait);
}

static int coda_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct coda_ctx *ctx = fh_to_ctx(file->private_data);

	return v4l2_m2m_mmap(file, ctx->m2m_ctx, vma);
}

static const struct v4l2_file_operations coda_fops = {
	.owner		= THIS_MODULE,
	.open		= coda_open,
	.release	= coda_release,
	.poll		= coda_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= coda_mmap,
};

static irqreturn_t coda_irq_handler(int irq, void *data)
{
	struct coda_dev *dev = data;

	/* read status register to attend the IRQ */
	coda_read(dev, CODA_REG_BIT_INT_STATUS);
	coda_write(dev, CODA_REG_BIT_INT_CLEAR_SET,
		      CODA_REG_BIT_INT_CLEAR);

	return coda_enc_isr(dev);
}

static int coda_hw_init(struct coda_dev *dev, const struct firmware *fw)
{
	u16 product, major, minor, release;
	u32 data;
	u16 *p;
	int i;

	clk_enable(dev->clk);

	/* Copy the whole firmware image to the code buffer */
	memcpy(dev->enc_codebuf.vaddr, fw->data, fw->size);
	/*
	 * Copy the first CODA_ISRAM_SIZE in the internal SRAM.
	 * This memory seems to be big-endian here, which is weird, since
	 * the internal ARM processor of the coda is little endian.
	 * Data in this SRAM survives a reboot.
	 */
	p = (u16 *)fw->data;
	for (i = 0; i < (CODA_ISRAM_SIZE / 2); i++)  {
		data = CODA_DOWN_ADDRESS_SET(i) |
			CODA_DOWN_DATA_SET(p[i ^ 1]);
		coda_write(dev, data, CODA_REG_BIT_CODE_DOWN);
	}
	release_firmware(fw);

	/* Tell the BIT where to find everything it needs */
	coda_write(dev, dev->enc_workbuf.paddr,
		      CODA_REG_BIT_WORK_BUF_ADDR);
	coda_write(dev, dev->enc_parabuf.paddr,
		      CODA_REG_BIT_PARA_BUF_ADDR);
	coda_write(dev, dev->enc_codebuf.paddr,
		      CODA_REG_BIT_CODE_BUF_ADDR);
	coda_write(dev, 0, CODA_REG_BIT_CODE_RUN);

	/* Set default values */
	coda_write(dev, CODA_STREAM_UNDOCUMENTED,
		      CODA_REG_BIT_STREAM_CTRL);
	coda_write(dev, 0, CODA_REG_BIT_FRAME_MEM_CTRL);
	coda_write(dev, CODA_INT_INTERRUPT_ENABLE,
		      CODA_REG_BIT_INT_ENABLE);

	/* Reset VPU and start processor */
	data = coda_read(dev, CODA_REG_BIT_CODE_RESET);
	data |= CODA_REG_RESET_ENABLE;
	coda_write(dev, data, CODA_REG_BIT_CODE_RESET);
	udelay(10);
	data &= ~CODA_REG_RESET_ENABLE;
	coda_write(dev, data, CODA_REG_BIT_CODE_RESET);
	coda_write(dev, CODA_REG_RUN_ENABLE, CODA_REG_BIT_CODE_RUN);

	/* Load firmware */
	coda_write(dev, 0, CODA_CMD_FIRMWARE_VERNUM);
	if (coda_command_sync(dev, 0, CODA_COMMAND_FIRMWARE_GET)) {
		v4l2_err(&dev->v4l2_dev, "firmware get command error\n");
		return -EIO;
	}

	/* Check we are compatible with the loaded firmware */
	data = coda_read(dev, CODA_CMD_FIRMWARE_VERNUM);
	product = CODA_FIRMWARE_PRODUCT(data);
	major = CODA_FIRMWARE_MAJOR(data);
	minor = CODA_FIRMWARE_MINOR(data);
	release = CODA_FIRMWARE_RELEASE(data);

	if ((product != CODA_SUPPORTED_PRODUCT_ID) ||
	    (major != CODA_SUPPORTED_MAJOR) ||
	    (minor != CODA_SUPPORTED_MINOR) ||
	    (release != CODA_SUPPORTED_RELEASE)) {
		v4l2_err(&dev->v4l2_dev, "Wrong firmware:\n product = 0x%04X\n"
			" major = %d\n minor = %d\n release = %d\n",
			product, major, minor, release);
		return -EINVAL;
	}

	clk_disable(dev->clk);

	v4l2_info(&dev->v4l2_dev, "Initialized. Fw version: %u.%u.%u.%u",
		  product, major, minor, release);

	return 0;
}

static void coda_fw_callback(const struct firmware *fw, void *context)
{
	struct coda_dev *dev = context;
	struct platform_device *pdev = dev->plat_dev;
	struct video_device *vfd;
	int ret;

	if (!fw) {
		v4l2_err(&dev->v4l2_dev, "firmware request failed\n");
		return;
	}

	ret = coda_hw_init(dev, fw);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "HW initialization failed\n");
		return;
	}

	/* Encoder device */
	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&dev->v4l2_dev, "Failed to allocate video device\n");
		return;
	}

	vfd->fops	= &coda_fops,
	vfd->ioctl_ops	= get_enc_v4l2_ioctl_ops();
	vfd->release	= video_device_release,
	vfd->lock	= &dev->dev_mutex;
	vfd->v4l2_dev	= &dev->v4l2_dev;
	snprintf(vfd->name, sizeof(vfd->name), "%s", CODA_ENC_NAME);
	dev->vfd_enc = vfd;
	video_set_drvdata(vfd, dev);

	dev->alloc_enc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(dev->alloc_enc_ctx)) {
		v4l2_err(&dev->v4l2_dev, "Failed to alloc vb2 context\n");
		goto rel_vdev;
	}

	dev->m2m_enc_dev = v4l2_m2m_init(get_enc_m2m_ops());
	if (IS_ERR(dev->m2m_enc_dev)) {
		v4l2_err(&dev->v4l2_dev, "Failed to init mem2mem device\n");
		goto rel_ctx;
	}

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register video device\n");
		goto rel_m2m_enc;
	}
	v4l2_info(&dev->v4l2_dev, "encoder registered as /dev/video%d\n",
		  vfd->num);

	return;

rel_m2m_enc:
	v4l2_m2m_release(dev->m2m_enc_dev);
rel_ctx:
	vb2_dma_contig_cleanup_ctx(dev->alloc_enc_ctx);
rel_vdev:
	video_device_release(vfd);

	return;
}

static int coda_firmware_request(struct coda_dev *dev)
{
	char *fw;

	if (cpu_is_mx27()) {
		fw = "v4l-codadx6-imx27.bin";
	} else {
		/* Not supported yet */
		return -ENOENT;
	}

	return request_firmware_nowait(THIS_MODULE, true,
		fw, &dev->plat_dev->dev,GFP_KERNEL, dev, coda_fw_callback);
}

static int __devinit coda_probe(struct platform_device *pdev)
{
	struct coda_dev *dev;
	struct resource *res;
	unsigned int bufsize;
	int ret;

	dev = kzalloc(sizeof *dev, GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Not enough memory for %s\n",
			CODA_NAME);
		return -ENOMEM;
	}

	spin_lock_init(&dev->irqlock);

	dev->plat_dev = pdev;
	if (!dev->plat_dev) {
		dev_err(&pdev->dev, "No platform data specified\n");
		ret = -ENODEV;
		goto free_dev;
	}

	dev->clk = clk_get(&pdev->dev, "vpu");
	if (IS_ERR(dev->clk)) {
		ret = PTR_ERR(dev->clk);
		goto free_dev;
	}

	/* Get  memory for physical registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region resource\n");
		ret = -ENOENT;
		goto free_clk;
	}

	if (devm_request_mem_region(&pdev->dev, res->start,
			resource_size(res), CODA_NAME) == NULL) {
		dev_err(&pdev->dev, "failed to request memory region\n");
		ret = -ENOENT;
		goto free_clk;
	}
	dev->regs_base = devm_ioremap(&pdev->dev, res->start,
				      resource_size(res));
	if (!dev->regs_base) {
		dev_err(&pdev->dev, "failed to ioremap address region\n");
		ret = -ENOENT;
		goto free_clk;
	}

	/* IRQ */
	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "failed to get irq resource\n");
		ret = -ENOENT;
		goto free_clk;
	}

	if (devm_request_irq(&pdev->dev, dev->irq, coda_irq_handler,
		0, CODA_NAME, dev) < 0) {
		dev_err(&pdev->dev, "failed to request irq\n");
		ret = -ENOENT;
		goto free_clk;
	}

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		goto free_clk;

	mutex_init(&dev->dev_mutex);

	/* Encoder */
	/* allocate auxiliary buffers for the BIT processor */
	bufsize = CODA_CODE_BUF_SIZE + CODA_WORK_BUF_SIZE +
		CODA_PARA_BUF_SIZE;
	dev->enc_codebuf.vaddr = dma_alloc_coherent(&pdev->dev, bufsize,
						    &dev->enc_codebuf.paddr,
						    GFP_KERNEL);
	if (!dev->enc_codebuf.vaddr) {
		dev_err(&pdev->dev, "failed to allocate aux buffers\n");
		ret = -ENOMEM;
		goto free_clk;
	}

	dev->enc_workbuf.vaddr = dev->enc_codebuf.vaddr + CODA_CODE_BUF_SIZE;
	dev->enc_workbuf.paddr = dev->enc_codebuf.paddr + CODA_CODE_BUF_SIZE;
	dev->enc_parabuf.vaddr = dev->enc_workbuf.vaddr + CODA_WORK_BUF_SIZE;
	dev->enc_parabuf.paddr = dev->enc_workbuf.paddr + CODA_WORK_BUF_SIZE;

	return coda_firmware_request(dev);

free_clk:
	clk_put(dev->clk);
free_dev:
	kfree(dev);
	return ret;
}

static int coda_remove(struct platform_device *pdev)
{
	struct coda_dev *dev = platform_get_drvdata(pdev);
	unsigned int bufsize = CODA_CODE_BUF_SIZE + CODA_WORK_BUF_SIZE +
				CODA_PARA_BUF_SIZE;

	video_unregister_device(dev->vfd_enc);
	v4l2_m2m_release(dev->m2m_enc_dev);
	vb2_dma_contig_cleanup_ctx(dev->alloc_enc_ctx);
	video_device_release(dev->vfd_enc);
	dma_free_coherent(&pdev->dev, bufsize, &dev->enc_codebuf.vaddr,
			  dev->enc_codebuf.paddr);
	clk_put(dev->clk);
	kfree(dev);

	return 0;
}

static struct platform_driver coda_driver = {
	.probe	= coda_probe,
	.remove	= __devexit_p(coda_remove),
	.driver	= {
		.name	= CODA_NAME,
		.owner	= THIS_MODULE,
		/* TODO: pm ops? */
	},
};

module_platform_driver(coda_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Javier Martin <javier.martin@vista-silicon.com>");
MODULE_DESCRIPTION("CodaDx6 multi-standard codec V4L2 driver");
