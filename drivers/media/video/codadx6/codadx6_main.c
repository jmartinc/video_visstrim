/*
 * CodaDx6 multi-standard codec IP
 *
 * Copyright (c) 2012 Vista Silicon S.L.
 * Javier Martin, <javier.martin@vista-silicon.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/codadx6.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "codadx6_common.h"
#include "codadx6_regs.h"
#include "codadx6_enc.h"

#define CODADX6_NAME		"codadx6"

#define CODADX6_FMO_BUF_SIZE	32
#define CODADX6_CODE_BUF_SIZE	(64 * 1024)
#define CODADX6_WORK_BUF_SIZE	(288 * 1024 + CODADX6_FMO_BUF_SIZE * 8 * 1024)
#define CODADX6_PARA_BUF_SIZE	(10 * 1024)
#define CODADX6_ISRAM_SIZE	(2048 * 2)

#define CODADX6_SUPPORTED_PRODUCT_ID	0xf001
#define CODADX6_SUPPORTED_MAJOR		2
#define CODADX6_SUPPORTED_MINOR		2
#define CODADX6_SUPPORTED_RELEASE	5

int codadx6_debug = 3;
module_param(codadx6_debug, int, 0);
MODULE_PARM_DESC(codadx6_debug, "Debug level (0-1)");

struct codadx6_q_data *get_q_data(struct codadx6_ctx *ctx,
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

static enum codadx6_node_type codadx6_get_node_type(struct file *file)
{
	struct video_device *vfd = video_devdata(file);

	if (vfd->index == 0)
		return CODADX6_NODE_ENCODER;
	else /* decoder not supported */
		return CODADX6_NODE_INVALID;
}

static int codadx6_queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct codadx6_ctx *ctx = priv;
	int ret;

	memset(src_vq, 0, sizeof(*src_vq));
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	if (ctx->inst_type == CODADX6_INST_ENCODER) {
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
	if (ctx->inst_type == CODADX6_INST_ENCODER) {
		dst_vq->ops = get_enc_qops();
	} else {
		v4l2_err(&ctx->dev->v4l2_dev, "Instance not supported\n");
		return -EINVAL;
	}
	dst_vq->mem_ops = &vb2_dma_contig_memops;

	return vb2_queue_init(dst_vq);
}

static int codadx6_open(struct file *file)
{
	struct codadx6_dev *dev = video_drvdata(file);
	struct codadx6_ctx *ctx = NULL;
	int ret = 0;

	ctx = kzalloc(sizeof *ctx, GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);
	ctx->dev = dev;

	if (codadx6_get_node_type(file) == CODADX6_NODE_ENCODER) {
		ctx->inst_type = CODADX6_INST_ENCODER;
		set_enc_default_params(ctx);
		ctx->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_enc_dev, ctx,
						 &codadx6_queue_init);
		if (IS_ERR(ctx->m2m_ctx)) {
			int ret = PTR_ERR(ctx->m2m_ctx);
			
			printk("%s return error (%d)\n", __func__, ret);
			goto err;
		}
		ret = codadx6_enc_ctrls_setup(ctx);
		if (ret) {
			v4l2_err(&dev->v4l2_dev, "failed to setup codadx6 controls\n");

			goto err;
		}
	} else {
		v4l2_err(&dev->v4l2_dev, "node type not supported\n");
		ret = -EINVAL;
		goto err;
	}

	ctx->fh.ctrl_handler = &ctx->ctrls;

	clk_enable(dev->clk);

	v4l2_dbg(1, codadx6_debug, &dev->v4l2_dev, "Created instance %p\n",
		 ctx);

	return 0;

err:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int codadx6_release(struct file *file)
{
	struct codadx6_dev *dev = video_drvdata(file);
	struct codadx6_ctx *ctx = fh_to_ctx(file->private_data);

	v4l2_dbg(1, codadx6_debug, &dev->v4l2_dev, "Releasing instance %p\n",
		 ctx);

	v4l2_ctrl_handler_free(&ctx->ctrls);
	clk_disable(dev->clk);
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static unsigned int codadx6_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct codadx6_ctx *ctx = fh_to_ctx(file->private_data);

	return v4l2_m2m_poll(file, ctx->m2m_ctx, wait);
}

static int codadx6_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct codadx6_ctx *ctx = fh_to_ctx(file->private_data);

	return v4l2_m2m_mmap(file, ctx->m2m_ctx, vma);
}

static const struct v4l2_file_operations codadx6_fops = {
	.owner		= THIS_MODULE,
	.open		= codadx6_open,
	.release	= codadx6_release,
	.poll		= codadx6_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= codadx6_mmap,
};

static irqreturn_t codadx6_irq_handler(int irq, void *data)
{
	/*TODO*/
	return IRQ_HANDLED;
}

static int codadx6_hw_init(struct codadx6_dev *dev, const struct firmware *fw)
{
	u16 product, major, minor, release;
	u32 data;
	u16 *p;
	int i;

	clk_enable(dev->clk);

	/* Copy the whole firmware image to the code buffer */
	memcpy(dev->enc_codebuf.vaddr, fw->data, fw->size);
	/*
	 * Copy the first CODADX6_ISRAM_SIZE in the internal SRAM.
	 * This memory seems to be big-endian here, which is weird, since
	 * the internal ARM processor of the codadx6 is little endian.
	 * Data in this SRAM survives a reboot.
	 */
	p = (u16 *)fw->data;
	for (i = 0; i < (CODADX6_ISRAM_SIZE / 2); i++)  {
		data = CODADX6_DOWN_ADDRESS_SET(i) |
			CODADX6_DOWN_DATA_SET(p[i ^ 1]);
		codadx6_write(dev, data, CODADX6_REG_BIT_CODE_DOWN);
	}
	release_firmware(fw);

	/* Tell the BIT where to find everything it needs */
	codadx6_write(dev, dev->enc_workbuf.paddr,
		      CODADX6_REG_BIT_WORK_BUF_ADDR);
	codadx6_write(dev, dev->enc_parabuf.paddr,
		      CODADX6_REG_BIT_PARA_BUF_ADDR);
	codadx6_write(dev, dev->enc_codebuf.paddr,
		      CODADX6_REG_BIT_CODE_BUF_ADDR);
	codadx6_write(dev, 0, CODADX6_REG_BIT_CODE_RUN);

	/* Set default values */
	codadx6_write(dev, CODADX6_STREAM_UNDOCUMENTED,
		      CODADX6_REG_BIT_STREAM_CTRL);
	codadx6_write(dev, 0, CODADX6_REG_BIT_FRAME_MEM_CTRL);
	codadx6_write(dev, CODADX6_INT_INTERRUPT_ENABLE,
		      CODADX6_REG_BIT_INT_ENABLE);

	/* Reset VPU and start processor */
	data = codadx6_read(dev, CODADX6_REG_BIT_CODE_RESET);
	data |= CODADX6_REG_RESET_ENABLE;
	codadx6_write(dev, data, CODADX6_REG_BIT_CODE_RESET);
	udelay(10);
	data &= ~CODADX6_REG_RESET_ENABLE;
	codadx6_write(dev, data, CODADX6_REG_BIT_CODE_RESET);
	codadx6_write(dev, CODADX6_REG_RUN_ENABLE, CODADX6_REG_BIT_CODE_RUN);

	/* Load firmware */
	codadx6_write(dev, 0, CODADX6_CMD_FIRMWARE_VERNUM);
	if (codadx6_command_sync(dev, 0, CODADX6_COMMAND_FIRMWARE_GET)) {
		v4l2_err(&dev->v4l2_dev, "firmware get command error\n");
		return -EIO;
	}

	/* Check we are compatible with the loaded firmware */
	data = codadx6_read(dev, CODADX6_CMD_FIRMWARE_VERNUM);
	product = CODADX6_FIRMWARE_PRODUCT(data);
	major = CODADX6_FIRMWARE_MAJOR(data);
	minor = CODADX6_FIRMWARE_MINOR(data);
	release = CODADX6_FIRMWARE_RELEASE(data);

	if ((product != CODADX6_SUPPORTED_PRODUCT_ID) ||
	    (major != CODADX6_SUPPORTED_MAJOR) ||
	    (minor != CODADX6_SUPPORTED_MINOR) ||
	    (release != CODADX6_SUPPORTED_RELEASE)) {
		v4l2_err(&dev->v4l2_dev, "Wrong firmware:\n product = 0x%04X\n"
			" major = %d\n minor = %d\n release = %d\n",
			product, major, minor, release);
		return -EINVAL;
	}

	clk_disable(dev->clk);

	v4l2_info(&dev->v4l2_dev, "Initialized");

	return 0;
}

static void codadx6_fw_callback(const struct firmware *fw, void *context)
{
	struct codadx6_dev *dev = context;
	struct platform_device *pdev = dev->plat_dev;
	struct codadx6_platform_data *pdata = pdev->dev.platform_data;
	struct video_device *vfd;
	int ret;

	if (!fw) {
		v4l2_err(&dev->v4l2_dev, "firmware request '%s' failed\n",
			 pdata->firmware);
		return;
	}

	ret = codadx6_hw_init(dev, fw);
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

	vfd->fops	= &codadx6_fops,
	vfd->ioctl_ops	= get_enc_v4l2_ioctl_ops();
	vfd->release	= video_device_release,
	vfd->lock	= &dev->dev_mutex;
	vfd->v4l2_dev	= &dev->v4l2_dev;
	snprintf(vfd->name, sizeof(vfd->name), "%s", CODADX6_ENC_NAME);
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
	v4l2_info(&dev->v4l2_dev, "encoder registered as /dev/video%d\n", vfd->num);

	return;

rel_m2m_enc:
	v4l2_m2m_release(dev->m2m_enc_dev);
rel_ctx:
	vb2_dma_contig_cleanup_ctx(dev->alloc_enc_ctx);
rel_vdev:
	video_device_release(vfd);

	return;
}

static int __devinit codadx6_probe(struct platform_device *pdev)
{
	struct codadx6_platform_data *pdata;
	struct codadx6_dev *dev;
	struct resource *res;
	unsigned int bufsize;
	int ret;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "Invalid platform data\n");
		return -EINVAL;
	}

	dev = kzalloc(sizeof *dev, GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Not enough memory for %s\n",
			CODADX6_NAME);
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
			resource_size(res), CODADX6_NAME) == NULL) {
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

	if (devm_request_irq(&pdev->dev, dev->irq, codadx6_irq_handler,
		0, CODADX6_NAME, dev) < 0) {
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
	bufsize = CODADX6_CODE_BUF_SIZE + CODADX6_WORK_BUF_SIZE +
		CODADX6_PARA_BUF_SIZE;
	dev->enc_codebuf.vaddr = dma_alloc_coherent(&pdev->dev, bufsize,
						    &dev->enc_codebuf.paddr,
						    GFP_KERNEL);
	if (!dev->enc_codebuf.vaddr) {
		dev_err(&pdev->dev, "failed to allocate aux buffers\n");
		ret = -ENOMEM;
		goto free_clk;
	}

	dev->enc_workbuf.vaddr = dev->enc_codebuf.vaddr + CODADX6_CODE_BUF_SIZE;
	dev->enc_workbuf.paddr = dev->enc_codebuf.paddr + CODADX6_CODE_BUF_SIZE;
	dev->enc_parabuf.vaddr = dev->enc_workbuf.vaddr + CODADX6_WORK_BUF_SIZE;
	dev->enc_parabuf.paddr = dev->enc_workbuf.paddr + CODADX6_WORK_BUF_SIZE;


	return request_firmware_nowait(THIS_MODULE, true, pdata->firmware,
			&pdev->dev, GFP_KERNEL, dev, codadx6_fw_callback);

free_clk:
	clk_put(dev->clk);
free_dev:
	kfree(dev);
	return ret;
}

static int codadx6_remove(struct platform_device *pdev)
{
	struct codadx6_dev *dev = platform_get_drvdata(pdev);
	unsigned int bufsize = CODADX6_CODE_BUF_SIZE + CODADX6_WORK_BUF_SIZE +
				CODADX6_PARA_BUF_SIZE;

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

static struct platform_driver codadx6_driver = {
	.probe	= codadx6_probe,
	.remove	= __devexit_p(codadx6_remove),
	.driver	= {
		.name	= CODADX6_NAME,
		.owner	= THIS_MODULE,
		/* TODO: pm ops? */
	},
};

module_platform_driver(codadx6_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Javier Martin <javier.martin@vista-silicon.com>");
MODULE_DESCRIPTION("CodaDx6 multi-standard codec V4L2 driver");
