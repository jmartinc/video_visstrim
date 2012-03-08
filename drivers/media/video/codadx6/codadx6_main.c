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
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "codadx6_enc.h"

#define CODADX6_NAME		"codadx6"
#define CODADX6_ENC_NAME	"codadx6-enc"

#define CODADX6_FMO_BUF_SIZE	32
#define CODADX6_CODE_BUF_SIZE	(64 * 1024)
#define CODADX6_WORK_BUF_SIZE	(288 * 1024 + CODADX6_FMO_BUF_SIZE * 8 * 1024)
#define CODADX6_PARA_BUF_SIZE	(10 * 1024)

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
	struct vb2_alloc_ctx	*alloc_ctx;
};

static int codadx6_open(struct file *file)
{
	/* TODO */
	return 0;
}

static int codadx6_release(struct file *file)
{
	/* TODO */
	return 0;
}

static unsigned int codadx6_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	/* TODO */
	return 0;
}

static int codadx6_mmap(struct file *file, struct vm_area_struct *vma)
{
	/* TODO */
	return 0;
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

static int codadx6_probe(struct platform_device *pdev)
{
	struct video_device *vfd;
	struct codadx6_dev *dev;
	struct resource *res;
	unsigned int bufsize;
	int ret;

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

	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&dev->v4l2_dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto free_buf;
	}
	vfd->fops	= &codadx6_fops,
	vfd->ioctl_ops	= get_enc_v4l2_ioctl_ops();
	vfd->release	= video_device_release,
	vfd->lock	= &dev->dev_mutex;
	vfd->v4l2_dev	= &dev->v4l2_dev;
	snprintf(vfd->name, sizeof(vfd->name), "%s", CODADX6_ENC_NAME);
	dev->vfd_enc = vfd;
	video_set_drvdata(vfd, dev);

	dev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(dev->alloc_ctx)) {
		v4l2_err(&dev->v4l2_dev, "Failed to alloc vb2 context\n");
		ret = PTR_ERR(dev->alloc_ctx);
		goto rel_vdev;
	}

	dev->m2m_enc_dev = v4l2_m2m_init(get_enc_m2m_ops());
	if (IS_ERR(dev->m2m_enc_dev)) {
		v4l2_err(&dev->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(dev->m2m_enc_dev);
		goto rel_ctx;
	}

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register video device\n");
		goto rel_m2m_enc;
	}
	v4l2_info(&dev->v4l2_dev, "encoder registered as /dev/video%d\n", vfd->num);

	return 0;

rel_m2m_enc:
	v4l2_m2m_release(dev->m2m_enc_dev);
rel_ctx:
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
rel_vdev:
	video_device_release(vfd);
free_buf:
	dma_free_coherent(&pdev->dev, bufsize, &dev->enc_codebuf.vaddr,
			  dev->enc_codebuf.paddr);
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
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
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
