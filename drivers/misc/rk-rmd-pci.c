// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip remote memory demo
 *
 * Copyright (C) 2021 Rockchip Inc.
 *
 * Author: Shawn Lin <shawn.lin@rock-chips.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/of.h>

#define DRV_NAME "rk-rmd"

static DEFINE_MUTEX(rmd_mutex);
#define BAR_0_SZ SZ_4M
#define RK_RMD_NUM_MSIX_VECTORS 8

struct rk_rmd_msix_context {
	struct pci_dev *dev;
	u16 msg_id;
};

struct rk_rmd {
	struct pci_dev *pdev;
	void __iomem *bar0;
	bool in_used;
	void *virt_mem;
	struct miscdevice dev;
	struct msix_entry msix_entries[RK_RMD_NUM_MSIX_VECTORS];
	struct rk_rmd_msix_context msix_ctx[RK_RMD_NUM_MSIX_VECTORS];
	struct rk_rmd_msix_context msi_ctx;
	bool msi_enable;
	bool msix_enable;
};

struct rk_rmd_elbi {
	u32 elbi_common_reg[12];
	u32 elbi_reg_index;
	u32  elbi_int_index; /* 32 interrupt vectors */
};

static int rk_rmd_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct rk_rmd *rk_rmd = container_of(miscdev, struct rk_rmd, dev);
	int ret = 0;

	mutex_lock(&rmd_mutex);

	if (rk_rmd->in_used)
		ret = -EINVAL;
	else
		rk_rmd->in_used = true;

	mutex_unlock(&rmd_mutex);

	return ret;
}

static int rk_rmd_release(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct rk_rmd *rk_rmd = container_of(miscdev, struct rk_rmd, dev);

	mutex_lock(&rmd_mutex);
	rk_rmd->in_used = false;
	mutex_unlock(&rmd_mutex);

	return 0;
}

static ssize_t rk_rmd_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct miscdevice *miscdev = file->private_data;
	struct rk_rmd *rk_rmd = container_of(miscdev, struct rk_rmd, dev);
	u32 *bar0_buf;
	int loop, i = 0;
	size_t raw_count = count;

	count = (count % 4) ? (count - count % 4) : count;

	if (count > BAR_0_SZ)
		return -EINVAL;

	bar0_buf = kzalloc(count, GFP_ATOMIC);
	if (!bar0_buf)
		return -ENOMEM;


	if (copy_from_user(bar0_buf, buf, count)) {
		count = -EFAULT;
		goto exit;
	}

	for (loop = 0; loop < count / 4; loop++) {
		iowrite32(bar0_buf[i], rk_rmd->bar0 + loop * 4);
		i++;
	}

exit:
	kfree(bar0_buf);

	return raw_count;
}

static ssize_t rk_rmd_read(struct file *file, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct miscdevice *miscdev = file->private_data;
	struct rk_rmd *rk_rmd = container_of(miscdev, struct rk_rmd, dev);
	u32 *bar0_buf;
	int loop, i = 0;
	size_t raw_count = count;

	count = (count % 4) ? (count - count % 4) : count;

	if (count > BAR_0_SZ)
		return -EINVAL;

	bar0_buf = kzalloc(count, GFP_ATOMIC);
	if (!bar0_buf)
		return -ENOMEM;


	for (loop = 0; loop < count / 4; loop++) {
		bar0_buf[i] = ioread32(rk_rmd->bar0 + loop * 4);
		i++;
	}

	if (copy_to_user(buf, bar0_buf, count)) {
		count = -EFAULT;
		goto exit;
	}

exit:
	kfree(bar0_buf);

	return raw_count;
}

int rk_rmd_mmap(struct file *file, struct vm_area_struct *vma)
{
	u64 addr;
	struct miscdevice *miscdev = file->private_data;
	struct rk_rmd *rk_rmd = container_of(miscdev, struct rk_rmd, dev);

	addr = virt_to_phys(rk_rmd->virt_mem);
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (io_remap_pfn_range(vma, vma->vm_start,
	    addr >> PAGE_SHIFT, BAR_0_SZ, vma->vm_page_prot)) {
		dev_err(&rk_rmd->pdev->dev, "io_remap_pfn_range failed\n");
		return -EAGAIN;
	}

	return 0;
}

long rk_rmd_ioctl(struct file *file, unsigned int cmd, unsigned long args)
{
	void __user *argp;
	u32 val;
	struct rk_rmd_elbi elbi;
	struct miscdevice *miscdev = file->private_data;
	struct rk_rmd *rk_rmd = container_of(miscdev, struct rk_rmd, dev);

	argp = (void __user *)args;
	if (copy_from_user(&elbi, argp, sizeof(elbi)))
		return -EFAULT;

	switch (cmd) {
	case 0x1:
		/* write ELBI */
		pci_write_config_dword(rk_rmd->pdev,
				       0xe10 + elbi.elbi_reg_index * 4,
				       elbi.elbi_common_reg[elbi.elbi_reg_index]);
		break;
	case 0x2:
		/* read ELBI */
		pci_read_config_dword(rk_rmd->pdev,
				      0xe10 + elbi.elbi_reg_index * 4,
				      &elbi.elbi_common_reg[elbi.elbi_reg_index]);
		if (copy_to_user(argp, &elbi, sizeof(elbi)))
			return -EFAULT;
		break;
	case 0x3:
		/* trigger ELBI interrupt */
		val = 0x1 << (elbi.elbi_int_index % 16); /* int bit */
		val |= 0x1 << ((elbi.elbi_int_index % 16) + 16); /* write mask */
		pci_write_config_dword(rk_rmd->pdev,
				       0xe00 + ((elbi.elbi_int_index  >> 4) << 2),
				       val);
		/* int mask bit was set by EP firmware when probing */
		break;
	default:
		break;
	}

	return 0;
}

static const struct file_operations rk_rmd_fops = {
	.owner		= THIS_MODULE,
	.open		= rk_rmd_open,
	.write		= rk_rmd_write,
	.read		= rk_rmd_read,
	.unlocked_ioctl = rk_rmd_ioctl,
	.mmap		= rk_rmd_mmap,
	.release	= rk_rmd_release,
	.llseek = no_llseek,
};

static irqreturn_t rk_rmd_pcie_interrupt(int irq, void *context)
{
	struct rk_rmd_msix_context *ctx = context;
	struct pci_dev *pdev = ctx->dev;
	struct rk_rmd *rk_rmd = pci_get_drvdata(pdev);

	if (!rk_rmd)
		return IRQ_HANDLED;

	if (rk_rmd->msix_enable)
		dev_info(&pdev->dev, "MSI-X is triggered for 0x%x\n", ctx->msg_id);

	else /* rk_rmd->msi_enable */
		dev_info(&pdev->dev, "MSI was triggered\n");

	return IRQ_HANDLED;
}

static int rk_rmd_request_irq(struct rk_rmd *rk_rmd)
{
        int ret, i, j;

	for (i = 0; i < RK_RMD_NUM_MSIX_VECTORS; i++)
		rk_rmd->msix_entries[i].entry = i;

	ret = pci_enable_msix_exact(rk_rmd->pdev, rk_rmd->msix_entries,
				    RK_RMD_NUM_MSIX_VECTORS);
	if (ret)
		goto msi;

	for (i = 0; i < RK_RMD_NUM_MSIX_VECTORS; i++) {
		rk_rmd->msix_ctx[i].dev = rk_rmd->pdev;
		rk_rmd->msix_ctx[i].msg_id = i;

		ret = request_irq(rk_rmd->msix_entries[i].vector,
				  rk_rmd_pcie_interrupt, 0, DRV_NAME,
				  &rk_rmd->msix_ctx[i]);

		if (ret) {
			dev_err(&rk_rmd->pdev->dev, "fail to allocate msix interrupt\n");
			break;
		} else {
			rk_rmd->msix_enable = true;
			return 0;
		}
	}

	if (ret) {
		for (j = 0; j < i; j++)
			free_irq(rk_rmd->msix_entries[j].vector,&rk_rmd->msix_ctx[i]);
	}

	pci_disable_msix(rk_rmd->pdev);

msi:
	if (pci_enable_msi(rk_rmd->pdev) != 0)
		pci_disable_msi(rk_rmd->pdev);

	rk_rmd->msi_ctx.dev = rk_rmd->pdev;
	rk_rmd->msi_ctx.msg_id = -1;

	ret = request_irq(rk_rmd->pdev->irq, rk_rmd_pcie_interrupt, IRQF_SHARED,
			  DRV_NAME, &rk_rmd->msi_ctx);
	if (ret) {
		dev_err(&rk_rmd->pdev->dev, "fail to allocate msi interrupt\n");
		return ret;
	}

	rk_rmd->msi_enable = true;

	return 0;
}

static int rk_rmd_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret;
	struct rk_rmd *rk_rmd;

	pr_err("%s %d\n", __func__, __LINE__);

	rk_rmd = kzalloc(sizeof(*rk_rmd), GFP_KERNEL);
	if (!rk_rmd)
		return -ENOMEM;

	rk_rmd->virt_mem = kzalloc(BAR_0_SZ, GFP_KERNEL);
	if (!rk_rmd->virt_mem) {
		ret = -ENOMEM;
		goto err_virt_mem;
	}

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "pci_enable_device failed %d", ret);
		goto err_pci_enable_dev;
	}

	ret = pci_request_regions(pdev, DRV_NAME);
	if (ret) {
		dev_err(&pdev->dev, "pci_request_regions failed %d", ret);
		goto err_req_regions;
	}

	rk_rmd->bar0 = pci_iomap(pdev, 0, 0);
	if (!rk_rmd->bar0) {
		dev_err(&pdev->dev, "pci_iomap failed");
		ret = -ENOMEM;
		goto err_pci_iomap;
	}

	dev_dbg(&pdev->dev, "get bar0 address is %p\n", rk_rmd->bar0);

	rk_rmd->dev.minor = MISC_DYNAMIC_MINOR;
	rk_rmd->dev.name = DRV_NAME;
	rk_rmd->dev.fops = &rk_rmd_fops;
	rk_rmd->dev.parent = NULL;

	ret = misc_register(&rk_rmd->dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register misc device.\n");
		goto err_register_mdev;
	}

	rk_rmd->pdev = pdev; /* Save pci device struct */

	pci_set_drvdata(pdev, rk_rmd);

	ret = rk_rmd_request_irq(rk_rmd);
	if (ret)
		dev_err(&pdev->dev, "failed to request msi or msi-x irq.\n");

	return 0;

err_register_mdev:
	pci_iounmap(pdev, rk_rmd->bar0);
err_pci_iomap:
	pci_release_regions(pdev);
err_req_regions:
	pci_disable_device(pdev);
err_pci_enable_dev:
	kfree(rk_rmd->virt_mem);
err_virt_mem:
	kfree(rk_rmd);
	return ret;
}

static void rk_rmd_remove(struct pci_dev *pdev)
{
	struct rk_rmd *rk_rmd = pci_get_drvdata(pdev);
	int i;

	pci_iounmap(pdev, rk_rmd->bar0);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	misc_deregister(&rk_rmd->dev);

	if (rk_rmd->msix_enable) {
		for (i = 0; i < RK_RMD_NUM_MSIX_VECTORS; i++)
			free_irq(rk_rmd->msix_entries[i].vector,&rk_rmd->msix_ctx[i]);
		pci_disable_msix(pdev);
	} else if (rk_rmd->msi_enable) {
		free_irq(pdev->irq, &rk_rmd->msi_ctx);
		pci_disable_msi(pdev);
	}

	kfree(rk_rmd->virt_mem);
	kfree(rk_rmd);
}

static const struct pci_device_id rk_rmd_pcidev_id[] = {
	{ PCI_VDEVICE(ROCKCHIP, 0x356a), 1,  },
	{ }
};
MODULE_DEVICE_TABLE(pci, rk_rmd_pcidev_id);

static struct pci_driver rk_rmd_driver = {
	.name = DRV_NAME,
	.id_table = rk_rmd_pcidev_id,
	.probe = rk_rmd_probe,
	.remove = rk_rmd_remove,
};
module_pci_driver(rk_rmd_driver);

MODULE_DESCRIPTION("Rockchip remote memory demo driver");
MODULE_LICENSE("GPL v2");
