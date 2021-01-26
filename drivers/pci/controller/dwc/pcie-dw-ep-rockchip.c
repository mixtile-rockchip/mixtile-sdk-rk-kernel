// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe EP controller driver for Rockchip SoCs
 *
 * Copyright (C) 2021 Rockchip Electronics Co., Ltd.
 *		http://www.rock-chips.com
 *
 * Author: Simon Xue <xxm@rock-chips.com>
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "pcie-designware.h"

/*
 * The upper 16 bits of PCIE_CLIENT_CONFIG are a write
 * mask for the lower 16 bits.
 */
#define HIWORD_UPDATE(mask, val) (((mask) << 16) | (val))
#define HIWORD_UPDATE_BIT(val)	HIWORD_UPDATE(val, val)

#define to_rockchip_pcie(x) dev_get_drvdata((x)->dev)

#define PCIE_CLIENT_RC_MODE		HIWORD_UPDATE_BIT(0x40)
#define PCIE_CLIENT_ENABLE_LTSSM	HIWORD_UPDATE_BIT(0xc)
#define PCIE_SMLH_LINKUP		BIT(16)
#define PCIE_RDLH_LINKUP		BIT(17)
#define PCIE_L0S_ENTRY			0x11
#define PCIE_CLIENT_GENERAL_CONTROL	0x0
#define PCIE_CLIENT_GENERAL_DEBUG	0x104
#define PCIE_CLIENT_HOT_RESET_CTRL      0x180
#define PCIE_CLIENT_LTSSM_STATUS	0x300
#define PCIE_CLIENT_INTR_MASK		0x24
#define PCIE_LTSSM_ENABLE_ENHANCE       BIT(4)
#define PCIE_ELBI_REG_NUM		0x2
#define PCIE_ELBI_LOCAL_BASE		0x200e00
#define PCIE_ELBI_LOCAL_ENABLE_OFF	0x8

#define PCIE_DIRECT_SPEED_CHANGE	BIT(17)

#define PCIE_TYPE0_STATUS_COMMAND_REG	0x4

#define PCIE_TYPE0_HDR_DBI2_OFFSET	0x100000

struct rockchip_pcie {
	struct dw_pcie			pci;
	void __iomem			*apb_base;
	struct phy			*phy;
	struct clk_bulk_data		*clks;
	unsigned int			clk_cnt;
	struct reset_control		*rst;
	struct gpio_desc		*rst_gpio;
	struct regulator                *vpcie3v3;
	unsigned long			*ib_window_map;
	unsigned long			*ob_window_map;
	u32				num_ib_windows;
	u32				num_ob_windows;
	phys_addr_t			*outbound_addr;
	u8				bar_to_atu[6];
	dma_addr_t			ib_target_address;
	u32				ib_target_size;
	void				*ib_target_base;
};

static const struct of_device_id rk_pcie_ep_of_match[] = {
	{
		.compatible = "rockchip,rk3568-pcie-std-ep",
	},
	{
		.compatible = "rockchip,rk3588-pcie-std-ep",
	},
	{},
};

MODULE_DEVICE_TABLE(of, rk_pcie_ep_of_match);

static int rockchip_pcie_readl_apb(struct rockchip_pcie *rockchip,
					     u32 reg)
{
	return readl(rockchip->apb_base + reg);
}

static void rockchip_pcie_writel_apb(struct rockchip_pcie *rockchip,
						u32 val, u32 reg)
{
	writel(val, rockchip->apb_base + reg);
}

static void *rockchip_pcie_map_kernel(phys_addr_t start, size_t len)
{
	int i;
	void *vaddr;
	pgprot_t pgprot;
	phys_addr_t phys;
	int npages = PAGE_ALIGN(len) / PAGE_SIZE;
	struct page **p = vmalloc(sizeof(struct page *) * npages);

	if (!p)
		return NULL;

	pgprot = pgprot_noncached(PAGE_KERNEL);

	phys = start;
	for (i = 0; i < npages; i++) {
		p[i] = phys_to_page(phys);
		phys += PAGE_SIZE;
	}

	vaddr = vmap(p, npages, VM_MAP, pgprot);
	vfree(p);

	return vaddr;
}

static int rockchip_pcie_resource_get(struct platform_device *pdev,
				      struct rockchip_pcie *rockchip)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	void *addr;
	struct resource *dbi_base;
	struct resource *apb_base;
	struct device_node *mem;
	struct resource reg;

	dbi_base = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"pcie-dbi");
	if (!dbi_base) {
		dev_err(&pdev->dev, "get pcie-dbi failed\n");
		return -ENODEV;
	}

	rockchip->pci.dbi_base = devm_ioremap_resource(dev, dbi_base);
	if (IS_ERR(rockchip->pci.dbi_base))
		return PTR_ERR(rockchip->pci.dbi_base);
	rockchip->pci.atu_base = rockchip->pci.dbi_base + DEFAULT_DBI_ATU_OFFSET;

	apb_base = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"pcie-apb");
	if (!apb_base) {
		dev_err(dev, "get pcie-apb failed\n");
		return -ENODEV;
	}

	rockchip->apb_base = devm_ioremap_resource(dev, apb_base);
	if (IS_ERR(rockchip->apb_base))
		return PTR_ERR(rockchip->apb_base);

	rockchip->rst_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(rockchip->rst_gpio))
		return PTR_ERR(rockchip->rst_gpio);

	ret = of_property_read_u32(np, "num-ib-windows",
				   &rockchip->num_ib_windows);
	if (ret < 0) {
		dev_err(dev, "unable to read *num-ib-windows* property\n");
		return ret;
	}

	if (rockchip->num_ib_windows > MAX_IATU_IN) {
		dev_err(dev, "Invalid *num-ib-windows*\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "num-ob-windows",
				   &rockchip->num_ob_windows);
	if (ret < 0) {
		dev_err(dev, "Unable to read *num-ob-windows* property\n");
		return ret;
	}

	if (rockchip->num_ob_windows > MAX_IATU_OUT) {
		dev_err(dev, "Invalid *num-ob-windows*\n");
		return -EINVAL;
	}

	rockchip->ib_window_map = devm_kcalloc(dev,
					BITS_TO_LONGS(rockchip->num_ib_windows),
					sizeof(long), GFP_KERNEL);
	if (!rockchip->ib_window_map)
		return -ENOMEM;

	rockchip->ob_window_map = devm_kcalloc(dev,
					BITS_TO_LONGS(rockchip->num_ob_windows),
					sizeof(long), GFP_KERNEL);
	if (!rockchip->ob_window_map)
		return -ENOMEM;

	addr = devm_kcalloc(dev, rockchip->num_ob_windows, sizeof(phys_addr_t),
			    GFP_KERNEL);
	if (!addr)
		return -ENOMEM;

	rockchip->outbound_addr = addr;

	mem = of_parse_phandle(np, "memory-region", 0);
	if (!mem) {
		dev_err(dev, "missing \"memory-region\" property\n");
		return -ENODEV;
	}

	ret = of_address_to_resource(mem, 0, &reg);
	if (ret < 0) {
		dev_err(dev, "missing \"reg\" property\n");
		return -ENODEV;
	}

	rockchip->ib_target_address = reg.start;
	rockchip->ib_target_size = resource_size(&reg);
	rockchip->ib_target_base = rockchip_pcie_map_kernel(reg.start,
						resource_size(&reg));

	return 0;
}

static void rockchip_pcie_enable_ltssm(struct rockchip_pcie *rockchip)
{
	/* Set ep mode */
	rockchip_pcie_writel_apb(rockchip, 0xf00000, 0x0);
	rockchip_pcie_writel_apb(rockchip, PCIE_CLIENT_ENABLE_LTSSM,
				 PCIE_CLIENT_GENERAL_CONTROL);
}

static int rockchip_pcie_link_up(struct dw_pcie *pci)
{
	struct rockchip_pcie *rockchip = to_rockchip_pcie(pci);
	u32 val = rockchip_pcie_readl_apb(rockchip, PCIE_CLIENT_LTSSM_STATUS);

	if ((val & (PCIE_RDLH_LINKUP | PCIE_SMLH_LINKUP)) == 0x30000)
		return 1;

	return 0;
}

static int rockchip_pcie_start_link(struct dw_pcie *pci)
{
	struct rockchip_pcie *rockchip = to_rockchip_pcie(pci);

	/* Reset device */
	gpiod_set_value_cansleep(rockchip->rst_gpio, 0);
	msleep(100);
	gpiod_set_value_cansleep(rockchip->rst_gpio, 1);

	rockchip_pcie_enable_ltssm(rockchip);

	return 0;
}

static int rockchip_pcie_phy_init(struct rockchip_pcie *rockchip)
{
	int ret;
	struct device *dev = rockchip->pci.dev;

	rockchip->phy = devm_phy_get(dev, "pcie-phy");
	if (IS_ERR(rockchip->phy)) {
		dev_err(dev, "missing phy\n");
		return PTR_ERR(rockchip->phy);
	}

	ret = phy_init(rockchip->phy);
	if (ret < 0)
		return ret;

	phy_power_on(rockchip->phy);

	return 0;
}

static void rockchip_pcie_phy_deinit(struct rockchip_pcie *rockchip)
{
	phy_exit(rockchip->phy);
	phy_power_off(rockchip->phy);
}

static int rockchip_pcie_reset_control_release(struct rockchip_pcie *rockchip)
{
	struct device *dev = rockchip->pci.dev;
	int ret;

	rockchip->rst = devm_reset_control_array_get_exclusive(dev);
	if (IS_ERR(rockchip->rst)) {
		dev_err(dev, "failed to get reset lines\n");
		return PTR_ERR(rockchip->rst);
	}

	ret = reset_control_deassert(rockchip->rst);

	return ret;
}

static int rockchip_pcie_clk_init(struct rockchip_pcie *rockchip)
{
	struct device *dev = rockchip->pci.dev;
	int ret;

	ret = devm_clk_bulk_get_all(dev, &rockchip->clks);
	if (ret < 0)
		return ret;

	rockchip->clk_cnt = ret;

	ret = clk_bulk_prepare_enable(rockchip->clk_cnt, rockchip->clks);
	if (ret)
		return ret;

	return 0;
}

static int rockchip_pci_find_resbar_capability(struct rockchip_pcie *rockchip)
{
	u32 header;
	int ttl;
	int start = 0;
	int pos = PCI_CFG_SPACE_SIZE;
	int cap = PCI_EXT_CAP_ID_REBAR;

	/* minimum 8 bytes per capability */
	ttl = (PCI_CFG_SPACE_EXP_SIZE - PCI_CFG_SPACE_SIZE) / 8;

	header = dw_pcie_readl_dbi(&rockchip->pci, pos);

	/*
	 * If we have no capabilities, this is indicated by cap ID,
	 * cap version and next pointer all being 0.
	 */
	if (header == 0)
		return 0;

	while (ttl-- > 0) {
		if (PCI_EXT_CAP_ID(header) == cap && pos != start)
			return pos;

		pos = PCI_EXT_CAP_NEXT(header);
		if (pos < PCI_CFG_SPACE_SIZE)
			break;

		header = dw_pcie_readl_dbi(&rockchip->pci, pos);
		if (!header)
			break;
	}

	return 0;
}

static int rockchip_pcie_ep_set_bar_flag(struct rockchip_pcie *rockchip, enum pci_barno barno, int flags)
{
	enum pci_barno bar = barno;
	u32 reg;

	reg = PCI_BASE_ADDRESS_0 + (4 * bar);

	/* Disabled the upper 32bits BAR to make a 64bits bar pair */
	if (flags & PCI_BASE_ADDRESS_MEM_TYPE_64) {
		dw_pcie_writel_dbi(&rockchip->pci, reg + PCIE_TYPE0_HDR_DBI2_OFFSET + 4, 0);
	}

	dw_pcie_writel_dbi(&rockchip->pci, reg, flags);
	if (flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
		dw_pcie_writel_dbi(&rockchip->pci, reg + 4, 0);

	return 0;
}

static void rockchip_pcie_resize_bar(struct rockchip_pcie *rockchip)
{
	struct dw_pcie *pci = &rockchip->pci;
	struct device *dev = pci->dev;
	struct device_node *np = dev->of_node;
	int bar, ret;
	u32 resbar_base, lanes, val;

	ret = of_property_read_u32(np, "num-lanes", &lanes);
	if (ret)
		lanes = 0;

	/* Set the number of lanes */
	val = dw_pcie_readl_dbi(pci, PCIE_PORT_LINK_CONTROL);
	val &= ~PORT_LINK_MODE_MASK;
	switch (lanes) {
	case 1:
		val |= PORT_LINK_MODE_1_LANES;
		break;
	case 2:
		val |= PORT_LINK_MODE_2_LANES;
		break;
	case 4:
		val |= PORT_LINK_MODE_4_LANES;
		break;
	case 8:
		val |= PORT_LINK_MODE_8_LANES;
		break;
	default:
		dev_err(dev, "num-lanes %u: invalid value\n", lanes);
		return;
	}

	dw_pcie_writel_dbi(pci, PCIE_PORT_LINK_CONTROL, val);

	/* Set link width speed control register */
	val = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val &= ~PORT_LOGIC_LINK_WIDTH_MASK;
	switch (lanes) {
	case 1:
		val |= PORT_LOGIC_LINK_WIDTH_1_LANES;
		break;
	case 2:
		val |= PORT_LOGIC_LINK_WIDTH_2_LANES;
		break;
	case 4:
		val |= PORT_LOGIC_LINK_WIDTH_4_LANES;
		break;
	case 8:
		val |= PORT_LOGIC_LINK_WIDTH_8_LANES;
		break;
	}

	val |= PCIE_DIRECT_SPEED_CHANGE;

	dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);

	/* Enable bus master and memory space */
	dw_pcie_writel_dbi(pci, PCIE_TYPE0_STATUS_COMMAND_REG, 0x6);

	resbar_base = rockchip_pci_find_resbar_capability(rockchip);

	/* Resize BAR0~1 to support 4M 32bits, BAR2~5 to support 64M 64bits-pref */
	dw_pcie_writel_dbi(pci, resbar_base + 0x4, 0xfffff0);
	dw_pcie_writel_dbi(pci, resbar_base + 0x8, 0x2c0);
	dw_pcie_writel_dbi(pci, resbar_base + 0xc, 0xfffff0);
	dw_pcie_writel_dbi(pci, resbar_base + 0x10, 0x2c0);
	for (bar = 2; bar < 6; bar++) {
		dw_pcie_writel_dbi(pci, resbar_base + 0x4 + bar * 0x8, 0xfffff0);
		dw_pcie_writel_dbi(pci, resbar_base + 0x8 + bar * 0x8, 0x6c0);
	}

	/* Set flags */
	rockchip_pcie_ep_set_bar_flag(rockchip, BAR_0, PCI_BASE_ADDRESS_MEM_TYPE_32);
	rockchip_pcie_ep_set_bar_flag(rockchip, BAR_1, PCI_BASE_ADDRESS_MEM_TYPE_32);
	rockchip_pcie_ep_set_bar_flag(rockchip, BAR_2, PCI_BASE_ADDRESS_MEM_PREFETCH | PCI_BASE_ADDRESS_MEM_TYPE_64);
	rockchip_pcie_ep_set_bar_flag(rockchip, BAR_4, PCI_BASE_ADDRESS_MEM_PREFETCH | PCI_BASE_ADDRESS_MEM_TYPE_64);
}

static void rockchip_pcie_init_id(struct rockchip_pcie *rockchip)
{
	struct dw_pcie *pci = &rockchip->pci;

	dw_pcie_writew_dbi(pci, PCI_DEVICE_ID, 0x356a);
	dw_pcie_writew_dbi(pci, PCI_CLASS_DEVICE, 0x0580);
}

static int rockchip_pcie_ep_set_bar(struct rockchip_pcie *rockchip)
{
	int ret;
	u32 free_win;
	struct dw_pcie *pci = &rockchip->pci;
	enum pci_barno bar;
	enum dw_pcie_as_type as_type;
	dma_addr_t cpu_addr;

	free_win = find_first_zero_bit(rockchip->ib_window_map,
				       rockchip->num_ib_windows);
	if (free_win >= rockchip->num_ib_windows) {
		dev_err(pci->dev, "No free inbound window\n");
		return -EINVAL;
	}

	as_type = DW_PCIE_AS_MEM;
	bar = BAR_0;
	cpu_addr = rockchip->ib_target_address;

	ret = dw_pcie_prog_inbound_atu(pci, 0, free_win, bar, cpu_addr, as_type);
	if (ret < 0) {
		dev_err(pci->dev, "Failed to program IB window\n");
		return ret;
	}

	rockchip->bar_to_atu[bar] = free_win;
	set_bit(free_win, rockchip->ib_window_map);

	return 0;

}

static void rockchip_pcie_fast_link_setup(struct rockchip_pcie *rockchip)
{
	u32 val;

	/* LTSSM EN ctrl mode */
	val = rockchip_pcie_readl_apb(rockchip, PCIE_CLIENT_HOT_RESET_CTRL);
	val |= PCIE_LTSSM_ENABLE_ENHANCE | (PCIE_LTSSM_ENABLE_ENHANCE << 16);
	rockchip_pcie_writel_apb(rockchip, val, PCIE_CLIENT_HOT_RESET_CTRL);
}

static u8 rockchip_pcie_iatu_unroll_enabled(struct dw_pcie *pci)
{
	u32 val;

	val = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT);
	if (val == 0xffffffff)
		return 1;

	return 0;
}

static void rockchip_pcie_local_elbi_enable(struct rockchip_pcie *rockchip)
{
	int i;
	u32 dlbi_reg;
	struct dw_pcie *pci = &rockchip->pci;

	for (i = 0; i < PCIE_ELBI_REG_NUM; i++) {
		dlbi_reg = PCIE_ELBI_LOCAL_BASE + PCIE_ELBI_LOCAL_ENABLE_OFF +
			   i * 4;
		dw_pcie_writel_dbi(pci, dlbi_reg, 0xffff0000);
	}
}

static void rockchip_pcie_elbi_clear(struct rockchip_pcie *rockchip)
{
	int i;
	u32 dlbi_reg;
	struct dw_pcie *pci = &rockchip->pci;
	u32 val;

	for (i = 0; i < PCIE_ELBI_REG_NUM; i++) {
		dlbi_reg = PCIE_ELBI_LOCAL_BASE + i * 4;
		val = dw_pcie_readl_dbi(pci, dlbi_reg);
		val <<= 16;
		dw_pcie_writel_dbi(pci, dlbi_reg, val);
	}
}

static irqreturn_t rockchip_pcie_sys_irq_handler(int irq, void *arg)
{
	struct rockchip_pcie *rockchip = arg;
	struct dw_pcie *pci = &rockchip->pci;
	u32 dlbi_reg;
	u32 dlbi[2];
	int i;

	for (i = 0; i < PCIE_ELBI_REG_NUM; i++) {
		dlbi_reg = PCIE_ELBI_LOCAL_BASE + i * 4;
		dlbi[i] = dw_pcie_readl_dbi(pci, dlbi_reg);

		dev_info(rockchip->pci.dev, "dlbi[%d] = 0x%x\n", i, dlbi[i]);
	}

	rockchip_pcie_elbi_clear(rockchip);

	return IRQ_HANDLED;
}

static int rockchip_pcie_request_sys_irq(struct rockchip_pcie *rockchip,
					struct platform_device *pdev)
{
	int irq;
	int ret;
	struct device *dev = rockchip->pci.dev;

	irq = platform_get_irq_byname(pdev, "sys");
	if (irq < 0) {
		dev_err(dev, "missing sys IRQ resource\n");
		return -EINVAL;
	}

	ret = devm_request_irq(dev, irq, rockchip_pcie_sys_irq_handler,
			       IRQF_SHARED, "pcie-sys", rockchip);
	if (ret) {
		dev_err(dev, "failed to request PCIe subsystem IRQ\n");
		return ret;
	}

	return 0;
}


static const struct dw_pcie_ops dw_pcie_ops = {
	.start_link = rockchip_pcie_start_link,
	.link_up = rockchip_pcie_link_up,
};

static int rockchip_pcie_ep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rockchip_pcie *rockchip;
	int ret;
	int retry;

	rockchip = devm_kzalloc(dev, sizeof(*rockchip), GFP_KERNEL);
	if (!rockchip)
		return -ENOMEM;

	platform_set_drvdata(pdev, rockchip);

	rockchip->pci.dev = dev;
	rockchip->pci.ops = &dw_pcie_ops;

	ret = rockchip_pcie_resource_get(pdev, rockchip);
	if (ret)
		return ret;

	/* DON'T MOVE ME: must be enable before phy init */
	rockchip->vpcie3v3 = devm_regulator_get_optional(dev, "vpcie3v3");
	if (IS_ERR(rockchip->vpcie3v3)) {
		if (PTR_ERR(rockchip->vpcie3v3) != -ENODEV)
			return PTR_ERR(rockchip->vpcie3v3);
		dev_info(dev, "no vpcie3v3 regulator found\n");
	}

	if (!IS_ERR(rockchip->vpcie3v3)) {
		ret = regulator_enable(rockchip->vpcie3v3);
		if (ret) {
			dev_err(dev, "fail to enable vpcie3v3 regulator\n");
			return ret;
		}
	}

	ret = rockchip_pcie_clk_init(rockchip);
	if (ret)
		goto disable_regulator;

	if (dw_pcie_link_up(&rockchip->pci)) {
		pr_info("%s, %d, already linkup\n", __func__, __LINE__);
		goto already_linkup;
	}

	ret = rockchip_pcie_phy_init(rockchip);
	if (ret)
		goto deinit_clk;

	ret = rockchip_pcie_reset_control_release(rockchip);
	if (ret)
		goto deinit_phy;

	dw_pcie_setup(&rockchip->pci);

	dw_pcie_dbi_ro_wr_en(&rockchip->pci);
	rockchip_pcie_resize_bar(rockchip);
	rockchip_pcie_init_id(rockchip);
	dw_pcie_dbi_ro_wr_dis(&rockchip->pci);

	rockchip_pcie_fast_link_setup(rockchip);

	rockchip_pcie_start_link(&rockchip->pci);

	for (retry = 0; retry < 10000; retry++) {
		if (dw_pcie_link_up(&rockchip->pci)) {
			/*
			 * We may be here in case of L0 in Gen1. But if EP is capable
			 * of Gen2 or Gen3, Gen switch may happen just in this time, but
			 * we keep on accessing devices in unstable link status. Given
			 * that LTSSM max timeout is 24ms per period, we can wait a bit
			 * more for Gen switch.
			 */
			msleep(2000);
			dev_info(dev, "PCIe EP Link up, LTSSM is 0x%x\n",
				 rockchip_pcie_readl_apb(rockchip, PCIE_CLIENT_LTSSM_STATUS));
			break;
		}

		dev_info_ratelimited(dev, "PCIe EP Linking... LTSSM is 0x%x\n",
				     rockchip_pcie_readl_apb(rockchip, PCIE_CLIENT_LTSSM_STATUS));
		msleep(20);
	}

	if (retry >= 10000)
		goto deinit_phy;

already_linkup:
	rockchip->pci.iatu_unroll_enabled = rockchip_pcie_iatu_unroll_enabled(&rockchip->pci);
	rockchip_pcie_ep_set_bar(rockchip);

	/* Enable client ELBI interrupt */
	rockchip_pcie_writel_apb(rockchip, 0x80000000, PCIE_CLIENT_INTR_MASK);
	/* Enable ELBI interrupt */
	rockchip_pcie_local_elbi_enable(rockchip);

	ret = rockchip_pcie_request_sys_irq(rockchip, pdev);
	if (ret)
		goto deinit_phy;

	return 0;

deinit_phy:
	rockchip_pcie_phy_deinit(rockchip);
deinit_clk:
	clk_bulk_disable_unprepare(rockchip->clk_cnt, rockchip->clks);
disable_regulator:
	if (!IS_ERR(rockchip->vpcie3v3))
		regulator_disable(rockchip->vpcie3v3);

	return ret;
}

static struct platform_driver rk_plat_pcie_driver = {
	.driver = {
		.name	= "rk-pcie-ep",
		.of_match_table = rk_pcie_ep_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = rockchip_pcie_ep_probe,
};

// module_platform_driver_probe(rk_plat_pcie_driver,
// 					 rockchip_pcie_ep_probe);

module_platform_driver(rk_plat_pcie_driver);

MODULE_AUTHOR("Simon Xue <xxm@rock-chips.com>");
MODULE_DESCRIPTION("RockChip PCIe Controller EP driver");
MODULE_LICENSE("GPL v2");
