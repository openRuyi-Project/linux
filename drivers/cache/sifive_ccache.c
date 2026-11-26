// SPDX-License-Identifier: GPL-2.0
/*
 * SiFive composable cache controller Driver
 *
 * Copyright (C) 2018-2022 SiFive, Inc.
 *
 */

#define pr_fmt(fmt) "CCACHE: " fmt

#include <linux/align.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/device.h>
#include <linux/bitfield.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/sizes.h>
#include <asm/cacheflush.h>
#include <asm/cacheinfo.h>
#include <asm/dma-noncoherent.h>
#include <soc/sifive/sifive_ccache.h>

#define SIFIVE_CCACHE_DIRECCFIX_LOW 0x100
#define SIFIVE_CCACHE_DIRECCFIX_HIGH 0x104
#define SIFIVE_CCACHE_DIRECCFIX_COUNT 0x108

#define SIFIVE_CCACHE_DIRECCFAIL_LOW 0x120
#define SIFIVE_CCACHE_DIRECCFAIL_HIGH 0x124
#define SIFIVE_CCACHE_DIRECCFAIL_COUNT 0x128

#define SIFIVE_CCACHE_DATECCFIX_LOW 0x140
#define SIFIVE_CCACHE_DATECCFIX_HIGH 0x144
#define SIFIVE_CCACHE_DATECCFIX_COUNT 0x148

#define SIFIVE_CCACHE_DATECCFAIL_LOW 0x160
#define SIFIVE_CCACHE_DATECCFAIL_HIGH 0x164
#define SIFIVE_CCACHE_DATECCFAIL_COUNT 0x168

#define SIFIVE_CCACHE_CONFIG 0x00
#define SIFIVE_CCACHE_CONFIG_BANK_MASK GENMASK_ULL(7, 0)
#define SIFIVE_CCACHE_CONFIG_WAYS_MASK GENMASK_ULL(15, 8)
#define SIFIVE_CCACHE_CONFIG_SETS_MASK GENMASK_ULL(23, 16)
#define SIFIVE_CCACHE_CONFIG_BLKS_MASK GENMASK_ULL(31, 24)

#define SIFIVE_CCACHE_FLUSH64 0x200
#define SIFIVE_CCACHE_FLUSH32 0x240

#define SIFIVE_CCACHE_WAYENABLE 0x08
#define SIFIVE_CCACHE_ECCINJECTERR 0x40

#define SIFIVE_CCACHE_MAX_ECCINTR 4
#define SIFIVE_CCACHE_LINE_SIZE 64

struct ccache_controller {
	void __iomem *base;
	struct device_node *node;
	int irq[SIFIVE_CCACHE_MAX_ECCINTR];
};

/* EIC7702 has one controller per die, indexed by die number. */
static struct ccache_controller ccache[2];
static unsigned int ccache_count;
static struct riscv_cacheinfo_ops ccache_cache_ops;
static int level;

enum {
	DIR_CORR = 0,
	DATA_CORR,
	DATA_UNCORR,
	DIR_UNCORR,
};

enum {
	QUIRK_NONSTANDARD_CACHE_OPS	= BIT(0),
	QUIRK_BROKEN_DATA_UNCORR	= BIT(1),
	QUIRK_ENABLE_ALL_WAYS		= BIT(2),
};

#ifdef CONFIG_DEBUG_FS
static struct dentry *sifive_test;

static ssize_t ccache_write(struct file *file, const char __user *data,
			    size_t count, loff_t *ppos)
{
	unsigned int val;

	if (kstrtouint_from_user(data, count, 0, &val))
		return -EINVAL;
	if ((val < 0xFF) || (val >= 0x10000 && val < 0x100FF))
		writel(val, ccache[0].base + SIFIVE_CCACHE_ECCINJECTERR);
	else
		return -EINVAL;
	return count;
}

static const struct file_operations ccache_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = ccache_write
};

static void setup_sifive_debug(void)
{
	sifive_test = debugfs_create_dir("sifive_ccache_cache", NULL);

	debugfs_create_file("sifive_debug_inject_error", 0200,
			    sifive_test, NULL, &ccache_fops);
}
#endif

static void ccache_config_read(void)
{
	u32 cfg;

	cfg = readl(ccache[0].base + SIFIVE_CCACHE_CONFIG);
	pr_info("%llu banks, %llu ways, sets/bank=%llu, bytes/block=%llu\n",
		FIELD_GET(SIFIVE_CCACHE_CONFIG_BANK_MASK, cfg),
		FIELD_GET(SIFIVE_CCACHE_CONFIG_WAYS_MASK, cfg),
		BIT_ULL(FIELD_GET(SIFIVE_CCACHE_CONFIG_SETS_MASK, cfg)),
		BIT_ULL(FIELD_GET(SIFIVE_CCACHE_CONFIG_BLKS_MASK, cfg)));

	cfg = readl(ccache[0].base + SIFIVE_CCACHE_WAYENABLE);
	pr_info("Index of the largest way enabled: %u\n", cfg);
}

static int ccache_enable_all_ways(void __iomem *base)
{
	u32 config, ways;

	config = readl(base + SIFIVE_CCACHE_CONFIG);
	ways = FIELD_GET(SIFIVE_CCACHE_CONFIG_WAYS_MASK, config);
	if (!ways) {
		pr_err("Invalid cache configuration: zero ways\n");
		return -EINVAL;
	}

	writel(ways - 1, base + SIFIVE_CCACHE_WAYENABLE);

	return 0;
}

static const struct of_device_id sifive_ccache_ids[] = {
	{ .compatible = "eswin,eic7700-l3-cache",
	  .data = (void *)(QUIRK_NONSTANDARD_CACHE_OPS | QUIRK_ENABLE_ALL_WAYS) },
	{ .compatible = "sifive,fu540-c000-ccache" },
	{ .compatible = "sifive,fu740-c000-ccache" },
	{ .compatible = "starfive,jh7100-ccache",
	  .data = (void *)(QUIRK_NONSTANDARD_CACHE_OPS | QUIRK_BROKEN_DATA_UNCORR) },
	{ .compatible = "starfive,jh7110-ccache",
	  .data = (void *)(QUIRK_NONSTANDARD_CACHE_OPS) },
	{ .compatible = "sifive,ccache0" },
	{ /* end of table */ }
};

static ATOMIC_NOTIFIER_HEAD(ccache_err_chain);

int register_sifive_ccache_error_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&ccache_err_chain, nb);
}
EXPORT_SYMBOL_GPL(register_sifive_ccache_error_notifier);

int unregister_sifive_ccache_error_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&ccache_err_chain, nb);
}
EXPORT_SYMBOL_GPL(unregister_sifive_ccache_error_notifier);

#ifdef CONFIG_RISCV_NONSTANDARD_CACHE_OPS
/*
 * EIC7702 keeps each DDR line in the L3 of the die that owns it. Die 1
 * owns its direct window and, in the striped window, the 256-byte lines
 * where address bits 8 and 18 differ.
 */
#define EIC7702_DIE1_DIRECT_BASE	SZ_128G
#define EIC7702_DIE1_DIRECT_SIZE	SZ_64G
#define EIC7702_STRIPED_BASE		SZ_256G
#define EIC7702_STRIPED_SIZE		SZ_128G

static void __iomem *ccache_flush_base(phys_addr_t line)
{
	unsigned int die;

	if (ccache_count == 1)
		return ccache[0].base;

	if (line >= EIC7702_STRIPED_BASE &&
	    line < EIC7702_STRIPED_BASE + EIC7702_STRIPED_SIZE)
		die = ((line >> 8) ^ (line >> 18)) & 1;
	else
		die = line >= EIC7702_DIE1_DIRECT_BASE &&
		      line < EIC7702_DIE1_DIRECT_BASE + EIC7702_DIE1_DIRECT_SIZE;

	return ccache[die].base;
}

static void ccache_flush_range(phys_addr_t start, size_t len)
{
	phys_addr_t end = start + len;
	phys_addr_t line;

	if (!len)
		return;

	mb(); /* complete earlier memory accesses before the cache flush */
	for (line = ALIGN_DOWN(start, SIFIVE_CCACHE_LINE_SIZE); line < end;
			line += SIFIVE_CCACHE_LINE_SIZE) {
		void __iomem *base = ccache_flush_base(line);

#ifdef CONFIG_32BIT
		writel_relaxed(line >> 4, base + SIFIVE_CCACHE_FLUSH32);
#else
		writeq_relaxed(line, base + SIFIVE_CCACHE_FLUSH64);
#endif
	}
	mb(); /* issue later memory accesses after the cache flush */
}

static const struct riscv_nonstd_cache_ops ccache_mgmt_ops __initconst = {
	.wback = &ccache_flush_range,
	.inv = &ccache_flush_range,
	.wback_inv = &ccache_flush_range,
};
#endif /* CONFIG_RISCV_NONSTANDARD_CACHE_OPS */

static struct ccache_controller *ccache_from_node(const struct device_node *np)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(ccache); i++)
		if (ccache[i].base && ccache[i].node == np)
			return &ccache[i];

	return NULL;
}

static ssize_t number_of_ways_enabled_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct cacheinfo *leaf = dev_get_drvdata(dev);
	struct ccache_controller *cc = ccache_from_node(leaf->fw_token) ?: &ccache[0];

	return sysfs_emit(buf, "%u\n", readl(cc->base + SIFIVE_CCACHE_WAYENABLE) & 0xff);
}

static DEVICE_ATTR_RO(number_of_ways_enabled);

static struct attribute *priv_attrs[] = {
	&dev_attr_number_of_ways_enabled.attr,
	NULL,
};

static const struct attribute_group priv_attr_group = {
	.attrs = priv_attrs,
};

static const struct attribute_group *ccache_get_priv_group(struct cacheinfo
							   *this_leaf)
{
	/* We want to use private group for composable cache only */
	if (this_leaf->level == level)
		return &priv_attr_group;
	else
		return NULL;
}

static irqreturn_t ccache_int_handler(int irq, void *device)
{
	struct ccache_controller *cc = device;
	unsigned int add_h, add_l;

	if (irq == cc->irq[DIR_CORR]) {
		add_h = readl(cc->base + SIFIVE_CCACHE_DIRECCFIX_HIGH);
		add_l = readl(cc->base + SIFIVE_CCACHE_DIRECCFIX_LOW);
		pr_err("DirError @ 0x%08X.%08X\n", add_h, add_l);
		/* Reading this register clears the DirError interrupt sig */
		readl(cc->base + SIFIVE_CCACHE_DIRECCFIX_COUNT);
		atomic_notifier_call_chain(&ccache_err_chain,
					   SIFIVE_CCACHE_ERR_TYPE_CE,
					   "DirECCFix");
	}
	if (irq == cc->irq[DIR_UNCORR]) {
		add_h = readl(cc->base + SIFIVE_CCACHE_DIRECCFAIL_HIGH);
		add_l = readl(cc->base + SIFIVE_CCACHE_DIRECCFAIL_LOW);
		/* Reading this register clears the DirFail interrupt sig */
		readl(cc->base + SIFIVE_CCACHE_DIRECCFAIL_COUNT);
		atomic_notifier_call_chain(&ccache_err_chain,
					   SIFIVE_CCACHE_ERR_TYPE_UE,
					   "DirECCFail");
		panic("CCACHE: DirFail @ 0x%08X.%08X\n", add_h, add_l);
	}
	if (irq == cc->irq[DATA_CORR]) {
		add_h = readl(cc->base + SIFIVE_CCACHE_DATECCFIX_HIGH);
		add_l = readl(cc->base + SIFIVE_CCACHE_DATECCFIX_LOW);
		pr_err("DataError @ 0x%08X.%08X\n", add_h, add_l);
		/* Reading this register clears the DataError interrupt sig */
		readl(cc->base + SIFIVE_CCACHE_DATECCFIX_COUNT);
		atomic_notifier_call_chain(&ccache_err_chain,
					   SIFIVE_CCACHE_ERR_TYPE_CE,
					   "DatECCFix");
	}
	if (irq == cc->irq[DATA_UNCORR]) {
		add_h = readl(cc->base + SIFIVE_CCACHE_DATECCFAIL_HIGH);
		add_l = readl(cc->base + SIFIVE_CCACHE_DATECCFAIL_LOW);
		pr_err("DataFail @ 0x%08X.%08X\n", add_h, add_l);
		/* Reading this register clears the DataFail interrupt sig */
		readl(cc->base + SIFIVE_CCACHE_DATECCFAIL_COUNT);
		atomic_notifier_call_chain(&ccache_err_chain,
					   SIFIVE_CCACHE_ERR_TYPE_UE,
					   "DatECCFail");
	}

	return IRQ_HANDLED;
}

static int sifive_ccache_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ccache_controller *cc;
	unsigned long quirks;
	int intr_num, rc;

	cc = ccache_from_node(dev->of_node);
	if (!cc)
		return -ENODEV;

	quirks = (unsigned long)device_get_match_data(dev);

	intr_num = platform_irq_count(pdev);
	if (intr_num < 0)
		return intr_num;
	if (!intr_num || intr_num > ARRAY_SIZE(cc->irq))
		return dev_err_probe(dev, -EINVAL, "Invalid interrupt count: %d\n", intr_num);

	for (int i = 0; i < intr_num; i++) {
		if (i == DATA_UNCORR && (quirks & QUIRK_BROKEN_DATA_UNCORR))
			continue;

		cc->irq[i] = platform_get_irq(pdev, i);
		if (cc->irq[i] < 0)
			return cc->irq[i];

		rc = devm_request_irq(dev, cc->irq[i], ccache_int_handler, 0, "ccache_ecc", cc);
		if (rc)
			return dev_err_probe(dev, rc, "Could not request IRQ %d\n", cc->irq[i]);
	}

	return 0;
}

static struct platform_driver sifive_ccache_driver = {
	.probe	= sifive_ccache_probe,
	.driver	= {
		.name		= "sifive_ccache",
		.of_match_table	= sifive_ccache_ids,
	},
};

static int __init ccache_map(struct device_node *np, const struct resource *res,
			     unsigned long quirks, struct ccache_controller *cc)
{
	int rc;

	cc->base = ioremap(res->start, resource_size(res));
	if (!cc->base)
		return -ENOMEM;

	if ((quirks & QUIRK_ENABLE_ALL_WAYS) &&
	    !of_property_present(np, "memory-region")) {
		rc = ccache_enable_all_ways(cc->base);
		if (rc) {
			iounmap(cc->base);
			cc->base = NULL;
			return rc;
		}
	}

	cc->node = of_node_get(np);
	ccache_count++;

	return 0;
}

static void __init ccache_unmap_all(void)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(ccache); i++) {
		if (!ccache[i].base)
			continue;

		iounmap(ccache[i].base);
		ccache[i].base = NULL;
		of_node_put(ccache[i].node);
		ccache[i].node = NULL;
	}
	ccache_count = 0;
}

/* The die 1 register windows of EIC7702 sit 512 MiB above die 0. */
static int __init eic7702_map_caches(unsigned long quirks)
{
	struct device_node *np;
	struct resource res;
	unsigned int die;
	int rc;

	for_each_compatible_node(np, NULL, "eswin,eic7700-l3-cache") {
		if (!of_device_is_available(np))
			continue;

		rc = of_address_to_resource(np, 0, &res);
		if (rc)
			goto err_put;

		die = !!(res.start & SZ_512M);
		if (ccache[die].base) {
			rc = -EINVAL;
			goto err_put;
		}

		rc = ccache_map(np, &res, quirks, &ccache[die]);
		if (rc)
			goto err_put;
	}

	if (ccache_count != 2) {
		pr_err("EIC7702 needs the L3 cache of both dies\n");
		return -ENODEV;
	}

	return 0;

err_put:
	of_node_put(np);
	return rc;
}

static int __init sifive_ccache_init(void)
{
	struct device_node *np;
	struct resource res;
	const struct of_device_id *match;
	unsigned long quirks;
	int rc;

	for_each_matching_node_and_match(np, sifive_ccache_ids, &match)
		if (of_device_is_available(np))
			break;
	if (!np)
		return -ENODEV;

	quirks = (uintptr_t)match->data;

	if (of_property_read_u32(np, "cache-level", &level)) {
		rc = -ENOENT;
		goto err_node_put;
	}

	if (of_machine_is_compatible("eswin,eic7702")) {
		rc = eic7702_map_caches(quirks);
	} else {
		rc = of_address_to_resource(np, 0, &res);
		if (!rc)
			rc = ccache_map(np, &res, quirks, &ccache[0]);
	}
	if (rc)
		goto err_unmap;

	rc = platform_driver_register(&sifive_ccache_driver);
	if (rc)
		goto err_unmap;

#ifdef CONFIG_RISCV_NONSTANDARD_CACHE_OPS
	if (quirks & QUIRK_NONSTANDARD_CACHE_OPS) {
		riscv_cbom_block_size = SIFIVE_CCACHE_LINE_SIZE;
		riscv_noncoherent_supported();
		riscv_noncoherent_register_cache_ops(&ccache_mgmt_ops);
	}
#endif

	ccache_config_read();

	ccache_cache_ops.get_priv_group = ccache_get_priv_group;
	riscv_set_cacheinfo_ops(&ccache_cache_ops);

#ifdef CONFIG_DEBUG_FS
	setup_sifive_debug();
#endif

	of_node_put(np);
	return 0;

err_unmap:
	ccache_unmap_all();
err_node_put:
	of_node_put(np);
	return rc;
}

arch_initcall(sifive_ccache_init);
