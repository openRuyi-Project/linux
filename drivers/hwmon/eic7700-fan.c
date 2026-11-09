// SPDX-License-Identifier: GPL-2.0
/*
 * ESWIN EIC7700 fan tachometer driver
 *
 * The fan controller measures the fan tachometer period in APB clock
 * cycles.
 *
 * Based on the ESWIN vendor driver, Copyright 2022 ESWIN Inc.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>

#define EIC7700_FAN_INT			0x0
#define EIC7700_FAN_INT_EN		BIT(0)
#define EIC7700_FAN_INT_STATUS		BIT(1)
#define EIC7700_FAN_INT_CLR		BIT(2)
#define EIC7700_FAN_INT_BLADE_LOCK	BIT(3)
#define EIC7700_FAN_RPM			0x4

#define EIC7700_FAN_TIMEOUT		msecs_to_jiffies(1500)

struct eic7700_fan {
	void __iomem *base;
	struct clk *clk;
	int irq;
	u32 ppr;
	/* serializes speed measurements */
	struct mutex lock;
	struct completion done;
	u32 cycles;
};

static irqreturn_t eic7700_fan_irq(int irq, void *data)
{
	struct eic7700_fan *fan = data;
	u32 status;

	status = readl(fan->base + EIC7700_FAN_INT);
	if (!(status & EIC7700_FAN_INT_STATUS))
		return IRQ_NONE;

	fan->cycles = readl(fan->base + EIC7700_FAN_RPM);
	if (status & EIC7700_FAN_INT_BLADE_LOCK)
		fan->cycles = 0;

	/* Stop the measurement and clear the interrupt */
	writel(EIC7700_FAN_INT_CLR, fan->base + EIC7700_FAN_INT);
	complete(&fan->done);

	return IRQ_HANDLED;
}

static int eic7700_fan_read_rpm(struct eic7700_fan *fan, long *val)
{
	unsigned long rate;

	guard(mutex)(&fan->lock);

	rate = clk_get_rate(fan->clk);
	if (!rate)
		return -EINVAL;

	reinit_completion(&fan->done);
	writel(EIC7700_FAN_INT_EN, fan->base + EIC7700_FAN_INT);

	/* No tachometer pulses at all means the fan is not spinning */
	if (!wait_for_completion_timeout(&fan->done, EIC7700_FAN_TIMEOUT)) {
		writel(EIC7700_FAN_INT_CLR, fan->base + EIC7700_FAN_INT);
		/* Drain a late interrupt before arming the next measurement. */
		readl(fan->base + EIC7700_FAN_INT);
		synchronize_irq(fan->irq);
		*val = 0;
		return 0;
	}

	if (!fan->cycles) {
		*val = 0;
	} else {
		u64 div = (u64)fan->ppr * fan->cycles;

		*val = div64_u64(60ULL * rate + div / 2, div);
	}

	return 0;
}

static int eic7700_fan_read(struct device *dev, enum hwmon_sensor_types type,
			    u32 attr, int channel, long *val)
{
	struct eic7700_fan *fan = dev_get_drvdata(dev);

	if (type != hwmon_fan || attr != hwmon_fan_input)
		return -EOPNOTSUPP;

	return eic7700_fan_read_rpm(fan, val);
}

static umode_t eic7700_fan_is_visible(const void *data,
				      enum hwmon_sensor_types type,
				      u32 attr, int channel)
{
	return type == hwmon_fan && attr == hwmon_fan_input ? 0444 : 0;
}

static const struct hwmon_channel_info * const eic7700_fan_info[] = {
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT),
	NULL
};

static const struct hwmon_ops eic7700_fan_hwmon_ops = {
	.is_visible = eic7700_fan_is_visible,
	.read = eic7700_fan_read,
};

static const struct hwmon_chip_info eic7700_fan_chip_info = {
	.ops = &eic7700_fan_hwmon_ops,
	.info = eic7700_fan_info,
};

static int eic7700_fan_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *rst;
	struct device *hwmon;
	struct eic7700_fan *fan;
	int ret;

	fan = devm_kzalloc(dev, sizeof(*fan), GFP_KERNEL);
	if (!fan)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &fan->lock);
	if (ret)
		return ret;
	init_completion(&fan->done);

	fan->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fan->base))
		return PTR_ERR(fan->base);

	ret = device_property_read_u32(dev, "pulses-per-revolution", &fan->ppr);
	if (ret)
		return dev_err_probe(dev, ret, "missing pulses-per-revolution\n");
	if (fan->ppr != 1 && fan->ppr != 2 && fan->ppr != 4)
		return dev_err_probe(dev, -EINVAL,
				     "invalid pulses-per-revolution %u\n", fan->ppr);

	fan->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(fan->clk))
		return dev_err_probe(dev, PTR_ERR(fan->clk), "failed to get clock\n");

	rst = devm_reset_control_get_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "failed to get reset\n");

	fan->irq = platform_get_irq(pdev, 0);
	if (fan->irq < 0)
		return fan->irq;

	/* Make sure no measurement is pending before requesting the IRQ */
	writel(EIC7700_FAN_INT_CLR, fan->base + EIC7700_FAN_INT);

	ret = devm_request_irq(dev, fan->irq, eic7700_fan_irq, 0, dev_name(dev), fan);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	hwmon = devm_hwmon_device_register_with_info(dev, "eic7700_fan", fan,
						     &eic7700_fan_chip_info,
						     NULL);
	return PTR_ERR_OR_ZERO(hwmon);
}

static const struct of_device_id eic7700_fan_of_match[] = {
	{ .compatible = "eswin,eic7700-fan-control" },
	{ }
};
MODULE_DEVICE_TABLE(of, eic7700_fan_of_match);

static struct platform_driver eic7700_fan_driver = {
	.driver = {
		.name = "eic7700-fan",
		.of_match_table = eic7700_fan_of_match,
	},
	.probe = eic7700_fan_probe,
};
module_platform_driver(eic7700_fan_driver);

MODULE_AUTHOR("Han Gao <gaohan@iscas.ac.cn>");
MODULE_DESCRIPTION("ESWIN EIC7700 fan tachometer driver");
MODULE_LICENSE("GPL");
