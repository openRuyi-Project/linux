// SPDX-License-Identifier: GPL-2.0
/*
 * ESWIN EIC7700 fan controller driver
 *
 * The fan controller measures the fan tachometer period in APB clock
 * cycles. The fan speed is controlled through an external PWM channel,
 * optionally as a thermal cooling device using "cooling-levels".
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
#include <linux/pwm.h>
#include <linux/reset.h>
#include <linux/thermal.h>

#define EIC7700_FAN_INT			0x0
#define EIC7700_FAN_INT_EN		BIT(0)
#define EIC7700_FAN_INT_STATUS		BIT(1)
#define EIC7700_FAN_INT_CLR		BIT(2)
#define EIC7700_FAN_INT_BLADE_LOCK	BIT(3)
#define EIC7700_FAN_RPM			0x4

#define EIC7700_FAN_TIMEOUT		msecs_to_jiffies(1500)
#define EIC7700_FAN_PWM_MIN		25
#define EIC7700_FAN_PWM_MAX		255

struct eic7700_fan {
	void __iomem *base;
	struct pwm_device *pwm;
	unsigned long clk_rate;
	u32 ppr;
	/* serializes speed measurements */
	struct mutex lock;
	/* serializes PWM updates from hwmon and thermal */
	struct mutex pwm_lock;
	u32 *levels;
	unsigned int max_state;
	unsigned long cur_state;
	struct completion done;
	u32 cycles;
	bool locked;
};

static irqreturn_t eic7700_fan_irq(int irq, void *data)
{
	struct eic7700_fan *fan = data;
	u32 status;

	status = readl(fan->base + EIC7700_FAN_INT);
	if (!(status & EIC7700_FAN_INT_STATUS))
		return IRQ_NONE;

	fan->locked = status & EIC7700_FAN_INT_BLADE_LOCK;
	fan->cycles = readl(fan->base + EIC7700_FAN_RPM);

	/* Stop the measurement and clear the interrupt */
	writel(EIC7700_FAN_INT_CLR, fan->base + EIC7700_FAN_INT);
	complete(&fan->done);

	return IRQ_HANDLED;
}

static int eic7700_fan_set_pwm(struct eic7700_fan *fan, unsigned long pwm)
{
	struct pwm_state state;

	lockdep_assert_held(&fan->pwm_lock);

	pwm_get_state(fan->pwm, &state);
	pwm_set_relative_duty_cycle(&state, pwm, EIC7700_FAN_PWM_MAX);
	state.enabled = true;

	return pwm_apply_might_sleep(fan->pwm, &state);
}

static void eic7700_fan_update_state(struct eic7700_fan *fan, unsigned long pwm)
{
	unsigned int i;

	if (!fan->levels)
		return;

	for (i = 0; i < fan->max_state; i++)
		if (pwm < fan->levels[i + 1])
			break;
	fan->cur_state = i;
}

static int eic7700_fan_read_rpm(struct eic7700_fan *fan, long *val)
{
	guard(mutex)(&fan->lock);

	reinit_completion(&fan->done);
	writel(EIC7700_FAN_INT_EN, fan->base + EIC7700_FAN_INT);

	/* No tachometer pulses at all means the fan is not spinning */
	if (!wait_for_completion_timeout(&fan->done, EIC7700_FAN_TIMEOUT)) {
		writel(EIC7700_FAN_INT_CLR, fan->base + EIC7700_FAN_INT);
		*val = 0;
		return 0;
	}

	if (fan->locked || !fan->cycles) {
		*val = 0;
	} else {
		u64 div = (u64)fan->ppr * fan->cycles;

		*val = div64_u64(60ULL * fan->clk_rate + div / 2, div);
	}

	return 0;
}

static int eic7700_fan_read(struct device *dev, enum hwmon_sensor_types type,
			    u32 attr, int channel, long *val)
{
	struct eic7700_fan *fan = dev_get_drvdata(dev);
	struct pwm_state state;

	switch (type) {
	case hwmon_fan:
		return eic7700_fan_read_rpm(fan, val);
	case hwmon_pwm:
		pwm_get_state(fan->pwm, &state);
		*val = pwm_get_relative_duty_cycle(&state, EIC7700_FAN_PWM_MAX);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int eic7700_fan_write(struct device *dev, enum hwmon_sensor_types type,
			     u32 attr, int channel, long val)
{
	struct eic7700_fan *fan = dev_get_drvdata(dev);
	int ret;

	if (type != hwmon_pwm)
		return -EOPNOTSUPP;

	if (val < EIC7700_FAN_PWM_MIN || val > EIC7700_FAN_PWM_MAX)
		return -EINVAL;

	guard(mutex)(&fan->pwm_lock);

	ret = eic7700_fan_set_pwm(fan, val);
	if (!ret)
		eic7700_fan_update_state(fan, val);

	return ret;
}

static umode_t eic7700_fan_is_visible(const void *data,
				      enum hwmon_sensor_types type,
				      u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		return 0444;
	case hwmon_pwm:
		return 0644;
	default:
		return 0;
	}
}

static const struct hwmon_channel_info * const eic7700_fan_info[] = {
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(pwm, HWMON_PWM_INPUT),
	NULL
};

static const struct hwmon_ops eic7700_fan_hwmon_ops = {
	.is_visible = eic7700_fan_is_visible,
	.read = eic7700_fan_read,
	.write = eic7700_fan_write,
};

static const struct hwmon_chip_info eic7700_fan_chip_info = {
	.ops = &eic7700_fan_hwmon_ops,
	.info = eic7700_fan_info,
};

static int eic7700_fan_get_max_state(struct thermal_cooling_device *cdev,
				     unsigned long *state)
{
	struct eic7700_fan *fan = cdev->devdata;

	*state = fan->max_state;

	return 0;
}

static int eic7700_fan_get_cur_state(struct thermal_cooling_device *cdev,
				     unsigned long *state)
{
	struct eic7700_fan *fan = cdev->devdata;

	*state = fan->cur_state;

	return 0;
}

static int eic7700_fan_set_cur_state(struct thermal_cooling_device *cdev,
				     unsigned long state)
{
	struct eic7700_fan *fan = cdev->devdata;
	int ret;

	if (state > fan->max_state)
		return -EINVAL;

	guard(mutex)(&fan->pwm_lock);

	if (state == fan->cur_state)
		return 0;

	ret = eic7700_fan_set_pwm(fan, fan->levels[state]);
	if (ret)
		return ret;

	fan->cur_state = state;

	return 0;
}

static const struct thermal_cooling_device_ops eic7700_fan_cooling_ops = {
	.get_max_state = eic7700_fan_get_max_state,
	.get_cur_state = eic7700_fan_get_cur_state,
	.set_cur_state = eic7700_fan_set_cur_state,
};

static int eic7700_fan_get_levels(struct device *dev, struct eic7700_fan *fan)
{
	int num, ret, i;

	if (!device_property_present(dev, "cooling-levels"))
		return 0;

	num = device_property_count_u32(dev, "cooling-levels");
	if (num <= 0)
		return dev_err_probe(dev, num ?: -EINVAL, "invalid cooling-levels\n");

	fan->levels = devm_kcalloc(dev, num, sizeof(*fan->levels), GFP_KERNEL);
	if (!fan->levels)
		return -ENOMEM;

	ret = device_property_read_u32_array(dev, "cooling-levels", fan->levels, num);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read cooling-levels\n");

	for (i = 0; i < num; i++) {
		if (fan->levels[i] > EIC7700_FAN_PWM_MAX ||
		    (i && fan->levels[i] < fan->levels[i - 1]))
			return dev_err_probe(dev, -EINVAL,
					     "cooling-levels must be ascending and at most %d\n",
					     EIC7700_FAN_PWM_MAX);
	}

	fan->max_state = num - 1;

	return 0;
}

static int eic7700_fan_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *rst;
	struct pwm_state state;
	struct device *hwmon;
	struct eic7700_fan *fan;
	struct clk *clk;
	int irq, ret;

	fan = devm_kzalloc(dev, sizeof(*fan), GFP_KERNEL);
	if (!fan)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &fan->lock);
	if (ret)
		return ret;
	ret = devm_mutex_init(dev, &fan->pwm_lock);
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

	ret = eic7700_fan_get_levels(dev, fan);
	if (ret)
		return ret;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get clock\n");

	fan->clk_rate = clk_get_rate(clk);
	if (!fan->clk_rate)
		return dev_err_probe(dev, -EINVAL, "invalid clock rate\n");

	rst = devm_reset_control_get_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "failed to get reset\n");

	fan->pwm = devm_pwm_get(dev, NULL);
	if (IS_ERR(fan->pwm))
		return dev_err_probe(dev, PTR_ERR(fan->pwm), "failed to get PWM\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* Make sure no measurement is pending before requesting the IRQ */
	writel(EIC7700_FAN_INT_CLR, fan->base + EIC7700_FAN_INT);

	ret = devm_request_irq(dev, irq, eic7700_fan_irq, 0, dev_name(dev), fan);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	/*
	 * Run the fan at its highest level until the thermal framework or
	 * userspace selects another one.
	 */
	pwm_init_state(fan->pwm, &state);
	pwm_set_relative_duty_cycle(&state,
				    fan->levels ? fan->levels[fan->max_state] :
						  EIC7700_FAN_PWM_MAX,
				    EIC7700_FAN_PWM_MAX);
	state.enabled = true;
	ret = pwm_apply_might_sleep(fan->pwm, &state);
	if (ret)
		return dev_err_probe(dev, ret, "failed to apply PWM state\n");
	fan->cur_state = fan->max_state;

	hwmon = devm_hwmon_device_register_with_info(dev, "eic7700_fan", fan,
						     &eic7700_fan_chip_info,
						     NULL);
	if (IS_ERR(hwmon))
		return dev_err_probe(dev, PTR_ERR(hwmon), "failed to register hwmon\n");

	if (fan->levels && IS_ENABLED(CONFIG_THERMAL)) {
		struct thermal_cooling_device *cdev;

		cdev = devm_thermal_of_cooling_device_register(dev, 0, "eic7700-fan",
							       fan,
							       &eic7700_fan_cooling_ops);
		if (IS_ERR(cdev))
			return dev_err_probe(dev, PTR_ERR(cdev),
					     "failed to register cooling device\n");
	}

	return 0;
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
MODULE_DESCRIPTION("ESWIN EIC7700 fan controller driver");
MODULE_LICENSE("GPL");
