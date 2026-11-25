// SPDX-License-Identifier: GPL-2.0-only
/*
 * SpacemiT K1/K3 timer clockevent driver
 *
 * Copyright (C) 2026 SpacemiT
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/limits.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

/* Use counter 0 and its first match comparator. */
#define TMR_CER			0x00
#define TMR_CMR			0x04
#define TMR_CCR			0x0c
#define TMR_MATCH		0x10
#define TMR_PRELOAD		0x40
#define TMR_PRELOAD_CTRL		0x50
#define TMR_IER			0x60
#define TMR_ICR			0x70
#define TMR_SR			0x80
#define TMR_COUNT		0x90

#define TMR_COUNTER_EN		BIT(0)
#define TMR_FREE_RUNNING		BIT(0)
#define TMR_CLOCK_MASK		GENMASK(1, 0)
#define TMR_MATCH0		BIT(0)
#define TMR_MATCH_MASK		GENMASK(2, 0)
/* Leave room for match and enable synchronization. */
#define TMR_MIN_DELTA		16
/* COUNT reaches the APB domain up to three counter clocks late. */
#define TMR_SYNC_CYCLES		3
/* The slowest counter clock is the 32.768 kHz clock. */
#define TMR_MIN_RATE		32768

struct spacemit_timer {
	void __iomem *base;
	struct device *dev;
	struct clock_event_device clkevt;
	raw_spinlock_t lock;
	unsigned int sync_us;
};

static struct spacemit_timer *to_spacemit_timer(struct clock_event_device *evt)
{
	return container_of(evt, struct spacemit_timer, clkevt);
}

static int spacemit_timer_write(struct spacemit_timer *timer, u32 reg, u32 val,
				u32 mask)
{
	u32 tmp;

	writel(val, timer->base + reg);
	return readl_poll_timeout_atomic(timer->base + reg, tmp,
					(tmp & mask) == (val & mask), 1,
					10 * timer->sync_us);
}

static int spacemit_timer_stop(struct spacemit_timer *timer)
{
	u32 val, prev;
	int ret, i;

	val = readl(timer->base + TMR_CER) & ~TMR_COUNTER_EN;
	ret = spacemit_timer_write(timer, TMR_CER, val, TMR_COUNTER_EN);
	if (ret)
		return ret;

	/*
	 * CER crosses into the counter clock domain. Its readback alone does
	 * not show that counting has stopped. COUNT is synchronized back to
	 * APB with a latency of TMR_SYNC_CYCLES. Wait for a stable count
	 * before changing the match or preload registers.
	 */
	prev = readl(timer->base + TMR_COUNT);
	for (i = 0; i < 10; i++) {
		udelay(timer->sync_us);
		val = readl(timer->base + TMR_COUNT);
		if (val == prev)
			return 0;
		prev = val;
	}

	return -ETIMEDOUT;
}

static int spacemit_timer_clear(struct spacemit_timer *timer)
{
	u32 val;

	/* ICR is write-only; completion is observed through SR. */
	writel(TMR_MATCH_MASK, timer->base + TMR_ICR);
	return readl_poll_timeout_atomic(timer->base + TMR_SR, val,
					!(val & TMR_MATCH_MASK), 1,
					10 * timer->sync_us);
}

static int spacemit_timer_quiesce(struct spacemit_timer *timer)
{
	int ret, err;

	ret = spacemit_timer_write(timer, TMR_IER, 0, TMR_MATCH_MASK);
	/* Still try to stop and acknowledge the timer after a failed write. */
	err = spacemit_timer_stop(timer);
	if (!ret)
		ret = err;
	err = spacemit_timer_clear(timer);

	return ret ?: err;
}

static int spacemit_timer_shutdown(struct clock_event_device *evt)
{
	struct spacemit_timer *timer = to_spacemit_timer(evt);
	unsigned long flags;
	int ret;

	raw_spin_lock_irqsave(&timer->lock, flags);
	ret = spacemit_timer_quiesce(timer);
	raw_spin_unlock_irqrestore(&timer->lock, flags);

	return ret;
}

static int spacemit_timer_next_event(unsigned long delta,
				     struct clock_event_device *evt)
{
	struct spacemit_timer *timer = to_spacemit_timer(evt);
	unsigned long flags;
	u32 val;
	int ret;

	raw_spin_lock_irqsave(&timer->lock, flags);

	ret = spacemit_timer_quiesce(timer);
	if (ret)
		goto out;

	ret = spacemit_timer_write(timer, TMR_MATCH, delta - 1, U32_MAX);
	if (ret)
		goto out;

	ret = spacemit_timer_write(timer, TMR_IER, TMR_MATCH0, TMR_MATCH_MASK);
	if (ret)
		goto out;

	/* Enabling the counter reloads the zero preload value. */
	val = readl(timer->base + TMR_CER) | TMR_COUNTER_EN;
	ret = spacemit_timer_write(timer, TMR_CER, val, TMR_COUNTER_EN);
out:
	if (ret) {
		/* Leave a failed programming attempt with interrupts masked. */
		writel(0, timer->base + TMR_IER);
		val = readl(timer->base + TMR_CER) & ~TMR_COUNTER_EN;
		writel(val, timer->base + TMR_CER);
	}
	raw_spin_unlock_irqrestore(&timer->lock, flags);

	return ret;
}

static irqreturn_t spacemit_timer_interrupt(int irq, void *data)
{
	struct spacemit_timer *timer = data;
	unsigned long flags;
	int ret;

	raw_spin_lock_irqsave(&timer->lock, flags);
	if (!(readl(timer->base + TMR_SR) & TMR_MATCH0)) {
		raw_spin_unlock_irqrestore(&timer->lock, flags);
		return IRQ_NONE;
	}

	ret = spacemit_timer_quiesce(timer);
	raw_spin_unlock_irqrestore(&timer->lock, flags);
	if (ret)
		dev_err_ratelimited(timer->dev, "Failed to acknowledge timer: %d\n", ret);

	timer->clkevt.event_handler(&timer->clkevt);
	return IRQ_HANDLED;
}

static int spacemit_timer_resume(struct clock_event_device *evt)
{
	struct spacemit_timer *timer = to_spacemit_timer(evt);
	unsigned long flags;
	u32 val;
	int ret;

	raw_spin_lock_irqsave(&timer->lock, flags);
	ret = spacemit_timer_quiesce(timer);
	if (ret)
		goto out;

	/* Select the functional clock and disable automatic match reload. */
	val = readl(timer->base + TMR_CCR) & ~TMR_CLOCK_MASK;
	ret = spacemit_timer_write(timer, TMR_CCR, val, TMR_CLOCK_MASK);
	if (ret)
		goto out;

	val = readl(timer->base + TMR_CMR) | TMR_FREE_RUNNING;
	ret = spacemit_timer_write(timer, TMR_CMR, val, TMR_FREE_RUNNING);
	if (ret)
		goto out;

	ret = spacemit_timer_write(timer, TMR_PRELOAD_CTRL, 0, GENMASK(2, 0));
	if (ret)
		goto out;

	ret = spacemit_timer_write(timer, TMR_PRELOAD, 0, U32_MAX);
out:
	raw_spin_unlock_irqrestore(&timer->lock, flags);
	return ret;
}

static int spacemit_timer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *reset;
	struct spacemit_timer *timer;
	struct clk *func, *bus;
	unsigned long rate;
	int irq, ret;

	timer = devm_kzalloc(dev, sizeof(*timer), GFP_KERNEL);
	if (!timer)
		return -ENOMEM;

	timer->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(timer->base))
		return PTR_ERR(timer->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* Keep both clocks enabled for the lifetime of the clockevent. */
	bus = devm_clk_get_enabled(dev, "bus");
	if (IS_ERR(bus))
		return dev_err_probe(dev, PTR_ERR(bus), "Failed to enable bus clock\n");

	func = devm_clk_get_enabled(dev, "func");
	if (IS_ERR(func))
		return dev_err_probe(dev, PTR_ERR(func), "Failed to enable functional clock\n");

	ret = devm_clk_rate_exclusive_get(dev, func);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to protect timer rate\n");

	/* The clockevent core takes the rate as a u32. */
	rate = clk_get_rate(func);
	if (rate < TMR_MIN_RATE || rate > U32_MAX)
		return dev_err_probe(dev, -EINVAL, "Invalid timer rate: %lu\n", rate);

	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "Failed to get reset\n");

	ret = reset_control_assert(reset);
	if (ret)
		return ret;
	ret = reset_control_deassert(reset);
	if (ret)
		return ret;

	timer->dev = dev;
	timer->sync_us = DIV_ROUND_UP(TMR_SYNC_CYCLES * USEC_PER_SEC, rate);
	raw_spin_lock_init(&timer->lock);
	timer->clkevt.name = dev_name(dev);
	timer->clkevt.rating = 200;
	timer->clkevt.features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_DYNIRQ;
	timer->clkevt.cpumask = cpu_possible_mask;
	timer->clkevt.irq = irq;
	timer->clkevt.set_next_event = spacemit_timer_next_event;
	timer->clkevt.set_state_shutdown = spacemit_timer_shutdown;
	timer->clkevt.set_state_oneshot = spacemit_timer_shutdown;
	timer->clkevt.tick_resume = spacemit_timer_resume;

	ret = spacemit_timer_resume(&timer->clkevt);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize timer\n");

	ret = devm_request_irq(dev, irq, spacemit_timer_interrupt, IRQF_TIMER,
			       dev_name(dev), timer);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request timer interrupt\n");

	clockevents_config_and_register(&timer->clkevt, rate, TMR_MIN_DELTA,
					U32_MAX - 1);
	return 0;
}

static const struct of_device_id spacemit_timer_match[] = {
	{ .compatible = "spacemit,k1-timer" },
	{ }
};

static struct platform_driver spacemit_timer_driver = {
	.probe = spacemit_timer_probe,
	.driver = {
		.name = "spacemit-timer",
		.of_match_table = spacemit_timer_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(spacemit_timer_driver);
