// SPDX-License-Identifier: GPL-2.0-only
/* Zhihe A210 application processor to E902 mailbox */
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

/* The local mailbox holds the INFO bank and, one page up, the AP interrupt. */
#define A210_MBOX_STATUS		0x1000
#define A210_MBOX_CLEAR		0x1004
#define A210_MBOX_MASK		0x100c
#define A210_MBOX_GENERATE	0x10
#define A210_MBOX_INFO(n)	(0x14 + 4 * (n))
#define A210_MBOX_E902_PENDING	BIT(0)
#define A210_MBOX_NOTIFY		BIT(6)

/* Channel 0 carries AP requests and completions, channel 1 E902 notifications. */
#define A210_MBOX_CHANS		2

struct a210_mbox {
	struct mbox_controller controller;
	struct mbox_chan channels[A210_MBOX_CHANS];
	void __iomem *local;
	void __iomem *remote;
	/* Protects the shared interrupt mask and the set of started channels. */
	spinlock_t lock;
	unsigned long started;
	int irq;
};

static void a210_mbox_mask(struct a210_mbox *mbox, unsigned int index, bool enable)
{
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&mbox->lock, flags);
	if (enable)
		__set_bit(index, &mbox->started);
	else
		__clear_bit(index, &mbox->started);
	value = readl(mbox->local + A210_MBOX_MASK);
	if (mbox->started)
		value |= A210_MBOX_E902_PENDING;
	else
		value &= ~A210_MBOX_E902_PENDING;
	writel(value, mbox->local + A210_MBOX_MASK);
	spin_unlock_irqrestore(&mbox->lock, flags);
}

static irqreturn_t a210_mbox_irq(int irq, void *data)
{
	struct a210_mbox *mbox = data;

	if (!(readl(mbox->local + A210_MBOX_MASK) & A210_MBOX_E902_PENDING))
		return IRQ_NONE;
	if (!(readl(mbox->local + A210_MBOX_STATUS) & A210_MBOX_E902_PENDING))
		return IRQ_NONE;

	writel(A210_MBOX_E902_PENDING, mbox->local + A210_MBOX_CLEAR);

	/* E902 sets INFO<n> before ringing the shared doorbell for channel n. */
	for (unsigned int i = 0; i < A210_MBOX_CHANS; i++) {
		if (!readl(mbox->local + A210_MBOX_INFO(i)))
			continue;
		writel(0, mbox->local + A210_MBOX_INFO(i));
		if (test_bit(i, &mbox->started))
			mbox_chan_received_data(&mbox->channels[i], NULL);
	}
	return IRQ_HANDLED;
}

static unsigned int a210_mbox_index(struct a210_mbox *mbox, struct mbox_chan *channel)
{
	return channel - mbox->channels;
}

static int a210_mbox_send(struct mbox_chan *channel, void *data)
{
	struct a210_mbox *mbox = channel->con_priv;

	/* E902 notifications are receive-only. */
	if (a210_mbox_index(mbox, channel))
		return -EINVAL;

	/* Payload and completion ownership belong to the shared-memory client. */
	writel(A210_MBOX_NOTIFY, mbox->remote + A210_MBOX_GENERATE);
	return 0;
}

static int a210_mbox_startup(struct mbox_chan *channel)
{
	struct a210_mbox *mbox = channel->con_priv;
	unsigned int index = a210_mbox_index(mbox, channel);

	writel(0, mbox->local + A210_MBOX_INFO(index));
	a210_mbox_mask(mbox, index, true);
	return 0;
}

static void a210_mbox_shutdown(struct mbox_chan *channel)
{
	struct a210_mbox *mbox = channel->con_priv;

	a210_mbox_mask(mbox, a210_mbox_index(mbox, channel), false);
	synchronize_irq(mbox->irq);
}

static const struct mbox_chan_ops a210_mbox_ops = {
	.send_data = a210_mbox_send,
	.startup = a210_mbox_startup,
	.shutdown = a210_mbox_shutdown,
};

static int a210_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct a210_mbox *mbox;
	int ret;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	mbox->local = devm_platform_ioremap_resource_byname(pdev, "local");
	if (IS_ERR(mbox->local))
		return PTR_ERR(mbox->local);
	mbox->remote = devm_platform_ioremap_resource_byname(pdev, "remote");
	if (IS_ERR(mbox->remote))
		return PTR_ERR(mbox->remote);
	mbox->irq = platform_get_irq(pdev, 0);
	if (mbox->irq < 0)
		return mbox->irq;

	spin_lock_init(&mbox->lock);
	a210_mbox_mask(mbox, 0, false);
	for (unsigned int i = 0; i < A210_MBOX_CHANS; i++)
		mbox->channels[i].con_priv = mbox;
	/*
	 * Neither txdone_irq nor txdone_poll is set: the client acknowledges
	 * completion after consuming shared memory.
	 */
	mbox->controller.dev = dev;
	mbox->controller.chans = mbox->channels;
	mbox->controller.num_chans = A210_MBOX_CHANS;
	mbox->controller.ops = &a210_mbox_ops;

	ret = devm_request_irq(dev, mbox->irq, a210_mbox_irq, 0, dev_name(dev),
			       mbox);
	if (ret)
		return ret;

	return devm_mbox_controller_register(dev, &mbox->controller);
}

static const struct of_device_id a210_mbox_match[] = {
	{ .compatible = "zhihe,a210-ap-mailbox" },
	{}
};
MODULE_DEVICE_TABLE(of, a210_mbox_match);

static struct platform_driver a210_mbox_driver = {
	.probe = a210_mbox_probe,
	.driver = {
		.name = "a210-mailbox",
		.of_match_table = a210_mbox_match,
		/* Running E902 firmware continuously accesses the remote bank. */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(a210_mbox_driver);

MODULE_AUTHOR("Han Gao <gaohan@iscas.ac.cn>");
MODULE_DESCRIPTION("Zhihe A210 AP to E902 mailbox");
MODULE_LICENSE("GPL");
