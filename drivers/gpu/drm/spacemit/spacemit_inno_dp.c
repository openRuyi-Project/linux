// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025-2026 SpacemiT Co., Ltd.
 *
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/phy/phy-dp.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <drm/display/drm_dp.h>
#include <drm/display/drm_dp_aux_bus.h>
#include <drm/display/drm_dp_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include "spacemit_inno_dp.h"

#define SPACEMIT_DP_SWING_MAX  2
#define SPACEMIT_DP_PREEMP_MAX 2
#define SPACEMIT_DP_AUX_MAX_RETRIES 3
#define SPACEMIT_DP_AUX_POLL_US     100
#define SPACEMIT_DP_AUX_TIMEOUT_US  (200 * USEC_PER_MSEC)

/* Adjust requests followed in one training phase before it is given up. */
#define SPACEMIT_DP_TRAIN_MAX_TRIES 8

#define SPACEMIT_DP_SINK_READY_DELAY_MS 120
#define SPACEMIT_DP_SINK_READY_RETRIES 3

/*
 * With DP routed through a Type-C PD controller the HPD pin is virtual and
 * can re-assert seconds after a transmitter power-cycle. Asserted HPD
 * returns immediately; the full wait is paid only when no sink shows up.
 */
#define SPACEMIT_DP_HPD_POLL_US     1000
#define SPACEMIT_DP_HPD_DEBOUNCE_US (1000 * USEC_PER_MSEC)

enum spacemit_dp_link_rate {
	SPACEMIT_DP_LINK_RATE_1_62 = 1620000, /* 1.62 Gbps */
	SPACEMIT_DP_LINK_RATE_2_70 = 2700000, /* 2.70 Gbps */
	SPACEMIT_DP_LINK_RATE_5_40 = 5400000, /* 5.40 Gbps */
	SPACEMIT_DP_LINK_RATE_8_10 = 8100000, /* 8.10 Gbps */
};

/* The K3 source tops out at HBR2: the PHY's MPLL has no 8.1 Gbps config. */
#define SPACEMIT_DP_SOURCE_MAX_RATE	SPACEMIT_DP_LINK_RATE_5_40

enum spacemit_dp_lane_count {
	SPACEMIT_DP_LANE_1 = 1,
	SPACEMIT_DP_LANE_2 = 2,
	SPACEMIT_DP_LANE_4 = 4,
};

/* DP_VIDEO_MAPPING encoding of the one stream format produced: RGB, 8 bpc. */
#define SPACEMIT_DP_VIDEO_RGB_8BIT	1
#define SPACEMIT_DP_BPP			24

enum spacemit_dp_ref_clk {
	SPACEMIT_DP_REF_CLK_24M = 24000,
};

static const struct spacemit_dp_link_config {
	enum spacemit_dp_link_rate rate;
	enum spacemit_dp_lane_count lanes;
} spacemit_dp_link_priority_table[] = {
	{SPACEMIT_DP_LINK_RATE_1_62, SPACEMIT_DP_LANE_1}, /* 1.62 Gbps */
	{SPACEMIT_DP_LINK_RATE_2_70, SPACEMIT_DP_LANE_1}, /* 2.70 Gbps */
	{SPACEMIT_DP_LINK_RATE_1_62, SPACEMIT_DP_LANE_2}, /* 3.24 Gbps */

	{SPACEMIT_DP_LINK_RATE_2_70, SPACEMIT_DP_LANE_2}, /* 5.40 Gbps */
	{SPACEMIT_DP_LINK_RATE_1_62, SPACEMIT_DP_LANE_4}, /* 6.48 Gbps */

	{SPACEMIT_DP_LINK_RATE_2_70, SPACEMIT_DP_LANE_4}, /* 10.8 Gbps */
	{SPACEMIT_DP_LINK_RATE_5_40, SPACEMIT_DP_LANE_2}, /* 10.8 Gbps */

	{SPACEMIT_DP_LINK_RATE_5_40, SPACEMIT_DP_LANE_4}, /* 21.6 Gbps */
};

/**
 * struct spacemit_dp_dev - Per-instance state for an Innosilicon DP/eDP
 * controller
 * @dev: Underlying platform device.
 * @drm: Parent DRM device, set at component bind.
 * @encoder: Passive encoder anchoring possible_crtcs; atomic goes through
 * @bridge.
 * @bridge: DRM bridge implementing this controller, attached NO_CONNECTOR.
 * @connector: Connector built on top of @bridge by drm_bridge_connector_init().
 * @connector_status: Last status .detect() reported; the HPD interrupt
 *		      compares against it.
 * @regs: Regmap over the DP controller register block.
 * @aux: DPCD AUX channel, registered at bind.
 * @dpcd: Raw DPCD capability bytes.
 * @lane_count: Lane count chosen by link training (1, 2 or 4).
 * @link_rate: Link rate chosen by link training, in kbit/s.
 * @link: Link limits: the sink's capabilities, clamped to the PHY and
 *	narrowed after a failed training until the sink is replugged.
 * @link.revision: DPCD revision byte.
 * @link.enhanced_framing: Whether the sink advertises enhanced framing.
 * @link.max_rate: Maximum usable link rate, in kbit/s.
 * @link.max_num_lanes: Maximum usable lane count.
 * @pxclk: Pixel clock mux, parented onto @pll_clk at probe.
 * @pll_clk: The PHY's pixel PLL; clk_set_rate() programs it.
 * @phy: DP PHY handle.
 * @phy_powered: Controller-side mirror of the PHY framework's power refcount.
 * @edp_mode: True when this instance drives an embedded panel.
 * @component_added: True once the component is registered; for eDP that is
 *	when the panel has probed.
 * @mode_lock: Serialises detect, EDID read, enable and disable.
 * @modeset_retry_work: Deferred "link is bad" notification, see its handler.
 * @irq: Controller interrupt line.
 */
struct spacemit_dp_dev {
	struct device *dev;

	struct drm_device *drm;
	struct drm_encoder encoder;
	struct drm_bridge bridge;
	struct drm_connector *connector;

	enum drm_connector_status connector_status;

	struct regmap *regs;
	struct drm_dp_aux aux;

	u8 dpcd[DP_RECEIVER_CAP_SIZE];
	int lane_count;
	u32 link_rate;

	struct {
		u8 revision;
		u8 enhanced_framing;
		u32 max_rate;
		u32 max_num_lanes;
	} link;

	struct clk *pxclk;
	struct clk *pll_clk;
	struct phy *phy;
	bool phy_powered;

	bool edp_mode;
	bool component_added;
	struct mutex mode_lock;	/* detect, EDID read, enable, disable */

	struct work_struct modeset_retry_work;

	int irq;
};

/*
 * drm_connector_set_link_status_property() takes connection_mutex, which the
 * commit already holds: calling it from .atomic_enable deadlocks against
 * itself, hence the work item.
 */
static void spacemit_dp_modeset_retry_work(struct work_struct *work)
{
	struct spacemit_dp_dev *dp = container_of(work, struct spacemit_dp_dev,
						  modeset_retry_work);
	struct drm_connector *connector = dp->connector;

	if (!connector)
		return;

	drm_connector_set_link_status_property(connector,
					       DRM_MODE_LINK_STATUS_BAD);
	drm_kms_helper_connector_hotplug_event(connector);
}

/**
 * struct spacemit_dp_bridge_state - Per-state planned link configuration.
 * @base: Parent &struct drm_bridge_state.
 * @link_cfg_idx: Index into spacemit_dp_link_priority_table[] picked by
 *	->atomic_check(); -1 means ->atomic_enable() walks the table from the start.
 */
struct spacemit_dp_bridge_state {
	struct drm_bridge_state base;
	int link_cfg_idx;
};

static inline struct spacemit_dp_bridge_state *
to_spacemit_dp_bridge_state(struct drm_bridge_state *state)
{
	return container_of(state, struct spacemit_dp_bridge_state, base);
}

static const struct regmap_config spacemit_dp_regmap_config = {
	.name		= "dp",
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= 0x3ffc,	/* 0x4000-byte register window */
};

static u32 spacemit_dp_aux_get_cmd(struct drm_dp_aux_msg *msg)
{
	switch (msg->request & ~DP_AUX_I2C_MOT) {
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE:
	case DP_AUX_I2C_WRITE_STATUS_UPDATE:
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		return msg->request;
	default:
		return 0;
	}
}

static void spacemit_dp_aux_hw_reset(struct spacemit_dp_dev *dp)
{
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_AUX_RESET,
			  FIELD_PREP(DP_AUX_RESET, 0x1));
	usleep_range(2000, 2500);
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_AUX_RESET,
			  FIELD_PREP(DP_AUX_RESET, 0x0));
	usleep_range(2000, 2500);
	regmap_write(dp->regs, DP_GENERAL_INTERRUPT,
		     DP_AUX_REPLY_EVENT_INT_STA);
}

static bool spacemit_dp_dpcd_caps_valid(const u8 *dpcd)
{
	u8 max_bw = dpcd[DP_MAX_LINK_RATE];
	u8 max_lanes = dpcd[DP_MAX_LANE_COUNT] & DP_MAX_LANE_COUNT_MASK;

	if (!dpcd[DP_DPCD_REV])
		return false;

	switch (max_bw) {
	case DP_LINK_BW_1_62:
	case DP_LINK_BW_2_7:
	case DP_LINK_BW_5_4:
	case DP_LINK_BW_8_1:
		break;
	default:
		return false;
	}

	return max_lanes == 1 || max_lanes == 2 || max_lanes == 4;
}

static ssize_t spacemit_dp_aux_transfer(struct drm_dp_aux *aux,
					struct drm_dp_aux_msg *msg)
{
	int ret, i, retries = 0;

	struct spacemit_dp_dev *dp = container_of(aux, struct spacemit_dp_dev,
						  aux);
	u32 cmd, len, val, status;
	u32 data[4] = {0};
	u8 *buf = msg->buffer;
	bool is_read = (msg->request & DP_AUX_I2C_READ) ||
		((msg->request & DP_AUX_NATIVE_READ) == DP_AUX_NATIVE_READ);

	/*
	 * Nothing attached: skip the 200 ms timeout + retries, fail at once.
	 * Read the live HPD-in level; the HPD interrupt is pixel-clock gated.
	 */
	if (!dp->edp_mode &&
	    !regmap_test_bits(dp->regs, DP_HPD_INTERRUPT_STATUS, DP_HPD_IN_STATUS))
		return -ENXIO;

	if (msg->size > 16)
		return -EINVAL;

	cmd = spacemit_dp_aux_get_cmd(msg);

retry_eio:
	if (!is_read) {
		/* Pack bytes into 32-bit words (Little Endian packing) */
		for (i = 0; i < msg->size; i++)
			data[i / 4] |= (u32)buf[i] << ((i % 4) * 8);

		/* Write data to registers: DATA1(LSB)..DATA4(MSB) */
		regmap_write_bits(dp->regs, DP_AUX_DATA1_REG, DP_AUX_DATA1,
				  FIELD_PREP(DP_AUX_DATA1, data[0]));
		regmap_write_bits(dp->regs, DP_AUX_DATA2_REG, DP_AUX_DATA2,
				  FIELD_PREP(DP_AUX_DATA2, data[1]));
		regmap_write_bits(dp->regs, DP_AUX_DATA3_REG, DP_AUX_DATA3,
				  FIELD_PREP(DP_AUX_DATA3, data[2]));
		regmap_write_bits(dp->regs, DP_AUX_DATA4_REG, DP_AUX_DATA4,
				  FIELD_PREP(DP_AUX_DATA4, data[3]));
	}

	/* HW expects Length - 1 */
	len = msg->size > 0 ? msg->size - 1 : 0;

	regmap_write_bits(dp->regs, DP_AUX_CMD_REG, DP_AUX_LENGTH,
			  FIELD_PREP(DP_AUX_LENGTH, len));
	regmap_write_bits(dp->regs, DP_AUX_CMD_REG, DP_AUX_ADDR,
			  FIELD_PREP(DP_AUX_ADDR, msg->address));
	regmap_write_bits(dp->regs, DP_AUX_CMD_REG, DP_AUX_CMD_TYPE,
			  FIELD_PREP(DP_AUX_CMD_TYPE, cmd));

	/*
	 * A transfer that timed out may complete late and latch the reply
	 * event; start from a clean status so the poll below cannot see it.
	 */
	regmap_write(dp->regs, DP_GENERAL_INTERRUPT,
		     DP_AUX_REPLY_EVENT_INT_STA);

	regmap_write_bits(dp->regs, DP_AUX_START_REG, DP_AUX_START,
			  FIELD_PREP(DP_AUX_START, 0));
	regmap_write_bits(dp->regs, DP_AUX_START_REG, DP_AUX_START,
			  FIELD_PREP(DP_AUX_START, 1));

	ret = regmap_read_poll_timeout(dp->regs, DP_GENERAL_INTERRUPT, val,
				       val & DP_AUX_REPLY_EVENT_INT_STA,
				       SPACEMIT_DP_AUX_POLL_US,
				       SPACEMIT_DP_AUX_TIMEOUT_US);
	if (ret) {
		if (retries < SPACEMIT_DP_AUX_MAX_RETRIES) {
			retries++;
			spacemit_dp_aux_hw_reset(dp);
			dev_dbg(dp->dev,
				"AUX timeout, cmd=0x%x addr=0x%x size=%zu retry %d/%d\n",
				cmd, msg->address, msg->size, retries, SPACEMIT_DP_AUX_MAX_RETRIES);
			goto retry_eio;
		}

		dev_err_ratelimited(dp->dev,
				    "AUX transfer timeout, req: 0x%x, cmd: 0x%x, addr: 0x%x, size %zu\n",
				    msg->request, cmd, msg->address, msg->size);
		return ret;
	}

	regmap_read(dp->regs, DP_AUX_STS_REG, &status);
	status = FIELD_GET(DP_AUX_STATUS, status);

	/* Write 1 to clear. */
	regmap_write(dp->regs, DP_GENERAL_INTERRUPT,
		     DP_AUX_REPLY_EVENT_INT_STA);

	/* Map HW status (the AUX reply command byte) to DRM reply codes */
	switch (status) {
	case 0: /* ACK */
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;
		break;
	case 1: /* NACK */
		msg->reply = DP_AUX_NATIVE_REPLY_NACK;
		return 0; /* the AUX helpers retry on a NACK */
	case 2: /* DEFER */
		msg->reply = DP_AUX_NATIVE_REPLY_DEFER;
		return 0;
	case DP_AUX_I2C_REPLY_NACK: /* native ACK, I2C NACK */
		msg->reply = DP_AUX_I2C_REPLY_NACK;
		return 0;
	case DP_AUX_I2C_REPLY_DEFER: /* native ACK, I2C DEFER */
		msg->reply = DP_AUX_I2C_REPLY_DEFER;
		return 0;
	default:
		regmap_read(dp->regs, DP_AUX_STS_REG, &val);
		val = FIELD_GET(DP_AUX_REPLY_ERR_CODE, val);
		if (retries < SPACEMIT_DP_AUX_MAX_RETRIES) {
			retries++;
			dev_dbg(dp->dev,
				"AUX status, cmd=0x%x addr=0x%x size=%zu retry %d/%d\n",
				cmd, msg->address, msg->size, retries, SPACEMIT_DP_AUX_MAX_RETRIES);
			usleep_range(1000, 1100);
			goto retry_eio;
		}
		dev_err_ratelimited(dp->dev,
				    "AUX error: cmd=0x%x addr=0x%x size=%zu status=0x%x code=0x%x\n",
				    cmd, msg->address, msg->size, status, val);
		return -EIO;
	}

	if (is_read && msg->size > 0) {
		regmap_read(dp->regs, DP_AUX_DATA1_REG, &data[0]);
		data[0] = FIELD_GET(DP_AUX_DATA1, data[0]);
		regmap_read(dp->regs, DP_AUX_DATA2_REG, &data[1]);
		data[1] = FIELD_GET(DP_AUX_DATA2, data[1]);
		regmap_read(dp->regs, DP_AUX_DATA3_REG, &data[2]);
		data[2] = FIELD_GET(DP_AUX_DATA3, data[2]);
		regmap_read(dp->regs, DP_AUX_DATA4_REG, &data[3]);
		data[3] = FIELD_GET(DP_AUX_DATA4, data[3]);

		/* Unpack 32-bit words back to bytes */
		for (i = 0; i < msg->size; i++)
			buf[i] = (data[i / 4] >> ((i % 4) * 8)) & 0xFF;
	}

	return msg->size;
}

static int spacemit_dp_update_edp_config(struct spacemit_dp_dev *dp, bool enable)
{
	u8 value;
	int ret;

	ret = drm_dp_dpcd_read_byte(&dp->aux, DP_EDP_CONFIGURATION_SET, &value);
	if (ret < 0) {
		drm_dbg(dp->drm,
			"failed to read DP_EDP_CONFIGURATION_SET: %d\n", ret);
		return ret;
	}

	if (enable)
		value |= DP_ALTERNATE_SCRAMBLER_RESET_ENABLE;
	else
		value &= ~DP_ALTERNATE_SCRAMBLER_RESET_ENABLE;

	ret = drm_dp_dpcd_write_byte(&dp->aux, DP_EDP_CONFIGURATION_SET, value);
	if (ret < 0) {
		drm_dbg(dp->drm,
			"failed to write DP_EDP_CONFIGURATION_SET: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * Read the sink's DPCD link-layer capabilities. EDID is handled separately.
 * A sink that does not answer is reported here and given assumed limits, so
 * there is nothing left for a caller to handle.
 */
static void spacemit_dp_hw_read_sink_caps(struct spacemit_dp_dev *dp)
{
	ssize_t ret;
	int retry;

	for (retry = 0; retry < SPACEMIT_DP_SINK_READY_RETRIES; retry++) {
		if (retry) {
			scoped_guard(mutex, &dp->aux.hw_mutex)
				spacemit_dp_aux_hw_reset(dp);
			msleep(SPACEMIT_DP_SINK_READY_DELAY_MS);
		}

		ret = drm_dp_read_dpcd_caps(&dp->aux, dp->dpcd);
		if (ret < 0)
			continue;

		if (!spacemit_dp_dpcd_caps_valid(dp->dpcd)) {
			drm_dbg(dp->drm,
				"DPCD caps not ready: rev=0x%02x bw=0x%02x lanes=0x%02x\n",
				dp->dpcd[DP_DPCD_REV], dp->dpcd[DP_MAX_LINK_RATE],
				dp->dpcd[DP_MAX_LANE_COUNT]);
			ret = -EAGAIN;
			continue;
		}

		break;
	}

	if (ret < 0) {
		drm_warn_ratelimited(dp->drm,
				     "failed to read DPCD after %d retries: %zd\n",
				     SPACEMIT_DP_SINK_READY_RETRIES, ret);
		dp->link.revision = DP_DPCD_REV_14;
		dp->link.max_rate = SPACEMIT_DP_LINK_RATE_5_40;
		dp->link.max_num_lanes = SPACEMIT_DP_LANE_2;
		dp->link.enhanced_framing = 1;
		return;
	}

	dp->link.revision = dp->dpcd[DP_DPCD_REV];

	/* drm_dp_max_link_rate() is in 10 kbit/s; the source stops at HBR2. */
	dp->link.max_rate = min_t(u32, drm_dp_max_link_rate(dp->dpcd) * 10,
				  SPACEMIT_DP_SOURCE_MAX_RATE);
	dp->link.max_num_lanes = drm_dp_max_lane_count(dp->dpcd);
	dp->link.enhanced_framing = drm_dp_enhanced_frame_cap(dp->dpcd);

	drm_dbg(dp->drm,
		"DPCD: Rev %x.%x, MaxRate %d kbit/s, MaxLanes %d, EnhFrame %d\n",
		dp->link.revision >> 4, dp->link.revision & 0xF,
		dp->link.max_rate, dp->link.max_num_lanes,
		dp->link.enhanced_framing);
}

static int spacemit_dp_phy_set_lanes(struct spacemit_dp_dev *dp,
				     enum spacemit_dp_lane_count lanes)
{
	union phy_configure_opts opts = {
		.dp = {
			.lanes = lanes,
			.set_lanes = true,
		},
	};
	int ret;

	ret = phy_configure(dp->phy, &opts);
	if (ret)
		return ret;

	dp->lane_count = lanes;
	return 0;
}

static int spacemit_dp_phy_set_rate(struct spacemit_dp_dev *dp,
				    enum spacemit_dp_link_rate rate)
{
	union phy_configure_opts opts = {
		.dp = {
			/*
			 * enum value is in kbit/s; PHY framework wants Mbit/s.
			 */
			.link_rate = rate / 1000,
			.set_rate = true,
		},
	};
	int ret;

	ret = phy_configure(dp->phy, &opts);
	if (ret)
		return ret;

	dp->link_rate = rate;
	return 0;
}

/*
 * The link PLL rides phy_configure(); the pixel PLL is a clock the PHY
 * registered. Link first: that is the order the analog blocks come up in.
 */
static int spacemit_dp_set_plls(struct spacemit_dp_dev *dp,
				enum spacemit_dp_link_rate rate, u32 pclk_khz)
{
	int ret;

	ret = spacemit_dp_phy_set_rate(dp, rate);
	if (ret)
		return ret;

	return clk_set_rate(dp->pll_clk, (unsigned long)pclk_khz * 1000);
}

/*
 * phy_power_on/off() are refcounted, but this driver's bring-up is idempotent
 * rather than pair-balanced. Mirror the state so the refcount cannot go
 * negative.
 */
static int spacemit_dp_phy_on(struct spacemit_dp_dev *dp)
{
	int ret;

	if (dp->phy_powered)
		return 0;

	ret = phy_power_on(dp->phy);
	if (ret)
		return ret;

	dp->phy_powered = true;
	return 0;
}

static void spacemit_dp_phy_off(struct spacemit_dp_dev *dp)
{
	if (!dp->phy_powered)
		return;

	phy_power_off(dp->phy);
	dp->phy_powered = false;
}

/* Pulse the AUX reset so the next bring-up starts from a clean state. */
static void spacemit_dp_link_disable(struct spacemit_dp_dev *dp)
{
	regmap_write_bits(dp->regs, DP_VIDEO_VSAMPLE_REG,
			  DP_VIDEO_STREAM_ENABLE,
			  FIELD_PREP(DP_VIDEO_STREAM_ENABLE, 0));
	spacemit_dp_phy_off(dp);

	/* Not under a transfer: a userspace AUX client may be in one. */
	guard(mutex)(&dp->aux.hw_mutex);
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_AUX_RESET,
			  FIELD_PREP(DP_AUX_RESET, 0x1));
	usleep_range(5000, 8000);
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_AUX_RESET,
			  FIELD_PREP(DP_AUX_RESET, 0x0));
	usleep_range(2000, 4000);
}

static enum drm_connector_status
spacemit_dp_hw_detect_hpd(struct spacemit_dp_dev *dp)
{
	u32 hpd_status;
	enum drm_connector_status connector_status =
			connector_status_disconnected;

	regmap_read(dp->regs, DP_HPD_INTERRUPT_STATUS, &hpd_status);
	hpd_status = FIELD_GET(DP_HPD_IN_STATUS, hpd_status);
	if (hpd_status)
		connector_status = connector_status_connected;
	else
		connector_status = connector_status_disconnected;

	return connector_status;
}

/* Returns true when a latched HPD event was pending and got acked. */
static bool spacemit_dp_hw_clean_hpd(struct spacemit_dp_dev *dp)
{
	u32 plug_event, unplug_event;

	regmap_read(dp->regs, DP_HPD_INTERRUPT_STATUS, &plug_event);
	plug_event = FIELD_GET(DP_HOT_PLUG_EVENT, plug_event);
	regmap_read(dp->regs, DP_HPD_INTERRUPT_STATUS, &unplug_event);
	unplug_event = FIELD_GET(DP_HOT_UNPLUG_EVENT, unplug_event);

	if (plug_event)
		regmap_write(dp->regs, DP_HPD_INTERRUPT_STATUS,
			     FIELD_PREP(DP_HOT_PLUG_EVENT, 0x1));

	if (unplug_event)
		regmap_write(dp->regs, DP_HPD_INTERRUPT_STATUS,
			     FIELD_PREP(DP_HOT_UNPLUG_EVENT, 0x1));

	return plug_event || unplug_event;
}

static int spacemit_dp_set_training_pattern(struct spacemit_dp_dev *dp,
					    u8 pattern)
{
	u32 tps_sel;
	u8 dpcd_pattern = pattern;
	int ret;

	if (pattern != DP_TRAINING_PATTERN_DISABLE)
		dpcd_pattern |= DP_LINK_SCRAMBLING_DISABLE;

	switch (pattern) {
	case DP_TRAINING_PATTERN_DISABLE:
		tps_sel = 0;
		regmap_write_bits(dp->regs, DP_CORE_CONTROL,
				  DP_SCRAMBLER_DISABLE,
				  FIELD_PREP(DP_SCRAMBLER_DISABLE, 0));
		break;
	case DP_TRAINING_PATTERN_1:
		tps_sel = 1;
		regmap_write_bits(dp->regs, DP_CORE_CONTROL,
				  DP_SCRAMBLER_DISABLE,
				  FIELD_PREP(DP_SCRAMBLER_DISABLE, 1));
		break;
	case DP_TRAINING_PATTERN_2:
		tps_sel = 2;
		regmap_write_bits(dp->regs, DP_CORE_CONTROL,
				  DP_SCRAMBLER_DISABLE,
				  FIELD_PREP(DP_SCRAMBLER_DISABLE, 1));
		break;
	case DP_TRAINING_PATTERN_3:
		tps_sel = 3;
		regmap_write_bits(dp->regs, DP_CORE_CONTROL,
				  DP_SCRAMBLER_DISABLE,
				  FIELD_PREP(DP_SCRAMBLER_DISABLE, 1));
		break;
	default:
		drm_err(dp->drm, "Unsupported training pattern: 0x%x\n",
			pattern);
		return -EINVAL;
	}

	regmap_write_bits(dp->regs, DP_PHYIF_CTRL_ADDR, DP_TPS_SEL,
			  FIELD_PREP(DP_TPS_SEL, tps_sel));

	ret = drm_dp_dpcd_write_byte(&dp->aux, DP_TRAINING_PATTERN_SET,
				     dpcd_pattern);
	if (ret < 0) {
		drm_dbg(dp->drm, "failed to set DPCD training pattern: %d\n",
			ret);
		return ret;
	}

	return 0;
}

/*
 * The adjust request has to reach the source PHY, not just the sink's DPCD:
 * a transmitter left at its reset default trains any level > 0 with a bad eye.
 */
static int spacemit_dp_link_apply_adjust(struct spacemit_dp_dev *dp,
					 const u8 *link_status,
				    enum spacemit_dp_lane_count lanes,
				    u8 *training_set)
{
	union phy_configure_opts opts = {
		.dp = {
			.lanes = lanes,
			.set_voltages = true,
		},
	};
	int i, ret;

	for (i = 0; i < lanes; i++) {
		u8 v = drm_dp_get_adjust_request_voltage(link_status, i);
		u8 p = drm_dp_get_adjust_request_pre_emphasis(link_status, i);
		u8 v_bits;

		if (v >= SPACEMIT_DP_SWING_MAX)
			v = SPACEMIT_DP_SWING_MAX;
		if (p >= SPACEMIT_DP_PREEMP_MAX)
			p = SPACEMIT_DP_PREEMP_MAX;

		opts.dp.voltage[i] = v;
		opts.dp.pre[i] = p;

		v_bits = v;
		if (v == SPACEMIT_DP_SWING_MAX)
			v_bits |= DP_TRAIN_MAX_SWING_REACHED;
		if (p == SPACEMIT_DP_PREEMP_MAX)
			v_bits |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;

		training_set[i] = v_bits | (p << DP_TRAIN_PRE_EMPHASIS_SHIFT);
	}

	ret = phy_configure(dp->phy, &opts);
	if (ret)
		return ret;

	return drm_dp_dpcd_write_data(&dp->aux, DP_TRAINING_LANE0_SET,
				      training_set, lanes);
}

static int spacemit_dp_link_train_clock_recovery(struct spacemit_dp_dev *dp,
						 enum spacemit_dp_link_rate rate,
					    enum spacemit_dp_lane_count lanes)
{
	u8 link_status[DP_LINK_STATUS_SIZE] = {0};
	u8 training_set[4] = {0};
	int retries = 0;
	int ret;

	/* Level 0 on both ends: the PHY keeps what the last training left. */
	ret = spacemit_dp_link_apply_adjust(dp, link_status, lanes,
					    training_set);
	if (ret < 0)
		return ret;

	ret = spacemit_dp_set_training_pattern(dp, DP_TRAINING_PATTERN_1);
	if (ret < 0)
		goto err_end_training;

	while (retries < SPACEMIT_DP_TRAIN_MAX_TRIES) {
		drm_dp_link_train_clock_recovery_delay(&dp->aux, dp->dpcd);

		ret = drm_dp_dpcd_read_link_status(&dp->aux, link_status);
		if (ret < 0)
			goto err_end_training;

		if (drm_dp_clock_recovery_ok(link_status, lanes))
			return 0;

		ret = spacemit_dp_link_apply_adjust(dp, link_status, lanes,
						    training_set);
		if (ret)
			goto err_end_training;

		retries++;
	}

	drm_dbg(dp->drm, "link training: clock recovery failed\n");
	ret = -ETIMEDOUT;

err_end_training:
	spacemit_dp_set_training_pattern(dp, DP_TRAINING_PATTERN_DISABLE);
	return ret;
}

static int spacemit_dp_link_train_channel_eq(struct spacemit_dp_dev *dp,
					     enum spacemit_dp_link_rate rate,
					enum spacemit_dp_lane_count lanes)
{
	u8 link_status[DP_LINK_STATUS_SIZE];
	u8 training_set[4] = {0};
	u8 training_pattern = DP_TRAINING_PATTERN_2;
	int retries = 0;
	int ret;

	if (drm_dp_tps3_supported(dp->dpcd)) {
		training_pattern = DP_TRAINING_PATTERN_3;
		drm_dbg(dp->drm, "link training: using TPS3\n");
	} else {
		drm_dbg(dp->drm, "link training: using TPS2\n");
	}

	ret = spacemit_dp_set_training_pattern(dp, training_pattern);
	if (ret < 0)
		goto err_end_training;

	while (retries < SPACEMIT_DP_TRAIN_MAX_TRIES) {
		drm_dp_link_train_channel_eq_delay(&dp->aux, dp->dpcd);

		ret = drm_dp_dpcd_read_link_status(&dp->aux, link_status);
		if (ret < 0)
			goto err_end_training;

		if (drm_dp_channel_eq_ok(link_status, lanes))
			return spacemit_dp_set_training_pattern(dp,
					DP_TRAINING_PATTERN_DISABLE);

		ret = spacemit_dp_link_apply_adjust(dp, link_status, lanes,
						    training_set);
		if (ret)
			goto err_end_training;

		retries++;
	}

	drm_dbg(dp->drm, "link training: channel EQ failed\n");
	ret = -ETIMEDOUT;

err_end_training:
	spacemit_dp_set_training_pattern(dp, DP_TRAINING_PATTERN_DISABLE);
	return ret;
}

static int spacemit_dp_link_train(struct spacemit_dp_dev *dp, enum spacemit_dp_link_rate rate,
				  enum spacemit_dp_lane_count lanes)
{
	int ret;
	u8 link_config[1];
	u8 bw_code = drm_dp_link_rate_to_bw_code(rate / 10);

	/*
	 * A sink may act on the DP_LINK_BW_SET write, so set the lane count
	 * first.
	 */
	link_config[0] = lanes;
	if (dp->link.enhanced_framing)
		link_config[0] |= DP_LANE_COUNT_ENHANCED_FRAME_EN;
	regmap_write_bits(dp->regs, DP_CORE_CONTROL, DP_ENHANCE_FRAMING_EN,
			  FIELD_PREP(DP_ENHANCE_FRAMING_EN,
				     dp->link.enhanced_framing));

	ret = drm_dp_dpcd_write_byte(&dp->aux, DP_LANE_COUNT_SET,
				     link_config[0]);
	if (ret < 0) {
		drm_dbg(dp->drm, "failed to set DPCD lane count\n");
		return ret;
	}

	ret = drm_dp_dpcd_write_byte(&dp->aux, DP_LINK_BW_SET, bw_code);
	if (ret < 0) {
		drm_dbg(dp->drm, "failed to set DPCD link rate\n");
		return ret;
	}

	ret = spacemit_dp_link_train_clock_recovery(dp, rate, lanes);
	if (ret)
		return ret;

	return spacemit_dp_link_train_channel_eq(dp, rate, lanes);
}

static void spacemit_dp_hw_set_msa_and_enable_video(struct spacemit_dp_dev *dp,
						    const struct drm_display_mode *mode,
						    enum spacemit_dp_link_rate rate,
						    enum spacemit_dp_lane_count lanes)
{
	u64 hb_num;
	u32 link_rate;
	u32 fp; /* Pixel clock in MHz */
	u32 tu, tu_frac, tu_int, rd_thres;
	u32 hsync_len, hstart, vstart;

	/* mode->clock unit is kHz, fp unit is MHz */
	fp = mode->clock / 1000;

	if (fp == 0)
		fp = 1; /* Prevent division by zero */

	/*
	 * HBlank interval = (htotal - hactive) * (LinkSymbolClock / 4) /
	 * PixelClock, where LinkSymbolClock = LinkRate * 100 (1.62G -> 162MHz).
	 * rate is in kbit/s, so link_rate = rate / 10000 gives the 162, 270,
	 * 540 form used below.
	 */
	link_rate = rate / 10000;

	/* Multiply before dividing: hb_num = hblank * (link_rate / 4) / fp */
	hb_num = (u64)(mode->htotal - mode->hdisplay) * link_rate;
	do_div(hb_num, 4 * fp);

	/* Transfer unit: tu = fp * bpp * 640 / (8 * lanes * link_rate) */
	tu = div_u64((u64)fp * SPACEMIT_DP_BPP * 640, 8 * lanes * link_rate);
	tu_frac = tu % 10;
	tu_int  = tu / 10;

	/* FIFO read threshold. */
	if (tu_int < 6)
		rd_thres = 32;
	else if ((mode->htotal - mode->hdisplay) < 80)
		rd_thres = 12;
	else
		rd_thres = 16;

	drm_dbg(dp->drm,
		"MSA: %dx%d, Rate:%d kbit/s, Lanes:%d, BPP:%d, TU:%d.%d\n",
		mode->hdisplay, mode->vdisplay, rate, lanes, SPACEMIT_DP_BPP,
		tu_int, tu_frac);

	regmap_write_bits(dp->regs, DP_VIDEO_VSAMPLE_REG, DP_VIDEO_MAPPING,
			  FIELD_PREP(DP_VIDEO_MAPPING, SPACEMIT_DP_VIDEO_RGB_8BIT));

	/*
	 * The Saturn TMG raster is always positive-sync (hsp/vsp fixed to 1).
	 */
	regmap_write_bits(dp->regs, DP_VINPUT_POLARITY, DP_HSYNC_IN_POLARITY,
			  FIELD_PREP(DP_HSYNC_IN_POLARITY, 1));
	regmap_write_bits(dp->regs, DP_VINPUT_POLARITY, DP_VSYNC_IN_POLARITY,
			  FIELD_PREP(DP_VSYNC_IN_POLARITY, 1));

	/* Basic timing */
	regmap_write_bits(dp->regs, DP_VIDEO_CONFIG1, DP_HACTIVE,
			  FIELD_PREP(DP_HACTIVE, mode->hdisplay));
	regmap_write_bits(dp->regs, DP_VIDEO_CONFIG2, DP_VACTIVE,
			  FIELD_PREP(DP_VACTIVE, mode->vdisplay));
	regmap_write_bits(dp->regs, DP_VIDEO_CONFIG1, DP_HBLANK,
			  FIELD_PREP(DP_HBLANK, mode->htotal - mode->hdisplay));
	regmap_write_bits(dp->regs, DP_VIDEO_CONFIG2, DP_VBLANK,
			  FIELD_PREP(DP_VBLANK, mode->vtotal - mode->vdisplay));

	/* MSA start values count from the leading edge of sync. */
	hstart = mode->htotal - mode->hsync_start;
	vstart = mode->vtotal - mode->vsync_start;

	regmap_write_bits(dp->regs, DP_VIDEO_MSA1, DP_HSTART,
			  FIELD_PREP(DP_HSTART, hstart));
	regmap_write_bits(dp->regs, DP_VIDEO_MSA1, DP_VSTART,
			  FIELD_PREP(DP_VSTART, vstart));

	hsync_len = mode->hsync_end - mode->hsync_start;

	regmap_write_bits(dp->regs, DP_VIDEO_CONFIG3, DP_H_SYNC_WIDTH,
			  FIELD_PREP(DP_H_SYNC_WIDTH, hsync_len));
	regmap_write_bits(dp->regs, DP_VIDEO_CONFIG4, DP_V_SYNC_WIDTH,
			  FIELD_PREP(DP_V_SYNC_WIDTH, mode->vsync_end - mode->vsync_start));
	regmap_write_bits(dp->regs, DP_VIDEO_CONFIG3, DP_H_FRONT_PORCH,
			  FIELD_PREP(DP_H_FRONT_PORCH, mode->hsync_start - mode->hdisplay));
	regmap_write_bits(dp->regs, DP_VIDEO_CONFIG4, DP_V_FRONT_PORCH,
			  FIELD_PREP(DP_V_FRONT_PORCH, mode->vsync_start - mode->vdisplay));

	/* MSA and MISC */
	regmap_write_bits(dp->regs, DP_VIDEO_MSA2, DP_MISC0,
			  FIELD_PREP(DP_MISC0, DP_MSA_MISC_8_BPC));
	regmap_write_bits(dp->regs, DP_VIDEO_MSA3, DP_MISC1,
			  FIELD_PREP(DP_MISC1, 0));
	regmap_write_bits(dp->regs, DP_VIDEO_MSA3, DP_NVID,
			  FIELD_PREP(DP_NVID, 0));

	/* Link layer parameters */
	regmap_write_bits(dp->regs, DP_VIDEO_HBLANK_INTERVAL,
			  DP_HBLANK_INTERVAL,
			  FIELD_PREP(DP_HBLANK_INTERVAL, (u32)hb_num));
	regmap_write_bits(dp->regs, DP_VIDEO_CONFIG5, DP_AVERAGE_BYTES_PER_TU,
			  FIELD_PREP(DP_AVERAGE_BYTES_PER_TU, tu_int));
	regmap_write_bits(dp->regs, DP_VIDEO_CONFIG5,
			  DP_AVERAGE_BYTES_PER_TU_FRAC,
			  FIELD_PREP(DP_AVERAGE_BYTES_PER_TU_FRAC, tu_frac));
	regmap_write_bits(dp->regs, DP_VIDEO_CONFIG5, DP_INIT_THRESHOLD,
			  FIELD_PREP(DP_INIT_THRESHOLD, rd_thres));

	regmap_write_bits(dp->regs, DP_PHYIF_CTRL_ADDR, DP_PHY_SSC_DIS,
			  FIELD_PREP(DP_PHY_SSC_DIS, 1));
	regmap_write_bits(dp->regs, DP_CLK_INV_MUX, DP_REG_VID_CLK_SEL,
			  FIELD_PREP(DP_REG_VID_CLK_SEL, 0));
	regmap_write_bits(dp->regs, DP_VIDEO_COLORBAR_CFG, DP_VID_BIST_EN,
			  FIELD_PREP(DP_VID_BIST_EN, 0));

	drm_dbg(dp->drm, "enabling video stream\n");
	regmap_write_bits(dp->regs, DP_VIDEO_VSAMPLE_REG,
			  DP_VIDEO_STREAM_ENABLE,
			  FIELD_PREP(DP_VIDEO_STREAM_ENABLE, 1));
}

static void spacemit_dp_hw_disable(struct spacemit_dp_dev *dp)
{
	dev_dbg(dp->dev, "disabling video and PHY\n");

	regmap_write_bits(dp->regs, DP_VIDEO_VSAMPLE_REG,
			  DP_VIDEO_STREAM_ENABLE,
			  FIELD_PREP(DP_VIDEO_STREAM_ENABLE, 0));

	regmap_write_bits(dp->regs, DP_PHYIF_CTRL_ADDR, DP_XMIT_ENABLE,
			  FIELD_PREP(DP_XMIT_ENABLE, 0));

	regmap_write_bits(dp->regs, DP_ANA_TX_ENABLE, DP_ANA_TX_EN_LDO_D0,
			  FIELD_PREP(DP_ANA_TX_EN_LDO_D0, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_ENABLE, DP_ANA_TX_EN_LDO_D1,
			  FIELD_PREP(DP_ANA_TX_EN_LDO_D1, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_ENABLE, DP_ANA_TX_EN_LDO_D2,
			  FIELD_PREP(DP_ANA_TX_EN_LDO_D2, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_ENABLE, DP_ANA_TX_EN_LDO_D3,
			  FIELD_PREP(DP_ANA_TX_EN_LDO_D3, 0));

	regmap_write_bits(dp->regs, DP_ANA_TX_ENABLE, DP_ANA_TX_EN_DRV_D0,
			  FIELD_PREP(DP_ANA_TX_EN_DRV_D0, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_ENABLE, DP_ANA_TX_EN_DRV_D1,
			  FIELD_PREP(DP_ANA_TX_EN_DRV_D1, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_ENABLE, DP_ANA_TX_EN_DRV_D2,
			  FIELD_PREP(DP_ANA_TX_EN_DRV_D2, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_ENABLE, DP_ANA_TX_EN_DRV_D3,
			  FIELD_PREP(DP_ANA_TX_EN_DRV_D3, 0));

	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_EN_P2S_D0,
			  FIELD_PREP(DP_ANA_TX_EN_P2S_D0, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_EN_P2S_D1,
			  FIELD_PREP(DP_ANA_TX_EN_P2S_D1, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_EN_P2S_D2,
			  FIELD_PREP(DP_ANA_TX_EN_P2S_D2, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_EN_P2S_D3,
			  FIELD_PREP(DP_ANA_TX_EN_P2S_D3, 0));

	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL6, DP_ANA_BG_EN,
			  FIELD_PREP(DP_ANA_BG_EN, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL7, DP_ANA_BG_EN_RCAL,
			  FIELD_PREP(DP_ANA_BG_EN_RCAL, 0));

	/* Both PLLs. */
	regmap_write_bits(dp->regs, DP_MPLL_CTRL0, DP_ANA_MPLL_PD,
			  FIELD_PREP(DP_ANA_MPLL_PD, 1));
	regmap_write_bits(dp->regs, DP_PREPLL_CTRL0, DP_ANA_PREPLL_PD,
			  FIELD_PREP(DP_ANA_PREPLL_PD, 1));
}

/* Calculate required bandwidth in kbit/s (Pixel Clock * Bits Per Pixel) */
static u32 spacemit_dp_calc_required_bw(const struct drm_display_mode *mode,
					int bpp)
{
	return mode->clock * bpp;
}

/*
 * Calculate available link capacity in kbit/s (taking 8b/10b overhead into
 * account)
 */
static u32 spacemit_dp_calc_link_capacity(enum spacemit_dp_link_rate rate,
					  enum spacemit_dp_lane_count lanes)
{
	u32 rate_khz = rate;
	u32 lane_count = lanes;

	/* Capacity = Rate(kbit/s) * Lanes * 0.8 */
	return (rate_khz * lane_count * 8) / 10;
}

/*
 * Every configuration able to carry @req_bw failed to train. Limit the link
 * to the best configuration below it, so that mode_valid() prunes the mode
 * and userspace falls back to a smaller one.
 */
static bool spacemit_dp_link_narrow(struct spacemit_dp_dev *dp, u32 req_bw)
{
	const struct spacemit_dp_link_config *best = NULL;
	u32 best_capacity = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(spacemit_dp_link_priority_table); i++) {
		const struct spacemit_dp_link_config *cfg =
				&spacemit_dp_link_priority_table[i];
		u32 capacity;

		if (cfg->rate > dp->link.max_rate ||
		    cfg->lanes > dp->link.max_num_lanes)
			continue;

		capacity = spacemit_dp_calc_link_capacity(cfg->rate, cfg->lanes);
		if (capacity >= req_bw || capacity <= best_capacity)
			continue;

		best = cfg;
		best_capacity = capacity;
	}

	if (!best)
		return false;

	dp->link.max_rate = best->rate;
	dp->link.max_num_lanes = best->lanes;
	return true;
}

static enum drm_connector_status
spacemit_dp_bridge_detect(struct drm_bridge *bridge, struct drm_connector *connector)
{
	struct spacemit_dp_dev *dp = container_of(bridge,
						  struct spacemit_dp_dev,
						  bridge);
	enum drm_connector_status status;

	/*
	 * The eDP panel is fixed, and DP_HPD_IN_STATUS does not read
	 * meaningfully on that path.
	 */
	if (dp->edp_mode)
		return connector_status_connected;

	/*
	 * The HPD interrupt is gated by the DP pixel clock, which is off
	 * exactly when a plug must be caught. Read the level register live
	 * instead.
	 */
	guard(mutex)(&dp->mode_lock);
	status = spacemit_dp_hw_detect_hpd(dp);

	/*
	 * The bind-time cap read only covers a sink that was already there.
	 * Without a re-read here, link.max_rate stays 0 and every link config
	 * is filtered out.
	 */
	if (status == connector_status_connected &&
	    dp->connector_status != connector_status_connected)
		spacemit_dp_hw_read_sink_caps(dp);

	WRITE_ONCE(dp->connector_status, status);

	return status;
}

static const struct drm_edid *
spacemit_dp_bridge_edid_read(struct drm_bridge *bridge, struct drm_connector *connector)
{
	struct spacemit_dp_dev *dp = container_of(bridge,
						  struct spacemit_dp_dev,
						  bridge);
	const struct drm_edid *edid = NULL;
	int retry;

	scoped_guard(mutex, &dp->mode_lock) {
		for (retry = 0; retry < SPACEMIT_DP_SINK_READY_RETRIES; retry++) {
			if (retry) {
				scoped_guard(mutex, &dp->aux.hw_mutex)
					spacemit_dp_aux_hw_reset(dp);
				msleep(SPACEMIT_DP_SINK_READY_DELAY_MS);
			}

			edid = drm_edid_read_ddc(connector, &dp->aux.ddc);
			if (edid)
				break;
		}
	}

	if (!edid)
		drm_err_ratelimited(dp->drm, "failed to read EDID\n");

	return edid;
}

static enum drm_mode_status
spacemit_dp_bridge_mode_valid(struct drm_bridge *bridge,
			      const struct drm_display_info *info,
			 const struct drm_display_mode *mode)
{
	struct spacemit_dp_dev *dp = container_of(bridge,
						  struct spacemit_dp_dev,
						  bridge);
	unsigned int max_link_rate, max_lanes;
	u64 max_bw, req_bw;

	/*
	 * Prefer the link limits the driver maintains: they start as the
	 * sink's capabilities clamped to the PHY and are narrowed after a
	 * failed training. drm_dp_max_link_rate() is in 10 kbit/s.
	 */
	if (dp->link.max_rate && dp->link.max_num_lanes) {
		max_link_rate = dp->link.max_rate / 10;
		max_lanes = dp->link.max_num_lanes;
	} else {
		max_link_rate = min_t(unsigned int,
				      drm_dp_max_link_rate(dp->dpcd),
				      SPACEMIT_DP_SOURCE_MAX_RATE / 10);
		max_lanes = drm_dp_max_lane_count(dp->dpcd);
	}
	if (!max_link_rate || !max_lanes) {
		/* Sink not enumerated yet: accept and re-check at modeset. */
		return MODE_OK;
	}

	/*
	 * max_link_rate is per lane in 10 kbit/s. The x10 to reach kbit/s and
	 * the 8b/10b coding overhead cancel down to a plain "* 8".
	 */
	max_bw = (u64)max_link_rate * max_lanes * 8;

	/* Required bandwidth: pixel clock (kHz) * bits per pixel. */
	req_bw = (u64)mode->clock * SPACEMIT_DP_BPP;

	if (req_bw > max_bw) {
		drm_dbg(bridge->dev,
			"rejecting %dx%d@%d: %llu > %llu kbit/s (%u kbit/s x%u lanes)\n",
			mode->hdisplay, mode->vdisplay, drm_mode_vrefresh(mode),
			req_bw, max_bw, max_link_rate * 10, max_lanes);
		return MODE_CLOCK_HIGH;
	}

	/* The pixel clock is made by the PHY PLL, which cannot hit every rate. */
	if (clk_round_rate(dp->pll_clk, mode->clock * 1000UL) <= 0)
		return MODE_NOCLOCK;

	return MODE_OK;
}

static const struct drm_encoder_funcs spacemit_dp_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static void spacemit_dp_bridge_atomic_enable(struct drm_bridge *bridge,
					     struct drm_atomic_state *state)
{
	struct spacemit_dp_dev *dp = container_of(bridge,
						  struct spacemit_dp_dev,
						  bridge);
	struct drm_bridge_state *new_bridge_state =
		drm_atomic_get_new_bridge_state(state, bridge);
	struct spacemit_dp_bridge_state *st =
			to_spacemit_dp_bridge_state(new_bridge_state);
	struct drm_connector *connector =
		drm_atomic_get_new_connector_for_encoder(state, bridge->encoder);
	struct drm_connector_state *conn_state =
		drm_atomic_get_new_connector_state(state, connector);
	struct drm_crtc_state *crtc_state =
		drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	struct drm_display_mode *adjusted_mode = &crtc_state->adjusted_mode;
	const struct spacemit_dp_link_config *cfg;
	bool hpd_lost = false;
	bool trained;
	u32 req_bw;
	u32 hpd;
	int start;
	int i;

	guard(mutex)(&dp->mode_lock);

	/* Enable-time retry for the caps .detect() could not read. */
	if (!dp->link.max_rate || !dp->link.max_num_lanes)
		spacemit_dp_hw_read_sink_caps(dp);

	req_bw = spacemit_dp_calc_required_bw(adjusted_mode, SPACEMIT_DP_BPP);

	/*
	 * Start where atomic_check() left off, but keep the upgrade walk as
	 * fallback.
	 */
	start = st->link_cfg_idx >= 0 ? st->link_cfg_idx : 0;

	for (trained = false, i = start;
	     i < ARRAY_SIZE(spacemit_dp_link_priority_table); i++) {
		u32 capacity;

		cfg = &spacemit_dp_link_priority_table[i];

		/* Source and sink limits */
		if (cfg->rate > dp->link.max_rate || cfg->lanes > dp->link.max_num_lanes)
			continue;

		/* Bandwidth requirement */
		capacity = spacemit_dp_calc_link_capacity(cfg->rate,
							  cfg->lanes);
		if (capacity < req_bw)
			continue;

		drm_dbg(dp->drm, "trying link config: rate %d lanes %d (capacity %d, required %d)\n",
			cfg->rate, cfg->lanes, capacity, req_bw);

		spacemit_dp_link_disable(dp);

		/* Apply Hardware Settings */
		if (spacemit_dp_set_plls(dp, cfg->rate, adjusted_mode->clock))
			continue;

		if (spacemit_dp_phy_set_lanes(dp, cfg->lanes))
			continue;

		if (spacemit_dp_phy_on(dp))
			continue;

		/*
		 * Debounce: the transmitter was power-cycled just now and a
		 * sink may legitimately drop HPD across a link-down. eDP is
		 * hardwired and noisy.
		 */
		if (!dp->edp_mode &&
		    regmap_read_poll_timeout(dp->regs, DP_HPD_INTERRUPT_STATUS, hpd,
					     hpd & DP_HPD_IN_STATUS,
					     SPACEMIT_DP_HPD_POLL_US,
					     SPACEMIT_DP_HPD_DEBOUNCE_US)) {
			drm_dbg(dp->drm,
				"sink is not asserting HPD; not training\n");
			hpd_lost = true;
			break;
		}

		if (dp->edp_mode) {
			regmap_write_bits(dp->regs, DP_CORE_CONTROL,
					  DP_ENABLE_EDP,
					  FIELD_PREP(DP_ENABLE_EDP, 0x1));
			regmap_write_bits(dp->regs, DP_VIDEO_VSAMPLE_REG,
					  DP_STREAM_ENC_EN,
					  FIELD_PREP(DP_STREAM_ENC_EN, 0x1));

			/* Do not train a link the panel was not set up for. */
			if (spacemit_dp_update_edp_config(dp, true))
				continue;
		}

		/* Execute Link Training */
		if (spacemit_dp_link_train(dp, cfg->rate, cfg->lanes) == 0) {
			drm_dbg(dp->drm,
				"link training succeeded: rate %d lanes %d\n",
				cfg->rate, cfg->lanes);
			trained = true;
			break;
		}

		drm_dbg(dp->drm,
			"link training failed: rate %d lanes %d, trying next config\n",
			cfg->rate, cfg->lanes);
	}

	/*
	 * The DPU's pixel clock comes from the PHY PLL, so scanning out into an
	 * untrained link never reaches CFG_RDY and wedges the CRTC on
	 * flip_done.
	 */
	if (!trained) {
		/* A sink that went away is not a link that failed. */
		if (!hpd_lost)
			drm_err_ratelimited(dp->drm,
					    "no usable link config; leaving video disabled (%u kbit/s x%u lanes)\n",
					    dp->link.max_rate,
					    dp->link.max_num_lanes);

		/*
		 * A panel has no smaller mode to fall back to and a sink that
		 * dropped HPD was not trained at all: retry those as they are,
		 * with the capabilities read again. Otherwise retrying the
		 * same mode would fail the same way, so narrow the link, and
		 * give up once nothing is left to narrow.
		 */
		if (dp->edp_mode || hpd_lost) {
			dp->link.max_rate = 0;
			dp->link.max_num_lanes = 0;
		} else if (!spacemit_dp_link_narrow(dp, req_bw)) {
			dp->link.max_rate = 0;
			dp->link.max_num_lanes = 0;
			return;
		}

		/*
		 * Deferred: we are inside the commit and hold the modeset
		 * locks.
		 */
		schedule_work(&dp->modeset_retry_work);
		return;
	}

	spacemit_dp_hw_set_msa_and_enable_video(dp, adjusted_mode,
						dp->link_rate, dp->lane_count);
}

static void spacemit_dp_bridge_atomic_disable(struct drm_bridge *bridge,
					      struct drm_atomic_state *state)
{
	struct spacemit_dp_dev *dp = container_of(bridge,
						  struct spacemit_dp_dev,
						  bridge);

	guard(mutex)(&dp->mode_lock);

	spacemit_dp_link_disable(dp);
}

static int spacemit_dp_bridge_atomic_check(struct drm_bridge *bridge,
					   struct drm_bridge_state *bridge_state,
				      struct drm_crtc_state *crtc_state,
				      struct drm_connector_state *conn_state)
{
	struct spacemit_dp_dev *dp = container_of(bridge,
						  struct spacemit_dp_dev,
						  bridge);
	struct spacemit_dp_bridge_state *st =
			to_spacemit_dp_bridge_state(bridge_state);
	const struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	u32 req_bw;
	int i;

	req_bw = spacemit_dp_calc_required_bw(mode, SPACEMIT_DP_BPP);

	/* No caps yet: leave idx unset and let ->atomic_enable() retry. */
	if (!dp->link.max_rate || !dp->link.max_num_lanes) {
		st->link_cfg_idx = -1;
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(spacemit_dp_link_priority_table); i++) {
		const struct spacemit_dp_link_config *cfg =
				&spacemit_dp_link_priority_table[i];

		if (cfg->rate > dp->link.max_rate ||
		    cfg->lanes > dp->link.max_num_lanes)
			continue;
		if (spacemit_dp_calc_link_capacity(cfg->rate, cfg->lanes) < req_bw)
			continue;

		st->link_cfg_idx = i;
		return 0;
	}

	drm_dbg(bridge->dev,
		"no DP link cfg fits %dx%d@%d (need %u kbit/s; sink %u kbit/s x %u lanes)\n",
		mode->hdisplay, mode->vdisplay, drm_mode_vrefresh(mode),
		req_bw, dp->link.max_rate, dp->link.max_num_lanes);
	return -EINVAL;
}

static struct drm_bridge_state *
spacemit_dp_bridge_atomic_duplicate_state(struct drm_bridge *bridge)
{
	struct spacemit_dp_bridge_state *cur =
		to_spacemit_dp_bridge_state(drm_priv_to_bridge_state(bridge->base.state));
	struct spacemit_dp_bridge_state *new;

	new = kzalloc_obj(*new);
	if (!new)
		return NULL;

	__drm_atomic_helper_bridge_duplicate_state(bridge, &new->base);
	new->link_cfg_idx = cur->link_cfg_idx;
	return &new->base;
}

static void
spacemit_dp_bridge_atomic_destroy_state(struct drm_bridge *bridge,
					struct drm_bridge_state *state)
{
	kfree(to_spacemit_dp_bridge_state(state));
}

static u32 *spacemit_dp_bridge_atomic_get_output_bus_fmts(struct drm_bridge *bridge,
							  struct drm_bridge_state *bridge_state,
				struct drm_crtc_state *crtc_state,
				struct drm_connector_state *conn_state,
				unsigned int *num_output_fmts)
{
	u32 *output_fmts;

	output_fmts = kmalloc_obj(*output_fmts, GFP_KERNEL);
	if (!output_fmts) {
		*num_output_fmts = 0;
		return NULL;
	}

	output_fmts[0] = MEDIA_BUS_FMT_RGB888_1X24;
	*num_output_fmts = 1;
	return output_fmts;
}

static struct drm_bridge_state *
spacemit_dp_bridge_atomic_reset(struct drm_bridge *bridge)
{
	struct spacemit_dp_bridge_state *new;

	new = kzalloc_obj(*new);
	if (!new)
		return ERR_PTR(-ENOMEM);

	__drm_atomic_helper_bridge_reset(bridge, &new->base);
	new->link_cfg_idx = -1;
	return &new->base;
}

static int spacemit_dp_bridge_attach(struct drm_bridge *bridge,
				     struct drm_encoder *encoder,
				enum drm_bridge_attach_flags flags)
{
	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR))
		return -EINVAL;

	if (!bridge->next_bridge)
		return 0;

	return drm_bridge_attach(encoder, bridge->next_bridge, bridge, flags);
}

static const struct drm_bridge_funcs spacemit_dp_bridge_funcs = {
	.attach				= spacemit_dp_bridge_attach,
	.atomic_enable			= spacemit_dp_bridge_atomic_enable,
	.atomic_disable			= spacemit_dp_bridge_atomic_disable,
	.atomic_check			= spacemit_dp_bridge_atomic_check,
	.atomic_duplicate_state		= spacemit_dp_bridge_atomic_duplicate_state,
	.atomic_destroy_state		= spacemit_dp_bridge_atomic_destroy_state,
	.atomic_reset		= spacemit_dp_bridge_atomic_reset,
	.atomic_get_output_bus_fmts	= spacemit_dp_bridge_atomic_get_output_bus_fmts,
	.mode_valid			= spacemit_dp_bridge_mode_valid,
	.detect				= spacemit_dp_bridge_detect,
	.edid_read			= spacemit_dp_bridge_edid_read,
};

static irqreturn_t spacemit_dp_irq_handler(int irq, void *data)
{
	struct spacemit_dp_dev *dp = data;
	irqreturn_t ret = IRQ_NONE;
	int idx;

	if (!drm_dev_enter(dp->drm, &idx))
		return IRQ_NONE;

	/*
	 * eDP .detect() ignores the HPD level (and never records it), so a
	 * level flip would wake the thread forever; ack and ignore.
	 */
	if (dp->edp_mode) {
		spacemit_dp_hw_clean_hpd(dp);
		drm_dev_exit(idx);
		return IRQ_HANDLED;
	}

	/*
	 * Only compare here: the authoritative connector_status update
	 * happens in the thread under mode_lock, via .detect().
	 */
	if (spacemit_dp_hw_detect_hpd(dp) != READ_ONCE(dp->connector_status))
		ret = IRQ_WAKE_THREAD;
	if (spacemit_dp_hw_clean_hpd(dp) && ret == IRQ_NONE)
		ret = IRQ_HANDLED;

	drm_dev_exit(idx);
	return ret;
}

static irqreturn_t spacemit_dp_hotplug_event_handler(int irq, void *data)
{
	struct spacemit_dp_dev *dp = data;
	enum drm_connector_status status;
	int idx;

	if (!drm_dev_enter(dp->drm, &idx))
		return IRQ_HANDLED;

	/*
	 * .detect() owns mode_lock and re-reads the sink caps on a fresh plug
	 */
	status = spacemit_dp_bridge_detect(&dp->bridge, dp->connector);
	drm_bridge_hpd_notify(&dp->bridge, status);

	drm_dev_exit(idx);
	return IRQ_HANDLED;
}

static int spacemit_dp_resource_init(struct spacemit_dp_dev *dp,
				     struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *base;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return dev_err_probe(dev, PTR_ERR(base),
				     "failed to map registers\n");
	dp->regs = devm_regmap_init_mmio(dev, base, &spacemit_dp_regmap_config);
	if (IS_ERR(dp->regs))
		return dev_err_probe(dev, PTR_ERR(dp->regs),
				     "failed to init DP regmap\n");

	dp->irq = platform_get_irq(pdev, 0);
	if (dp->irq < 0)
		return dp->irq;

	return 0;
}

static int spacemit_dp_dev_init(struct spacemit_dp_dev *dp)
{
	int ret;
	u32 m_isel = 0x5, m_mainsel = 0x19;
	u32 m_pre = 0x0, m_post = 0x2;
	u32 tx_mode = 0x1, tx_pre = 0x0;
	u32 clk_div = 24 * 1000 / 100;

	regmap_write_bits(dp->regs, DP_PHYIF_CTRL_ADDR, DP_XMIT_ENABLE,
			  FIELD_PREP(DP_XMIT_ENABLE, 0));
	usleep_range(2000, 4000);

	regmap_write_bits(dp->regs, DP_MPLL_CTRL0, DP_ANA_MPLL_PD,
			  FIELD_PREP(DP_ANA_MPLL_PD, 1));
	regmap_write_bits(dp->regs, DP_PREPLL_CTRL0, DP_ANA_PREPLL_PD,
			  FIELD_PREP(DP_ANA_PREPLL_PD, 1));
	usleep_range(2000, 4000);

	/* Reset Controller and PHY */
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_CONTROLLER_RESET,
			  FIELD_PREP(DP_CONTROLLER_RESET, 0x1));
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_PHY_RESET,
			  FIELD_PREP(DP_PHY_RESET, 0x1));
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_HDCP_RESET,
			  FIELD_PREP(DP_HDCP_RESET, 0x1));
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_AUX_RESET,
			  FIELD_PREP(DP_AUX_RESET, 0x1));
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_VIDEO_RESET,
			  FIELD_PREP(DP_VIDEO_RESET, 0x1));
	usleep_range(5000, 8000);

	/* Release the resets */
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_CONTROLLER_RESET,
			  FIELD_PREP(DP_CONTROLLER_RESET, 0x0));
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_PHY_RESET,
			  FIELD_PREP(DP_PHY_RESET, 0x0));
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_HDCP_RESET,
			  FIELD_PREP(DP_HDCP_RESET, 0x0));
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_AUX_RESET,
			  FIELD_PREP(DP_AUX_RESET, 0x0));
	regmap_write_bits(dp->regs, DP_SOFT_RESET, DP_VIDEO_RESET,
			  FIELD_PREP(DP_VIDEO_RESET, 0x0));
	usleep_range(2000, 4000);

	regmap_write(dp->regs, DP_GENERAL_INTERRUPT,
		     DP_AUX_REPLY_EVENT_INT_STA);

	regmap_write_bits(dp->regs, DP_CORE_CONTROL,
			  DP_DEFAULT_FAST_LINK_TRAIN_EN,
			  FIELD_PREP(DP_DEFAULT_FAST_LINK_TRAIN_EN, 0x0));
	regmap_write_bits(dp->regs, DP_CORE_CONTROL, DP_SCRAMBLER_DISABLE,
			  FIELD_PREP(DP_SCRAMBLER_DISABLE, 0x0));
	regmap_write_bits(dp->regs, DP_CORE_CONTROL, DP_SCALE_DOWN_MODE,
			  FIELD_PREP(DP_SCALE_DOWN_MODE, 0x0));
	regmap_write_bits(dp->regs, DP_PHYIF_CTRL_ADDR, DP_XMIT_ENABLE,
			  FIELD_PREP(DP_XMIT_ENABLE, 0));

	regmap_write_bits(dp->regs, DP_VIDEO_VSAMPLE_REG,
			  DP_VIDEO_STREAM_ENABLE,
			  FIELD_PREP(DP_VIDEO_STREAM_ENABLE, 0));

	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK,
			  DP_AUX_REPLY_EVENT_INT_STA_MSK,
			  FIELD_PREP(DP_AUX_REPLY_EVENT_INT_STA_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK,
			  DP_HDCP_INT_STA_MSK,
			  FIELD_PREP(DP_HDCP_INT_STA_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK,
			  DP_ILLEGAL_AUX_CMD_INT_STA_MSK,
			  FIELD_PREP(DP_ILLEGAL_AUX_CMD_INT_STA_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK,
			  DP_TYPE_C_EVENT_MSK,
			  FIELD_PREP(DP_TYPE_C_EVENT_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK, DP_DSC_EVENT_MSK,
			  FIELD_PREP(DP_DSC_EVENT_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK,
			  DP_SDP_INT_STA_S3_MSK,
			  FIELD_PREP(DP_SDP_INT_STA_S3_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK,
			  DP_SDP_INT_STA_S2_MSK,
			  FIELD_PREP(DP_SDP_INT_STA_S2_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK,
			  DP_SDP_INT_STA_S1_MSK,
			  FIELD_PREP(DP_SDP_INT_STA_S1_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK,
			  DP_SDP_INT_STA_S0_MSK,
			  FIELD_PREP(DP_SDP_INT_STA_S0_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK,
			  DP_VIDEO_FIFO_OVERFLOW_INT_STA_S3_MSK,
			  FIELD_PREP(DP_VIDEO_FIFO_OVERFLOW_INT_STA_S3_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK,
			  DP_VIDEO_FIFO_OVERFLOW_INT_STA_S2_MSK,
			  FIELD_PREP(DP_VIDEO_FIFO_OVERFLOW_INT_STA_S2_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK,
			  DP_VIDEO_FIFO_OVERFLOW_INT_STA_S1_MSK,
			  FIELD_PREP(DP_VIDEO_FIFO_OVERFLOW_INT_STA_S1_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK,
			  DP_VIDEO_FIFO_OVERFLOW_INT_STA_S0_MSK,
			  FIELD_PREP(DP_VIDEO_FIFO_OVERFLOW_INT_STA_S0_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_HPD_INTERRUPT_ENABLE,
			  DP_SINK_IRQ_EVENT_MSK,
			  FIELD_PREP(DP_SINK_IRQ_EVENT_MSK, 0x0));
	regmap_write_bits(dp->regs, DP_GENERAL_INTERRUPT_MASK,
			  DP_HPD_INT_STA_MSK,
			  FIELD_PREP(DP_HPD_INT_STA_MSK, 0x1));
	regmap_write_bits(dp->regs, DP_HPD_INTERRUPT_ENABLE,
			  DP_HOT_PLUG_EVENT_MSK,
			  FIELD_PREP(DP_HOT_PLUG_EVENT_MSK, 0x1));
	regmap_write_bits(dp->regs, DP_HPD_INTERRUPT_ENABLE,
			  DP_HOT_UNPLUG_EVENT_MSK,
			  FIELD_PREP(DP_HOT_UNPLUG_EVENT_MSK, 0x1));
	regmap_write_bits(dp->regs, DP_HPD_INTERRUPT_ENABLE,
			  DP_SINK_UNPLUG_ERROR_EVENT_MSK,
			  FIELD_PREP(DP_SINK_UNPLUG_ERROR_EVENT_MSK, 0x0));
	usleep_range(2000, 4000);

	regmap_write_bits(dp->regs, DP_PHYIF_CTRL_ADDR, DP_PHY_BUSY_BYP,
			  FIELD_PREP(DP_PHY_BUSY_BYP, 0x1));

	/* Enable enhanced framing */
	regmap_write_bits(dp->regs, DP_CORE_CONTROL, DP_ENHANCE_FRAMING_EN,
			  FIELD_PREP(DP_ENHANCE_FRAMING_EN, 0x1));

	ret = spacemit_dp_set_plls(dp, SPACEMIT_DP_LINK_RATE_2_70, 148500);
	if (ret)
		return ret;

	ret = spacemit_dp_phy_set_lanes(dp, SPACEMIT_DP_LANE_2);
	if (ret)
		return ret;

	ret = spacemit_dp_phy_on(dp);
	if (ret)
		return ret;

	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_MODE_D0,
			  FIELD_PREP(DP_ANA_TX_MODE_D0, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_MODE_D1,
			  FIELD_PREP(DP_ANA_TX_MODE_D1, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_MODE_D2,
			  FIELD_PREP(DP_ANA_TX_MODE_D2, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_MODE_D3,
			  FIELD_PREP(DP_ANA_TX_MODE_D3, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL7, DP_ANA_RTCAL_FREQDIV_HBIT,
			  FIELD_PREP(DP_ANA_RTCAL_FREQDIV_HBIT, (clk_div >> 8) & 0x7f));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL7, DP_ANA_RTCAL_BYPASS,
			  FIELD_PREP(DP_ANA_RTCAL_BYPASS, 1));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL7, DP_ANA_RTCAL_FREQDIV_LBIT,
			  FIELD_PREP(DP_ANA_RTCAL_FREQDIV_LBIT, clk_div & 0xff));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL7, DP_ANA_BG_RCAL_SEL,
			  FIELD_PREP(DP_ANA_BG_RCAL_SEL, 0));

	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL8, DP_ANA_TX_RTM_D3,
			  FIELD_PREP(DP_ANA_TX_RTM_D3, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL8, DP_ANA_TX_RTM_D2,
			  FIELD_PREP(DP_ANA_TX_RTM_D2, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL8, DP_ANA_TX_RTM_D1,
			  FIELD_PREP(DP_ANA_TX_RTM_D1, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL8, DP_ANA_TX_RTM_D0,
			  FIELD_PREP(DP_ANA_TX_RTM_D0, 0));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL7, DP_ANA_RTCAL_BYPASS,
			  FIELD_PREP(DP_ANA_RTCAL_BYPASS, 1));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL7, DP_ANA_RTCAL_BYPASS,
			  FIELD_PREP(DP_ANA_RTCAL_BYPASS, 0));
	msleep(100);

	regmap_write_bits(dp->regs, DP_ANA_LANE_EMPH, DP_ANA_TX_MODE_PRE_D3,
			  FIELD_PREP(DP_ANA_TX_MODE_PRE_D3, 1));
	regmap_write_bits(dp->regs, DP_ANA_LANE_EMPH, DP_ANA_TX_MODE_PRE_D2,
			  FIELD_PREP(DP_ANA_TX_MODE_PRE_D2, 1));
	regmap_write_bits(dp->regs, DP_ANA_LANE_EMPH, DP_ANA_TX_MODE_PRE_D1,
			  FIELD_PREP(DP_ANA_TX_MODE_PRE_D1, 1));
	regmap_write_bits(dp->regs, DP_ANA_LANE_EMPH, DP_ANA_TX_MODE_PRE_D0,
			  FIELD_PREP(DP_ANA_TX_MODE_PRE_D0, 1));
	regmap_write_bits(dp->regs, DP_ANA_LANE_EMPH, DP_ANA_TX_MODE_DE_D3,
			  FIELD_PREP(DP_ANA_TX_MODE_DE_D3, 1));
	regmap_write_bits(dp->regs, DP_ANA_LANE_EMPH, DP_ANA_TX_MODE_DE_D2,
			  FIELD_PREP(DP_ANA_TX_MODE_DE_D2, 1));
	regmap_write_bits(dp->regs, DP_ANA_LANE_EMPH, DP_ANA_TX_MODE_DE_D1,
			  FIELD_PREP(DP_ANA_TX_MODE_DE_D1, 1));
	regmap_write_bits(dp->regs, DP_ANA_LANE_EMPH, DP_ANA_TX_MODE_DE_D0,
			  FIELD_PREP(DP_ANA_TX_MODE_DE_D0, 1));
	regmap_write_bits(dp->regs, DP_ANA_LANE_EMPH, DP_ANA_TX_POSTSEL_PRE_D3,
			  FIELD_PREP(DP_ANA_TX_POSTSEL_PRE_D3, tx_pre));
	regmap_write_bits(dp->regs, DP_ANA_LANE_EMPH, DP_ANA_TX_POSTSEL_PRE_D2,
			  FIELD_PREP(DP_ANA_TX_POSTSEL_PRE_D2, tx_pre));
	regmap_write_bits(dp->regs, DP_ANA_LANE_EMPH, DP_ANA_TX_POSTSEL_PRE_D1,
			  FIELD_PREP(DP_ANA_TX_POSTSEL_PRE_D1, tx_pre));
	regmap_write_bits(dp->regs, DP_ANA_LANE_EMPH, DP_ANA_TX_POSTSEL_PRE_D0,
			  FIELD_PREP(DP_ANA_TX_POSTSEL_PRE_D0, tx_pre));
	regmap_write_bits(dp->regs, DP_ANA_TX_ENABLE, DP_ANA_TX_ISEL_DRV_D3,
			  FIELD_PREP(DP_ANA_TX_ISEL_DRV_D3, m_isel));
	regmap_write_bits(dp->regs, DP_ANA_TX_ENABLE, DP_ANA_TX_ISEL_DRV_D2,
			  FIELD_PREP(DP_ANA_TX_ISEL_DRV_D2, m_isel));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL1, DP_ANA_TX_MAINSEL_D2,
			  FIELD_PREP(DP_ANA_TX_MAINSEL_D2, m_mainsel));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL1, DP_ANA_TX_MAINSEL_D3,
			  FIELD_PREP(DP_ANA_TX_MAINSEL_D3, m_mainsel));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL1, DP_ANA_TX_ISEL_DRV_D1,
			  FIELD_PREP(DP_ANA_TX_ISEL_DRV_D1, m_isel));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL1, DP_ANA_TX_ISEL_DRV_D0,
			  FIELD_PREP(DP_ANA_TX_ISEL_DRV_D0, m_isel));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL2, DP_ANA_TX_POSTSEL_D1,
			  FIELD_PREP(DP_ANA_TX_POSTSEL_D1, m_post));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL2, DP_ANA_TX_POSTSEL_D0,
			  FIELD_PREP(DP_ANA_TX_POSTSEL_D0, m_post));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL2, DP_ANA_TX_POSTSEL_D3,
			  FIELD_PREP(DP_ANA_TX_POSTSEL_D3, m_post));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL2, DP_ANA_TX_POSTSEL_D2,
			  FIELD_PREP(DP_ANA_TX_POSTSEL_D2, m_post));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL2, DP_DA_TX_MAINSEL_D0_4_0,
			  FIELD_PREP(DP_DA_TX_MAINSEL_D0_4_0, m_mainsel));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL2, DP_ANA_TX_MAINSEL_D1,
			  FIELD_PREP(DP_ANA_TX_MAINSEL_D1, m_mainsel));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_PRESEL_D1,
			  FIELD_PREP(DP_ANA_TX_PRESEL_D1, m_pre));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_PRESEL_D0,
			  FIELD_PREP(DP_ANA_TX_PRESEL_D0, m_pre));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_PRESEL_D3,
			  FIELD_PREP(DP_ANA_TX_PRESEL_D3, m_pre));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_PRESEL_D2,
			  FIELD_PREP(DP_ANA_TX_PRESEL_D2, m_pre));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_MODE_D3,
			  FIELD_PREP(DP_ANA_TX_MODE_D3, tx_mode));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_MODE_D2,
			  FIELD_PREP(DP_ANA_TX_MODE_D2, tx_mode));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_MODE_D1,
			  FIELD_PREP(DP_ANA_TX_MODE_D1, tx_mode));
	regmap_write_bits(dp->regs, DP_ANA_TX_CTRL3, DP_ANA_TX_MODE_D0,
			  FIELD_PREP(DP_ANA_TX_MODE_D0, tx_mode));
	regmap_write_bits(dp->regs, DP_ANA_TX_AUX_CTRL, DP_ANA_TX_AUX_RX_VSEL,
			  FIELD_PREP(DP_ANA_TX_AUX_RX_VSEL, 0x0));

	return 0;
}

static int spacemit_dp_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct spacemit_dp_dev *dp = dev_get_drvdata(dev);
	struct drm_device *drm = data;
	struct device_node *ep;
	int ret;

	dp->drm = drm;
	dp->aux.drm_dev = drm;
	ret = drm_dp_aux_register(&dp->aux);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register DP AUX\n");
		goto err_drm;
	}

	ret = drm_encoder_init(drm, &dp->encoder, &spacemit_dp_encoder_funcs,
			       DRM_MODE_ENCODER_NONE, NULL);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init encoder\n");
		goto err_aux;
	}

	ep = of_graph_get_endpoint_by_regs(dev->of_node, 0, -1);
	if (ep) {
		struct device_node *remote = of_graph_get_remote_port(ep);

		dp->encoder.possible_crtcs = drm_of_crtc_port_mask(drm, remote);
		of_node_put(remote);
		of_node_put(ep);
	}

	ret = drm_bridge_attach(&dp->encoder, &dp->bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret) {
		dev_err_probe(dev, ret, "failed to attach bridge\n");
		goto err_encoder;
	}

	dp->connector = drm_bridge_connector_init(drm, &dp->encoder);
	if (IS_ERR(dp->connector)) {
		ret = PTR_ERR(dp->connector);
		dev_err_probe(dev, ret, "failed to init bridge connector\n");
		goto err_encoder;
	}

	ret = spacemit_dp_dev_init(dp);
	if (ret)
		goto err_encoder;

	spacemit_dp_link_disable(dp);

	WRITE_ONCE(dp->connector_status, spacemit_dp_hw_detect_hpd(dp));
	spacemit_dp_hw_clean_hpd(dp);

	if (dp->connector_status == connector_status_connected)
		spacemit_dp_hw_read_sink_caps(dp);

	drm_bridge_hpd_notify(&dp->bridge,
			      spacemit_dp_bridge_detect(&dp->bridge,
							dp->connector));

	enable_irq(dp->irq);

	return 0;

err_encoder:
	drm_encoder_cleanup(&dp->encoder);
err_aux:
	drm_dp_aux_unregister(&dp->aux);
err_drm:
	/* The master releases the drm_device; the AUX channel outlives it. */
	dp->drm = NULL;
	dp->connector = NULL;
	dp->aux.drm_dev = NULL;
	return ret;
}

static void spacemit_dp_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct spacemit_dp_dev *dp = dev_get_drvdata(dev);

	disable_irq(dp->irq);

	/* The connector this work touches dies with the DRM device. */
	cancel_work_sync(&dp->modeset_retry_work);

	drm_dp_aux_unregister(&dp->aux);

	spacemit_dp_hw_disable(dp);

	drm_encoder_cleanup(&dp->encoder);

	dp->drm = NULL;
	dp->connector = NULL;
	dp->aux.drm_dev = NULL;
}

static const struct component_ops spacemit_dp_ops = {
	.bind = spacemit_dp_bind,
	.unbind = spacemit_dp_unbind,
};

/*
 * AUX-bus done-probing callback. The controller joins the DRM device only
 * now, so the bind always finds the panel bridge at the end of the chain.
 */
static int spacemit_dp_edp_link_panel(struct drm_dp_aux *aux)
{
	struct spacemit_dp_dev *dp = container_of(aux, struct spacemit_dp_dev,
						  aux);
	struct drm_bridge *next;
	int ret;

	/* A panel that probes again cannot join a registered controller. */
	if (dp->component_added)
		return -EBUSY;

	next = devm_drm_of_get_bridge(dp->dev, dp->dev->of_node, 1, 0);
	if (IS_ERR(next))
		return PTR_ERR(next);

	/* Held until the bridge is freed; the core drops it then. */
	dp->bridge.next_bridge = drm_bridge_get(next);

	ret = component_add(dp->dev, &spacemit_dp_ops);
	if (ret) {
		dp->bridge.next_bridge = NULL;
		drm_bridge_put(next);
		return ret;
	}
	dp->component_added = true;

	return 0;
}

/* Leave the DRM device, or power down a controller that never joined one. */
static void spacemit_dp_unregister(struct spacemit_dp_dev *dp)
{
	if (dp->component_added)
		component_del(dp->dev, &spacemit_dp_ops);
	else
		spacemit_dp_hw_disable(dp);
}

/*
 * Registered after devm_of_platform_populate(), so devres runs it before
 * the PHY child (and the PLL clock it provides) is torn down: unregistering
 * a clock that still has an enabled child warns, a disabled child is
 * orphaned silently.
 */
static void spacemit_dp_pxclk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int spacemit_dp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *aux_bus_np;
	struct spacemit_dp_dev *dp;
	struct reset_control *rstc;
	int ret;

	dp = devm_drm_bridge_alloc(dev, struct spacemit_dp_dev, bridge,
				   &spacemit_dp_bridge_funcs);
	if (IS_ERR(dp))
		return PTR_ERR(dp);

	dp->dev = dev;
	INIT_WORK(&dp->modeset_retry_work, spacemit_dp_modeset_retry_work);

	/* eDP instances have an aux-bus child node; DP instances do not. */
	aux_bus_np = of_get_child_by_name(dev->of_node, "aux-bus");
	dp->edp_mode = !!aux_bus_np;
	of_node_put(aux_bus_np);

	dp->bridge.of_node = dev->of_node;
	dp->connector_status = connector_status_disconnected;
	ret = devm_mutex_init(dev, &dp->mode_lock);
	if (ret)
		return ret;

	rstc = devm_reset_control_get_exclusive_deasserted(&pdev->dev, NULL);
	if (IS_ERR(rstc))
		return dev_err_probe(dev, PTR_ERR(rstc),
				     "failed to get reset\n");

	dp->pxclk = devm_clk_get(dev, "pxclk");
	if (IS_ERR(dp->pxclk))
		return dev_err_probe(dev, PTR_ERR(dp->pxclk),
				     "failed to get pxclk\n");

	if (dp->edp_mode) {
		dp->bridge.type = DRM_MODE_CONNECTOR_eDP;
		/* .detect() reports the fixed panel as connected. */
		dp->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_HPD;
	} else {
		dp->bridge.type = DRM_MODE_CONNECTOR_DisplayPort;
		/*
		 * No OP_HPD: the HPD interrupt is gated by the DP pixel clock.
		 * Leaving it out marks the connector POLL_CONNECT |
		 * POLL_DISCONNECT instead.
		 */
		dp->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID;
	}

	ret = clk_prepare_enable(dp->pxclk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable pxclk\n");

	/*
	 * The PHY child shares this device's regmap, so the regmap must exist
	 * before devm_of_platform_populate() lets the PHY probe and
	 * devm_phy_get() succeed.
	 */
	ret = spacemit_dp_resource_init(dp, pdev);
	if (ret)
		goto err_disable_pxclk;

	ret = devm_of_platform_populate(dev);
	if (ret) {
		dev_err_probe(dev, ret, "failed to populate child devices\n");
		goto err_disable_pxclk;
	}

	dp->phy = devm_phy_get(dev, "dp");
	if (IS_ERR(dp->phy)) {
		ret = dev_err_probe(dev, PTR_ERR(dp->phy),
				    "failed to get DP PHY\n");
		goto err_disable_pxclk;
	}

	/*
	 * The PHY exposes its PLL as the APMU pixel-clock mux's external
	 * parent.
	 */
	dp->pll_clk = devm_clk_get(dev, "pll");
	if (IS_ERR(dp->pll_clk)) {
		ret = dev_err_probe(dev, PTR_ERR(dp->pll_clk),
				    "failed to get PHY pixel clock\n");
		goto err_disable_pxclk;
	}

	ret = clk_set_parent(dp->pxclk, dp->pll_clk);
	if (ret) {
		dev_err_probe(dev, ret,
			      "failed to route the pixel mux to the PHY PLL\n");
		goto err_disable_pxclk;
	}

	ret = devm_add_action_or_reset(dev, spacemit_dp_pxclk_disable,
				       dp->pxclk);
	if (ret)
		return ret;

	/*
	 * Panel discovery needs the AUX channel here; registration waits for
	 * bind.
	 */
	dp->aux.name = "spacemit-dp-aux";
	dp->aux.dev = dev;
	dp->aux.transfer = spacemit_dp_aux_transfer;
	dp->aux.no_zero_sized = true;
	drm_dp_aux_init(&dp->aux);
	dp->bridge.ddc = &dp->aux.ddc;

	/*
	 * drm_bridge_attach() expects the bridge to already be on the global
	 * list.
	 */
	ret = devm_drm_bridge_add(dev, &dp->bridge);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register DP bridge\n");

	platform_set_drvdata(pdev, dp);

	/*
	 * The handler needs bind-created state, so keep the line disabled until
	 * bind.
	 */
	ret = devm_request_threaded_irq(dev, dp->irq, spacemit_dp_irq_handler,
					spacemit_dp_hotplug_event_handler,
					IRQF_NO_AUTOEN, dev_name(dev), dp);
	if (ret)
		return ret;

	if (!dp->edp_mode) {
		ret = component_add(dev, &spacemit_dp_ops);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add component\n");
		dp->component_added = true;

		return 0;
	}

	/*
	 * eDP hands panel power, enable-GPIO and backlight to panel-edp over
	 * the AUX bus, and panel-edp reads the EDID over AUX while it probes:
	 * bring the controller up first. The component is added once the panel
	 * is there, from spacemit_dp_edp_link_panel(); that may be before this
	 * call returns.
	 */
	ret = spacemit_dp_dev_init(dp);
	if (ret) {
		spacemit_dp_hw_disable(dp);
		return dev_err_probe(dev, ret,
				     "eDP: failed to bring up the controller\n");
	}
	spacemit_dp_link_disable(dp);

	ret = devm_of_dp_aux_populate_bus(&dp->aux, spacemit_dp_edp_link_panel);
	if (ret) {
		spacemit_dp_unregister(dp);
		return dev_err_probe(dev, ret,
				     "eDP: failed to populate aux-bus panel\n");
	}

	return 0;

err_disable_pxclk:
	clk_disable_unprepare(dp->pxclk);
	return ret;
}

static void spacemit_dp_remove(struct platform_device *pdev)
{
	spacemit_dp_unregister(platform_get_drvdata(pdev));
}

static void spacemit_dp_shutdown(struct platform_device *pdev)
{
	spacemit_dp_hw_disable(platform_get_drvdata(pdev));
}

static const struct of_device_id spacemit_dp_match[] = {
	{ .compatible = "spacemit,k3-inno-dp" },
	{}
};
MODULE_DEVICE_TABLE(of, spacemit_dp_match);

static struct platform_driver spacemit_dp_driver = {
	.probe = spacemit_dp_probe,
	.remove = spacemit_dp_remove,
	.shutdown = spacemit_dp_shutdown,
	.driver = {
		.name = "spacemit-inno-dp-drv",
		.of_match_table = spacemit_dp_match,
	},
};

module_platform_driver(spacemit_dp_driver);

MODULE_DESCRIPTION("SpacemiT K3 Innosilicon DP/eDP bridge driver");
MODULE_LICENSE("GPL");
MODULE_SOFTDEP("pre: phy-k3-inno-dp");
