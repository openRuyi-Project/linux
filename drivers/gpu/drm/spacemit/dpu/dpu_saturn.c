// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025-2026 SpacemiT Co., Ltd.
 *
 */

#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/types.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "../spacemit_cmdlist.h"
#include "../spacemit_dmmu.h"
#include "../spacemit_dpu_ids.h"
#include "../spacemit_dpu_reg.h"
#include "../spacemit_drm.h"
#include "dpu_saturn.h"
#include "saturn_regs/ops_hee.h"

static const struct dpu_format_id primary_fmts[] = {
	{ DRM_FORMAT_XRGB8888, 8, 32 },
	{ DRM_FORMAT_ARGB8888, 4, 32 },
	{ DRM_FORMAT_XBGR8888, 9, 32 },
	{ DRM_FORMAT_ABGR8888, 5, 32 },
	{ DRM_FORMAT_RGBX8888, 10, 32 },
	{ DRM_FORMAT_RGBA8888, 6, 32 },
	{ DRM_FORMAT_BGRX8888, 11, 32 },
	{ DRM_FORMAT_BGRA8888, 7, 32 },
	{ DRM_FORMAT_ABGR2101010, 1, 32 },
	{ DRM_FORMAT_RGB888, 12, 24 },
	{ DRM_FORMAT_BGR888, 13, 24 },
	{ DRM_FORMAT_RGB565, 22, 16 },
	{ DRM_FORMAT_BGR565, 23, 16 },
};

/* Read channels per instance. */
#define SATURN_HEE_RDMA_NUM	4

/* A commit whose CFG_RDY is not taken within this time is reported. */
#define SATURN_CFG_RDY_TIMEOUT_MS	3000

/* Wait for the previous CFG_RDY to be consumed before the block is set up. */
#define SATURN_CFG_IDLE_POLL_US		100
#define SATURN_CFG_IDLE_TIMEOUT_US	(200 * USEC_PER_MSEC)

static const u32 saturn_hee_fbcmem_sizes[] = {
	89600, /* 87.5 KiB */
	15360, /* 15 KiB */
};

const struct spacemit_hw_device spacemit_k3_saturn_dpu = {
	.rdma_nums = SATURN_HEE_RDMA_NUM,
	.n_formats = ARRAY_SIZE(primary_fmts),
	.formats = primary_fmts,
	.n_fbcmems = ARRAY_SIZE(saturn_hee_fbcmem_sizes),
	.fbcmem_sizes = saturn_hee_fbcmem_sizes,
};

void saturn_enable_irq_mask(struct spacemit_crtc *a_crtc, bool enable,
			    u32 offset, u32 mask)
{
	struct spacemit_drm_private *priv = a_crtc->crtc.dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;

	if (!priv->clocks_on)
		return;

	regmap_update_bits(hwdev->regmap, offset, mask, enable ? mask : 0);
}

int spacemit_dpu_parse_clocks(struct device *dev, struct dpu_clk_context *clk_ctx)
{
	clk_ctx->pxclk = devm_clk_get(dev, "pxclk");
	if (IS_ERR(clk_ctx->pxclk))
		return dev_err_probe(dev, PTR_ERR(clk_ctx->pxclk),
				     "failed to get pxclk\n");

	clk_ctx->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(clk_ctx->mclk))
		return dev_err_probe(dev, PTR_ERR(clk_ctx->mclk),
				     "failed to get mclk\n");

	clk_ctx->escclk = devm_clk_get(dev, "escclk");
	if (IS_ERR(clk_ctx->escclk))
		return dev_err_probe(dev, PTR_ERR(clk_ctx->escclk),
				     "failed to get escclk\n");

	clk_ctx->aclk = devm_clk_get(dev, "aclk");
	if (IS_ERR(clk_ctx->aclk))
		return dev_err_probe(dev, PTR_ERR(clk_ctx->aclk),
				     "failed to get aclk\n");

	clk_ctx->dscclk = devm_clk_get(dev, "dscclk");
	if (IS_ERR(clk_ctx->dscclk))
		return dev_err_probe(dev, PTR_ERR(clk_ctx->dscclk),
				     "failed to get dscclk\n");

	return 0;
}

/*
 * Bring @clk to the rate nearest @rate. A clock that refuses is reported and
 * left as it is: the rates are targets, not requirements.
 */
static void spacemit_dpu_clk_set_rate(struct device *dev, struct clk *clk,
				      unsigned long rate, const char *name)
{
	long rounded = clk_round_rate(clk, rate);
	int ret;

	if (rounded < 0) {
		dev_warn_ratelimited(dev, "%s: no rate near %lu Hz: %ld\n",
				     name, rate, rounded);
		return;
	}

	ret = clk_set_rate(clk, rounded);
	if (ret)
		dev_warn_ratelimited(dev, "%s: failed to set %ld Hz: %d\n",
				     name, rounded, ret);
}

int spacemit_dpu_enable_clocks(struct spacemit_drm_private *priv)
{
	struct dpu_clk_context *clk_ctx = &priv->clk_ctx;
	int ret;

	/*
	 * Rate left alone: the mode's pixel clock is the DP PHY's PLL. This
	 * input only has to run for the "lcd" reset to reach the DPU.
	 */
	ret = clk_prepare_enable(clk_ctx->pxclk);
	if (ret)
		goto err;

	ret = clk_prepare_enable(clk_ctx->mclk);
	if (ret)
		goto err_pxclk;
	spacemit_dpu_clk_set_rate(priv->dev, clk_ctx->mclk, DPU_MCLK_DEFAULT,
				  "mclk");

	/* The escape clock only has to run for the "esc" reset to propagate. */
	ret = clk_prepare_enable(clk_ctx->escclk);
	if (ret)
		goto err_mclk;

	ret = clk_prepare_enable(clk_ctx->aclk);
	if (ret)
		goto err_escclk;
	spacemit_dpu_clk_set_rate(priv->dev, clk_ctx->aclk, DPU_AXICLK_DEFAULT,
				  "aclk");

	ret = clk_prepare_enable(clk_ctx->dscclk);
	if (ret)
		goto err_aclk;

	return 0;

err_aclk:
	clk_disable_unprepare(clk_ctx->aclk);
err_escclk:
	clk_disable_unprepare(clk_ctx->escclk);
err_mclk:
	clk_disable_unprepare(clk_ctx->mclk);
err_pxclk:
	clk_disable_unprepare(clk_ctx->pxclk);
err:
	dev_err(priv->dev, "failed to enable DPU clocks: %d\n", ret);
	return ret;
}

void spacemit_dpu_disable_clocks(struct spacemit_drm_private *priv)
{
	struct dpu_clk_context *clk_ctx = &priv->clk_ctx;

	clk_disable_unprepare(clk_ctx->dscclk);
	clk_disable_unprepare(clk_ctx->aclk);
	clk_disable_unprepare(clk_ctx->escclk);
	clk_disable_unprepare(clk_ctx->mclk);
	clk_disable_unprepare(clk_ctx->pxclk);
}

u8 spacemit_plane_hw_get_format_id(u32 format)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(primary_fmts); i++) {
		if (primary_fmts[i].format == format)
			return primary_fmts[i].id;
	}

	return SPACEMIT_DPU_INVALID_FORMAT_ID;
}

/* Complete a commit whose lists the engine will not confirm any more. */
void spacemit_dpu_complete_event(struct spacemit_crtc *a_crtc)
{
	struct drm_crtc *crtc = &a_crtc->crtc;
	struct drm_pending_vblank_event *event;
	unsigned long flags;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	event = a_crtc->event;
	a_crtc->event = NULL;
	if (event)
		drm_crtc_send_vblank_event(crtc, event);
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);

	if (event)
		drm_crtc_vblank_put(crtc);
}

void spacemit_cfg_rdy_timer_handler(struct timer_list *t)
{
	struct spacemit_crtc *a_crtc = timer_container_of(a_crtc, t,
							  cfg_rdy_timer);

	drm_err_ratelimited(a_crtc->crtc.dev, "CFG_RDY handshake timeout\n");
	/* A CFG_RDY that still comes belongs to no commit any more. */
	saturn_hee_enable_irq(a_crtc, INT_CFG_RDY, false);
	spacemit_dpu_complete_event(a_crtc);
}

void spacemit_dpu_init(struct spacemit_crtc *a_crtc,
		       const struct drm_display_mode *mode)
{
	struct spacemit_drm_private *priv = a_crtc->crtc.dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;
	u32 val;

	if (!priv->clocks_on)
		return;

	if (read_poll_timeout(saturn_hee_get_cfg_rdy, val, val == 0,
			      SATURN_CFG_IDLE_POLL_US,
			      SATURN_CFG_IDLE_TIMEOUT_US, false, a_crtc, hwdev))
		drm_err(a_crtc->crtc.dev, "wait for cfg ready timed out\n");

	saturn_hee_dpu_init(a_crtc, mode);
}

void spacemit_dpu_uninit(struct spacemit_crtc *a_crtc)
{
	struct spacemit_drm_private *priv = a_crtc->crtc.dev->dev_private;

	if (!priv->clocks_on)
		return;

	saturn_hee_irq_enable(a_crtc, false);
}

/* The read channels of the planes this CRTC scans out after the commit. */
u32 saturn_conf_dpuctrl_rdma(struct drm_atomic_state *state,
			     const struct drm_crtc_state *crtc_state)
{
	struct drm_plane *plane;
	u32 rdma_en = 0;

	drm_for_each_plane_mask(plane, crtc_state->crtc->dev,
				crtc_state->plane_mask) {
		const struct drm_plane_state *plane_state =
				drm_atomic_get_new_plane_state(state, plane);

		/* A plane the commit does not touch keeps its current state. */
		if (!plane_state)
			plane_state = plane->state;
		if (plane_state->visible)
			rdma_en |= BIT(to_spacemit_plane(plane)->rdma_id);
	}

	return rdma_en;
}

/* Returns false when none of the pending status bits was this CRTC's. */
bool spacemit_dpu_online_isr(struct spacemit_crtc *a_crtc)
{
	u32 irq_raw, irq_bit, irq_ur_bit, handled;
	struct spacemit_drm_private *priv = a_crtc->crtc.dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;

	irq_raw = saturn_hee_get_int_sts(hwdev);
	irq_bit = saturn_hee_get_irq_bit(INT_UNDERRUN);
	irq_ur_bit = irq_raw & irq_bit;
	handled = irq_ur_bit;
	if (irq_ur_bit && !a_crtc->ur_reported) {
		saturn_hee_clr_int_sts(a_crtc, irq_ur_bit);
		drm_err_ratelimited(a_crtc->crtc.dev, "underrun\n");
		a_crtc->ur_reported = true;
	}
	irq_bit = saturn_hee_get_irq_bit(INT_CFG_RDY);
	if (irq_raw & irq_bit) {
		saturn_hee_clr_int_sts(a_crtc, irq_bit);
		timer_delete(&a_crtc->cfg_rdy_timer);
		a_crtc->ur_reported = false;
		saturn_hee_enable_irq(a_crtc, INT_CFG_RDY, false);
		handled |= irq_bit;

		/* The lists are taken: the next vblank completes the commit. */
		spin_lock(&a_crtc->crtc.dev->event_lock);
		if (a_crtc->event) {
			drm_crtc_arm_vblank_event(&a_crtc->crtc, a_crtc->event);
			a_crtc->event = NULL;
		}
		spin_unlock(&a_crtc->crtc.dev->event_lock);
	}
	irq_bit = saturn_hee_get_irq_bit(INT_VSYNC);
	if (irq_raw & irq_bit) {
		saturn_hee_clr_int_sts(a_crtc, irq_bit);
		drm_crtc_handle_vblank(&a_crtc->crtc);
		handled |= irq_bit;
	}
	irq_bit = saturn_hee_get_irq_bit(INT_REST) | irq_ur_bit;
	if (irq_raw & irq_bit) {
		saturn_hee_clr_int_sts(a_crtc, irq_bit);
		handled |= irq_raw & irq_bit;
	}

	return handled;
}

void spacemit_dpu_run(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct spacemit_crtc *a_crtc = to_spacemit_crtc(crtc);
	struct spacemit_drm_private *priv = a_crtc->crtc.dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;

	saturn_hee_conf_dpuctrl(crtc, state);

	/* flush all config register writes before CFG_RDY is raised */
	mb();

	/* Ack first: one left by a commit that timed out is not this one's. */
	saturn_hee_clr_int_sts(a_crtc, saturn_hee_get_irq_bit(INT_CFG_RDY));
	saturn_hee_enable_irq(a_crtc, INT_CFG_RDY, true);

	mod_timer(&a_crtc->cfg_rdy_timer,
		  jiffies + msecs_to_jiffies(SATURN_CFG_RDY_TIMEOUT_MS));
	saturn_hee_cfg_ready(a_crtc, hwdev);

	if (unlikely(a_crtc->is_1st_f)) {
		drm_dbg(a_crtc->crtc.dev, "DPU start\n");
		a_crtc->is_1st_f = false;
		saturn_hee_sw_start(a_crtc, hwdev);
	}
}

