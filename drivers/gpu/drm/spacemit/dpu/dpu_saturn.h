/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025-2026 SpacemiT Co., Ltd.
 *
 */

#ifndef _DPU_SATURN_H_
#define _DPU_SATURN_H_

#include "../spacemit_crtc.h"

struct dpu_format_id {
	u32 format;		/* DRM fourcc */
	u8 id;
	u8 bpp;			/* bits per pixel */
};

#define SPACEMIT_DPU_INVALID_FORMAT_ID	0xff

int spacemit_dpu_parse_clocks(struct device *dev, struct dpu_clk_context *clk_ctx);
int spacemit_dpu_enable_clocks(struct spacemit_drm_private *priv);
void spacemit_dpu_disable_clocks(struct spacemit_drm_private *priv);
void spacemit_dpu_init(struct spacemit_crtc *a_crtc,
		       const struct drm_display_mode *mode);
void spacemit_dpu_uninit(struct spacemit_crtc *a_crtc);
void spacemit_dpu_run(struct drm_crtc *crtc, struct drm_atomic_state *state);
bool spacemit_dpu_online_isr(struct spacemit_crtc *a_crtc);
u8 spacemit_plane_hw_get_format_id(u32 format);
u32 saturn_conf_dpuctrl_rdma(struct drm_atomic_state *state,
			     const struct drm_crtc_state *crtc_state);
extern const struct spacemit_hw_device spacemit_k3_saturn_dpu;
void saturn_enable_irq_mask(struct spacemit_crtc *a_crtc, bool enable,
			    u32 offset, u32 mask);
void spacemit_cfg_rdy_timer_handler(struct timer_list *t);
void spacemit_dpu_complete_event(struct spacemit_crtc *a_crtc);
#endif
