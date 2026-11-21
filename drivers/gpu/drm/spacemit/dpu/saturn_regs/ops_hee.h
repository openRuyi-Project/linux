/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025-2026 SpacemiT Co., Ltd.
 *
 */

#ifndef _SATURN_OPS_HEE_H_
#define _SATURN_OPS_HEE_H_

#include "../../spacemit_crtc.h"

void saturn_hee_enable_irq(struct spacemit_crtc *a_crtc,
			   enum spacemit_dpu_irq irq_id, bool enable);
void saturn_hee_cfg_ready(struct spacemit_crtc *a_crtc,
			  struct spacemit_hw_device *hwdev);
void saturn_hee_sw_start(struct spacemit_crtc *a_crtc,
			 struct spacemit_hw_device *hwdev);
void saturn_hee_dpu_init(struct spacemit_crtc *a_crtc,
			 const struct drm_display_mode *mode);
void saturn_hee_plane_update_hw_channel(struct drm_plane *plane,
					struct drm_plane_state *state,
					const struct drm_display_mode *mode);
void saturn_hee_plane_disable_hw_channel(struct drm_plane *plane,
					 struct drm_crtc *crtc);
void saturn_hee_irq_enable(struct spacemit_crtc *a_crtc, bool enable);
void saturn_hee_conf_dpuctrl(struct drm_crtc *crtc,
			     struct drm_atomic_commit *state);
u32 saturn_hee_get_cfg_rdy(struct spacemit_crtc *a_crtc,
			   struct spacemit_hw_device *hwdev);
u32 saturn_hee_get_int_sts(struct spacemit_hw_device *hwdev);
u32 saturn_hee_get_irq_bit(enum spacemit_dpu_irq irq_id);
void saturn_hee_clr_int_sts(struct spacemit_crtc *a_crtc, u32 data);
void saturn_hee_enable_cmdlist(struct spacemit_crtc *a_crtc,
			       struct spacemit_hw_device *hwdev, int id,
			       bool enable);
void saturn_hee_cfg_cmdlist(struct spacemit_hw_device *hwdev, int id, u32 chy,
			    u32 addrl, u32 addrh);
void saturn_hee_rdma_dmmu(struct spacemit_hw_device *hwdev, u8 tbu_id,
			  struct tbu_instance *tbu, struct drm_framebuffer *fb,
			  struct cmdlist_regs *cl_rdma,
			  struct drm_plane *plane);
bool saturn_hee_cmdlist_fill_data_row(struct cmdlist *cl, u32 strobe,
				      u32 offset, u32 value[]);
void saturn_hee_cmdlist_fill_conf_row(struct cmdlist *cl, u32 ch_y);
void saturn_hee_crtc_cmdlist(struct spacemit_hw_device *hwdev, dma_addr_t pa,
			     u8 crtc_id);
#endif
