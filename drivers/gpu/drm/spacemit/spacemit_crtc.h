/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025-2026 SpacemiT Co., Ltd.
 *
 */

#ifndef _SPACEMIT_CRTC_H_
#define _SPACEMIT_CRTC_H_

#include <linux/container_of.h>
#include <linux/timer.h>
#include <linux/types.h>

#include <drm/drm_crtc.h>
#include <drm/drm_plane.h>

#include "spacemit_cmdlist.h"
#include "spacemit_drm.h"

struct regmap;

/* RDMA stride is a 16-bit byte count; img_width/height are 16-bit fields. */
#define SPACEMIT_DPU_MAX_FB_WIDTH	(U16_MAX / 4)
#define SPACEMIT_DPU_MAX_FB_HEIGHT	U16_MAX
#define SPACEMIT_DPU_MAX_PITCH		U16_MAX
/* K3 user manual ch.12: each DPU output is at most 3840x2160. */
#define SPACEMIT_DPU_MAX_HDISPLAY	3840
#define SPACEMIT_DPU_MAX_VDISPLAY	2160

#define DPU_MCLK_DEFAULT	307200000
#define DPU_AXICLK_DEFAULT	409000000

enum spacemit_dpu_irq {
	INT_UNDERRUN,
	INT_CFG_RDY,
	INT_VSYNC,
	INT_EOF,
	INT_REST,
};

/**
 * struct spacemit_crtc - Per-instance state for a Saturn DPU CRTC
 * @crtc: Embedded DRM CRTC; reachable via to_spacemit_crtc().
 * @cfg_rdy_timer: Watchdog for a CFG_RDY that never arrives.
 * @is_1st_f: True until the first frame of a new modeset is transferred.
 * @event: The commit's event, held until the engine has taken its lists;
 *	   under &drm_device.event_lock.
 * @cl_cmp: Scratch image of the composer registers.
 * @cl_pp: Scratch image of the post-pipe registers.
 * @cl: The composer channel's command lists, written alternately.
 * @cl_cur: Index of the list being built for the current commit.
 * @ur_reported: True once an underrun has been logged since the last CFG_RDY.
 * @ur_mask_armed: True while saturn_hee_clr_int_sts() may still mask the
 *		   underrun IRQ.
 */
struct spacemit_crtc {
	struct drm_crtc crtc;
	struct timer_list cfg_rdy_timer;

	bool is_1st_f;
	struct drm_pending_vblank_event *event;

	struct cmdlist_regs *cl_cmp;
	struct cmdlist_regs *cl_pp;
	struct cmdlist cl[CMDLIST_RING];
	u8 cl_cur;

	bool ur_reported;
	bool ur_mask_armed;
};

static inline struct spacemit_crtc *to_spacemit_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct spacemit_crtc, crtc);
}

struct spacemit_hw_device;

/**
 * struct spacemit_plane - Subclassed drm_plane for the Saturn DPU
 * @plane: Base DRM plane.
 * @rdma_id: The RDMA read channel this plane scans out through, fixed at init.
 * @cl: The channel's command lists, written alternately.
 * @cl_cur: Index of the list being built for the current commit.
 * @cl_rdma: Scratch image of the read channel's registers.
 * @cl_tbu: Scratch image of the channel's TBU registers.
 */
struct spacemit_plane {
	struct drm_plane plane;
	u8 rdma_id;
	struct cmdlist_regs *cl_rdma;
	struct cmdlist_regs *cl_tbu;
	struct cmdlist cl[CMDLIST_RING];
	u8 cl_cur;
};

/**
 * struct spacemit_plane_state - Subclassed drm_plane_state for the Saturn DPU
 * @state: Embedded drm_plane_state; reachable via to_spacemit_plane_state().
 * @format: DPU-internal pixel-format token translated from the fourcc.
 * @fbcmem_size: Line-buffer units (FBCMEM_UNIT bytes) this plane needs.
 */
struct spacemit_plane_state {
	struct drm_plane_state state;
	u8 format;
	u32 fbcmem_size;
};

struct spacemit_plane *to_spacemit_plane(struct drm_plane *plane);

static inline struct
spacemit_plane_state *
to_spacemit_plane_state(const struct drm_plane_state *state)
{
	return container_of(state, struct spacemit_plane_state, state);
}

struct drm_plane *spacemit_plane_init(struct drm_device *drm);

/*
 * Per-revision Saturn DPU description. One static instance per DT compatible,
 * copied at probe.
 */
struct spacemit_hw_device {
	void __iomem *base;
	struct regmap *regmap;
	u8 rdma_nums;
	u8 n_formats;
	const struct dpu_format_id *formats;
	u8 n_fbcmems;
	const u32 *fbcmem_sizes;
};
#endif
