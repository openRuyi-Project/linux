// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025-2026 SpacemiT Co., Ltd.
 *
 */

#include <linux/bits.h>
#include <linux/slab.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_drv.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>

#include "dpu/dpu_saturn.h"
#include "dpu/saturn_fbcmem.h"
#include "dpu/saturn_regs/ops_hee.h"
#include "dpu/saturn_regs/saturn_fields.h"
#include "spacemit_cmdlist.h"
#include "spacemit_dmmu.h"
#include "spacemit_drm.h"

/* The AFBC decoder exists but no modifier has been validated on hardware. */
static const u64 supported_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

struct spacemit_plane *to_spacemit_plane(struct drm_plane *plane)
{
	return container_of(plane, struct spacemit_plane, plane);
}

static int spacemit_plane_atomic_check(struct drm_plane *plane,
				       struct drm_atomic_commit *atomic_state)
{
	struct drm_plane_state *state =
			drm_atomic_get_new_plane_state(atomic_state, plane);
	struct drm_framebuffer *fb = state->fb;
	struct spacemit_plane_state *cur_state = to_spacemit_plane_state(state);
	struct drm_crtc_state *crtc_state = NULL;
	int ret;

	if (state->crtc)
		crtc_state = drm_atomic_get_new_crtc_state(atomic_state,
							   state->crtc);

	ret = drm_atomic_helper_check_plane_state(state, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  true, false);
	if (ret)
		return ret;

	if (!state->visible)
		return 0;

	/* The width limit bounds the default pitch; userspace may pass a larger one. */
	if (fb->pitches[0] > SPACEMIT_DPU_MAX_PITCH) {
		drm_dbg(plane->dev, "pitch %u exceeds the stride field\n",
			fb->pitches[0]);
		return -EINVAL;
	}

	/* Scanout starts at the first byte of the buffer object. */
	if (fb->offsets[0]) {
		drm_dbg(plane->dev, "framebuffer offset %u is not supported\n",
			fb->offsets[0]);
		return -EINVAL;
	}

	if (state->src.x1 >> 16 > SPACEMIT_DMMU_MAX_SRC_X) {
		drm_dbg(plane->dev, "src_x %d exceeds the DMMU table padding\n",
			state->src.x1 >> 16);
		return -EINVAL;
	}

	if (fb->obj[0]->size >> PAGE_SHIFT > SPACEMIT_DMMU_MAX_PAGES) {
		drm_dbg(plane->dev, "buffer of %zu bytes exceeds one TBU\n",
			fb->obj[0]->size);
		return -EINVAL;
	}

	if (saturn_cal_layer_fbcmem_size(plane, state)) {
		drm_dbg(plane->dev, "invalid fbcmem size\n");
		return -EINVAL;
	}

	cur_state->format = spacemit_plane_hw_get_format_id(fb->format->format);
	if (cur_state->format == SPACEMIT_DPU_INVALID_FORMAT_ID) {
		drm_dbg(plane->dev, "unsupported format %p4cc\n",
			&fb->format->format);
		return -EINVAL;
	}

	return 0;
}

static void spacemit_plane_program(struct drm_plane *plane,
				   struct drm_plane_state *state,
				   const struct drm_display_mode *mode)
{
	struct spacemit_plane *p = to_spacemit_plane(plane);
	u32 rdma_id = p->rdma_id;

	spacemit_cmdlist_ring_next(p->cl, &p->cl_cur);

	saturn_hee_plane_update_hw_channel(plane, state, mode);
	spacemit_dmmu_map(state->fb, rdma_id * 2, p->cl_rdma, plane);
	spacemit_cmdlist_regs_packing(spacemit_plane_to_cl(plane), CMDLIST_MOD_RDMA,
				      p->cl_rdma);

	spacemit_cmdlist_submit_plane(plane, state);
}

/* Take the layer out of the composition and stop its read channel. */
static void spacemit_plane_off(struct drm_plane *plane, struct drm_crtc *crtc)
{
	struct spacemit_drm_private *priv = plane->dev->dev_private;
	struct spacemit_plane *p = to_spacemit_plane(plane);

	saturn_hee_plane_disable_hw_channel(plane, crtc);
	saturn_hee_enable_cmdlist(to_spacemit_crtc(crtc), priv->hwdev,
				  p->rdma_id, false);
}

static void spacemit_plane_atomic_update(struct drm_plane *plane,
					 struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_state =
			drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state =
			drm_atomic_get_new_crtc_state(state, new_state->crtc);
	struct spacemit_drm_private *priv = plane->dev->dev_private;
	int idx;

	if (priv->clocks_on && drm_dev_enter(plane->dev, &idx)) {
		/* A plane moved off the screen has no rectangle to program. */
		if (new_state->visible)
			spacemit_plane_program(plane, new_state,
					       &crtc_state->adjusted_mode);
		else
			spacemit_plane_off(plane, new_state->crtc);
		drm_dev_exit(idx);
	}
}

static void spacemit_plane_atomic_disable(struct drm_plane *plane,
					  struct drm_atomic_commit *state)
{
	struct drm_plane_state *old_state =
			drm_atomic_get_old_plane_state(state, plane);
	struct spacemit_drm_private *priv = plane->dev->dev_private;
	int idx;

	if (priv->clocks_on && drm_dev_enter(plane->dev, &idx)) {
		spacemit_plane_off(plane, old_state->crtc);
		drm_dev_exit(idx);
	}
}

static struct drm_plane_state *
spacemit_plane_atomic_create_state(struct drm_plane *plane)
{
	struct spacemit_plane_state *s;

	s = kzalloc_obj(*s);
	if (!s)
		return ERR_PTR(-ENOMEM);

	__drm_atomic_helper_plane_state_init(&s->state, plane);

	return &s->state;
}

static struct drm_plane_state *
spacemit_plane_atomic_duplicate_state(struct drm_plane *plane)
{
	struct spacemit_plane_state *s;
	struct spacemit_plane_state *old_state =
			to_spacemit_plane_state(plane->state);

	s = kzalloc_obj(*s);
	if (!s)
		return NULL;

	__drm_atomic_helper_plane_duplicate_state(plane, &s->state);

	s->format = old_state->format;
	s->fbcmem_size = old_state->fbcmem_size;

	return &s->state;
}

static void spacemit_plane_atomic_destroy_state(struct drm_plane *plane,
						struct drm_plane_state *state)
{
	__drm_atomic_helper_plane_destroy_state(state);

	kfree(to_spacemit_plane_state(state));
}

static int spacemit_plane_create_properties(struct spacemit_plane *p)
{
	unsigned int support_modes = BIT(DRM_MODE_BLEND_PIXEL_NONE) |
		BIT(DRM_MODE_BLEND_PREMULTI) |
		BIT(DRM_MODE_BLEND_COVERAGE);
	int ret;

	ret = drm_plane_create_alpha_property(&p->plane);
	if (ret)
		return ret;

	return drm_plane_create_blend_mode_property(&p->plane, support_modes);
}

/* Collect the buffer's fences and build its DMMU table once. */
static int spacemit_plane_prepare_fb(struct drm_plane *plane,
				     struct drm_plane_state *state)
{
	int ret;

	if (!state->fb)
		return 0;

	ret = drm_gem_plane_helper_prepare_fb(plane, state);
	if (ret)
		return ret;

	return spacemit_dmmu_prepare(state->fb->obj[0]);
}

static const struct drm_plane_helper_funcs spacemit_plane_helper_funcs = {
	.prepare_fb = spacemit_plane_prepare_fb,
	.atomic_check = spacemit_plane_atomic_check,
	.atomic_update = spacemit_plane_atomic_update,
	.atomic_disable = spacemit_plane_atomic_disable,
};

static void spacemit_plane_atomic_print_state(struct drm_printer *p,
					      const struct drm_plane_state *state)
{
	const struct spacemit_plane_state *s = to_spacemit_plane_state(state);

	drm_printf(p, "\tformat-id=%u\n", s->format);
	drm_printf(p, "\tfbcmem-size=%u\n", s->fbcmem_size);
}

static const struct drm_plane_funcs spacemit_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.atomic_create_state = spacemit_plane_atomic_create_state,
	.atomic_duplicate_state = spacemit_plane_atomic_duplicate_state,
	.atomic_destroy_state = spacemit_plane_atomic_destroy_state,
	.atomic_print_state = spacemit_plane_atomic_print_state,
};

struct drm_plane *spacemit_plane_init(struct drm_device *drm)
{
	struct spacemit_plane *p;
	int err, j;
	u32 *formats;
	struct spacemit_drm_private *priv = drm->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;
	u8 n_formats = hwdev->n_formats;
	u8 n_fbcmems = hwdev->n_fbcmems;
	u8 n_rdmas = hwdev->rdma_nums;
	u32 plane_crtc_mask;

	if (n_fbcmems * 2 != n_rdmas) {
		drm_err(drm, "mismatched rdma/fbcmem counts: %d vs %d\n",
			n_rdmas, n_fbcmems);
		return ERR_PTR(-EINVAL);
	}

	for (j = 0; j < n_fbcmems; j++) {
		if (hwdev->fbcmem_sizes[j] < FBCMEM_UNIT) {
			drm_err(drm, "invalid fbcmem_sizes[%d] = %u\n",
				j, hwdev->fbcmem_sizes[j]);
			return ERR_PTR(-EINVAL);
		}
	}

	formats = kcalloc(n_formats, sizeof(*formats), GFP_KERNEL);
	if (!formats)
		return ERR_PTR(-ENOMEM);

	/*
	 * The CRTC is created right after this, so its index is the current
	 * count.
	 */
	plane_crtc_mask = BIT(drm->mode_config.num_crtc);

	for (j = 0; j < n_formats; j++)
		formats[j] = hwdev->formats[j].format;

	p = drmm_universal_plane_alloc(drm, struct spacemit_plane, plane,
				       plane_crtc_mask, &spacemit_plane_funcs,
				       formats, n_formats,
				       supported_format_modifiers,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	kfree(formats);
	if (IS_ERR(p)) {
		drm_err(drm, "failed to init primary plane\n");
		return ERR_CAST(p);
	}

	/* The single primary plane owns the first read channel. */
	p->rdma_id = 0;

	drm_plane_helper_add(&p->plane, &spacemit_plane_helper_funcs);

	err = spacemit_cmdlist_ring_init(drm, p->cl, CMDLIST_PLANE, &p->plane, NULL);
	if (err)
		return ERR_PTR(err);

	err = spacemit_plane_create_properties(p);
	if (err)
		return ERR_PTR(err);

	p->cl_rdma = spacemit_cmdlist_regs_alloc(drm, RDMA_PATH_X_REG__NWORDS);
	p->cl_tbu = spacemit_cmdlist_regs_alloc(drm, MMU_TBU_REG__NWORDS);
	if (!p->cl_rdma || !p->cl_tbu)
		return ERR_PTR(-ENOMEM);

	return &p->plane;
}
