// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025-2026 SpacemiT Co., Ltd.
 *
 */

#include <linux/dma-mapping.h>
#include <linux/string.h>

#include <drm/drm_atomic.h>
#include <drm/drm_managed.h>
#include <drm/drm_print.h>

#include "dpu/dpu_saturn.h"
#include "dpu/saturn_regs/ops_hee.h"
#include "spacemit_cmdlist.h"
#include "spacemit_crtc.h"
#include "spacemit_dpu_reg.h"
#include "spacemit_drm.h"

struct cmdlist *spacemit_plane_to_cl(struct drm_plane *plane)
{
	struct spacemit_plane *p = to_spacemit_plane(plane);

	return &p->cl[p->cl_cur];
}

struct cmdlist *spacemit_crtc_to_cl(struct drm_crtc *crtc)
{
	struct spacemit_crtc *a_crtc = to_spacemit_crtc(crtc);

	return &a_crtc->cl[a_crtc->cl_cur];
}

static void spacemit_cmdlist_ring_release(struct drm_device *drm, void *res)
{
	struct cmdlist *cl = res;
	int i;

	for (i = 0; i < CMDLIST_RING; i++) {
		if (cl[i].va)
			dma_free_coherent(drm->dev, cl[i].size, cl[i].va,
					  cl[i].pa);
	}
}

/* Allocate a channel's lists once, for the life of the DRM device. */
int spacemit_cmdlist_ring_init(struct drm_device *drm, struct cmdlist cl[CMDLIST_RING],
			       enum cmdlist_type type, struct drm_plane *plane,
		      struct drm_crtc *crtc)
{
	int i;

	for (i = 0; i < CMDLIST_RING; i++) {
		cl[i].type = type;
		cl[i].plane = plane;
		cl[i].crtc = crtc;
		cl[i].size = PER_CMDLIST_SIZE;
		cl[i].va = dma_alloc_coherent(drm->dev, cl[i].size, &cl[i].pa,
					      GFP_KERNEL);
		if (!cl[i].va) {
			spacemit_cmdlist_ring_release(drm, cl);
			return -ENOMEM;
		}
	}

	return drmm_add_action_or_reset(drm, spacemit_cmdlist_ring_release, cl);
}

/* Switch to the other list and empty it for this commit. */
struct cmdlist *spacemit_cmdlist_ring_next(struct cmdlist cl[CMDLIST_RING], u8 *cur)
{
	struct cmdlist *next;

	*cur ^= 1;
	next = &cl[*cur];
	next->nod_len = 0;
	next->mode_mask = 0;
	memset(next->va, 0, CMDLIST_HEADER_SIZE);

	return next;
}

/* Allocated once at init: the commit tail has no way to fail. */
struct cmdlist_regs *spacemit_cmdlist_regs_alloc(struct drm_device *drm,
						 u16 nwords)
{
	struct cmdlist_regs *regs;

	regs = drmm_kzalloc(drm, sizeof(*regs), GFP_KERNEL);
	if (!regs)
		return NULL;

	regs->size = nwords;
	regs->module = drmm_kcalloc(drm, nwords, sizeof(*regs->module),
				    GFP_KERNEL);
	regs->flags = drmm_kcalloc(drm, nwords, sizeof(*regs->flags),
				   GFP_KERNEL);
	if (!regs->module || !regs->flags)
		return NULL;

	return regs;
}

/* Append the words written to @cl_regs to @cl and empty the image. */
void spacemit_cmdlist_regs_packing(struct cmdlist *cl, enum cmdlist_mode_type mod,
				   struct cmdlist_regs *cl_regs)
{
	struct drm_device *drm;
	u32 *reg_base;
	int i;
	u32 index;

	drm = cl->type == CMDLIST_PLANE ? cl->plane->dev : cl->crtc->dev;

	reg_base = cl_regs->module;

	cl->mode_mask |= mod;

	/* Allocated in spacemit_cmdlist_ring_init(); without it the writes vanish. */
	if (drm_WARN_ON(drm, !cl->va))
		goto out;

	for (i = 0; i < cl_regs->size;) {
		u32 value[CMDLIST_ROW_REGS] = {0x0};
		u32 offset, strobe = 0;

		if (cl_regs->flags[i]) {
			offset = cl_regs->base + i * sizeof(u32);
			for (u8 j = 0; j < CMDLIST_ROW_REGS; j++) {
				index = i + j;
				if (likely(index < cl_regs->size)) {
					if (cl_regs->flags[index]) {
						value[j] = reg_base[index];
						strobe |= CMDLIST_REG_STROBE(j);
					}
				} else {
					break;
				}
			}
			if (!saturn_hee_cmdlist_fill_data_row(cl, strobe, offset,
							      value)) {
				drm_WARN_ONCE(drm, 1,
					      "cmdlist buffer full (rows %u)\n",
					      cl->nod_len);
				goto out;
			}
			/*
			 * the row covers the whole window; a shorter stride
			 * would revisit covered words as duplicate rows
			 */
			i += CMDLIST_ROW_REGS;
			cl->nod_len++;
		} else {
			i++;
		}
	}

	drm_dbg(drm, "row_num = %d\n", cl->nod_len);
out:
	memset(cl_regs->flags, 0, cl_regs->size);
}

/*
 * Terminate the plane's list and point its read channel at it; the engine
 * replays it at the next CFG_RDY.
 */
void spacemit_cmdlist_submit_plane(struct drm_plane *plane,
				   struct drm_plane_state *state)
{
	struct spacemit_plane *p = to_spacemit_plane(plane);
	struct spacemit_crtc *a_crtc = to_spacemit_crtc(state->crtc);
	struct spacemit_drm_private *priv = plane->dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;
	struct cmdlist *cl = spacemit_plane_to_cl(plane);
	u32 addrl, addrh;

	saturn_hee_cmdlist_fill_conf_row(cl, state->dst.y1);

	addrl = (lower_32_bits(cl->pa) & CMDLIST_ADDRL_ALIGN_MASK) >>
		CMDLIST_ADDRL_ALIGN_BITS;
	addrh = upper_32_bits(cl->pa);

	saturn_hee_enable_cmdlist(a_crtc, hwdev, p->rdma_id, false);
	saturn_hee_cfg_cmdlist(hwdev, p->rdma_id, state->dst.y1, addrl, addrh);
	saturn_hee_enable_cmdlist(a_crtc, hwdev, p->rdma_id, true);
}

/* Terminate the CRTC's compose list and point the composer channel at it. */
void spacemit_cmdlist_submit_crtc(struct spacemit_crtc *a_crtc)
{
	struct drm_crtc *crtc = &a_crtc->crtc;
	struct spacemit_drm_private *priv = crtc->dev->dev_private;
	struct cmdlist *cl = spacemit_crtc_to_cl(crtc);

	saturn_hee_cmdlist_fill_conf_row(cl, 0);
	saturn_hee_crtc_cmdlist(priv->hwdev, cl->pa, drm_crtc_index(crtc));
}
