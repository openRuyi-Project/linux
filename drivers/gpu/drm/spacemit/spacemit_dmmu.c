// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023-2026 SpacemiT Co., Ltd.
 *
 */

#include <linux/cleanup.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/stddef.h>

#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_shmem_helper.h>

#include "dpu/saturn_regs/ops_hee.h"
#include "spacemit_cmdlist.h"
#include "spacemit_crtc.h"
#include "spacemit_dmmu.h"
#include "spacemit_dpu_reg.h"
#include "spacemit_drm.h"

static inline void spacemit_dmmu_fill_pgtable(struct drm_device *drm,
					      struct dpu_mmu_tbl *tbl,
					      struct sg_table *sgt)
{
	struct sg_dma_page_iter dma_iter;
	u32 max_entries = tbl->size / 4;
	u32 *ttbr = tbl->va;
	u32 i = 0, n = 0, temp = 0;

	for_each_sgtable_dma_page(sgt, &dma_iter, 0) {
		/*
		 * The table was sized from this object, so running past it is
		 * a driver bug.
		 */
		if (drm_WARN_ON_ONCE(drm, n + HW_ALIGN_TTB_NUM >= max_entries))
			break;
		ttbr[n++] = SPACEMIT_DMMU_PFN_MASK &
			    (sg_page_iter_dma_address(&dma_iter) >> PAGE_SHIFT);
	}

	if (drm_WARN_ON_ONCE(drm, !n))
		return;

	/*
	 * The silicon over-fetches past the last entry; pad it to avoid a DDR
	 * fault.
	 */
	temp = ttbr[n - 1];
	for (i = 0; i < HW_ALIGN_TTB_NUM; i++)
		ttbr[n++] = temp;
}

/*
 * Build the translation table for @obj once. The table describes the buffer,
 * not a plane state, so it lives with the object and is reused by every
 * commit that scans the buffer out.
 */
int spacemit_dmmu_prepare(struct drm_gem_object *obj)
{
	struct spacemit_gem_object *sgem = to_spacemit_gem(obj);
	struct drm_gem_shmem_object *shmem = &sgem->base;
	struct device *dev = obj->dev->dev;
	struct dpu_mmu_tbl tbl;
	struct sg_table *sgt;

	guard(mutex)(&sgem->tbl_lock);

	if (sgem->mmu_tbl.va)
		return 0;

	/* An imported dma-buf has its sg table mapped for this device. */
	if (drm_gem_is_imported(obj)) {
		sgt = shmem->sgt;
		if (!sgt)
			return -EINVAL;
	} else {
		sgt = drm_gem_shmem_get_pages_sgt(shmem);
		if (IS_ERR(sgt))
			return PTR_ERR(sgt);
	}

	tbl.size = ((PAGE_ALIGN(obj->size) >> PAGE_SHIFT) + HW_ALIGN_TTB_NUM) * 4;
	tbl.va = dma_alloc_coherent(dev, tbl.size, &tbl.pa, GFP_KERNEL);
	if (!tbl.va)
		return -ENOMEM;

	spacemit_dmmu_fill_pgtable(obj->dev, &tbl, sgt);
	sgem->mmu_tbl = tbl;

	return 0;
}

void spacemit_dmmu_release(struct drm_gem_object *obj)
{
	struct spacemit_gem_object *sgem = to_spacemit_gem(obj);

	if (sgem->mmu_tbl.va)
		dma_free_coherent(obj->dev->dev, sgem->mmu_tbl.size,
				  sgem->mmu_tbl.va, sgem->mmu_tbl.pa);
	mutex_destroy(&sgem->tbl_lock);
}

/* Point the plane's TBU at the framebuffer's table; built in prepare_fb. */
void spacemit_dmmu_map(struct drm_framebuffer *fb, u8 tbu_id,
		       struct cmdlist_regs *cl_rdma, struct drm_plane *plane)
{
	struct spacemit_drm_private *priv = fb->dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;
	struct spacemit_gem_object *sgem = to_spacemit_gem(fb->obj[0]);
	struct tbu_instance tbu = { };

	/* Every advertised format is single-planar */
	tbu.ttb_pa[0] = sgem->mmu_tbl.pa;
	tbu.tbu_va[0] = TBU_BASE_VA(tbu_id);
	tbu.ttb_size[0] = roundup(fb->obj[0]->size, PAGE_SIZE) >> PAGE_SHIFT;

	saturn_hee_rdma_dmmu(hwdev, tbu_id, &tbu, fb, cl_rdma, plane);
}
