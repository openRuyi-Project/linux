/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025-2026 SpacemiT Co., Ltd.
 *
 */

#ifndef _SPACEMIT_DMMU_H_
#define _SPACEMIT_DMMU_H_

#include <linux/bits.h>
#include <linux/mutex.h>
#include <linux/types.h>

#include <drm/drm_file.h>
#include <drm/drm_gem_shmem_helper.h>

#include "spacemit_cmdlist.h"

#define DPU_QOS_URGENT	4
/* The TLB over-fetches past the table's end, further the larger src_x is. */
#define SPACEMIT_DMMU_MAX_SRC_X	4096
/* Entries of padding that cover the over-fetch at SPACEMIT_DMMU_MAX_SRC_X. */
#define HW_ALIGN_TTB_NUM	60

/* tbu_size is a 16-bit page count. */
#define SPACEMIT_DMMU_MAX_PAGES	U16_MAX
/* A table entry holds a 26-bit page frame number. */
#define SPACEMIT_DMMU_PFN_MASK	GENMASK(25, 0)

#define RD_OUTS_NUM             16
#define RDMA_TIMELIMIT  0xFFFF

#define BASE_VA         0x10000000ULL
#define VA_STEP_PER_TBU 0x40000000ULL

#define TBU_BASE_VA(tbu_id) ((u64)BASE_VA + (u64)VA_STEP_PER_TBU * (tbu_id))

struct tbu_instance {
	u64 ttb_pa[3];
	u64 tbu_va[3];
	u32 ttb_size[3];
};

/**
 * struct dpu_mmu_tbl - DMMU translation table for one buffer object
 * @size: Bytes of table, including the over-fetch padding.
 * @va: CPU address of the table.
 * @pa: DMA address programmed into the TBU.
 */
struct dpu_mmu_tbl {
	u32 size;
	void *va;
	dma_addr_t pa;
};

/**
 * struct spacemit_gem_object - shmem object plus its DMMU table
 * @base: The shmem object.
 * @mmu_tbl: Built on the first scanout of this buffer, freed with it.
 * @tbl_lock: Serialises the lazy build between commits.
 */
struct spacemit_gem_object {
	struct drm_gem_shmem_object base;
	struct dpu_mmu_tbl mmu_tbl;
	struct mutex tbl_lock;	/* serialises the build of @mmu_tbl */
};

static inline struct spacemit_gem_object *
to_spacemit_gem(struct drm_gem_object *obj)
{
	return container_of(to_drm_gem_shmem_obj(obj), struct spacemit_gem_object,
			    base);
}

int spacemit_dmmu_prepare(struct drm_gem_object *obj);
void spacemit_dmmu_release(struct drm_gem_object *obj);
void spacemit_dmmu_map(struct drm_framebuffer *fb, u8 tbu_id,
		       struct cmdlist_regs *cl_rdma, struct drm_plane *plane);

#endif /* _SPACEMIT_DMMU_H_ */
