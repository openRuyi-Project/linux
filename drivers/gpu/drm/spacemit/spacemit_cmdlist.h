/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025-2026 SpacemiT Co., Ltd.
 *
 */

#ifndef _SPACEMIT_CMDLIST_H_
#define _SPACEMIT_CMDLIST_H_

#include <linux/bits.h>
#include <linux/io.h>
#include <linux/types.h>

#include <drm/drm_crtc.h>
#include <drm/drm_plane.h>

struct spacemit_crtc;
struct spacemit_plane_state;

#define PER_CMDLIST_SIZE             4096
/* The engine's node header; rows follow it. Layout in dpu_saturn_hee.c. */
#define CMDLIST_HEADER_SIZE          16
#define CMDLIST_ROW_REGS	(3)

/**
 * enum cmdlist_type - what surface owns a cmdlist instance
 * @CMDLIST_PLANE: Per-plane RDMA register batches.
 * @CMDLIST_CRTC: Compose-side batches submitted on every flush.
 */
enum cmdlist_type {
	CMDLIST_PLANE = 1,
	CMDLIST_CRTC,
};

/**
 * struct cmdlist - A DMA-backed batch of register writes replayed at CFG_RDY
 * @nod_len: Number of register-write rows currently filled.
 * @size: Capacity of the DMA buffer in bytes.
 * @mode_mask: &enum cmdlist_mode_type bits of the blocks packed into the
 *	       list; selects the wait events and CFG_RDY bits when it is closed.
 * @va: Kernel virtual address of the DMA buffer.
 * @pa: DMA address programmed into the cmdlist base register.
 * @type: Originating surface; see &enum cmdlist_type.
 * @plane: Owning plane for a %CMDLIST_PLANE list.
 * @crtc: Owning CRTC for a %CMDLIST_CRTC list.
 */
struct cmdlist {
	u16 nod_len;
	u32 size;
	u32 mode_mask;
	void *va;
	dma_addr_t pa;
	u8 type;
	struct drm_plane *plane;
	struct drm_crtc *crtc;
};

/*
 * Two lists per channel, used alternately: the engine may still re-read the
 * list of the previous commit until the next CFG_RDY has been consumed, and
 * every commit waits for that before it completes.
 */
#define CMDLIST_RING	2

enum cmdlist_mode_type {
	CMDLIST_MOD_RDMA = BIT(0),
	CMDLIST_MOD_COMP = BIT(3),
	CMDLIST_MOD_DMMU = BIT(6),
};

#define CMDLIST_REG_STROBE(index)	(0xf << ((index) * 4))

#define CMDLIST_ADDRL_ALIGN_BITS               (4) /* From cmdlist_reg_0[] in CMDLIST_REG */
#define CMDLIST_ADDRL_ALIGN_MASK \
	((u32)(~(BIT(CMDLIST_ADDRL_ALIGN_BITS) - 1)))
/*
 * First touch seeds the word from a live MMIO read: only register blocks that
 * read back what was written may be routed through cmdlist.
 */
#define cmdlist_write_reg(hwdev, module_name, module_base, field, data, cl_regs) \
do { \
	struct cmdlist_regs *cl_p = (cl_regs); \
	u32 *__words = cl_p->module; \
	const unsigned int __w = dpu_field_word(module_name, field); \
	if (cl_p->flags[__w] == 0) { \
		__words[__w] = readl((hwdev)->base + (module_base) + 4 * __w); \
		cl_p->base = (module_base); \
		cl_p->flags[__w] = 1; \
	} \
	__words[__w] &= ~dpu_field_mask(module_name, field); \
	__words[__w] |= FIELD_PREP(dpu_field_mask(module_name, field), \
				   (u32)(data)); \
} while (0)

/* Raw whole-word variant; no seeding needed. */
#define cmdlist_write_word(module_base, word, data, cl_regs) \
do { \
	struct cmdlist_regs *cl_p = (cl_regs); \
	u32 *__words = cl_p->module; \
	cl_p->base = (module_base); \
	cl_p->flags[(word)] = 1; \
	__words[(word)] = (u32)(data); \
} while (0)

/**
 * struct cmdlist_regs - Scratch image of one register block
 * @base: Offset of the block the image currently describes.
 * @size: Number of words in the block.
 * @module: The words.
 * @flags: Nonzero for each word written since the image was last packed.
 */
struct cmdlist_regs {
	u32 base;
	u16 size;
	u32 *module;
	u8 *flags;
};

struct cmdlist_regs *spacemit_cmdlist_regs_alloc(struct drm_device *drm,
						 u16 nwords);
void spacemit_cmdlist_regs_packing(struct cmdlist *cl, enum cmdlist_mode_type mod,
				   struct cmdlist_regs *cl_regs);
int spacemit_cmdlist_ring_init(struct drm_device *drm, struct cmdlist cl[CMDLIST_RING],
			       enum cmdlist_type type, struct drm_plane *plane,
		      struct drm_crtc *crtc);
struct cmdlist *spacemit_cmdlist_ring_next(struct cmdlist cl[CMDLIST_RING], u8 *cur);
void spacemit_cmdlist_submit_plane(struct drm_plane *plane,
				   struct drm_plane_state *state);
void spacemit_cmdlist_submit_crtc(struct spacemit_crtc *a_crtc);
struct cmdlist *spacemit_plane_to_cl(struct drm_plane *plane);
struct cmdlist *spacemit_crtc_to_cl(struct drm_crtc *crtc);

#endif /* _SPACEMIT_CMDLIST_H_ */
