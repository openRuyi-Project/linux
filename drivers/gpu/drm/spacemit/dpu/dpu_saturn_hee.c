// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025-2026 SpacemiT Co., Ltd.
 *
 */

#include <linux/regmap.h>

#include <drm/drm_atomic.h>
#include <drm/drm_blend.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_print.h>

#include "../spacemit_cmdlist.h"
#include "../spacemit_dmmu.h"
#include "../spacemit_dpu_ids.h"
#include "../spacemit_dpu_reg.h"
#include "dpu_saturn.h"
#include "saturn_fbcmem.h"
#include "saturn_regs/ops_hee.h"
#include "saturn_regs/reg_map_hee.h"

#define SATURN_WRITE_TBU_REGS(hwdev, base, reg_id, cl_tbu) \
do {\
	dpu_write(hwdev, MMU_TBU_REG, base, tbu_base_addr##reg_id##_low, \
		  tbu->ttb_pa[reg_id] & 0xFFFFFFFF, cl_tbu); \
	dpu_write(hwdev, MMU_TBU_REG, base, tbu_base_addr##reg_id##_high, \
		  (tbu->ttb_pa[reg_id] >> 32) & 0xFFF, cl_tbu); \
	dpu_write(hwdev, MMU_TBU_REG, base, tbu_va##reg_id, \
		 (tbu->tbu_va[reg_id] >> 12), cl_tbu); \
	dpu_write(hwdev, MMU_TBU_REG, base, \
		  tbu_size##reg_id, tbu->ttb_size[reg_id], cl_tbu); \
	dpu_write(hwdev, MMU_TBU_REG, base, \
		  vsync_update_en, 1, cl_tbu); \
} while (0)

#define SATURN_WRITE_RDMA_ADDR_REG(hwdev, reg_id, rdma_id, addr, cl_rdma) \
do { \
	dpu_write(hwdev, RDMA_PATH_X_REG, RDMA_BASE_ADDR[rdma_id], \
		  base_addr##reg_id##_low_ly0, (addr) & 0xFFFFFFFF, cl_rdma); \
	dpu_write(hwdev, RDMA_PATH_X_REG, RDMA_BASE_ADDR[rdma_id], \
		  base_addr##reg_id##_high_ly0, ((addr) >> 32) & 0xFFF, \
		  cl_rdma); \
} while (0)

static const u32 RDMA_BASE_ADDR[] = {
	RDMA0_BASE_ADDR,
	RDMA1_BASE_ADDR,
	RDMA2_BASE_ADDR,
	RDMA3_BASE_ADDR,
};

static const u32 MMU_TBU_BASE_ADDR_ARRAY[] = {
	TBU0_ADDR, TBU1_ADDR, TBU2_ADDR, TBU3_ADDR,
	TBU4_ADDR, TBU5_ADDR, TBU6_ADDR, TBU7_ADDR,
};

void saturn_hee_enable_irq(struct spacemit_crtc *a_crtc,
			   enum spacemit_dpu_irq irq_id, bool enable)
{
	saturn_enable_irq_mask(a_crtc, enable,
			       DPU_INT_BASE_ADDR + DPU_ONLINE_IRQ_MSK,
			       saturn_hee_get_irq_bit(irq_id));
}

void saturn_hee_cfg_ready(struct spacemit_crtc *a_crtc,
			  struct spacemit_hw_device *hwdev)
{
	u32 base = DPU_SCENE_CTL_ADDR(COMPOSER1);

	saturn_hee_enable_irq(a_crtc, INT_UNDERRUN, true);
	a_crtc->ur_mask_armed = true;
	dpu_write(hwdev, DPU_CTL_REG, base, both_cfg_rdy, 1);
}

void saturn_hee_sw_start(struct spacemit_crtc *a_crtc,
			 struct spacemit_hw_device *hwdev)
{
	u32 base;

	base = DPU_SCENE_CTL_ADDR(COMPOSER1);
	dpu_write_reg(hwdev, DPU_CTL_REG, base, sw_start, 1);
}

static void saturn_init_tmg(struct spacemit_crtc *a_crtc,
			    const struct drm_display_mode *mode)
{
	u32 base = TMG0_BASE_ADDR;
	struct spacemit_drm_private *priv = a_crtc->crtc.dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;
	u16 vfp, vbp, vsync, hfp, hbp, hsync;
	void __iomem *tmg_addr;
	u32 value;

	tmg_addr = hwdev->base + TMG0_BASE_ADDR;

	hsync = mode->hsync_end - mode->hsync_start;
	hbp = mode->htotal - mode->hsync_end;
	hfp = mode->hsync_start - mode->hdisplay;
	vsync = mode->vsync_end - mode->vsync_start;
	vbp = mode->vtotal - mode->vsync_end;
	vfp = mode->vsync_start - mode->vdisplay;
	drm_dbg(a_crtc->crtc.dev, "timing " DRM_MODE_FMT "\n", DRM_MODE_ARG(mode));

	dpu_write(hwdev, TMG_REG, base, disp_ready_man_en, 0);
	dpu_write(hwdev, TMG_REG, base, background_r, 0);
	dpu_write(hwdev, TMG_REG, base, background_g, 0x0);
	dpu_write(hwdev, TMG_REG, base, background_b, 0xff);
	dpu_write(hwdev, TMG_REG, base, eof_1st_ln_dly_num, vfp - 7);
	dpu_write(hwdev, TMG_REG, base, eof_2nd_ln_dly_num, vfp - 6);
	dpu_write(hwdev, TMG_REG, base, split_en, 0);
	dpu_write(hwdev, TMG_REG, base, cmd_screen, 0);
	dpu_write(hwdev, TMG_REG, base, cmd_wait_en, 0);
	dpu_write(hwdev, TMG_REG, base, cmd_wait_te, 0);
	dpu_write(hwdev, TMG_REG, base, sof_pre_ln_num, 0);
	dpu_write(hwdev, TMG_REG, base, hfp, hfp);
	dpu_write(hwdev, TMG_REG, base, hbp, hbp);
	dpu_write(hwdev, TMG_REG, base, vfp, vfp);
	dpu_write(hwdev, TMG_REG, base, vbp, vbp);
	dpu_write(hwdev, TMG_REG, base, hsync_width, hsync);
	dpu_write(hwdev, TMG_REG, base, vsync_width, vsync);
	dpu_write(hwdev, TMG_REG, base, hsp, 1);
	dpu_write(hwdev, TMG_REG, base, vsp, 1);
	dpu_write(hwdev, TMG_REG, base, h_active, mode->hdisplay);
	dpu_write(hwdev, TMG_REG, base, v_active, mode->vdisplay);
	dpu_write(hwdev, TMG_REG, base, fm_timing_en, 1);
	/* The pipeline always hands RGB888 to the DP/eDP encoder. */
	dpu_write(hwdev, TMG_REG, base, user, OUTFMT_RGB888);

	value = readl(tmg_addr + 4 * dpu_field_word(TMG_REG, vfp));
	value &= ~dpu_field_mask(TMG_REG, vfp);
	value |= vfp;
	writel(value, tmg_addr + 4 * dpu_field_word(TMG_REG, vfp));

	dpu_write(hwdev, TMG_REG, base, vsync_align_with_hsync, 1);
}

static void saturn_init_regs(struct spacemit_crtc *a_crtc,
			     const struct drm_display_mode *mode)
{
	struct spacemit_drm_private *priv = a_crtc->crtc.dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;
	u32 base;

	base = CMP1_BASE_ADDR;
	dpu_write(hwdev, CMPS_X_REG, base, bg_color_b, 0x0);
	dpu_write(hwdev, CMPS_X_REG, base, bg_color_r, 0x0);
	dpu_write(hwdev, CMPS_X_REG, base, bg_color_g, 0x0);
	dpu_write(hwdev, CMPS_X_REG, base, bg_color_a, 0xFF);
	dpu_write(hwdev, CMPS_X_REG, base, module_enable, 1);
	dpu_write(hwdev, CMPS_X_REG, base, layer00_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer01_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer02_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer03_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer04_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer05_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer06_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer07_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer08_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer09_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer10_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer11_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer12_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer13_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer14_en, 0);
	dpu_write(hwdev, CMPS_X_REG, base, layer15_en, 0);

	base = EE_ADDR;
	dpu_write(hwdev, EE_REG, base, m_benable, 0);

	saturn_init_tmg(a_crtc, mode);
}

static void saturn_setup_dma_top(struct spacemit_crtc *a_crtc)
{
	u32 base = DMA_TOP_BASE_ADDR;
	struct spacemit_drm_private *priv = a_crtc->crtc.dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;

	dpu_write(hwdev, DMA_TOP_REG, base, image_rr_ratio, 0x10);
	dpu_write(hwdev, DMA_TOP_REG, base, pixel_num_th, 4);

	dpu_write(hwdev, DMA_TOP_REG, base, rdma_timeout_limit, 0xFFFE);
	dpu_write(hwdev, DMA_TOP_REG, base, wdma_timeout_limit, 0xFFFF);
}

static void saturn_setup_mmu_top(struct spacemit_crtc *a_crtc)
{
	unsigned int rd_outs_num = RD_OUTS_NUM / 2;
	u32 base = MMU_TOP_BASE_ADDR;
	struct spacemit_drm_private *priv = a_crtc->crtc.dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;

	dpu_write(hwdev, MMU_TOP_REG, base, rdma_timelimit, RDMA_TIMELIMIT);

	dpu_write(hwdev, MMU_TOP_REG, base, sram0_tlb_axi_port_sel, 0);
	dpu_write(hwdev, MMU_TOP_REG, base, sram1_tlb_axi_port_sel, 0);
	dpu_write(hwdev, MMU_TOP_REG, base, dmac0_rd_outs_num, rd_outs_num);
	dpu_write(hwdev, MMU_TOP_REG, base, dmac1_rd_outs_num, rd_outs_num);
}

void saturn_hee_irq_enable(struct spacemit_crtc *a_crtc, bool enable)
{
	u32 offset = DPU_INT_BASE_ADDR + DPU_ONLINE_IRQ_MSK;
	u32 mask;

	mask = saturn_hee_get_irq_bit(INT_UNDERRUN) |
	       saturn_hee_get_irq_bit(INT_CFG_RDY);
	if (enable) {
		struct spacemit_drm_private *priv =
				a_crtc->crtc.dev->dev_private;

		/*
		 * Ack first: a surviving status bit would fire as the mask
		 * opens.
		 */
		regmap_write(priv->hwdev->regmap,
			     DPU_INT_BASE_ADDR + DPU_ONLINE_IRQ_STS, mask);
	}
	saturn_enable_irq_mask(a_crtc, enable, offset, mask);
	a_crtc->ur_mask_armed = enable;
}

void saturn_hee_dpu_init(struct spacemit_crtc *a_crtc,
			 const struct drm_display_mode *mode)
{
	saturn_init_regs(a_crtc, mode);
	saturn_setup_dma_top(a_crtc);
	saturn_setup_mmu_top(a_crtc);
	saturn_hee_irq_enable(a_crtc, true);
}

#define SATURN_WRITE_COMPOSER_LAYER(id_name) \
do {\
	dpu_write(hwdev, CMPS_X_REG, base, layer ## id_name ## _colorkey_en,	\
		  0, cl_cmp);					\
	dpu_write(hwdev, CMPS_X_REG, base, layer ## id_name ## _blend_mode,	\
		  blend_mode, cl_cmp);				\
	dpu_write(hwdev, CMPS_X_REG, base, layer ## id_name ## _blend_sel,	\
		  alpha_sel, cl_cmp);				\
	dpu_write(hwdev, CMPS_X_REG, base, layer ## id_name ## _layer_alpha,	\
		  alpha, cl_cmp);					\
	dpu_write(hwdev, CMPS_X_REG, base, layer ## id_name ## _alpha_ratio,	\
		  0, cl_cmp);					\
	dpu_write(hwdev, CMPS_X_REG, base, layer ## id_name ## _area_left,	\
		  crtc_x, cl_cmp);					\
	dpu_write(hwdev, CMPS_X_REG, base, layer ## id_name ## _area_top,	\
		  crtc_y, cl_cmp);					\
	dpu_write(hwdev, CMPS_X_REG, base, layer ## id_name ## _area_right,	\
		  crtc_x + crtc_w - 1, cl_cmp);			\
	dpu_write(hwdev, CMPS_X_REG, base, layer ## id_name ## _area_bottom,	\
		  crtc_y + crtc_h - 1, cl_cmp);			\
	dpu_write(hwdev, CMPS_X_REG, base, layer ## id_name ## _solid_en,	\
		  0, cl_cmp);				\
	dpu_write(hwdev, CMPS_X_REG, base, layer ## id_name ## _dma_id,	\
		  rdma_id, cl_cmp);			\
	dpu_write(hwdev, CMPS_X_REG, base, layer ## id_name ## _en, 1, cl_cmp); \
} while (0)

static void saturn_write_fbcmem_regs(struct spacemit_hw_device *hwdev,
				     struct drm_plane_state *state, u32 base,
				     struct cmdlist_regs *cl_rdma)
{
	u32 size = to_spacemit_plane_state(state)->fbcmem_size;
	u32 val;

	/* The plane owns the front of its bank: the base address stays 0. */
	val = FIELD_PREP(dpu_field_mask(RDMA_PATH_X_REG, fbc_mem_map), 1) |
	      FIELD_PREP(dpu_field_mask(RDMA_PATH_X_REG, fbc_mem_size), size);
	dpu_write_word(hwdev, base,
		       dpu_field_word(RDMA_PATH_X_REG, fbc_mem_size), val,
		       cl_rdma);
}

void saturn_hee_plane_update_hw_channel(struct drm_plane *plane,
					struct drm_plane_state *state,
					const struct drm_display_mode *mode)
{
	struct spacemit_crtc *a_crtc = to_spacemit_crtc(state->crtc);
	struct drm_framebuffer *fb = state->fb;
	struct spacemit_plane_state *spacemit_plane_state =
			to_spacemit_plane_state(state);
	struct cmdlist_regs *cl_cmp = a_crtc->cl_cmp;
	u32 rdma_id = to_spacemit_plane(plane)->rdma_id;
	u8 alpha = state->alpha >> 8;
	u16 pixel_alpha = state->pixel_blend_mode;
	u32 src_w, src_h, src_x, src_y;
	u32 crtc_w, crtc_h, crtc_x, crtc_y;
	u32 alpha_sel, blend_mode = 0;
	u32 base;
	struct spacemit_drm_private *priv = plane->dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;
	struct cmdlist_regs *cl_rdma = to_spacemit_plane(plane)->cl_rdma;

	/* Clipped by drm_atomic_helper_check_plane_state(); src is 16.16. */
	src_x = state->src.x1 >> 16;
	src_y = state->src.y1 >> 16;
	src_w = drm_rect_width(&state->src) >> 16;
	src_h = drm_rect_height(&state->src) >> 16;

	crtc_x = state->dst.x1;
	crtc_y = state->dst.y1;
	crtc_w = drm_rect_width(&state->dst);
	crtc_h = drm_rect_height(&state->dst);

	drm_dbg(plane->dev, "crtc_x %u crtc_y %u\n", crtc_x, crtc_y);

	base = RDMA_BASE_ADDR[rdma_id];
	/* linear scanout only: the FBC decode path is not exposed */
	dpu_write(hwdev, RDMA_PATH_X_REG, base, layer_mode, 0, cl_rdma);
	dpu_write(hwdev, RDMA_PATH_X_REG, base, layer_cmpsr_id, COMPOSER1,
		  cl_rdma);
	/*
	 * force online-mode routing whatever the bootloader left behind
	 */
	dpu_write(hwdev, RDMA_PATH_X_REG, base, is_offline, 0, cl_rdma);
	dpu_write(hwdev, RDMA_PATH_X_REG, base, is_two_layers, 0, cl_rdma);

	dpu_write(hwdev, RDMA_PATH_X_REG, base, img_width_ly0, fb->width,
		  cl_rdma);
	dpu_write(hwdev, RDMA_PATH_X_REG, base, img_height_ly0, fb->height,
		  cl_rdma);
	dpu_write(hwdev, RDMA_PATH_X_REG, base, bbox_start_x_ly0, src_x,
		  cl_rdma);
	dpu_write(hwdev, RDMA_PATH_X_REG, base, bbox_start_y_ly0, src_y,
		  cl_rdma);
	dpu_write(hwdev, RDMA_PATH_X_REG, base, bbox_end_x_ly0,
		  src_x + src_w - 1, cl_rdma);
	dpu_write(hwdev, RDMA_PATH_X_REG, base, bbox_end_y_ly0,
		  src_y + src_h - 1, cl_rdma);

	saturn_write_fbcmem_regs(hwdev, state, base, cl_rdma);

	dpu_write(hwdev, RDMA_PATH_X_REG, base, rot_mode_ly0, 0, cl_rdma);
	dpu_write(hwdev, RDMA_PATH_X_REG, base, uv_swap, 0, cl_rdma);
	dpu_write(hwdev, RDMA_PATH_X_REG, base, pixel_format,
		  spacemit_plane_state->format, cl_rdma);

	switch (pixel_alpha) {
	case DRM_MODE_BLEND_COVERAGE:
		blend_mode = 0x0;
		break;
	case DRM_MODE_BLEND_PREMULTI:
		blend_mode = 0x1;
		break;
	case DRM_MODE_BLEND_PIXEL_NONE:
		blend_mode = 0x0;
		break;
	default:
		drm_err(plane->dev, "unsupported blend mode for pixel alpha\n");
	}

	if (fb->format->has_alpha && pixel_alpha != DRM_MODE_BLEND_PIXEL_NONE) {
		if (alpha != 0xff)
			alpha_sel = 0x2;
		else
			alpha_sel = 0x1;
	} else {
		blend_mode = 0x0;
		alpha_sel = 0x0;
	}

	base = CMP1_BASE_ADDR;
	dpu_write(hwdev, CMPS_X_REG, base, dst_w, mode->hdisplay, cl_cmp);
	dpu_write(hwdev, CMPS_X_REG, base, dst_h, mode->vdisplay, cl_cmp);

	SATURN_WRITE_COMPOSER_LAYER(00);
	spacemit_cmdlist_regs_packing(spacemit_crtc_to_cl(state->crtc), CMDLIST_MOD_COMP,
				      cl_cmp);
}

void saturn_hee_plane_disable_hw_channel(struct drm_plane *plane,
					 struct drm_crtc *crtc)
{
	u32 base = CMP1_BASE_ADDR;
	struct spacemit_drm_private *priv = plane->dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;
	struct cmdlist_regs *cl_cmp = to_spacemit_crtc(crtc)->cl_cmp;

	dpu_write(hwdev, CMPS_X_REG, base, layer00_en, 0, cl_cmp);

	spacemit_cmdlist_regs_packing(spacemit_crtc_to_cl(crtc), CMDLIST_MOD_COMP,
				      cl_cmp);
}

void saturn_hee_conf_dpuctrl(struct drm_crtc *crtc,
			     struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state =
			drm_atomic_get_new_crtc_state(state, crtc);
	u32 rdma_en;
	u32 base;
	u32 pp_base = POST_PIPE_ADDR;
	struct spacemit_drm_private *priv = crtc->dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;
	struct spacemit_crtc *a_crtc = to_spacemit_crtc(crtc);
	const struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	struct cmdlist_regs *pp_cl = a_crtc->cl_pp;

	base = DPU_SCENE_CTL_ADDR(COMPOSER1);
	rdma_en = saturn_conf_dpuctrl_rdma(state, crtc_state);

	/* The post-pipe needs its input size even when it processes nothing. */
	dpu_write_word(hwdev, pp_base, 0, 0, pp_cl);
	dpu_write(hwdev, POSTPIPE_REG, pp_base, m_inwidth, mode->hdisplay,
		  pp_cl);
	dpu_write(hwdev, POSTPIPE_REG, pp_base, m_inheight, mode->vdisplay,
		  pp_cl);

	spacemit_cmdlist_regs_packing(spacemit_crtc_to_cl(crtc), CMDLIST_MOD_COMP, pp_cl);

	dpu_write(hwdev, DPU_CTL_REG, base, nml_scl_en, 0);
	dpu_write(hwdev, DPU_CTL_REG, base, nml_rch_en, rdma_en);
	dpu_write(hwdev, DPU_CTL_REG, base, timing_inter0, 8);
	dpu_write(hwdev, DPU_CTL_REG, base, timing_inter1, 8);
	dpu_write(hwdev, DPU_CTL_REG, base, video_mod, 1);
	dpu_write(hwdev, DPU_CTL_REG, base, dbg_mod, 0);

	dpu_write(hwdev, DPU_CTL_REG, base, nml_outctl_en, 0x1);
	dpu_write(hwdev, DPU_CTL_REG, base, nml_frm_timing_en, 2);

	spacemit_cmdlist_submit_crtc(a_crtc);
}

u32 saturn_hee_get_cfg_rdy(struct spacemit_crtc *a_crtc,
			   struct spacemit_hw_device *hwdev)
{
	u32 base = DPU_SCENE_CTL_ADDR(COMPOSER1);

	return dpu_read_word(hwdev, base,
			     dpu_field_word(DPU_CTL_REG, both_cfg_rdy));
}

u32 saturn_hee_get_int_sts(struct spacemit_hw_device *hwdev)
{
	return dpu_read_word(hwdev, DPU_INT_BASE_ADDR, DPU_ONLINE_IRQ_STS / 4);
}

static const struct spacemit_crtc_irq_bitfield {
	enum spacemit_dpu_irq irq_id;
	u32 irq_bit;
} irq_list[] = {
	[INT_UNDERRUN] = {INT_UNDERRUN, DPU_INT_FRM_TIMING_UNFLOW},
	[INT_CFG_RDY] = {INT_CFG_RDY, DPU_INT_CFG_RDY_CLR},
	[INT_VSYNC] = {INT_VSYNC, DPU_INT_FRM_TIMING_VSYNC},
	[INT_EOF] = {INT_EOF, DPU_INT_FRM_TIMING_EOF},
	[INT_REST] = {INT_REST, DPU_REST_INT_BITS},
};

u32 saturn_hee_get_irq_bit(enum spacemit_dpu_irq irq_id)
{
	return irq_list[irq_id].irq_bit;
}

void saturn_hee_clr_int_sts(struct spacemit_crtc *a_crtc, u32 data)
{
	struct spacemit_drm_private *priv = a_crtc->crtc.dev->dev_private;
	struct spacemit_hw_device *hwdev = priv->hwdev;

	regmap_write(hwdev->regmap, DPU_INT_BASE_ADDR + DPU_ONLINE_IRQ_STS,
		     data);
	if ((saturn_hee_get_irq_bit(INT_UNDERRUN) & data) &&
	    a_crtc->ur_mask_armed) {
		saturn_hee_enable_irq(a_crtc, INT_UNDERRUN, false);
		a_crtc->ur_mask_armed = false;
	}
}

void saturn_hee_enable_cmdlist(struct spacemit_crtc *a_crtc, struct spacemit_hw_device *hwdev,
			       int id, bool enable)
{
	u32 base = DPU_SCENE_CTL_ADDR(COMPOSER1);
	u32 val;

	dpu_write_reg_arr(hwdev, DPU_CTL_TOP_REG, DPU_CTRL_BASE_ADDR,
			  dpu_ctl_top_reg_0, id, cmdlist_rch_en,
			  enable ? 1 : 0);
	/* One layer per channel: clear its vertical-reuse bit. */
	val = dpu_read_reg(hwdev, DPU_CTL_REG, base, nml_rch_vrt_reuse);
	val &= ~BIT(id);
	dpu_write(hwdev, DPU_CTL_REG, base, nml_rch_vrt_reuse, val);
}

void saturn_hee_cfg_cmdlist(struct spacemit_hw_device *hwdev, int id, u32 chy,
			    u32 addrl, u32 addrh)
{
	dpu_write_reg_arr(hwdev, CMDLIST_REG, CMDLIST_BASE_ADDR,
			  cmdlist_reg_48, id, cmdlist_ch_y_first, chy);
	dpu_write_reg_arr(hwdev, CMDLIST_REG, CMDLIST_BASE_ADDR,
			  cmdlist_reg_0, id, cmdlist_ch_start_addrl, addrl);
	dpu_write_reg_arr(hwdev, CMDLIST_REG, CMDLIST_BASE_ADDR,
			  cmdlist_reg_16, id, cmdlist_ch_start_addrh, addrh);
}

void saturn_hee_rdma_dmmu(struct spacemit_hw_device *hwdev, u8 tbu_id, struct tbu_instance *tbu,
			  struct drm_framebuffer *fb,
			  struct cmdlist_regs *cl_rdma,
			  struct drm_plane *plane)
{
	u32 base, val;
	u8 rdma_id = tbu_id / 2;
	struct cmdlist_regs *cl_tbu = to_spacemit_plane(plane)->cl_tbu;

	SATURN_WRITE_RDMA_ADDR_REG(hwdev, 0, rdma_id, tbu->tbu_va[0], cl_rdma);
	SATURN_WRITE_RDMA_ADDR_REG(hwdev, 1, rdma_id, tbu->tbu_va[1], cl_rdma);
	SATURN_WRITE_RDMA_ADDR_REG(hwdev, 2, rdma_id, tbu->tbu_va[2], cl_rdma);

	base = RDMA_BASE_ADDR[rdma_id];
	dpu_write(hwdev, RDMA_PATH_X_REG, base, rdma_stride0_layer0,
		  fb->pitches[0], cl_rdma);
	dpu_write(hwdev, RDMA_PATH_X_REG, base, rdma_stride1_layer0,
		  fb->pitches[1], cl_rdma);

	base = MMU_TBU_BASE_ADDR_ARRAY[tbu_id];
	SATURN_WRITE_TBU_REGS(hwdev, base, 0, cl_tbu);
	SATURN_WRITE_TBU_REGS(hwdev, base, 1, cl_tbu);
	SATURN_WRITE_TBU_REGS(hwdev, base, 2, cl_tbu);

	/* Linear scanout of one plane: the mode and plane count stay 0. */
	val = FIELD_PREP(dpu_field_mask(MMU_TBU_REG, tbu_en), 1) |
	      FIELD_PREP(dpu_field_mask(MMU_TBU_REG, tbu_burst_limit_en), 1) |
	      FIELD_PREP(dpu_field_mask(MMU_TBU_REG, tlb_fetch_active_en), 1) |
	      FIELD_PREP(dpu_field_mask(MMU_TBU_REG, tbu_qos), DPU_QOS_URGENT);
	dpu_write_word(hwdev, base, dpu_field_word(MMU_TBU_REG, tbu_en), val,
		       cl_tbu);
	spacemit_cmdlist_regs_packing(spacemit_plane_to_cl(plane), CMDLIST_MOD_DMMU, cl_tbu);
}

struct cmdlist_header {
	u64 next_list_addr : 39;
	/* reserved */
	u32: 1;
	u32 nod_len : 16;
	/* 1: the last cmdlist node, 2: pending node, otherwise 0 */
	u32 nod_type : 2;
	u32 next_nod_secu : 1;
	u64 wait_event_low    : 5;
	u64 wait_event    : 59;
	/* reserved */
	u32: 5;
};

struct cmdlist_row {
	u32 module_cfg_addr   : 19;
	u32 module_cfg_strobe : 12;
	/* the last row tag = 1, else tag = 0. */
	u32 row_eof_tag       : 1;
	u32 module_regs[3];
};

/* DMA stream layout consumed by the cmdlist engine; it relies on the sizes. */
static_assert(sizeof(struct cmdlist_header) == CMDLIST_HEADER_SIZE);
static_assert(sizeof(struct cmdlist_row) == 16);

#define CL_HEADER_SZ sizeof(struct cmdlist_header)
#define CL_ROW_SZ    sizeof(struct cmdlist_row)

#define CMDLIST_RDMA_CFG_RDY(ch) BIT(ch)
#define CMDLIST_CMPS_CFG_RDY(ch) BIT((ch) + 20)

#define CMDLIST_WAIT_EVENT_BIT_RDMA_RELOAD BIT(4)
#define CMDLIST_WAIT_EVENT_BIT_CMPS_RELOAD BIT(6)

/* Returns false, without writing, when the row would overrun the buffer. */
bool saturn_hee_cmdlist_fill_data_row(struct cmdlist *cl, u32 strobe,
				      u32 offset, u32 value[])
{
	struct cmdlist_row *row;
	u8 i;

	if (CL_HEADER_SZ + (cl->nod_len + 1) * CL_ROW_SZ > cl->size)
		return false;

	/* The buffer is reused: write the whole row, end tag included. */
	row = (struct cmdlist_row *)((char *)cl->va + CL_HEADER_SZ) + cl->nod_len;
	*row = (struct cmdlist_row) {
		.module_cfg_addr = offset >> 2,
		.module_cfg_strobe = strobe,
	};

	for (i = 0; i < CMDLIST_ROW_REGS; i++) {
		if (strobe & CMDLIST_REG_STROBE(i))
			row->module_regs[i] = value[i];
	}

	return true;
}

/*
 * Close a list: the compose-start row (plane lists only), the CFG_RDY row that
 * ends it, and the header. A list is a single node, so the header never
 * chains to another.
 */
void saturn_hee_cmdlist_fill_conf_row(struct cmdlist *cl, u32 ch_y)
{
	struct cmdlist_header *header;
	struct cmdlist_row *row;
	struct drm_device *drm;
	u8 rch_id = 0;
	u32 wait_event_bits = 0, cfg_rdy_bits = 0;

	if (cl->type == CMDLIST_PLANE) {
		rch_id = to_spacemit_plane(cl->plane)->rdma_id;
		drm = cl->plane->dev;
	} else {
		drm = cl->crtc->dev;
	}

	/* At most two rows are appended below; bail before overrunning. */
	if (drm_WARN_ONCE(drm, CL_HEADER_SZ + (cl->nod_len + 2) * CL_ROW_SZ > cl->size,
			  "cmdlist buffer full (type %u, rows %u)\n",
			  cl->type, cl->nod_len))
		return;

	header = (struct cmdlist_header *)(cl->va);
	row = (struct cmdlist_row *)((char *)header + CL_HEADER_SZ + cl->nod_len * CL_ROW_SZ);

	if (cl->type == CMDLIST_PLANE) {
		/* The composer must start fetching 24 lines ahead of the layer. */
		*row = (struct cmdlist_row) {
			.module_cfg_addr = (DPU_CTRL_BASE_ADDR +
					    CMDLIST_CH_START_CMPS_Y + rch_id * 4) >> 2,
			.module_cfg_strobe = CMDLIST_REG_STROBE(0),
			.module_regs[0] = ch_y >= 24 ? ch_y - 24 : 0,
		};
		cl->nod_len++;
		row++;
	}

	if (cl->mode_mask & CMDLIST_MOD_RDMA) {
		wait_event_bits |= CMDLIST_WAIT_EVENT_BIT_RDMA_RELOAD;
		cfg_rdy_bits |= CMDLIST_RDMA_CFG_RDY(rch_id);
	}
	if (cl->mode_mask & CMDLIST_MOD_COMP) {
		wait_event_bits |= CMDLIST_WAIT_EVENT_BIT_CMPS_RELOAD;
		cfg_rdy_bits |= CMDLIST_CMPS_CFG_RDY(COMPOSER1);
	}
	*row = (struct cmdlist_row) {
		.module_cfg_addr = (DPU_CTRL_BASE_ADDR + CMDLIST_CFG_READY) >> 2,
		.module_cfg_strobe = CMDLIST_REG_STROBE(0),
		.row_eof_tag = 1,
		.module_regs[0] = cfg_rdy_bits,
	};
	cl->nod_len++;

	header->nod_type = 1;			/* last (and only) node */
	header->nod_len = cl->nod_len + 1;	/* include header */
	header->wait_event_low = wait_event_bits & 0x1F;
	header->wait_event = wait_event_bits >> 5;
}

void saturn_hee_crtc_cmdlist(struct spacemit_hw_device *hwdev, dma_addr_t pa,
			     u8 crtc_id)
{
	u8 ch = hwdev->rdma_nums + 2 + crtc_id;
	u32 val;

	dpu_write_reg_arr(hwdev, CMDLIST_REG, CMDLIST_BASE_ADDR, cmdlist_reg_48,
			  ch, cmdlist_ch_y_first, 0);
	val = (lower_32_bits(pa) & CMDLIST_ADDRL_ALIGN_MASK) >>
	      CMDLIST_ADDRL_ALIGN_BITS;
	dpu_write_reg_arr(hwdev, CMDLIST_REG, CMDLIST_BASE_ADDR, cmdlist_reg_0,
			  ch, cmdlist_ch_start_addrl, val);
	dpu_write_reg_arr(hwdev, CMDLIST_REG, CMDLIST_BASE_ADDR, cmdlist_reg_16,
			  ch, cmdlist_ch_start_addrh, upper_32_bits(pa));
	dpu_write_reg_arr(hwdev, DPU_CTL_TOP_REG, DPU_CTRL_BASE_ADDR,
			  dpu_ctl_top_reg_10, COMPOSER1, cmdlist_cmps_top_en, 1);
	dpu_write_reg_arr(hwdev, DPU_CTL_TOP_REG, DPU_CTRL_BASE_ADDR,
			  dpu_ctl_top_reg_10, COMPOSER1, cmdlist_cmps_other_en, 1);
}
