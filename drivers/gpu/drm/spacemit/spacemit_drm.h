/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025-2026 SpacemiT Co., Ltd.
 *
 */

#ifndef _SPACEMIT_DRM_H_
#define _SPACEMIT_DRM_H_

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/reset.h>

#include <drm/drm_atomic.h>
#include <drm/drm_drv.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "spacemit_dmmu.h"

/* The DPU's clock inputs, as named by the binding. */
struct dpu_clk_context {
	struct clk *pxclk;
	struct clk *mclk;
	struct clk *escclk;
	struct clk *aclk;
	struct clk *dscclk;
};

/**
 * struct spacemit_drm_private - Top-level driver state for the SpacemiT DRM
 * card
 * @ddev: DRM device; lives from the component master bind to its unbind.
 * @dev: The DPU platform device; the DPU is the drm subsystem root.
 * @a_crtc: Per-DPU CRTC state; lives as long as @ddev.
 * @hwdev: Per-revision Saturn DPU description.
 * @clk_ctx: The DPU's clocks, acquired once at probe.
 * @clocks_on: True while clocks run; MMIO on a gated DPU stalls the bus.
 * @pipe_resets: The DPU reset lines, in assert order; bulk-deassert
 *		 releases them in reverse.
 * @irq_online: Online-compose IRQ; requested disabled, enabled at bind.
 */
struct spacemit_drm_private {
	struct drm_device *ddev;
	struct device *dev;
	struct spacemit_crtc *a_crtc;
	struct spacemit_hw_device *hwdev;

	struct dpu_clk_context clk_ctx;
	bool clocks_on;
	struct reset_control_bulk_data pipe_resets[5];
	int irq_online;
};

extern const struct drm_driver spacemit_drm_drv;
extern const struct component_master_ops spacemit_drm_master_ops;

int spacemit_drm_mode_config_init(struct drm_device *drm);
int spacemit_drm_of_component_probe(struct device *dev);

#endif /* _SPACEMIT_DRM_H_ */
