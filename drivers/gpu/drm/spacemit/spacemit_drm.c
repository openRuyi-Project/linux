// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025-2026 SpacemiT Co., Ltd.
 *
 */

#include <linux/component.h>
#include <linux/of_graph.h>
#include <linux/slab.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_dumb_buffers.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_of.h>

#include "spacemit_crtc.h"
#include "spacemit_dmmu.h"
#include "spacemit_drm.h"

#define DRIVER_NAME	"spacemit"
#define DRIVER_DESC	"SpacemiT SoC DRM driver"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static void spacemit_drm_atomic_commit_tail(struct drm_atomic_state *old_state)
{
	struct drm_device *dev = old_state->dev;

	/*
	 * Enables before planes, and planes on active CRTCs only: the DPU is
	 * powered from atomic_enable to atomic_disable.
	 */
	drm_atomic_helper_commit_modeset_disables(dev, old_state);

	drm_atomic_helper_commit_modeset_enables(dev, old_state);

	drm_atomic_helper_commit_planes(dev, old_state,
					DRM_PLANE_COMMIT_ACTIVE_ONLY);

	/*
	 * Flip first, hw_done second: each channel has two command lists
	 * written alternately, and the engine may re-read the previous one
	 * until the next CFG_RDY has been consumed. Holding the next commit
	 * back until this one's vblank keeps it from rewriting a list still
	 * in use.
	 */
	drm_atomic_helper_wait_for_flip_done(dev, old_state);

	drm_atomic_helper_commit_hw_done(old_state);

	drm_atomic_helper_cleanup_planes(dev, old_state);
}

static const struct drm_mode_config_helper_funcs spacemit_drm_mode_config_helper = {
	.atomic_commit_tail = spacemit_drm_atomic_commit_tail,
};

static const struct drm_mode_config_funcs spacemit_drm_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

int spacemit_drm_mode_config_init(struct drm_device *drm)
{
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = 1;
	drm->mode_config.min_height = 1;
	/*
	 * Framebuffer bounds come from the RDMA register widths: a 16-bit
	 * byte stride (16383 pixels at 32 bpp) and 16-bit image dimensions.
	 * The 3840x2160 output limit is enforced per CRTC in mode_valid.
	 */
	drm->mode_config.max_width = SPACEMIT_DPU_MAX_FB_WIDTH;
	drm->mode_config.max_height = SPACEMIT_DPU_MAX_FB_HEIGHT;

	drm->mode_config.funcs = &spacemit_drm_mode_config_funcs;
	drm->mode_config.helper_private = &spacemit_drm_mode_config_helper;

	return 0;
}

static void spacemit_gem_free(struct drm_gem_object *obj)
{
	spacemit_dmmu_release(obj);
	drm_gem_shmem_free(to_drm_gem_shmem_obj(obj));
}

/* drm_gem_shmem_funcs with a free that also drops the DMMU table. */
static const struct drm_gem_object_funcs spacemit_gem_funcs = {
	.free = spacemit_gem_free,
	.print_info = drm_gem_shmem_object_print_info,
	.pin = drm_gem_shmem_object_pin,
	.unpin = drm_gem_shmem_object_unpin,
	.get_sg_table = drm_gem_shmem_object_get_sg_table,
	.vmap = drm_gem_shmem_object_vmap,
	.vunmap = drm_gem_shmem_object_vunmap,
	.mmap = drm_gem_shmem_object_mmap,
	.vm_ops = &drm_gem_shmem_vm_ops,
};

static struct drm_gem_object *spacemit_gem_create_object(struct drm_device *drm,
							 size_t size)
{
	struct spacemit_gem_object *sgem;

	sgem = kzalloc_obj(*sgem);
	if (!sgem)
		return ERR_PTR(-ENOMEM);

	mutex_init(&sgem->tbl_lock);
	sgem->base.map_wc = true;
	sgem->base.base.funcs = &spacemit_gem_funcs;

	return &sgem->base.base;
}

/*
 * A new buffer's pages are cleared through the cached linear map, while
 * everything drawn afterwards goes through a write-combining mapping. Build
 * the sg table now: mapping it writes the cached zeroes back before anything
 * is drawn. Left to the first scanout, they would land on top of the picture.
 */
static int spacemit_gem_dumb_create(struct drm_file *file,
				    struct drm_device *drm,
				    struct drm_mode_create_dumb *args)
{
	struct drm_gem_shmem_object *shmem;
	struct sg_table *sgt;
	int ret;

	ret = drm_mode_size_dumb(drm, args, 0, 0);
	if (ret)
		return ret;

	shmem = drm_gem_shmem_create(drm, args->size);
	if (IS_ERR(shmem))
		return PTR_ERR(shmem);

	sgt = drm_gem_shmem_get_pages_sgt(shmem);
	if (IS_ERR(sgt))
		ret = PTR_ERR(sgt);
	else
		ret = drm_gem_handle_create(file, &shmem->base, &args->handle);

	/* On success the handle holds the reference. */
	drm_gem_object_put(&shmem->base);

	return ret;
}

DEFINE_DRM_GEM_FOPS(spacemit_drm_fops);

const struct drm_driver spacemit_drm_drv = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &spacemit_drm_fops,

	.gem_prime_import_sg_table = drm_gem_shmem_prime_import_sg_table,
	.dumb_create = spacemit_gem_dumb_create,
	.gem_create_object = spacemit_gem_create_object,
	DRM_FBDEV_SHMEM_DRIVER_OPS,

	.name		= DRIVER_NAME,
	.desc		= DRIVER_DESC,
	.major		= DRIVER_MAJOR,
	.minor		= DRIVER_MINOR,
};

int spacemit_drm_of_component_probe(struct device *dev)
{
	struct component_match *match = NULL;
	struct device_node *remote, *ep;

	/* The DPU is the drm root: its OF graph names the DP/eDP components. */
	for_each_endpoint_of_node(dev->of_node, ep) {
		remote = of_graph_get_remote_port_parent(ep);
		if (!remote)
			continue;
		if (of_device_is_available(remote))
			drm_of_component_match_add(dev, &match,
						   component_compare_of,
						   remote);
		of_node_put(remote);
		if (IS_ERR(match)) {
			of_node_put(ep);
			return PTR_ERR(match);
		}
	}

	if (!match)
		return dev_err_probe(dev, -ENODEV,
				     "no downstream component available\n");

	return component_master_add_with_match(dev, &spacemit_drm_master_ops,
					       match);
}
