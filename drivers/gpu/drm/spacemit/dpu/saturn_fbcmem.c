// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025-2026 SpacemiT Co., Ltd.
 *
 */

#include <linux/stddef.h>

#include <drm/drm_atomic.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_mode.h>

#include "../spacemit_cmdlist.h"
#include "saturn_fbcmem.h"

static int saturn_raw_plane_mem_size(u32 drm_4cc_fmt, u32 plane_crop_width,
				     u32 *output_mem_size)
{
	const struct drm_format_info *info;
	u32 ret_mem_size;
	u32 line_bytes;

	info = drm_format_info(drm_4cc_fmt);
	if (info->num_planes != 1 || info->is_yuv)
		return -EINVAL;

	/* The crop width is at most the 16-bit image width, so this fits. */
	line_bytes = drm_format_info_min_pitch(info, 0, plane_crop_width);
	if (!line_bytes)
		return -EINVAL;

	/* The hardware requires 64-byte rounding. */
	ret_mem_size = roundup(line_bytes, 64);
	ret_mem_size = ret_mem_size / 32;

	if (output_mem_size)
		*output_mem_size = ret_mem_size;

	return 0;
}

/*
 * Size the plane's share of the RDMA line buffer. Each pair of read
 * channels owns one bank; the primary plane is alone on channel 0 and takes
 * the front of bank 0, so the only check is that it fits.
 */
int saturn_cal_layer_fbcmem_size(struct drm_plane *plane,
				 struct drm_plane_state *state)
{
	struct spacemit_plane_state *pstate = to_spacemit_plane_state(state);
	struct spacemit_drm_private *priv = plane->dev->dev_private;
	struct spacemit_plane *p = to_spacemit_plane(plane);
	u32 bank = priv->hwdev->fbcmem_sizes[p->rdma_id / 2] / FBCMEM_UNIT;
	u32 drm_4cc_fmt = pstate->state.fb->format->format;
	u32 crop_w = drm_rect_width(&pstate->state.src) >> 16;
	int ret;

	/* Only linear sizing: the compressed path is not exposed. */
	ret = saturn_raw_plane_mem_size(drm_4cc_fmt, crop_w,
					&pstate->fbcmem_size);
	if (ret)
		return ret;

	return pstate->fbcmem_size > bank ? -ENOSPC : 0;
}
