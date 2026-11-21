/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _SATURN_FBCMEM_H_
#define _SATURN_FBCMEM_H_

#include <drm/drm_plane.h>

#include "../spacemit_crtc.h"
#include "../spacemit_dpu_reg.h"
#include "dpu_saturn.h"

#define FBCMEM_UNIT	(32) /* fbcmem is 32 bytes per unit */

int saturn_cal_layer_fbcmem_size(struct drm_plane *plane,
				 struct drm_plane_state *state);

#endif
