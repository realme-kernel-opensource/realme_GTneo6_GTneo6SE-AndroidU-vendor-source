// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <sde_hw_mdss.h>
#include <sde_hw_sspp.h>
#include "dsi_iris_api.h"
#include "dsi_iris.h"
#include "dsi_iris_log.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lp.h"

void iris_sde_plane_setup_csc(void *csc_ptr)
{
	static const struct sde_csc_cfg hdrYUV = {
		{
			0x00010000, 0x00000000, 0x00000000,
			0x00000000, 0x00010000, 0x00000000,
			0x00000000, 0x00000000, 0x00010000,
		},
		{ 0x0, 0x0, 0x0,},
		{ 0x0, 0x0, 0x0,},
		{ 0x0, 0x3ff, 0x0, 0x3ff, 0x0, 0x3ff,},
		{ 0x0, 0x3ff, 0x0, 0x3ff, 0x0, 0x3ff,},
	};
	static const struct sde_csc_cfg hdrRGB10 = {
		/* S15.16 format */
		{
			0x00012A15, 0x00000000, 0x0001ADBE,
			0x00012A15, 0xFFFFD00B, 0xFFFF597E,
			0x00012A15, 0x0002244B, 0x00000000,
		},
		/* signed bias */
		{ 0xffc0, 0xfe00, 0xfe00,},
		{ 0x0, 0x0, 0x0,},
		/* unsigned clamp */
		{ 0x40, 0x3ac, 0x40, 0x3c0, 0x40, 0x3c0,},
		{ 0x00, 0x3ff, 0x00, 0x3ff, 0x00, 0x3ff,},
	};

	if (!iris_is_chip_supported())
		return;

	if (iris_get_hdr_enable() == 1)
		csc_ptr = (void *)&hdrYUV;
	else if (iris_get_hdr_enable() == 2)
		csc_ptr = (void *)&hdrRGB10;
}


int iris_sde_kms_iris_operate(struct msm_kms *kms,
		u32 operate_type, struct msm_iris_operate_value *operate_value)
{
	int ret = -EINVAL;

	if (!iris_is_chip_supported() && !iris_is_softiris_supported())
		return 0;

	if (operate_type == DRM_MSM_IRIS_OPERATE_CONF)
		ret = iris_operate_conf(operate_value);
	else if (operate_type == DRM_MSM_IRIS_OPERATE_TOOL)
		ret = iris_operate_tool(operate_value);

	return ret;
}


void iris_sde_update_dither_depth_map(uint32_t *map, uint32_t depth)
{
	if (!iris_is_chip_supported())
		return;

	if (depth >= 9) {
		map[5] = 1;
		map[6] = 2;
		map[7] = 3;
		map[8] = 2;
	}
}

void iris_sde_prepare_commit(uint32_t num_phys_encs, void *phys_enc)
{
}

void iris_sde_prepare_for_kickoff(uint32_t num_phys_encs, void *phys_enc)
{
	if (!iris_is_chip_supported() && !iris_is_softiris_supported())
		return;

	if (num_phys_encs == 0)
		return;

	if (iris_is_chip_supported())
		iris_prepare_for_kickoff(phys_enc);
	iris_sync_panel_brightness(1, phys_enc);
}

void iris_sde_encoder_kickoff(uint32_t num_phys_encs, void *phys_enc)
{
	if (!iris_is_chip_supported() && !iris_is_softiris_supported())
		return;

	if (num_phys_encs == 0)
		return;

	if (iris_is_chip_supported())
		iris_kickoff(phys_enc);
	iris_sync_panel_brightness(2, phys_enc);
}

void iris_sde_encoder_sync_panel_brightness(uint32_t num_phys_encs, void *phys_enc)
{
	if (!iris_is_chip_supported() && !iris_is_softiris_supported())
		return;

	if (num_phys_encs == 0)
		return;

	iris_sync_panel_brightness(3, phys_enc);
}

void iris_sde_encoder_wait_for_event(uint32_t num_phys_encs, void *phys_enc,
		uint32_t event)
{
	if (!iris_is_chip_supported() && !iris_is_softiris_supported())
		return;

	if (num_phys_encs == 0)
		return;

	if (event != MSM_ENC_COMMIT_DONE)
		return;

	iris_sync_panel_brightness(4, phys_enc);
}
void iris_sde_crtc_atomic_begin(struct drm_crtc *crtc,
				struct drm_crtc_state *old_state)
{
	if (iris_is_chip_supported())
		iris_set_metadata(true);
}
