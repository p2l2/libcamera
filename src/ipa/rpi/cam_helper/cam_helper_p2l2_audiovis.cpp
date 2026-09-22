/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * camera helper for the p2l2 audiovis FPGA camera
 */

#include "cam_helper.h"

using namespace RPiController;

/*
 * Same FPGA control philosophy as p2l2vis (see cam_helper_p2l2_vis.cpp
 * and custom_camera/linux_256x_cam/REGISTERS.md): CHIP_ID, MODE_SELECT,
 * FRAME_LENGTH, EXPOSURE and ANALOGUE_GAIN, no embedded data line, no
 * PDAF, no AE histogram/HDR readout, and a single fixed sensor mode
 * (256x256 monochrome RAW16 @ 30fps rather than p2l2vis's 1536x864
 * Bayer). CamHelper's defaults (no embedded data, uniform sensitivity,
 * no extra hidden/mistrusted frames) are used unchanged.
 */
class CamHelperP2L2Audiovis : public CamHelper
{
public:
	CamHelperP2L2Audiovis();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines. Carried over from p2l2vis's placeholder value -
	 * tune it against the audiovis FPGA's actual minimum frame-length/
	 * exposure gap once known.
	 */
	static constexpr int frameIntegrationDiff = 4;
};

CamHelperP2L2Audiovis::CamHelperP2L2Audiovis()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperP2L2Audiovis::gainCode(double gain) const
{
	/*
	 * Same linear gain*256 formula as p2l2vis (see REGISTERS.md) - an
	 * assumption about what the FPGA expects ANALOGUE_GAIN's value to
	 * mean, confirm it matches the FPGA's actual gain curve/scale.
	 */
	return static_cast<uint32_t>(gain * 256);
}

double CamHelperP2L2Audiovis::gain(uint32_t gainCode) const
{
	return static_cast<double>(gainCode) / 256.0;
}

static CamHelper *create()
{
	return new CamHelperP2L2Audiovis();
}

/*
 * Must match the sensor model name libcamera reports (the kernel driver's
 * p2l2_audiovis_probe() sets the subdev name to the fixed string
 * "p2l2-audiovis", no variant suffixes) - CamHelper::create() looks this
 * up with a substring match against that name.
 */
static RegisterCamHelper reg("p2l2-audiovis", &create);
