/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * camera helper for the p2l2 FPGA vision camera
 */

#include "cam_helper.h"

using namespace RPiController;

/*
 * The FPGA's I2C register map implements CHIP_ID, MODE_SELECT,
 * FRAME_LENGTH, EXPOSURE and ANALOGUE_GAIN (see
 * custom_camera/kernel_driver/REGISTERS.md) - but there is no embedded
 * data line, no PDAF, no AE histogram/HDR readout, and a single fixed
 * sensor mode. None of the Sony imx708-specific parsing this file was
 * copied from applies here, so it has all been removed; CamHelper's
 * defaults (no embedded data, uniform sensitivity, no extra
 * hidden/mistrusted frames) are used unchanged.
 */
class CamHelperP2L2Vis : public CamHelper
{
public:
	CamHelperP2L2Vis();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines. FRAME_LENGTH and EXPOSURE are now real FPGA
	 * registers, but this value itself is still an unverified guess -
	 * tune it against the FPGA's actual minimum frame-length/exposure
	 * gap once that's known.
	 */
	static constexpr int frameIntegrationDiff = 4;
};

CamHelperP2L2Vis::CamHelperP2L2Vis()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperP2L2Vis::gainCode(double gain) const
{
	/*
	 * The kernel driver writes this to a real, FPGA-implemented
	 * V4L2_CID_ANALOGUE_GAIN register now (see REGISTERS.md), but this
	 * linear gain*256 formula is still an assumption about what the FPGA
	 * expects that register's value to mean - confirm it matches the
	 * FPGA's actual gain curve/scale, and replace if not.
	 */
	return static_cast<uint32_t>(gain * 256);
}

double CamHelperP2L2Vis::gain(uint32_t gainCode) const
{
	return static_cast<double>(gainCode) / 256.0;
}

static CamHelper *create()
{
	return new CamHelperP2L2Vis();
}

/*
 * Must match the sensor model name libcamera reports (the kernel driver's
 * p2l2vis_identify_module() sets the subdev name to "p2l2vis" plus an
 * optional "_wide"/"_noir" suffix) - CamHelper::create() looks this up with
 * a substring match against that name.
 */
static RegisterCamHelper reg("p2l2vis", &create);
