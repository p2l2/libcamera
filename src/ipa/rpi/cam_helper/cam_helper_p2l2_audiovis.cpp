/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * camera helper for the p2l2 audiovis FPGA camera
 */

#include <iomanip>
#include <sstream>

#include <libcamera/base/log.h>

#include "cam_helper.h"

using namespace RPiController;
using namespace libcamera;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

/*
 * Same FPGA control philosophy as p2l2vis (see cam_helper_p2l2_vis.cpp
 * and custom_camera/linux_256x_cam/REGISTERS.md): CHIP_ID, MODE_SELECT,
 * FRAME_LENGTH, EXPOSURE and ANALOGUE_GAIN, no PDAF, no AE histogram/HDR
 * readout, and a single fixed sensor mode (128x128 monochrome RAW10).
 *
 * Every frame carries one embedded data line (DT 0x12, 256 bytes), written
 * by the FPGA's stream_to_mipi_tx_metadata unit, little endian:
 *   bytes 0..3  : peak value, Q16.16 beamformer power of the brightest pixel
 *   bytes 4..7  : highest_bit, the slicer window: pixel = bits
 *                 [highest_bit : highest_bit - 9] of the Q16.16 power
 *   bytes 8..11 : peak index, row major pixel index of the peak
 * The rest of the line is zero.
 */
class CamHelperP2L2Audiovis : public CamHelper
{
public:
	CamHelperP2L2Audiovis();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	bool sensorEmbeddedDataPresent() const override;
	void prepare(Span<const uint8_t> buffer, Metadata &metadata) override;

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

bool CamHelperP2L2Audiovis::sensorEmbeddedDataPresent() const
{
	return true;
}

/*
 * The layout is not SMIA/CCS (MdParserSmia only understands Sony's tag
 * format), so the three words are decoded here directly. The raw buffer is
 * dumped as well to check what arrives.
 */
void CamHelperP2L2Audiovis::prepare(Span<const uint8_t> buffer, Metadata &metadata)
{
	if (buffer.empty()) {
		LOG(IPARPI, Warning) << "No embedded data buffer received";
		return;
	}

	std::ostringstream oss;
	for (uint8_t byte : buffer.first(std::min<size_t>(buffer.size(), 512)))
		oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(byte) << ' ';

	LOG(IPARPI, Info) << "Embedded data buffer (" << buffer.size() << " bytes): " << oss.str();

	if (buffer.size() < 12) {
		LOG(IPARPI, Warning) << "Embedded data buffer too short: " << buffer.size() << " bytes";
		return;
	}

	/* Little endian, reinterpreted as int32_t, see controls::rpi::Audiovis*. */
	auto word = [&buffer](size_t offset) {
		uint32_t bits = static_cast<uint32_t>(buffer[offset]) |
				(static_cast<uint32_t>(buffer[offset + 1]) << 8) |
				(static_cast<uint32_t>(buffer[offset + 2]) << 16) |
				(static_cast<uint32_t>(buffer[offset + 3]) << 24);
		return static_cast<int32_t>(bits);
	};
	metadata.set("audiovis.embedded_value", word(0));
	metadata.set("audiovis.highest_bit", word(4));
	metadata.set("audiovis.peak_index", word(8));
}

/*
 * Must match the sensor model name libcamera reports (the kernel driver's
 * p2l2_audiovis_probe() sets the subdev name to the fixed string
 * "p2l2-audiovis", no variant suffixes) - CamHelper::create() looks this
 * up with a substring match against that name.
 */
static RegisterCamHelper reg("p2l2-audiovis", &create);
