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
 *   bytes 0..3  : peak value, unsigned Q9.23 beamformer power of the
 *                 brightest pixel
 *   bytes 4..7  : highest_bit, the slicer window: pixel = bits
 *                 [highest_bit : highest_bit - 9] of the Q9.23 power
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
 * format), so the three words are decoded here directly. The FPGA sends one
 * image line of embedded data (kEmbeddedLineBytes), only the first
 * kEmbeddedWordBytes carry data and the rest is zero. The buffer handed in is
 * the whole allocation, past the line it holds whatever was there before.
 */
void CamHelperP2L2Audiovis::prepare(Span<const uint8_t> buffer, Metadata &metadata)
{
	constexpr size_t kEmbeddedWordBytes = 12;
	constexpr size_t kEmbeddedLineBytes = 256;

	if (buffer.empty()) {
		LOG(IPARPI, Warning) << "No embedded data buffer received";
		return;
	}

	if (buffer.size() < kEmbeddedWordBytes) {
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
	const int32_t peakValue = word(0);
	const int32_t highestBit = word(4);
	const int32_t peakIndex = word(8);

	LOG(IPARPI, Debug) << "Embedded data: peak value 0x" << std::hex << std::setw(8)
			   << std::setfill('0') << static_cast<uint32_t>(peakValue) << std::dec
			   << ", highest bit " << highestBit << ", peak index " << peakIndex;

	/* Anything else in the line means the line is not what the FPGA sent. */
	const size_t lineEnd = std::min(buffer.size(), kEmbeddedLineBytes);
	for (size_t i = kEmbeddedWordBytes; i < lineEnd; i++) {
		if (buffer[i] == 0)
			continue;

		std::ostringstream oss;
		for (size_t j = i; j < std::min(lineEnd, i + 16); j++)
			oss << std::hex << std::setw(2) << std::setfill('0')
			    << static_cast<unsigned int>(buffer[j]) << ' ';

		LOG(IPARPI, Warning) << "Embedded data line not zero from byte " << i << ": "
				     << oss.str();
		break;
	}

	metadata.set("audiovis.embedded_value", peakValue);
	metadata.set("audiovis.highest_bit", highestBit);
	metadata.set("audiovis.peak_index", peakIndex);
}

/*
 * Must match the sensor model name libcamera reports (the kernel driver's
 * p2l2_audiovis_probe() sets the subdev name to the fixed string
 * "p2l2-audiovis", no variant suffixes) - CamHelper::create() looks this
 * up with a substring match against that name.
 */
static RegisterCamHelper reg("p2l2-audiovis", &create);
