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
 * readout, and a single fixed sensor mode (256x256 monochrome RAW16).
 *
 * Every frame carries one embedded data line (DT 0x12, 16 bytes) on its own
 * virtual channel (VC1), sent right after the image frame. It is written by
 * the FPGA's EmbeddedDataStreamer from the PeakDetector results of the
 * frame, two 64 bit words, little endian:
 *   bytes 0..7  : peak index, row major index of the first pixel holding
 *                 the maximum
 *   bytes 8..15 : maximum value of the frame's 32 bit input samples, before
 *                 the slicer
 * Both values fit 32 bits, the upper 4 bytes of each word are zero.
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
 * format), so the two words are decoded here directly. The buffer handed in
 * is the whole allocation, past the line it holds whatever was there before.
 */
void CamHelperP2L2Audiovis::prepare(Span<const uint8_t> buffer, Metadata &metadata)
{
	constexpr size_t kWordBytes = 8;
	constexpr size_t kEmbeddedLineBytes = 2 * kWordBytes;

	if (buffer.empty()) {
		LOG(IPARPI, Warning) << "No embedded data buffer received";
		return;
	}

	if (buffer.size() < kEmbeddedLineBytes) {
		LOG(IPARPI, Warning) << "Embedded data buffer too short: " << buffer.size() << " bytes";
		return;
	}

	auto word = [&buffer](size_t offset) {
		uint64_t bits = 0;
		for (size_t i = 0; i < kWordBytes; i++)
			bits |= static_cast<uint64_t>(buffer[offset + i]) << (8 * i);
		return bits;
	};
	const uint64_t peakIndex = word(0);
	const uint64_t maxValue = word(kWordBytes);

	LOG(IPARPI, Debug) << "Embedded data: peak index " << peakIndex
			   << ", max value 0x" << std::hex << std::setw(8)
			   << std::setfill('0') << maxValue;

	/* Set upper halves mean the line is not what the FPGA sent. */
	if ((peakIndex | maxValue) >> 32) {
		std::ostringstream oss;
		for (size_t i = 0; i < kEmbeddedLineBytes; i++)
			oss << std::hex << std::setw(2) << std::setfill('0')
			    << static_cast<unsigned int>(buffer[i]) << ' ';

		LOG(IPARPI, Warning) << "Embedded data line out of range: " << oss.str();
		return;
	}

	/* Reinterpreted as int32_t, see controls::rpi::Audiovis*. */
	metadata.set("audiovis.embedded_value", static_cast<int32_t>(maxValue));
	metadata.set("audiovis.peak_index", static_cast<int32_t>(peakIndex));
}

/*
 * Must match the sensor model name libcamera reports (the kernel driver's
 * p2l2_audiovis_probe() sets the subdev name to the fixed string
 * "p2l2-audiovis", no variant suffixes) - CamHelper::create() looks this
 * up with a substring match against that name.
 */
static RegisterCamHelper reg("p2l2-audiovis", &create);
