/**
 * @file config.h
 * SoftHdDevice config header file
 *
 * @copyright (c) 2011, 2015 by Johns.  All Rights Reserved.
 * @copyright (c) 2025 by Andreas Baierl. All Rights Reserved.
 *
 * @license{AGPLv3
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.}
 */

#ifndef __SOFTHDCONFIG_H
#define __SOFTHDCONFIG_H

#include <mutex>

/*****************************************************************************
 * Config
 ****************************************************************************/
class cSoftHdConfig
{
public:
	cSoftHdConfig(void) = default;

	bool SetupParse(const char *, const char *);

	// setup conf parameters
#ifdef USE_GLES
	int ConfigMaxSizeGPUImageCache = 128;       ///< config max gpu image cache size
	int ConfigDisableOglOsd = 0;                ///< config disable ogl osd
#endif
	int ConfigAdditionalBufferLengthMs = 0;     ///< config size ms of a/v buffer
	int ConfigVideoAudioDelayMs = 0;            ///< config audio delay
	int ConfigAudioPassthroughMask = 0;         ///< config audio pass-through mask
	bool ConfigAudioPassthroughState = false;   ///< flag audio-passthrough on/off
	bool ConfigAudioDownmix = false;            ///< config ffmpeg audio downmix
	bool ConfigAudioSoftvol = false;            ///< config use software volume
	bool ConfigAudioNormalize = false;          ///< config use normalize volume
	int ConfigAudioMaxNormalize = 0;            ///< config max normalize factor
	bool ConfigAudioCompression = false;        ///< config use volume compression
	int ConfigAudioMaxCompression = 0;          ///< config max volume compression
	int ConfigAudioStereoDescent = 0;           ///< config reduce stereo loudness
	int ConfigAudioAutoAES = 0;                 ///< config automatic AES handling
	int ConfigAudioEq = 0;                      ///< config equalizer filter
	int ConfigAudioEqBand[18] =
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0 };      ///< config equalizer filter bands
	bool ConfigHideMainMenuEntry = false;       ///< config hide main menu entry
	bool ConfigLogState = true;                 ///< flag logging on/off
	int ConfigLogLevels = 0;                    ///< loglevel config
	bool ConfigDisableDeint = false;            ///< disable deinterlacer
	bool ConfigDecoderNeedsIFrame = false;      ///< start h264 decoder only when an I-Frame arrives
	bool ConfigParseH264Dimensions = false;     ///< parse h264 stream for width and height for decoder init
	bool ConfigDecoderFallbackToSw = false;     ///< fallback to software decoder if the hardware decoder fails
	int ConfigDecoderFallbackToSwNumPkts = 22;  ///< maximum number of packets sent before fallback to sw decoder
	bool ConfigParseH264StreamStart = false;    ///< log the nal units at stream start until a second i-Frame arrives

	// pip - default position at right top, 25% scaled
	int ConfigPipScalePercent = 25;             ///< scale factor of pip video
	int ConfigPipLeftPercent = 100;             ///< 0 = aligned to left, 100 = aligned to right
	int ConfigPipTopPercent = 0;                ///< 0 = aligned to top, 100 = aligned to bottom

	int ConfigPipUseAlt = false;
	// alternative position at left top, 25% scaled
	int ConfigPipAltScalePercent = 25;          ///< alternative scale factor of pip video
	int ConfigPipAltLeftPercent = 0;            ///< 0 = aligned to left, 100 = aligned to right
	int ConfigPipAltTopPercent = 0;             ///< 0 = aligned to top, 100 = aligned to bottom

	// command line parameters
	const char *ConfigAudioPCMDevice = nullptr;         ///< audio PCM device
	const char *ConfigAudioPassthroughDevice = nullptr; ///< audio passthrough device
	const char *ConfigAudioMixerChannel = nullptr;      ///< audio mixer channel name
	const char *ConfigDisplayResolution = nullptr;      ///< display resolution (syntax: "1920x1080@50")

	// stats
	const char *CurrentDecoderName = "unknown";
	const char *CurrentDecoderType = "unknown";

	void PrintLogLevel(int);

	void SetDecoderNeedsMaxPackets(int);
	int GetDecoderNeedsMaxPackets(void);

private:
	int m_decoderNeedsMaxPackets = 0;
	std::mutex m_mutex;
};

#endif
