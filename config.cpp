/**
 * @file config.cpp
 * SoftHdDevice config class
 *
 * This file defines cSoftHdConfig, which is used to keep all
 * the config settings, which are set via setup.conf, commandline
 * or setup menu.
 *
 * @copyright (c) 2011, 2015 by Johns.  All Rights Reserved.
 * @copyright (c) 2018 zille.  All Rights Reserved.
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

#include <cstring>
#include <cstdlib>
#include <mutex>

#include "config.h"
#include "logger.h"

/*****************************************************************************
 * cSoftHdConfig - Setup parameters
 ****************************************************************************/

/**
 * Parse setup parameters
 *
 * @param name      paramter name (case sensetive)
 * @param value     value as string
 *
 * @returns         true if the parameter is supported, false otherwise
 */
bool cSoftHdConfig::SetupParse(const char *name, const char *value)
{
	//LOGDEBUG("config: %s: '%s' = '%s'", __FUNCTION__, name, value);

	if        (!strcasecmp(name, "HideMainMenuEntry"))     { ConfigHideMainMenuEntry = atoi(value);
	} else if (!strcasecmp(name, "AdditionalBufferLengthMs")) { ConfigAdditionalBufferLengthMs = atoi(value);
	} else if (!strcasecmp(name, "LogLevel"))              { ConfigLogLevels = abs(atoi(value));
	                                                         ConfigLogState = atoi(value) > 0;
                                                                 PrintLogLevel(ConfigLogState ? ConfigLogLevels : 0);
	                                                         cSoftHdLogger::GetLogger()->SetLogLevel(ConfigLogState ? ConfigLogLevels : 0);
	} else if (!strcasecmp(name, "DisableDeint"))          { ConfigDisableDeint = atoi(value);
	} else if (!strcasecmp(name, "DecoderNeedsIFrame"))    { ConfigDecoderNeedsIFrame = atoi(value);
	} else if (!strcasecmp(name, "ParseH264Dimensions"))   { ConfigParseH264Dimensions = atoi(value);
	} else if (!strcasecmp(name, "DecoderFallbackToSw"))   { ConfigDecoderFallbackToSw = atoi(value);
	} else if (!strcasecmp(name, "DecoderFallbackToSwNumPkts")) { ConfigDecoderFallbackToSwNumPkts = atoi(value);
	} else if (!strcasecmp(name, "ParseH264StreamStart"))  { ConfigParseH264StreamStart = atoi(value);
	} else if (!strcasecmp(name, "AudioDelay"))            { ConfigVideoAudioDelayMs = atoi(value);
	} else if (!strcasecmp(name, "AudioPassthrough"))      { ConfigAudioPassthroughMask = abs(atoi(value)); ConfigAudioPassthroughState = atoi(value) > 0;
	} else if (!strcasecmp(name, "AudioDownmix"))          { ConfigAudioDownmix = atoi(value);
	} else if (!strcasecmp(name, "AudioSoftvol"))          { ConfigAudioSoftvol = atoi(value);
	} else if (!strcasecmp(name, "AudioNormalize"))        { ConfigAudioNormalize = atoi(value);
	} else if (!strcasecmp(name, "AudioMaxNormalize"))     { ConfigAudioMaxNormalize = atoi(value);
	} else if (!strcasecmp(name, "AudioCompression"))      { ConfigAudioCompression = atoi(value);
	} else if (!strcasecmp(name, "AudioMaxCompression"))   { ConfigAudioMaxCompression = atoi(value);
	} else if (!strcasecmp(name, "AudioStereoDescent"))    { ConfigAudioStereoDescent = atoi(value);
	} else if (!strcasecmp(name, "AudioAutoAES"))          { ConfigAudioAutoAES = atoi(value);
	} else if (!strcasecmp(name, "AudioEq"))               { ConfigAudioEq = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand01b"))        { ConfigAudioEqBand[0] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand02b"))        { ConfigAudioEqBand[1] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand03b"))        { ConfigAudioEqBand[2] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand04b"))        { ConfigAudioEqBand[3] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand05b"))        { ConfigAudioEqBand[4] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand06b"))        { ConfigAudioEqBand[5] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand07b"))        { ConfigAudioEqBand[6] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand08b"))        { ConfigAudioEqBand[7] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand09b"))        { ConfigAudioEqBand[8] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand10b"))        { ConfigAudioEqBand[9] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand11b"))        { ConfigAudioEqBand[10] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand12b"))        { ConfigAudioEqBand[11] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand13b"))        { ConfigAudioEqBand[12] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand14b"))        { ConfigAudioEqBand[13] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand15b"))        { ConfigAudioEqBand[14] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand16b"))        { ConfigAudioEqBand[15] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand17b"))        { ConfigAudioEqBand[16] = atoi(value);
	} else if (!strcasecmp(name, "AudioEqBand18b"))        { ConfigAudioEqBand[17] = atoi(value);
	} else if (!strcasecmp(name, "PipScalePercent"))       { ConfigPipScalePercent = atoi(value);
	} else if (!strcasecmp(name, "PipLeftPercent"))        { ConfigPipLeftPercent = atoi(value);
	} else if (!strcasecmp(name, "PipTopPercent"))         { ConfigPipTopPercent = atoi(value);
	} else if (!strcasecmp(name, "PipUseAlt"))             { ConfigPipUseAlt = atoi(value);
	} else if (!strcasecmp(name, "PipAltScalePercent"))    { ConfigPipAltScalePercent = atoi(value);
	} else if (!strcasecmp(name, "PipAltLeftPercent"))     { ConfigPipAltLeftPercent = atoi(value);
	} else if (!strcasecmp(name, "PipAltTopPercent"))      { ConfigPipAltTopPercent = atoi(value);
#ifdef USE_GLES
	} else if (!strcasecmp(name, "MaxSizeGPUImageCache"))  { ConfigMaxSizeGPUImageCache = atoi(value);
#endif
	} else
		return false;

	return true;
}

void cSoftHdConfig::PrintLogLevel(int loglevel)
{
	if (!loglevel)
		return;

	char prefix[256] = "Set loglevels:";
	if (loglevel & L_DEBUG)
		strcat(prefix, " standard debugs,");
	if (loglevel & L_AV_SYNC)
		strcat(prefix, " AV-Sync,");
	if (loglevel & L_SOUND)
		strcat(prefix, " sound,");
	if (loglevel & L_OSD)
		strcat(prefix, " osd,");
	if (loglevel & L_DRM)
		strcat(prefix, " drm,");
	if (loglevel & L_CODEC)
		strcat(prefix, " codec,");
	if (loglevel & L_FFMPEG)
		strcat(prefix, " ffmpeg,");
	if (loglevel & L_STILL)
		strcat(prefix, " stillpicture,");
	if (loglevel & L_TRICK)
		strcat(prefix, " trickspeed,");
	if (loglevel & L_MEDIA)
		strcat(prefix, " mediaplayer,");
	if ((loglevel & L_OPENGL) ||
	    (loglevel & L_OPENGL_TIME) ||
	    (loglevel & L_OPENGL_TIME_ALL))
		strcat(prefix, " OpenGL OSD,");
	if (loglevel & L_PACKET)
		strcat(prefix, " packet tracking,");
	if (loglevel & L_GRAB)
		strcat(prefix, " grabbing");

	LOGINFO("%s", prefix);
}

void cSoftHdConfig::SetDecoderNeedsMaxPackets(int num)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_decoderNeedsMaxPackets = num;
}

int cSoftHdConfig::GetDecoderNeedsMaxPackets(void)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_decoderNeedsMaxPackets;
}
