/**
 * @file softhdsetupmenu.h
 * Setup menu class header file
 *
 * @copyright (c) 2011, 2014 by Johns.  All Rights Reserved.
 * @copyright (c) 2018 - 2019 zille.  All Rights Reserved.
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

#ifndef __SOFTHDSETUPMENU_H
#define __SOFTHDSETUPMENU_H

#include <vdr/menuitems.h>

class cSoftHdAudio;
class cSoftHdConfig;
class cSoftHdDevice;

/*****************************************************************************
 * cMenuSetupSoft - Setup menu
 ****************************************************************************/

/**
 * cMenuSetupSoft - SoftHdDevice plugin menu setup page class
 */
class cMenuSetupSoft : public cMenuSetupPage
{
public:
	cMenuSetupSoft(cSoftHdDevice *);
	virtual eOSState ProcessKey(eKeys);

protected:
	// local copies of global setup variables:

	// General
	int m_cGeneralMenu;
	int m_cHideMainMenuEntry;

	// Audio
	int m_cAudioMenu;
	int m_cAudioSoftvol;
	int m_cAudioDownmix;
	int m_cAudioPassthroughDefault;
	int m_cAudioPassthroughAC3;
	int m_cAudioPassthroughEAC3;
	int m_cAudioPassthroughDTS;
	int m_cAudioAutoAES;
	int m_cAudioDelay;
	int m_cAudioNormalize;
	int m_cAudioMaxNormalize;
	int m_cAudioCompression;
	int m_cAudioMaxCompression;
	int m_cAudioStereoDescent;

	// Audio equalizer
	int m_cAudioFilterMenu;
	int m_cAudioEq;
	int m_cAudioEqBand[18];

	// Picture-in-Picture
	int m_cPipMenu;
	int m_cPipScalePercent;
	int m_cPipLeftPercent;
	int m_cPipTopPercent;
	int m_cPipUseAlt;
	int m_cPipAltScalePercent;
	int m_cPipAltLeftPercent;
	int m_cPipAltTopPercent;

	// Logging
	int m_cLoggingMenu;
	int m_cLogDefault;
	int m_cLogDebug_;
	int m_cLogDRM;
	int m_cLogCodec;
	int m_cLogAVSync;
	int m_cLogSound;
	int m_cLogFFmpeg;
	int m_cLogPacket;
	int m_cLogOSD;
	int m_cLogGrab;
	int m_cLogStill;
	int m_cLogTrick;
	int m_cLogMedia;
	int m_cLogGL;
	int m_cLogGLTime;
	int m_cLogGLTimeAll;

	// Statistics
	int m_cStatisticsMenu;

	// Expert settings
	int m_cExpertMenu;
	int m_cAdditionalBufferLengthMs;
	int m_cDisableDeint;
	int m_cDecoderNeedsIFrame;
	int m_cParseH264Dimensions;
	int m_cDecoderFallbackToSw;
	int m_cDecoderFallbackToSwNumPkts;
	int m_cParseH264StreamStart;
#ifdef USE_GLES
	int m_cMaxSizeGPUImageCache;
#endif

private:
	cSoftHdDevice *m_pDevice;
	cSoftHdConfig *m_pConfig;
	cSoftHdAudio *m_pAudioDevice;

	inline cOsdItem * CollapsedItem(const char *, int &, const char * = NULL);
	void Create(void);

protected:
	virtual void Store(void);
};

#endif
