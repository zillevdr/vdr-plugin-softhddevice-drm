/**
 * @file softhdsetupmenu.cpp
 * Setup menu class
 *
 * This file defines cMenuSetupSoft which describes the
 * setup menu and sets the config paramaters.
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

#include <vdr/menuitems.h>

#include "audio.h"
#include "codec_audio.h"
#include "config.h"
#include "logger.h"
#include "softhddevice.h"
#include "softhdsetupmenu.h"

/*****************************************************************************
 * cMenuSetupSoft - Setup menu
 ****************************************************************************/

/**
 * Create a seperator named item
 *
 * @param label       text inside separator
 */
static inline cOsdItem *SeparatorName(const char *label)
{
	return new cOsdItem(cString::sprintf("%s:", label), osUnknown, false);
}

/**
 * Create a collapsed item
 *
 * @param label     text inside collapsed
 * @param flag      flag handling collapsed or opened
 * @param msg       open message
 */
inline cOsdItem *cMenuSetupSoft::CollapsedItem(const char *label, int &flag, const char *msg)
{
	cOsdItem *item;

	item = new cMenuEditBoolItem(cString::sprintf("* %s", label), &flag,
		msg ? msg : tr("show"), tr("hide"));

	return item;
}

/**
 * Create setup menu.
 */
void cMenuSetupSoft::Create(void)
{
	int current;

	current = Current();	// get current menu item index
	Clear();            	// clear the menu

	//
	// General
	//
	Add(CollapsedItem(tr("General"), m_cGeneralMenu));
	if (m_cGeneralMenu) {
		Add(new cMenuEditBoolItem(tr(" Hide main menu entry"), &m_cHideMainMenuEntry, trVDR("no"), trVDR("yes")));
	}

	//
	// Audio
	//
	Add(CollapsedItem(tr("Audio"), m_cAudioMenu));
	if (m_cAudioMenu) {
		Add(new cMenuEditBoolItem(tr(" Volume control"), &m_cAudioSoftvol, tr("hardware"), tr("software")));
		Add(new cMenuEditBoolItem(tr(" Enable stereo downmix"), &m_cAudioDownmix, trVDR("no"), trVDR("yes")));
		Add(new cMenuEditBoolItem(tr(" Enable passthrough"), &m_cAudioPassthroughDefault, trVDR("off"), trVDR("on")));
		if (m_cAudioPassthroughDefault) {
			Add(new cMenuEditBoolItem(tr("  AC-3 passthrough"), &m_cAudioPassthroughAC3, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  E-AC-3 passthrough"), &m_cAudioPassthroughEAC3, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  DTS passthrough"), &m_cAudioPassthroughDTS, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  Enable automatic AES"), &m_cAudioAutoAES, trVDR("no"), trVDR("yes")));
		}
		Add(new cMenuEditIntItem(tr(" Audio/Video delay (ms)"), &m_cAudioDelay, -1000, 1000));
		Add(new cMenuEditBoolItem(tr(" Enable normalize volume"), &m_cAudioNormalize, trVDR("no"), trVDR("yes")));
		if (m_cAudioNormalize)
			Add(new cMenuEditIntItem(tr("  Max normalize factor (/1000)"), &m_cAudioMaxNormalize, 0, 10000));
		Add(new cMenuEditBoolItem(tr(" Enable volume compression"), &m_cAudioCompression, trVDR("no"), trVDR("yes")));
		if (m_cAudioCompression)
			Add(new cMenuEditIntItem(tr("  Max compression factor (/1000)"), &m_cAudioMaxCompression, 0, 10000));
		Add(new cMenuEditIntItem(tr(" Reduce stereo volume (/1000)"), &m_cAudioStereoDescent, 0, 1000));
	}

	//
	// Audio filter
	//
	Add(CollapsedItem(tr("Audio equalizer"), m_cAudioFilterMenu));
	if (m_cAudioFilterMenu) {
		Add(new cMenuEditBoolItem(tr(" Enable audio equalizer"), &m_cAudioEq, trVDR("no"), trVDR("yes")));
		if (m_cAudioEq) {
			Add(new cMenuEditIntItem(tr("  60 Hz band gain"),   &m_cAudioEqBand[0], -15, 1));
			Add(new cMenuEditIntItem(tr("  72 Hz band gain"),   &m_cAudioEqBand[1], -15, 1));
			Add(new cMenuEditIntItem(tr("  107 Hz band gain"),   &m_cAudioEqBand[2], -15, 1));
			Add(new cMenuEditIntItem(tr("  150 Hz band gain"),   &m_cAudioEqBand[3], -15, 1));
			Add(new cMenuEditIntItem(tr("  220 Hz band gain"),   &m_cAudioEqBand[4], -15, 1));
			Add(new cMenuEditIntItem(tr("  310 Hz band gain"),   &m_cAudioEqBand[5], -15, 1));
			Add(new cMenuEditIntItem(tr("  430 Hz band gain"),   &m_cAudioEqBand[6], -15, 1));
			Add(new cMenuEditIntItem(tr("  620 Hz band gain"),   &m_cAudioEqBand[7], -15, 1));
			Add(new cMenuEditIntItem(tr("  860 Hz band gain"),   &m_cAudioEqBand[8], -15, 1));
			Add(new cMenuEditIntItem(tr("  1200 Hz band gain"),  &m_cAudioEqBand[9], -15, 1));
			Add(new cMenuEditIntItem(tr("  1700 Hz band gain"),  &m_cAudioEqBand[10], -15, 1));
			Add(new cMenuEditIntItem(tr("  2500 Hz band gain"),  &m_cAudioEqBand[11], -15, 1));
			Add(new cMenuEditIntItem(tr("  3500 Hz band gain"),  &m_cAudioEqBand[12], -15, 1));
			Add(new cMenuEditIntItem(tr("  4800 Hz band gain"),  &m_cAudioEqBand[13], -15, 1));
			Add(new cMenuEditIntItem(tr("  7000 Hz band gain"),  &m_cAudioEqBand[14], -15, 1));
			Add(new cMenuEditIntItem(tr("  9500 Hz band gain"),  &m_cAudioEqBand[15], -15, 1));
			Add(new cMenuEditIntItem(tr("  13500 Hz band gain"), &m_cAudioEqBand[16], -15, 1));
			Add(new cMenuEditIntItem(tr("  17200 Hz band gain"), &m_cAudioEqBand[17], -15, 1));
		}
	}

	//
	// PiP
	//
	if (m_pDevice->UsePip()) {
		Add(CollapsedItem(tr("Picture-in-picture"), m_cPipMenu));
		if (m_cPipMenu) {
			Add(new cMenuEditIntItem(tr(" video scaling factor (%)"), &m_cPipScalePercent, 10, 100));
			Add(new cMenuEditIntItem(tr(" video left (%)"), &m_cPipLeftPercent, 0, 100));
			Add(new cMenuEditIntItem(tr(" video top (%)"), &m_cPipTopPercent, 0, 100));
			Add(new cMenuEditBoolItem(tr(" use alternative position as default"), &m_cPipUseAlt, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditIntItem(tr(" alternative video scaling factor (%)"), &m_cPipAltScalePercent, 10, 100));
			Add(new cMenuEditIntItem(tr(" alternative video left (%)"), &m_cPipAltLeftPercent, 0, 100));
			Add(new cMenuEditIntItem(tr(" alternative video top (%)"), &m_cPipAltTopPercent, 0, 100));
		}
	}

	//
	// Logging
	//
	Add(CollapsedItem(tr("Logging"), m_cLoggingMenu));
	if (m_cLoggingMenu) {
		Add(new cMenuEditBoolItem(tr(" Enable logging"), &m_cLogDefault, trVDR("off"), trVDR("on")));
		if (m_cLogDefault) {
			Add(new cMenuEditBoolItem(tr("  Standard debug logs"), &m_cLogDebug_, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  DRM debug logs"), &m_cLogDRM, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  Codec debug logs"), &m_cLogCodec, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  AV Sync debug logs"), &m_cLogAVSync, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  Sound debug logs"), &m_cLogSound, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  FFmpeg debug logs"), &m_cLogFFmpeg, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  Packet tracking logs"), &m_cLogPacket, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  OSD debug logs"), &m_cLogOSD, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  Grabbing debug logs"), &m_cLogGrab, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  Stillpicture debug logs"), &m_cLogStill, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  Trickspeed debug logs"), &m_cLogTrick, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  Mediaplayer debug logs"), &m_cLogMedia, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  OpenGL OSD debug logs"), &m_cLogGL, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  OpenGL OSD time measurement"), &m_cLogGLTime, trVDR("no"), trVDR("yes")));
			Add(new cMenuEditBoolItem(tr("  OpenGL OSD time measurement (extensive)"), &m_cLogGLTimeAll, trVDR("no"), trVDR("yes")));
		}
	}

	//
	// Statistics
	//
	Add(CollapsedItem(tr("Statistics"), m_cStatisticsMenu));
	if (m_cStatisticsMenu) {
		int duped;
		int dropped;
		int counter;
		m_pDevice->GetStats(&duped, &dropped, &counter);
		Add(new cOsdItem(cString::sprintf(tr(" Frames duped(%d) dropped(%d) total(%d)"), duped, dropped, counter), osUnknown, false));
#ifdef USE_GLES
		Add(new cOsdItem(cString::sprintf(tr(" OSD: Using %s rendering"), m_pConfig->ConfigDisableOglOsd ? "software" : "hardware"), osUnknown, false));
#else
		Add(new cOsdItem(cString::sprintf(tr(" OSD: Using software rendering")), osUnknown, false));
#endif
		Add(new cOsdItem(cString::sprintf(tr(" Video decoder: %s (%s)"), m_pConfig->CurrentDecoderName, m_pConfig->CurrentDecoderType), osUnknown, false));
	}

	//
	// Expert settings
	//
	Add(CollapsedItem(tr("Expert settings"), m_cExpertMenu));
	if (m_cExpertMenu) {
		Add(SeparatorName(tr(" Audio settings")));
		Add(new cMenuEditIntItem(tr(" Additional buffer size (ms)"), &m_cAdditionalBufferLengthMs, 0, 1000));

		Add(SeparatorName(tr(" Video settings")));
		Add(new cMenuEditBoolItem(tr(" Disable deinterlacer"), &m_cDisableDeint, trVDR("no"), trVDR("yes")));
		Add(new cMenuEditBoolItem(tr(" H.264 HW dec needs I-Frame"), &m_cDecoderNeedsIFrame, trVDR("no"), trVDR("yes")));
		Add(new cMenuEditBoolItem(tr(" H.264 HW dec needs width and height"), &m_cParseH264Dimensions, trVDR("no"), trVDR("yes")));
		Add(new cMenuEditBoolItem(tr(" Enable SW decoder fallback"), &m_cDecoderFallbackToSw, trVDR("no"), trVDR("yes")));
		if (m_cDecoderFallbackToSw) {
			Add(new cOsdItem(cString::sprintf(tr("  (minimum: %d)"), m_pConfig->GetDecoderNeedsMaxPackets() + 1), osUnknown, false));
			Add(new cMenuEditIntItem(tr("  fallback after num packets"), &m_cDecoderFallbackToSwNumPkts, 22));
		}
		Add(new cMenuEditBoolItem(tr(" Parse H.264 stream start"), &m_cParseH264StreamStart, trVDR("no"), trVDR("yes")));
#ifdef USE_GLES
		Add(SeparatorName(tr(" OSD settings")));
		if (!m_pConfig->ConfigDisableOglOsd) {
			Add(new cMenuEditIntItem(tr(" GPU mem used for image caching (MB)"), &m_cMaxSizeGPUImageCache, 0, 4000));
		}
#endif
	}

	SetCurrent(Get(current));	// restore selected menu entry
	Display();               	// display build menu
}

/**
 * Process key for setup menu.
 *
 * @param key          pressed key
 */
eOSState cMenuSetupSoft::ProcessKey(eKeys key)
{
	int old_cGeneralMenu = m_cGeneralMenu;
	int old_cAudioMenu = m_cAudioMenu;
	int old_cAudioPassthroughDefault = m_cAudioPassthroughDefault;
	int old_cAudioNormalize = m_cAudioNormalize;
	int old_cAudioCompression = m_cAudioCompression;
	int old_cAudioFilterMenu = m_cAudioFilterMenu;
	int old_cAudioEq = m_cAudioEq;
	int old_cPipMenu = m_cPipMenu;
	int old_cLoggingMenu = m_cLoggingMenu;
	int old_cLogDefault = m_cLogDefault;
	int old_cStatisticsMenu = m_cStatisticsMenu;
	int old_cExpertMenu = m_cExpertMenu;
	int old_cDecoderFallbackToSw = m_cDecoderFallbackToSw;

	eOSState state = cMenuSetupPage::ProcessKey(key);

	if (key != kNone) {
		// update menu only, if something on the structure has changed
		// this is needed because VDR menus are evil slow
		if (old_cGeneralMenu             != m_cGeneralMenu ||
		    old_cAudioMenu               != m_cAudioMenu ||
		    old_cAudioPassthroughDefault != m_cAudioPassthroughDefault ||
		    old_cAudioNormalize          != m_cAudioNormalize ||
		    old_cAudioCompression        != m_cAudioCompression ||
		    old_cAudioFilterMenu         != m_cAudioFilterMenu ||
		    old_cAudioEq                 != m_cAudioEq ||
		    old_cPipMenu                 != m_cPipMenu ||
		    old_cLoggingMenu             != m_cLoggingMenu ||
		    old_cLogDefault              != m_cLogDefault ||
		    old_cStatisticsMenu          != m_cStatisticsMenu ||
		    old_cExpertMenu              != m_cExpertMenu ||
		    old_cDecoderFallbackToSw     != m_cDecoderFallbackToSw) {

			Create();	// update menu
		}
	}

	return state;
}

/**
 * cMenuSetupSoft constructor
 *
 * Import global config variables into setup
 */
cMenuSetupSoft::cMenuSetupSoft(cSoftHdDevice *device)
	: m_pDevice(device),
	  m_pConfig(m_pDevice->Config()),
	  m_pAudioDevice(m_pDevice->Audio())
{
	//
	// General
	//
	m_cGeneralMenu = 0;
	m_cHideMainMenuEntry = m_pConfig->ConfigHideMainMenuEntry;

	//
	// Audio
	//
	m_cAudioMenu = 0;
	m_cAudioSoftvol            = m_pConfig->ConfigAudioSoftvol;
	m_cAudioDownmix            = m_pConfig->ConfigAudioDownmix;
	m_cAudioPassthroughDefault = m_pConfig->ConfigAudioPassthroughState;
	m_cAudioPassthroughAC3     = m_pConfig->ConfigAudioPassthroughMask & CODEC_AC3;
	m_cAudioPassthroughEAC3    = m_pConfig->ConfigAudioPassthroughMask & CODEC_EAC3;
	m_cAudioPassthroughDTS     = m_pConfig->ConfigAudioPassthroughMask & CODEC_DTS;
	m_cAudioAutoAES            = m_pConfig->ConfigAudioAutoAES;
	m_cAudioDelay              = m_pConfig->ConfigVideoAudioDelayMs;
	m_cAudioNormalize          = m_pConfig->ConfigAudioNormalize;
	m_cAudioMaxNormalize       = m_pConfig->ConfigAudioMaxNormalize;
	m_cAudioCompression        = m_pConfig->ConfigAudioCompression;
	m_cAudioMaxCompression     = m_pConfig->ConfigAudioMaxCompression;
	m_cAudioStereoDescent      = m_pConfig->ConfigAudioStereoDescent;

	//
	// Audio equalizer
	//
	m_cAudioFilterMenu = 0;
	m_cAudioEq = m_pConfig->ConfigAudioEq;
	for (int i = 0; i < 18; i++) {
		m_cAudioEqBand[i] = m_pConfig->ConfigAudioEqBand[i];
	}

	//
	// Picture-in-picture
	//
	m_cPipMenu = 0;
	m_cPipScalePercent = m_pConfig->ConfigPipScalePercent;
	m_cPipLeftPercent = m_pConfig->ConfigPipLeftPercent;
	m_cPipTopPercent = m_pConfig->ConfigPipTopPercent;
	m_cPipUseAlt = m_pConfig->ConfigPipUseAlt;
	m_cPipAltScalePercent = m_pConfig->ConfigPipAltScalePercent;
	m_cPipAltLeftPercent = m_pConfig->ConfigPipAltLeftPercent;
	m_cPipAltTopPercent = m_pConfig->ConfigPipAltTopPercent;

	//
	// Logging
	//
	m_cLoggingMenu = 0;
	m_cLogDefault   = m_pConfig->ConfigLogState;
	m_cLogDebug_    = m_pConfig->ConfigLogLevels & L_DEBUG;
	m_cLogDRM       = m_pConfig->ConfigLogLevels & L_DRM;
	m_cLogCodec     = m_pConfig->ConfigLogLevels & L_CODEC;
	m_cLogAVSync    = m_pConfig->ConfigLogLevels & L_AV_SYNC;
	m_cLogSound     = m_pConfig->ConfigLogLevels & L_SOUND;
	m_cLogFFmpeg    = m_pConfig->ConfigLogLevels & L_FFMPEG;
	m_cLogPacket    = m_pConfig->ConfigLogLevels & L_PACKET;
	m_cLogOSD       = m_pConfig->ConfigLogLevels & L_OSD;
	m_cLogGrab      = m_pConfig->ConfigLogLevels & L_GRAB;
	m_cLogStill     = m_pConfig->ConfigLogLevels & L_STILL;
	m_cLogTrick     = m_pConfig->ConfigLogLevels & L_TRICK;
	m_cLogMedia     = m_pConfig->ConfigLogLevels & L_MEDIA;
	m_cLogGL        = m_pConfig->ConfigLogLevels & L_OPENGL;
	m_cLogGLTime    = m_pConfig->ConfigLogLevels & L_OPENGL_TIME;
	m_cLogGLTimeAll = m_pConfig->ConfigLogLevels & L_OPENGL_TIME_ALL;

	//
	// Statistics
	//
	m_cStatisticsMenu = 0;

	//
	// Expert settings
	//
	m_cExpertMenu = 0;
	m_cAdditionalBufferLengthMs= m_pConfig->ConfigAdditionalBufferLengthMs;
	m_cDisableDeint = m_pConfig->ConfigDisableDeint;
	m_cDecoderNeedsIFrame = m_pConfig->ConfigDecoderNeedsIFrame;
	m_cParseH264Dimensions = m_pConfig->ConfigParseH264Dimensions;
	m_cDecoderFallbackToSw = m_pConfig->ConfigDecoderFallbackToSw;
	m_cDecoderFallbackToSwNumPkts = m_pConfig->ConfigDecoderFallbackToSwNumPkts;
	m_cParseH264StreamStart = m_pConfig->ConfigParseH264StreamStart;
#ifdef USE_GLES
	m_cMaxSizeGPUImageCache = m_pConfig->ConfigMaxSizeGPUImageCache;
#endif

	Create();
}

/**
 * Store setup
 */
void cMenuSetupSoft::Store(void)
{
	//
	// General
	//
	SetupStore("HideMainMenuEntry", m_pConfig->ConfigHideMainMenuEntry = m_cHideMainMenuEntry);

	//
	// Audio
	//
	SetupStore("AudioSoftvol", m_pConfig->ConfigAudioSoftvol = m_cAudioSoftvol);
	m_pAudioDevice->SetSoftvol(m_pConfig->ConfigAudioSoftvol);
	SetupStore("AudioDownmix", m_pConfig->ConfigAudioDownmix = m_cAudioDownmix);
	m_pAudioDevice->SetDownmix(m_pConfig->ConfigAudioDownmix);
	// FIXME: can handle more audio state changes here
	// downmix changed reset audio, to get change direct
	if (m_pConfig->ConfigAudioDownmix != m_cAudioDownmix) {
		m_pDevice->ResetChannelId();
	}
	m_pConfig->ConfigAudioPassthroughMask = (m_cAudioPassthroughAC3 ? CODEC_AC3 : 0)
	                                      | (m_cAudioPassthroughEAC3 ? CODEC_EAC3 : 0)
	                                      | (m_cAudioPassthroughDTS ? CODEC_DTS : 0);
	m_pConfig->ConfigAudioPassthroughState = m_cAudioPassthroughDefault;
	if (m_pConfig->ConfigAudioPassthroughState) {
		SetupStore("AudioPassthrough", m_pConfig->ConfigAudioPassthroughMask);
		m_pDevice->SetPassthrough(m_pConfig->ConfigAudioPassthroughMask);
	} else {
		SetupStore("AudioPassthrough", -m_pConfig->ConfigAudioPassthroughMask);
		m_pDevice->SetPassthrough(0);
	}
	SetupStore("AudioAutoAES", m_pConfig->ConfigAudioAutoAES = m_cAudioAutoAES);
	m_pAudioDevice->SetAutoAES(m_pConfig->ConfigAudioAutoAES);
	SetupStore("AudioDelay", m_pConfig->ConfigVideoAudioDelayMs = m_cAudioDelay);
	SetupStore("AudioNormalize", m_pConfig->ConfigAudioNormalize = m_cAudioNormalize);
	SetupStore("AudioMaxNormalize", m_pConfig->ConfigAudioMaxNormalize = m_cAudioMaxNormalize);
	m_pAudioDevice->SetNormalize(m_pConfig->ConfigAudioNormalize, m_pConfig->ConfigAudioMaxNormalize);
	SetupStore("AudioCompression", m_pConfig->ConfigAudioCompression = m_cAudioCompression);
	SetupStore("AudioMaxCompression", m_pConfig->ConfigAudioMaxCompression = m_cAudioMaxCompression);
	m_pAudioDevice->SetCompression(m_pConfig->ConfigAudioCompression, m_pConfig->ConfigAudioMaxCompression);
	SetupStore("AudioStereoDescent", m_pConfig->ConfigAudioStereoDescent = m_cAudioStereoDescent);
	m_pAudioDevice->SetStereoDescent(m_pConfig->ConfigAudioStereoDescent);

	//
	// Audio equalizer
	//
	SetupStore("AudioEq", m_pConfig->ConfigAudioEq = m_cAudioEq);
	SetupStore("AudioEqBand01b", m_pConfig->ConfigAudioEqBand[0]  = m_cAudioEqBand[0]);
	SetupStore("AudioEqBand02b", m_pConfig->ConfigAudioEqBand[1]  = m_cAudioEqBand[1]);
	SetupStore("AudioEqBand03b", m_pConfig->ConfigAudioEqBand[2]  = m_cAudioEqBand[2]);
	SetupStore("AudioEqBand04b", m_pConfig->ConfigAudioEqBand[3]  = m_cAudioEqBand[3]);
	SetupStore("AudioEqBand05b", m_pConfig->ConfigAudioEqBand[4]  = m_cAudioEqBand[4]);
	SetupStore("AudioEqBand06b", m_pConfig->ConfigAudioEqBand[5]  = m_cAudioEqBand[5]);
	SetupStore("AudioEqBand07b", m_pConfig->ConfigAudioEqBand[6]  = m_cAudioEqBand[6]);
	SetupStore("AudioEqBand08b", m_pConfig->ConfigAudioEqBand[7]  = m_cAudioEqBand[7]);
	SetupStore("AudioEqBand09b", m_pConfig->ConfigAudioEqBand[8]  = m_cAudioEqBand[8]);
	SetupStore("AudioEqBand10b", m_pConfig->ConfigAudioEqBand[9]  = m_cAudioEqBand[9]);
	SetupStore("AudioEqBand11b", m_pConfig->ConfigAudioEqBand[10] = m_cAudioEqBand[10]);
	SetupStore("AudioEqBand12b", m_pConfig->ConfigAudioEqBand[11] = m_cAudioEqBand[11]);
	SetupStore("AudioEqBand13b", m_pConfig->ConfigAudioEqBand[12] = m_cAudioEqBand[12]);
	SetupStore("AudioEqBand14b", m_pConfig->ConfigAudioEqBand[13] = m_cAudioEqBand[13]);
	SetupStore("AudioEqBand15b", m_pConfig->ConfigAudioEqBand[14] = m_cAudioEqBand[14]);
	SetupStore("AudioEqBand16b", m_pConfig->ConfigAudioEqBand[15] = m_cAudioEqBand[15]);
	SetupStore("AudioEqBand17b", m_pConfig->ConfigAudioEqBand[16] = m_cAudioEqBand[16]);
	SetupStore("AudioEqBand18b", m_pConfig->ConfigAudioEqBand[17] = m_cAudioEqBand[17]);
	m_pAudioDevice->SetEq(m_pConfig->ConfigAudioEqBand, m_pConfig->ConfigAudioEq);

	//
	// Picture-in-picture
	//
	SetupStore("PipScalePercent", m_pConfig->ConfigPipScalePercent = m_cPipScalePercent);
	SetupStore("PipLeftPercent", m_pConfig->ConfigPipLeftPercent = m_cPipLeftPercent);
	SetupStore("PipTopPercent", m_pConfig->ConfigPipTopPercent = m_cPipTopPercent);
	SetupStore("PipUseAlt", m_pConfig->ConfigPipUseAlt = m_cPipUseAlt);
	SetupStore("PipAltScalePercent", m_pConfig->ConfigPipAltScalePercent = m_cPipAltScalePercent);
	SetupStore("PipAltLeftPercent", m_pConfig->ConfigPipAltLeftPercent = m_cPipAltLeftPercent);
	SetupStore("PipAltTopPercent", m_pConfig->ConfigPipAltTopPercent = m_cPipAltTopPercent);
	if (m_pDevice->UsePip())
		m_pDevice->PipSetSize();

	//
	// Logging
	//
	m_pConfig->ConfigLogLevels =
		(m_cLogDebug_    ? L_DEBUG : 0) |
		(m_cLogDRM       ? L_DRM : 0) |
		(m_cLogCodec     ? L_CODEC : 0) |
		(m_cLogAVSync    ? L_AV_SYNC : 0) |
		(m_cLogSound     ? L_SOUND : 0) |
		(m_cLogFFmpeg    ? L_FFMPEG : 0) |
		(m_cLogPacket    ? L_PACKET : 0) |
		(m_cLogOSD       ? L_OSD : 0) |
		(m_cLogGrab      ? L_GRAB : 0) |
		(m_cLogStill     ? L_STILL : 0) |
		(m_cLogTrick     ? L_TRICK : 0) |
		(m_cLogMedia     ? L_MEDIA : 0) |
		(m_cLogGL        ? L_OPENGL : 0) |
		(m_cLogGLTime    ? L_OPENGL_TIME : 0) |
		(m_cLogGLTimeAll ? L_OPENGL_TIME_ALL : 0);
	m_pConfig->ConfigLogState = m_cLogDefault;
	if (m_pConfig->ConfigLogState) {
		SetupStore("LogLevel", m_pConfig->ConfigLogLevels);
		m_pConfig->PrintLogLevel(m_pConfig->ConfigLogLevels);
		cSoftHdLogger::GetLogger()->SetLogLevel(m_pConfig->ConfigLogLevels);
	} else {
		SetupStore("LogLevel", -m_pConfig->ConfigLogLevels);
		cSoftHdLogger::GetLogger()->SetLogLevel(0);
	}

	//
	// Expert settings
	//
	SetupStore("AdditionalBufferLengthMs", m_pConfig->ConfigAdditionalBufferLengthMs = m_cAdditionalBufferLengthMs);
	SetupStore("DisableDeint", m_pConfig->ConfigDisableDeint = m_cDisableDeint);
	if (m_pConfig->ConfigDisableDeint) {
		LOGDEBUG("Disable deinterlacer!");
	}
	m_pDevice->SetDisableDeint();
	SetupStore("DecoderNeedsIFrame", m_pConfig->ConfigDecoderNeedsIFrame = m_cDecoderNeedsIFrame);
	m_pDevice->SetDecoderNeedsIFrame();
	SetupStore("ParseH264Dimensions", m_pConfig->ConfigParseH264Dimensions = m_cParseH264Dimensions);
	m_pDevice->SetParseH264Dimensions();
	SetupStore("DecoderFallbackToSw", m_pConfig->ConfigDecoderFallbackToSw = m_cDecoderFallbackToSw);
	SetupStore("DecoderFallbackToSwNumPkts", m_pConfig->ConfigDecoderFallbackToSwNumPkts = m_cDecoderFallbackToSwNumPkts);
	m_pDevice->SetDecoderFallbackToSw(m_pConfig->ConfigDecoderFallbackToSw);
	SetupStore("ParseH264StreamStart", m_pConfig->ConfigParseH264StreamStart = m_cParseH264StreamStart);
#ifdef USE_GLES
	SetupStore("MaxSizeGPUImageCache", m_pConfig->ConfigMaxSizeGPUImageCache = m_cMaxSizeGPUImageCache);
#endif
}
