///
///	@file audio.c		@brief Audio module
///
///	Copyright (c) 2009 - 2014 by Johns.  All Rights Reserved.
///	Copyright (c) 2018 by zille.  All Rights Reserved.
///
///	Contributor(s):
///
///	License: AGPLv3
///
///	This program is free software: you can redistribute it and/or modify
///	it under the terms of the GNU Affero General Public License as
///	published by the Free Software Foundation, either version 3 of the
///	License.
///
///	This program is distributed in the hope that it will be useful,
///	but WITHOUT ANY WARRANTY; without even the implied warranty of
///	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
///	GNU Affero General Public License for more details.
///
///	$Id$
//////////////////////////////////////////////////////////////////////////////

///
///	@defgroup Audio The audio module.
///
///		This module contains all audio output functions.
///
///		ALSA PCM/Mixer api is supported.
///		@see http://www.alsa-project.org/alsa-doc/alsa-lib
///
///	@note alsa async playback is broken, don't use it!
///
///	@todo FIXME: there can be problems with little/big endian.
///

#define USE_AUDIO_MIXER		///< use audio module mixer

#include <stdint.h>
#include <math.h>

#include <libintl.h>
#define _(str) gettext(str)		///< gettext shortcut
#define _N(str) str			///< gettext_noop shortcut

#include <alsa/asoundlib.h>

#ifndef __USE_GNU
#define __USE_GNU
#endif
#include <pthread.h>

#include <libavutil/channel_layout.h>
#include <libavutil/opt.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>

#include "iatomic.h"			// portable atomic_t

#include "ringbuffer.h"
#include "misc.h"
#include "audio.h"


//----------------------------------------------------------------------------
//	Variables
//----------------------------------------------------------------------------

static const char *AudioPCMDevice;	///< PCM device name
static const char *AudioPassthroughDevice;	///< Passthrough device name
static char AudioAppendAES;		///< flag automatic append AES
static const char *AudioMixerDevice;	///< mixer device name
static const char *AudioMixerChannel;	///< mixer channel name
static char AudioDoingInit;		///> flag in init, reduce error
static volatile char AudioRunning;	///< thread running / stopped
static volatile char AudioPaused;	///< audio paused
static volatile char AudioVideoIsReady;	///< video ready start early
static int AudioSkip;			///< skip audio to sync to video

static const int AudioBytesProSample = 2;	///< number of bytes per sample

static int AudioBufferTime = 600;	///< audio buffer time in ms

static pthread_t AudioThread;		///< audio play thread
static pthread_mutex_t AudioRbMutex;	///< audio condition mutex
static pthread_mutex_t AudioStartMutex;	///< audio condition mutex
static pthread_cond_t AudioStartCond;	///< condition variable
static char AudioThreadStop;		///< stop audio thread
static char AlsaPlayerStop;		///< stop audio thread

static char AudioSoftVolume;		///< flag use soft volume
static char AudioNormalize;		///< flag use volume normalize
static char AudioCompression;		///< flag use compress volume
static char AudioMute;			///< flag muted
static int AudioAmplifier;		///< software volume factor
static int AudioNormalizeFactor;	///< current normalize factor
static const int AudioMinNormalize = 100;	///< min. normalize factor
static int AudioMaxNormalize;		///< max. normalize factor
static int AudioCompressionFactor;	///< current compression factor
static int AudioMaxCompression;		///< max. compression factor
static int AudioStereoDescent;		///< volume descent for stereo
static int AudioVolume;			///< current volume (0 .. 1000)

extern int VideoAudioDelay;		///< import audio/video delay

    /// default ring buffer size ~2s 8ch 16bit (3 * 5 * 7 * 8)
static const unsigned AudioRingBufferSize = 3 * 5 * 7 * 8 * 2 * 1000;

static int AudioChannelsInHw[9];	///< table which channels are supported
enum _audio_rates
{					///< sample rates enumeration
    // HW: 32000 44100 48000 88200 96000 176400 192000
    //Audio32000,				///< 32.0Khz
    Audio44100,				///< 44.1Khz
    Audio48000,				///< 48.0Khz
    //Audio88200,				///< 88.2Khz
    //Audio96000,				///< 96.0Khz
    //Audio176400,				///< 176.4Khz
    Audio192000,			///< 192.0Khz
    AudioRatesMax			///< max index
};

    /// table which rates are supported
static int AudioRatesInHw[AudioRatesMax];

    /// input to hardware channel matrix
static int AudioChannelMatrix[AudioRatesMax][9];

    /// rates tables (must be sorted by frequency)
static const unsigned AudioRatesTable[AudioRatesMax] = {
    44100, 48000, 192000
};

//----------------------------------------------------------------------------
//	filter
//----------------------------------------------------------------------------

static const int AudioNormSamples = 4096;	///< number of samples

#define AudioNormMaxIndex 128		///< number of average values
    /// average of n last sample blocks
static uint32_t AudioNormAverage[AudioNormMaxIndex];
static int AudioNormIndex;		///< index into average table
static int AudioNormReady;		///< index counter
static int AudioNormCounter;		///< sample counter

AVFilterGraph *filter_graph;
AVFilterContext *abuffersrc_ctx, *abuffersink_ctx;
int FilterInit;
float AudioEqBand[18];
int AudioEq;
int Filterchanged;

/**
**	Audio normalizer.
**
**	@param samples	sample buffer
**	@param count	number of bytes in sample buffer
*/
static void AudioNormalizer(int16_t * samples, int count)
{
    int i;
    int l;
    int n;
    uint32_t avg;
    int factor;
    int16_t *data;

    // average samples
    l = count / AudioBytesProSample;
    data = samples;
    do {
	n = l;
	if (AudioNormCounter + n > AudioNormSamples) {
	    n = AudioNormSamples - AudioNormCounter;
	}
	avg = AudioNormAverage[AudioNormIndex];
	for (i = 0; i < n; ++i) {
	    int t;

	    t = data[i];
	    avg += (t * t) / AudioNormSamples;
	}
	AudioNormAverage[AudioNormIndex] = avg;
	AudioNormCounter += n;
	if (AudioNormCounter >= AudioNormSamples) {
	    if (AudioNormReady < AudioNormMaxIndex) {
		AudioNormReady++;
	    } else {
		avg = 0;
		for (i = 0; i < AudioNormMaxIndex; ++i) {
		    avg += AudioNormAverage[i] / AudioNormMaxIndex;
		}

		// calculate normalize factor
		if (avg > 0) {
		    factor = ((INT16_MAX / 8) * 1000U) / (uint32_t) sqrt(avg);
		    // smooth normalize
		    AudioNormalizeFactor =
			(AudioNormalizeFactor * 500 + factor * 500) / 1000;
		    if (AudioNormalizeFactor < AudioMinNormalize) {
			AudioNormalizeFactor = AudioMinNormalize;
		    }
		    if (AudioNormalizeFactor > AudioMaxNormalize) {
			AudioNormalizeFactor = AudioMaxNormalize;
		    }
		} else {
		    factor = 1000;
		}
		Debug(4, "audio/noramlize: avg %8d, fac=%6.3f, norm=%6.3f\n",
		    avg, factor / 1000.0, AudioNormalizeFactor / 1000.0);
	    }

	    AudioNormIndex = (AudioNormIndex + 1) % AudioNormMaxIndex;
	    AudioNormCounter = 0;
	    AudioNormAverage[AudioNormIndex] = 0U;
	}
	data += n;
	l -= n;
    } while (l > 0);

    // apply normalize factor
    for (i = 0; i < count / AudioBytesProSample; ++i) {
	int t;

	t = (samples[i] * AudioNormalizeFactor) / 1000;
	if (t < INT16_MIN) {
	    t = INT16_MIN;
	} else if (t > INT16_MAX) {
	    t = INT16_MAX;
	}
	samples[i] = t;
    }
}

/**
**	Reset normalizer.
*/
static void AudioResetNormalizer(void)
{
    int i;

    AudioNormCounter = 0;
    AudioNormReady = 0;
    for (i = 0; i < AudioNormMaxIndex; ++i) {
		AudioNormAverage[i] = 0U;
    }
    AudioNormalizeFactor = 1000;
}

/**
**	Audio compression.
**
**	@param samples	sample buffer
**	@param count	number of bytes in sample buffer
*/
static void AudioCompressor(int16_t * samples, int count)
{
    int max_sample;
    int i;
    int factor;

    // find loudest sample
    max_sample = 0;
    for (i = 0; i < count / AudioBytesProSample; ++i) {
		int t;

		t = abs(samples[i]);
		if (t > max_sample) {
			max_sample = t;
		}
    }

    // calculate compression factor
    if (max_sample > 0) {
		factor = (INT16_MAX * 1000) / max_sample;
		// smooth compression (FIXME: make configurable?)
		AudioCompressionFactor =
			(AudioCompressionFactor * 950 + factor * 50) / 1000;
		if (AudioCompressionFactor > factor) {
			AudioCompressionFactor = factor;	// no clipping
		}
		if (AudioCompressionFactor > AudioMaxCompression) {
			AudioCompressionFactor = AudioMaxCompression;
		}
    } else {
		return;				// silent nothing todo
    }

    Debug(4, "audio/compress: max %5d, fac=%6.3f, com=%6.3f\n", max_sample,
	factor / 1000.0, AudioCompressionFactor / 1000.0);

    // apply compression factor
    for (i = 0; i < count / AudioBytesProSample; ++i) {
		int t;

		t = (samples[i] * AudioCompressionFactor) / 1000;
		if (t < INT16_MIN) {
			t = INT16_MIN;
		} else if (t > INT16_MAX) {
			t = INT16_MAX;
		}
		samples[i] = t;
    }
}

/**
**	Reset compressor.
*/
static void AudioResetCompressor(void)
{
    AudioCompressionFactor = 2000;
    if (AudioCompressionFactor > AudioMaxCompression) {
		AudioCompressionFactor = AudioMaxCompression;
    }
}

/**
**	Audio software amplifier.
**
**	@param samples	sample buffer
**	@param count	number of bytes in sample buffer
**
**	@todo FIXME: this does hard clipping
*/
static void AudioSoftAmplifier(int16_t * samples, int count)
{
    int i;

    // silence
    if (AudioMute || !AudioAmplifier) {
		memset(samples, 0, count);
		return;
    }

    for (i = 0; i < count / AudioBytesProSample; ++i) {
		int t;

		t = (samples[i] * AudioAmplifier) / 1000;
		if (t < INT16_MIN) {
			t = INT16_MIN;
		} else if (t > INT16_MAX) {
			t = INT16_MAX;
		}
		samples[i] = t;
	}
}

#ifdef USE_AUDIO_MIXER

/**
**	Upmix mono to stereo.
**
**	@param in	input sample buffer
**	@param frames	number of frames in sample buffer
**	@param out	output sample buffer
*/
static void AudioMono2Stereo(const int16_t * in, int frames, int16_t * out)
{
    int i;

    for (i = 0; i < frames; ++i) {
	int t;

	t = in[i];
	out[i * 2 + 0] = t;
	out[i * 2 + 1] = t;
    }
}

/**
**	Downmix stereo to mono.
**
**	@param in	input sample buffer
**	@param frames	number of frames in sample buffer
**	@param out	output sample buffer
*/
static void AudioStereo2Mono(const int16_t * in, int frames, int16_t * out)
{
    int i;

    for (i = 0; i < frames; i += 2) {
	out[i / 2] = (in[i + 0] + in[i + 1]) / 2;
    }
}

/**
**	Downmix surround to stereo.
**
**	ffmpeg L  R  C	Ls Rs		-> alsa L R  Ls Rs C
**	ffmpeg L  R  C	LFE Ls Rs	-> alsa L R  Ls Rs C  LFE
**	ffmpeg L  R  C	LFE Ls Rs Rl Rr	-> alsa L R  Ls Rs C  LFE Rl Rr
**
**	@param in	input sample buffer
**	@param in_chan	nr. of input channels
**	@param frames	number of frames in sample buffer
**	@param out	output sample buffer
*/
static void AudioSurround2Stereo(const int16_t * in, int in_chan, int frames,
    int16_t * out)
{
    while (frames--) {
	int l;
	int r;

	switch (in_chan) {
	    case 3:			// stereo or surround? =>stereo
		l = in[0] * 600;	// L
		r = in[1] * 600;	// R
		l += in[2] * 400;	// C
		r += in[2] * 400;
		break;
	    case 4:			// quad or surround? =>quad
		l = in[0] * 600;	// L
		r = in[1] * 600;	// R
		l += in[2] * 400;	// Ls
		r += in[3] * 400;	// Rs
		break;
	    case 5:			// 5.0
		l = in[0] * 500;	// L
		r = in[1] * 500;	// R
		l += in[2] * 200;	// Ls
		r += in[3] * 200;	// Rs
		l += in[4] * 300;	// C
		r += in[4] * 300;
		break;
	    case 6:			// 5.1
		l = in[0] * 400;	// L
		r = in[1] * 400;	// R
		l += in[2] * 200;	// Ls
		r += in[3] * 200;	// Rs
		l += in[4] * 300;	// C
		r += in[4] * 300;
		l += in[5] * 100;	// LFE
		r += in[5] * 100;
		break;
	    case 7:			// 7.0
		l = in[0] * 400;	// L
		r = in[1] * 400;	// R
		l += in[2] * 200;	// Ls
		r += in[3] * 200;	// Rs
		l += in[4] * 300;	// C
		r += in[4] * 300;
		l += in[5] * 100;	// RL
		r += in[6] * 100;	// RR
		break;
	    case 8:			// 7.1
		l = in[0] * 400;	// L
		r = in[1] * 400;	// R
		l += in[2] * 150;	// Ls
		r += in[3] * 150;	// Rs
		l += in[4] * 250;	// C
		r += in[4] * 250;
		l += in[5] * 100;	// LFE
		r += in[5] * 100;
		l += in[6] * 100;	// RL
		r += in[7] * 100;	// RR
		break;
	    default:
		abort();
	}
	in += in_chan;

	out[0] = l / 1000;
	out[1] = r / 1000;
	out += 2;
    }
}

/**
**	Upmix @a in_chan channels to @a out_chan.
**
**	@param in	input sample buffer
**	@param in_chan	nr. of input channels
**	@param frames	number of frames in sample buffer
**	@param out	output sample buffer
**	@param out_chan	nr. of output channels
*/
static void AudioUpmix(const int16_t * in, int in_chan, int frames,
    int16_t * out, int out_chan)
{
    while (frames--) {
	int i;

	for (i = 0; i < in_chan; ++i) {	// copy existing channels
	    *out++ = *in++;
	}
	for (; i < out_chan; ++i) {	// silents missing channels
	    *out++ = 0;
	}
    }
}

/**
**	Resample ffmpeg sample format to hardware format.
**
**	FIXME: use libswresample for this and move it to codec.
**	FIXME: ffmpeg to alsa conversion is already done in codec.c.
**
**	ffmpeg L  R  C	Ls Rs		-> alsa L R  Ls Rs C
**	ffmpeg L  R  C	LFE Ls Rs	-> alsa L R  Ls Rs C  LFE
**	ffmpeg L  R  C	LFE Ls Rs Rl Rr	-> alsa L R  Ls Rs C  LFE Rl Rr
**
**	@param in	input sample buffer
**	@param in_chan	nr. of input channels
**	@param frames	number of frames in sample buffer
**	@param out	output sample buffer
**	@param out_chan	nr. of output channels
*/
static void AudioResample(const int16_t * in, int in_chan, int frames,
    int16_t * out, int out_chan)
{
    switch (in_chan * 8 + out_chan) {
	case 1 * 8 + 1:
	case 2 * 8 + 2:
	case 3 * 8 + 3:
	case 4 * 8 + 4:
	case 5 * 8 + 5:
	case 6 * 8 + 6:
	case 7 * 8 + 7:
	case 8 * 8 + 8:		// input = output channels
	    memcpy(out, in, frames * in_chan * AudioBytesProSample);
	    break;
	case 2 * 8 + 1:
	    AudioStereo2Mono(in, frames, out);
	    break;
	case 1 * 8 + 2:
	    AudioMono2Stereo(in, frames, out);
	    break;
	case 3 * 8 + 2:
	case 4 * 8 + 2:
	case 5 * 8 + 2:
	case 6 * 8 + 2:
	case 7 * 8 + 2:
	case 8 * 8 + 2:
	    AudioSurround2Stereo(in, in_chan, frames, out);
	    break;
	case 5 * 8 + 6:
	case 3 * 8 + 8:
	case 5 * 8 + 8:
	case 6 * 8 + 8:
	    AudioUpmix(in, in_chan, frames, out, out_chan);
	    break;

	default:
	    Error("audio: unsupported %d -> %d channels resample\n", in_chan,
		out_chan);
	    // play silence
	    memset(out, 0, frames * out_chan * AudioBytesProSample);
	    break;
    }
}

#endif

/**
**	Set filter bands.
**
**	@param band		setting frequenz bands
*/
void AudioSetEq(int band[17], int onoff)
{
	int i;

/*	fprintf(stderr, "AudioSetEq %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i onoff %d\n",
		band[0], band[1], band[2], band[3], band[4], band[5], band[6], band[7],
		band[8], band[9], band[10], band[11], band[12], band[13], band[14],
		band[15], band[16], band[17], onoff);*/

	for (i = 0; i < 18; i++) {
		switch (band[i]) {
			case 1:
				AudioEqBand[i] = 1.5;
				break;
			case 0:
				AudioEqBand[i] = 1;
				break;
			case -1:
				AudioEqBand[i] = 0.95;
				break;
			case -2:
				AudioEqBand[i] = 0.9;
				break;
			case -3:
				AudioEqBand[i] = 0.85;
				break;
			case -4:
				AudioEqBand[i] = 0.8;
				break;
			case -5:
				AudioEqBand[i] = 0.75;
				break;
			case -6:
				AudioEqBand[i] = 0.7;
				break;
			case -7:
				AudioEqBand[i] = 0.65;
				break;
			case -8:
				AudioEqBand[i] = 0.6;
				break;
			case -9:
				AudioEqBand[i] = 0.55;
				break;
			case -10:
				AudioEqBand[i] = 0.5;
				break;
			case -11:
				AudioEqBand[i] = 0.45;
				break;
			case -12:
				AudioEqBand[i] = 0.4;
				break;
			case -13:
				AudioEqBand[i] = 0.35;
				break;
			case -14:
				AudioEqBand[i] = 0.3;
				break;
			case -15:
				AudioEqBand[i] = 0.25;
				break;
		}
	}

	Filterchanged = 1;
	AudioEq = onoff;
}

/**
**	Filter init.
*/
void AudioFilterInit(AVFrame *frame)
{
	const AVFilter  *abuffer;
	AVFilterContext *filter_ctx[3];
	const AVFilter *eq;
	const AVFilter *aformat;
	const AVFilter *abuffersink;
	char ch_layout[64];
	char options_str[1024];
	int err, i, n_filter = 0;

#if LIBAVFILTER_VERSION_INT < AV_VERSION_INT(7,16,100)
	avfilter_register_all();
#endif

	if (!(filter_graph = avfilter_graph_alloc()))
		fprintf(stderr, "Unable to create filter graph.\n");

	// input buffer
	if (!(abuffer = avfilter_get_by_name("abuffer")))
		fprintf(stderr, "AudioFilterInit: Could not find the abuffer filter.\n");
	if (!(abuffersrc_ctx = avfilter_graph_alloc_filter(filter_graph, abuffer, "src")))
		fprintf(stderr, "AudioFilterInit: Could not allocate the abuffersrc_ctx instance.\n");
	av_get_channel_layout_string(ch_layout, sizeof(ch_layout), frame->channels, frame->channel_layout);
#ifdef DEBUG
	fprintf(stderr, "AudioFilterInit: ch_layout %s sample_fmt %s sample_rate %d channels %d\n",
		ch_layout, av_get_sample_fmt_name(frame->format), frame->sample_rate, frame->channels);
#endif
	av_opt_set    (abuffersrc_ctx, "channel_layout", ch_layout,                             AV_OPT_SEARCH_CHILDREN);
	av_opt_set    (abuffersrc_ctx, "sample_fmt",     av_get_sample_fmt_name(frame->format), AV_OPT_SEARCH_CHILDREN);
	av_opt_set_q  (abuffersrc_ctx, "time_base",      (AVRational){ 1, frame->sample_rate }, AV_OPT_SEARCH_CHILDREN);
	av_opt_set_int(abuffersrc_ctx, "sample_rate",    frame->sample_rate,                    AV_OPT_SEARCH_CHILDREN);
	// initialize the filter with NULL options, set all options above.
	if (avfilter_init_str(abuffersrc_ctx, NULL) < 0)
		fprintf(stderr, "AudioFilterInit: Could not initialize the abuffer filter.\n");

	if (AudioEq) {
		// superequalizer
		if (!(eq = avfilter_get_by_name("superequalizer")))
			fprintf(stderr, "AudioFilterInit: Could not find the superequalizer filter.\n");
		if (!(filter_ctx[n_filter] = avfilter_graph_alloc_filter(filter_graph, eq, "superequalizer")))
			fprintf(stderr, "AudioFilterInit: Could not allocate the superequalizer instance.\n");
		snprintf(options_str, sizeof(options_str),"1b=%.2f:2b=%.2f:3b=%.2f:4b=%.2f:5b=%.2f"
			":6b=%.2f:7b=%.2f:8b=%.2f:9b=%.2f:10b=%.2f:11b=%.2f:12b=%.2f:13b=%.2f:14b=%.2f:"
			"15b=%.2f:16b=%.2f:17b=%.2f:18b=%.2f ", AudioEqBand[0], AudioEqBand[1],
			AudioEqBand[2], AudioEqBand[3], AudioEqBand[4], AudioEqBand[5],
			AudioEqBand[6], AudioEqBand[7], AudioEqBand[8], AudioEqBand[9],
			AudioEqBand[10], AudioEqBand[11], AudioEqBand[12], AudioEqBand[13],
			AudioEqBand[14], AudioEqBand[15], AudioEqBand[16], AudioEqBand[17]);
		if (avfilter_init_str(filter_ctx[n_filter], options_str) < 0)
			fprintf(stderr, "AudioFilterInit: Could not initialize the superequalizer filter.\n");
		n_filter++;
	}
	// aformat
	if (!(aformat = avfilter_get_by_name("aformat")))
		fprintf(stderr, "AudioFilterInit: Could not find the aformat filter.\n");
	if (!(filter_ctx[n_filter] = avfilter_graph_alloc_filter(filter_graph, aformat, "aformat")))
		fprintf(stderr, "AudioFilterInit: Could not allocate the aformat instance.\n");
	snprintf(options_str, sizeof(options_str),
		"sample_fmts=%s:sample_rates=%d:channel_layouts=%s",
		av_get_sample_fmt_name(AV_SAMPLE_FMT_S16),
		frame->sample_rate, ch_layout);
	if (avfilter_init_str(filter_ctx[n_filter], options_str) < 0)
		fprintf(stderr, "AudioFilterInit: Could not initialize the aformat filter.\n");
	n_filter++;

	// abuffersink
	if (!(abuffersink = avfilter_get_by_name("abuffersink")))
		fprintf(stderr, "AudioFilterInit: Could not find the abuffersink filter.\n");
	if (!(filter_ctx[n_filter] = avfilter_graph_alloc_filter(filter_graph, abuffersink, "sink")))
		fprintf(stderr, "AudioFilterInit: Could not allocate the abuffersink instance.\n");
	if (avfilter_init_str(filter_ctx[n_filter], NULL) < 0)
		fprintf(stderr, "AudioFilterInit: Could not initialize the abuffersink instance.\n");
	n_filter++;

	// Connect the filters
	for (i = 0; i < n_filter; i++) {
		if (i == 0) {
			err = avfilter_link(abuffersrc_ctx, 0, filter_ctx[i], 0);
		} else {
			err = avfilter_link(filter_ctx[i - 1], 0, filter_ctx[i], 0);
		}
	}
	if (err < 0)
		fprintf(stderr, "AudioFilterInit: Error connecting audio filters\n");

	// Configure the graph.
	if (avfilter_graph_config(filter_graph, NULL) < 0)
		fprintf(stderr, "AudioFilterInit: Error configuring the audio filter graph\n");

	abuffersink_ctx = filter_ctx[n_filter - 1];
	Filterchanged = 0;
	FilterInit = 1;
}

//----------------------------------------------------------------------------
//	ring buffer
//----------------------------------------------------------------------------

/**
**	Audio ring buffer.
*/
//char FlushBuffers;			///< flag: flush buffers
char Passthrough;			///< flag: use pass-through (AC-3, ...)
unsigned int HwSampleRate;		///< hardware sample rate in Hz
unsigned int HwChannels;		///< hardware number of channels
unsigned InSampleRate;		///< input sample rate in Hz
unsigned InChannels;		///< input number of channels
int64_t PTS;			///< pts clock

RingBuffer *AudioRingBuffer;		///< sample ring buffer

static unsigned AudioStartThreshold;	///< start play, if filled

/**
**	Setup audio ring.
*/
static void AudioRingInit(void)
{
	// ~2s 8ch 16bit
	AudioRingBuffer = RingBufferNew(AudioRingBufferSize);
}

/**
**	Cleanup audio ring.
*/
static void AudioRingExit(void)
{
	if (AudioRingBuffer) {
		RingBufferDel(AudioRingBuffer);
		AudioRingBuffer = NULL;
	}
	HwSampleRate = 0;	// checked for valid setup
	InSampleRate = 0;
}


//============================================================================
//	A L S A
//============================================================================

//----------------------------------------------------------------------------
//	Alsa variables
//----------------------------------------------------------------------------

static snd_pcm_t *AlsaPCMHandle;	///< alsa pcm handle
static char AlsaCanPause;		///< hw supports pause
static int AlsaUseMmap;			///< use mmap

static snd_mixer_t *AlsaMixer;		///< alsa mixer handle
static snd_mixer_elem_t *AlsaMixerElem;	///< alsa pcm mixer element
static int AlsaRatio;			///< internal -> mixer ratio * 1000

//----------------------------------------------------------------------------
//	alsa pcm
//----------------------------------------------------------------------------

/**
**	Flush alsa buffers.
*/
static void AlsaFlushBuffers(void)
{
	int err;
	snd_pcm_state_t state;

//	fprintf(stderr, "AlsaFlushBuffers: AlsaFlushBuffers\n");

	state = snd_pcm_state(AlsaPCMHandle);
	Debug(3, "audio/alsa: flush state %s\n", snd_pcm_state_name(state));
	if (state != SND_PCM_STATE_OPEN) {
		if ((err = snd_pcm_drop(AlsaPCMHandle)) < 0) {
			Error(_("audio: snd_pcm_drop(): %s\n"), snd_strerror(err));
		}
		// ****ing alsa crash, when in open state here
		if ((err = snd_pcm_prepare(AlsaPCMHandle)) < 0) {
			Error(_("audio: snd_pcm_prepare(): %s\n"), snd_strerror(err));
			fprintf(stderr, "AlsaFlushBuffers: snd_pcm_prepare(): %s\n",
				snd_strerror(err));
		}
	}

	RingBufferReset(AudioRingBuffer);
	PTS = AV_NOPTS_VALUE;
	AudioVideoIsReady = 0;
}

//----------------------------------------------------------------------------
//	thread playback
//----------------------------------------------------------------------------

/**
**	Alsa thread
**
**	Play some samples and return.
**
**	@retval	-1	error
**	@retval	1	running
*/
static int AlsaPlayer(void)
{
	for (;;) {
		int avail;
		int n;
		int err;
		int frames;
		const void *p;

		if (AudioPaused || AlsaPlayerStop) {
			return 1;
		}

		// wait for space in kernel buffers
		if ((err = snd_pcm_wait(AlsaPCMHandle, 150)) < 0) {
			fprintf(stderr, "AlsaPlayer: snd_pcm_wait error? '%s'\n", snd_strerror(err));
			err = snd_pcm_recover(AlsaPCMHandle, err, 0);
			printf("AlsaPlayer: snd_pcm_wait error: snd_pcm_recover %s\n", snd_strerror(err));
		}

		if (AudioPaused || AlsaPlayerStop) {
			return 1;
		}

		// how many bytes can be written?
		n = snd_pcm_avail_update(AlsaPCMHandle);
		if (n < 0) {
			if (n == -EAGAIN) {
				continue;
			}
			err = snd_pcm_recover(AlsaPCMHandle, n, 0);
			if (err >= 0) {
				continue;
			}
			Error(_("audio/alsa: snd_pcm_avail_update(): %s\n"),
				snd_strerror(n));
			return -1;
		}
		avail = snd_pcm_frames_to_bytes(AlsaPCMHandle, n);
		if (avail < 256) {		// too much overhead
			Debug(4, "audio/alsa: break state '%s'\n",
				snd_pcm_state_name(snd_pcm_state(AlsaPCMHandle)));
			break;
		}

		n = RingBufferGetReadPointer(AudioRingBuffer, &p);
		if (!n) {			// ring buffer empty
			fprintf(stderr, "AlsaPlayer: ring buffer empty\n");
		}
		if (n < avail) {		// not enough bytes in ring buffer
			avail = n;
		}
		if (!avail) {			// full or buffer empty
			break;
		}
		// muting pass-through AC-3, can produce disturbance
		if (AudioMute || (AudioSoftVolume
			&& !Passthrough)) {
			// FIXME: quick&dirty cast
			AudioSoftAmplifier((int16_t *) p, avail);
			// FIXME: if not all are written, we double amplify them
		}

		frames = snd_pcm_bytes_to_frames(AlsaPCMHandle, avail);

		pthread_mutex_lock(&AudioRbMutex);
//		if (AlsaUseMmap) {
//			err = snd_pcm_mmap_writei(AlsaPCMHandle, p, frames);
//		} else {
			err = snd_pcm_writei(AlsaPCMHandle, p, frames);
//		}
		RingBufferReadAdvance(AudioRingBuffer, avail);
		pthread_mutex_unlock(&AudioRbMutex);
		if (err != frames) {
			if (err < 0) {
				if (err == -EAGAIN) {
					continue;
				}
				Warning(_("audio/alsa: writei underrun error? '%s'\n"),
					snd_strerror(err));
				err = snd_pcm_recover(AlsaPCMHandle, err, 0);
				if (err >= 0) {
					continue;
				}
				Error(_("audio/alsa: snd_pcm_writei failed: %s\n"),
					snd_strerror(err));
				return -1;
			}
			// this could happen, if underrun happened
			Warning(_("audio/alsa: not all frames written\n"));
			avail = snd_pcm_frames_to_bytes(AlsaPCMHandle, err);
			break;
		}
	}
	return 0;
}

//----------------------------------------------------------------------------

/**
**	Open alsa pcm device.
**
**	@param passthrough	use pass-through (AC-3, ...) device
*/
static snd_pcm_t *AlsaOpenPCM(int passthrough)
{
    const char *device;
    snd_pcm_t *handle;
    int err;

    // &&|| hell
    if (!(passthrough && ((device = AudioPassthroughDevice)
		|| (device = getenv("ALSA_PASSTHROUGH_DEVICE"))))
		&& !(device = AudioPCMDevice) && !(device = getenv("ALSA_DEVICE"))) {

		device = "default";
    }
    if (!AudioDoingInit) {		// reduce blabla during init
		Info(_("audio/alsa: using %sdevice '%s'\n"),
			passthrough ? "pass-through " : "", device);
    }
#if 0
    // for AC3 pass-through try to set the non-audio bit, use AES0=6 to set spdif in raw mode
    if (passthrough && AudioAppendAES) {
	// FIXME: not yet finished
	char *buf;
	const char *s;
	int n;

	n = strlen(device);
	buf = alloca(n + sizeof(":AES0=6") + 1);
	strcpy(buf, device);
	if (!(s = strchr(buf, ':'))) {
	    // no alsa parameters
	    strcpy(buf + n, ":AES=6");
	}
	Debug(3, "audio/alsa: try '%s'\n", buf);
    }
#endif
    // open none blocking; if device is already used, we don't want wait
    if ((err = snd_pcm_open(&handle, device, SND_PCM_STREAM_PLAYBACK,
		SND_PCM_NONBLOCK)) < 0) {

		Error(_("audio/alsa: playback open '%s' error: %s\n"), device,
			snd_strerror(err));
		return NULL;
	}

    if ((err = snd_pcm_nonblock(handle, 0)) < 0) {
		Error(_("audio/alsa: can't set block mode: %s\n"), snd_strerror(err));
    }
    return handle;
}

/**
**	Initialize alsa pcm device.
**
**	@see AudioPCMDevice
*/
static void AlsaInitPCM(void)
{
    snd_pcm_t *handle;
    snd_pcm_hw_params_t *hw_params;
    int err;

    if (!(handle = AlsaOpenPCM(0))) {
		return;
    }
    // FIXME: pass-through and pcm out can support different features
    snd_pcm_hw_params_alloca(&hw_params);
    // choose all parameters
    if ((err = snd_pcm_hw_params_any(handle, hw_params)) < 0) {
		Error(_
			("audio: snd_pcm_hw_params_any: no configurations available: %s\n"),
			snd_strerror(err));
    }
    AlsaCanPause = snd_pcm_hw_params_can_pause(hw_params);
    Info(_("audio/alsa: supports pause: %s\n"), AlsaCanPause ? "yes" : "no");

    AlsaPCMHandle = handle;
#ifdef SOUND_DEBUG
	static snd_output_t *output = NULL;
	err = snd_output_stdio_attach(&output, stdout, 0);
	if (err < 0) {
		printf("Output failed: %s\n", snd_strerror(err));
	} else {
		printf("snd_pcm_dump_setup VOR Setup\n");
		snd_pcm_dump_setup(AlsaPCMHandle, output);
	}
#endif
}

//----------------------------------------------------------------------------
//	Alsa Mixer
//----------------------------------------------------------------------------

/**
**	Set alsa mixer volume (0-1000)
**
**	@param volume	volume (0 .. 1000)
*/
static void AlsaSetVolume(int volume)
{
    int v;

    if (AlsaMixer && AlsaMixerElem) {
		v = (volume * AlsaRatio) / (1000 * 1000);
		snd_mixer_selem_set_playback_volume(AlsaMixerElem, 0, v);
		snd_mixer_selem_set_playback_volume(AlsaMixerElem, 1, v);
    }
}

/**
**	Initialize alsa mixer.
*/
static void AlsaInitMixer(void)
{
    const char *device;
    const char *channel;
    snd_mixer_t *alsa_mixer;
    snd_mixer_elem_t *alsa_mixer_elem;
    long alsa_mixer_elem_min;
    long alsa_mixer_elem_max;

    if (!(device = AudioMixerDevice)) {
		if (!(device = getenv("ALSA_MIXER"))) {
			device = "default";
		}
    }
    if (!(channel = AudioMixerChannel)) {
		if (!(channel = getenv("ALSA_MIXER_CHANNEL"))) {
			channel = "PCM";
		}
    }
    Debug(3, "audio/alsa: mixer %s - %s open\n", device, channel);
    snd_mixer_open(&alsa_mixer, 0);
    if (alsa_mixer && snd_mixer_attach(alsa_mixer, device) >= 0
		&& snd_mixer_selem_register(alsa_mixer, NULL, NULL) >= 0
		&& snd_mixer_load(alsa_mixer) >= 0) {

		const char *const alsa_mixer_elem_name = channel;

		alsa_mixer_elem = snd_mixer_first_elem(alsa_mixer);
		while (alsa_mixer_elem) {
			const char *name;

			name = snd_mixer_selem_get_name(alsa_mixer_elem);
			if (!strcasecmp(name, alsa_mixer_elem_name)) {
			snd_mixer_selem_get_playback_volume_range(alsa_mixer_elem,
				&alsa_mixer_elem_min, &alsa_mixer_elem_max);
			AlsaRatio = 1000 * (alsa_mixer_elem_max - alsa_mixer_elem_min);
			Debug(3, "audio/alsa: PCM mixer found %ld - %ld ratio %d\n",
				alsa_mixer_elem_min, alsa_mixer_elem_max, AlsaRatio);
			break;
			}

			alsa_mixer_elem = snd_mixer_elem_next(alsa_mixer_elem);
		}

		AlsaMixer = alsa_mixer;
		AlsaMixerElem = alsa_mixer_elem;
    } else {
		Error(_("audio/alsa: can't open mixer '%s'\n"), device);
    }
}

//----------------------------------------------------------------------------
//	Alsa API
//----------------------------------------------------------------------------

/**
**	Setup alsa audio for requested format.
**
**	@param freq		sample frequency
**	@param channels		number of channels
**	@param passthrough	use pass-through (AC-3, ...) device
**
**	@retval 0	everything ok
**	@retval 1	didn't support frequency/channels combination
**	@retval -1	something gone wrong
**
**	@todo FIXME: remove pointer for freq + channels
*/
static int AlsaSetup(unsigned int *rate, unsigned int *channels, int passthrough)
{
    snd_pcm_uframes_t buffer_size;
    snd_pcm_uframes_t period_size;
    int err;
    int delay;

    if (!AlsaPCMHandle) {		// alsa not running yet
		// FIXME: if open fails for fe. pass-through, we never recover
		return -1;
    }

	if ((err =
		snd_pcm_set_params(AlsaPCMHandle, SND_PCM_FORMAT_S16,
			AlsaUseMmap ? SND_PCM_ACCESS_MMAP_INTERLEAVED :
			SND_PCM_ACCESS_RW_INTERLEAVED, *channels, *rate, 1,
			150000))) {
		// try reduced buffer size (needed for sunxi)
		// FIXME: alternativ make this configurable
		if ((err =
			snd_pcm_set_params(AlsaPCMHandle, SND_PCM_FORMAT_S16,
			AlsaUseMmap ? SND_PCM_ACCESS_MMAP_INTERLEAVED :
			SND_PCM_ACCESS_RW_INTERLEAVED, *channels, *rate, 1,
			72 * 1000))) {

		if (!AudioDoingInit) {
			Error(_("audio/alsa: set params error: %s\n"),
				snd_strerror(err));
			fprintf(stderr, "audio/AlsaSetup: set params error: %s\n",
				snd_strerror(err));
		}
		// FIXME: must stop sound, AudioChannels ... invalid
		return -1;
		}
	}

    // update buffer

    snd_pcm_get_params(AlsaPCMHandle, &buffer_size, &period_size);
    Debug(3, "audio/alsa: buffer size %lu %zdms, period size %lu %zdms\n",
		buffer_size, snd_pcm_frames_to_bytes(AlsaPCMHandle,
		buffer_size) * 1000 / (*rate * *channels * AudioBytesProSample),
		period_size, snd_pcm_frames_to_bytes(AlsaPCMHandle,
		period_size) * 1000 / (*rate * *channels * AudioBytesProSample));
    Debug(3, "audio/alsa: state %s\n",
		snd_pcm_state_name(snd_pcm_state(AlsaPCMHandle)));

    AudioStartThreshold = snd_pcm_frames_to_bytes(AlsaPCMHandle, period_size);

    // buffer time/delay in ms
    delay = AudioBufferTime;
    if (VideoAudioDelay > 0) {
		delay += VideoAudioDelay / 90;
    }
    if (AudioStartThreshold <
		(*rate * *channels * AudioBytesProSample * delay) / 1000U) {

		AudioStartThreshold =
			(*rate * *channels * AudioBytesProSample * delay) / 1000U;
   }
    // no bigger, than 1/3 the buffer
    if (AudioStartThreshold > AudioRingBufferSize / 3) {
		AudioStartThreshold = AudioRingBufferSize / 3;
    }

    if (!AudioDoingInit) {
		Info(_("audio/alsa: start delay %ums\n"), (AudioStartThreshold * 1000)
			/ (*rate * *channels * AudioBytesProSample));
    }

#ifdef SOUND_DEBUG
	if (!AudioDoingInit) {
		static snd_output_t *output = NULL;
		err = snd_output_stdio_attach(&output, stdout, 0);
		if (err < 0) {
			printf("Output failed: %s\n", snd_strerror(err));
		} else {
			printf("snd_pcm_dump_setup nach Setup\n");
			snd_pcm_dump_setup(AlsaPCMHandle, output);
		}
	}
#endif
    return 0;
}

/**
**	Play audio.
*/
/*static void AlsaPlay(void)
{
    int err;

    if (AlsaCanPause) {
	if ((err = snd_pcm_pause(AlsaPCMHandle, 0))) {
	    Error(_("audio/alsa: snd_pcm_pause(): %s\n"), snd_strerror(err));
	}
    } else {
	if ((err = snd_pcm_prepare(AlsaPCMHandle)) < 0) {
	    Error(_("audio/alsa: snd_pcm_prepare(): %s\n"), snd_strerror(err));
	}
    }
#ifdef DEBUG
    if (snd_pcm_state(AlsaPCMHandle) == SND_PCM_STATE_PAUSED) {
	Error(_("audio/alsa: still paused\n"));
    }
#endif
}*/

/**
**	Pause audio.
*/
/*static void AlsaPause(void)
{
    int err;

    if (AlsaCanPause) {
	if ((err = snd_pcm_pause(AlsaPCMHandle, 1))) {
	    Error(_("snd_pcm_pause(): %s\n"), snd_strerror(err));
	}
    } else {
	if ((err = snd_pcm_drop(AlsaPCMHandle)) < 0) {
	    Error(_("snd_pcm_drop(): %s\n"), snd_strerror(err));
	}
    }
}*/

/**
**	Empty log callback
*/
static void AlsaNoopCallback( __attribute__ ((unused))
    const char *file, __attribute__ ((unused))
    int line, __attribute__ ((unused))
    const char *function, __attribute__ ((unused))
    int err, __attribute__ ((unused))
    const char *fmt, ...)
{
}

/**
**	Initialize alsa audio output module.
*/
static void AlsaInit(void)
{
#ifdef DEBUG
    (void)AlsaNoopCallback;
#else
    // disable display of alsa error messages
    snd_lib_error_set_handler(AlsaNoopCallback);
#endif

    AlsaInitPCM();
    AlsaInitMixer();
}

/**
**	Cleanup alsa audio output module.
*/
static void AlsaExit(void)
{
    if (AlsaPCMHandle) {
		snd_pcm_close(AlsaPCMHandle);
		AlsaPCMHandle = NULL;
    }
    if (AlsaMixer) {
		snd_mixer_close(AlsaMixer);
		AlsaMixer = NULL;
		AlsaMixerElem = NULL;
    }
}


//----------------------------------------------------------------------------
//	thread playback
//----------------------------------------------------------------------------

/**
**	Audio play thread.
**
**	@param dummy	unused thread argument
*/
static void *AudioPlayHandlerThread(void *dummy)
{
	for (;;) {
		// check if we should stop the thread
		if (AudioThreadStop) {
			Debug(3, "audio: play thread stopped\n");
			fprintf(stderr, "AudioPlayHandlerThread: play thread stopped\n");
			return PTHREAD_CANCELED;
		}

		Debug(3, "audio: wait on start condition\n");
		AlsaFlushBuffers();
		AudioResetCompressor();
		AudioResetNormalizer();
		AudioRunning = 0;
		AlsaPlayerStop = 0;
		pthread_mutex_lock(&AudioStartMutex);
#ifdef AV_SYNC_DEBUG
		fprintf(stderr, "AudioPlayHandlerThread: pthread_cond_wait\n");
#endif
		pthread_cond_wait(&AudioStartCond, &AudioStartMutex);
		pthread_mutex_unlock(&AudioStartMutex);

#ifdef AV_SYNC_DEBUG
		fprintf(stderr, "AudioPlayHandlerThread: nach pthread_cond_wait ----> %dms start\n",
			(AudioUsedBytes() * 1000) / (HwSampleRate * HwChannels * AudioBytesProSample));
		Debug(3, "audio: ----> %dms start\n", (AudioUsedBytes() * 1000)
			/ (!HwSampleRate + !HwChannels +
			HwSampleRate * HwChannels * AudioBytesProSample));
#endif

		do {
			if (AudioThreadStop) {
				Debug(3, "audio: play thread stopped\n");
				return PTHREAD_CANCELED;
			}

			// try to play some samples
			AlsaPlayer();

			// FIXME: check AudioPaused ...Thread()
			if (AudioPaused || AlsaPlayerStop) {
				break;
			}
		} while (RingBufferUsedBytes(AudioRingBuffer));
	}
	return dummy;
}

/**
**	Initialize audio thread.
*/
static void AudioInitThread(void)
{
    AudioThreadStop = 0;
    pthread_mutex_init(&AudioRbMutex, NULL);
    pthread_mutex_init(&AudioStartMutex, NULL);
    pthread_cond_init(&AudioStartCond, NULL);
    pthread_create(&AudioThread, NULL, AudioPlayHandlerThread, NULL);
    pthread_setname_np(AudioThread, "softhddev audio");
}

/**
**	Cleanup audio thread.
*/
static void AudioExitThread(void)
{
    void *retval;

    Debug(3, "audio: %s\n", __FUNCTION__);

    if (AudioThread) {
	AudioThreadStop = 1;
	AudioRunning = 1;		// wakeup thread, if needed
	pthread_cond_signal(&AudioStartCond);
	if (pthread_join(AudioThread, &retval) || retval != PTHREAD_CANCELED) {
	    Error(_("audio: can't cancel play thread\n"));
	}
	pthread_cond_destroy(&AudioStartCond);
	pthread_mutex_destroy(&AudioRbMutex);
	pthread_mutex_destroy(&AudioStartMutex);
	AudioThread = 0;
    }
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

/**
**	Place samples in audio output queue.
**
**	@param samples	sample buffer
**	@param count	number of bytes in sample buffer
*/
void AudioEnqueue(AVFrame *frame)
{
	size_t n;
	int16_t *buffer;
	unsigned u;

#ifdef AV_SYNC_DEBUG
	// Control PTS is possible
//	if (frame->pts == (int64_t) AV_NOPTS_VALUE) {
//		fprintf(stderr, "AudioEnqueue: NO VALID PTS\n");
//	}
//	if (frame->pts != PTS) {
//		fprintf(stderr, "AudioEnqueue: frame->pts %s PTS %s Diff %lli\n",
//			PtsTimestamp2String(frame->pts), PtsTimestamp2String(PTS),
//			(frame->pts - PTS) / 90);
//	}
#endif
	if (frame->sample_rate != (int)HwSampleRate ||
		frame->channels != (int)HwChannels) {

		for (u = 0; u < AudioRatesMax; ++u) {
			if ((int)AudioRatesTable[u] == frame->sample_rate) {
				goto found;
			}
			if ((int)AudioRatesTable[u] > frame->sample_rate) {
				break;
			}
		}
		fprintf(stderr, "AudioEnqueue: %dHz sample-rate unsupported\n",
			frame->sample_rate);

found:
		// Test this with HW more then 2 channels
		if (AudioChannelMatrix[u][frame->channels] != frame->channels) {
			fprintf(stderr, "AudioEnqueue: Frame %d channels Matrix %d channels\n",
				frame->channels, AudioChannelMatrix[u][frame->channels]);
			HwChannels = AudioChannelMatrix[u][frame->channels];
		} else {
			HwChannels = frame->channels;
		}
		HwSampleRate = frame->sample_rate;

		AlsaSetup(&HwSampleRate, &HwChannels, 0);
	}

	if (AlsaPlayerStop) {
		av_frame_unref(frame);
		fprintf(stderr, "AudioEnqueue: AlsaPlayerStop!!!\n");
		return;
	}

	int count = frame->nb_samples * frame->channels * AudioBytesProSample;
	buffer = (void *)frame->data[0];

	// audio sample modification allowed and needed?
	if (!Passthrough && (frame->channels != (int)HwChannels)) {
		fprintf(stderr, "AudioEnqueue: audio sample modification allowed and needed\n");

		// resample into ring-buffer is too complex in the case of a roundabout
		// just use a temporary buffer
		buffer = alloca(frame->nb_samples * HwChannels * AudioBytesProSample);
#ifdef USE_AUDIO_MIXER
		// Convert / resample input to hardware format
		AudioResample((void *)frame->data[0], frame->channels, frame->nb_samples,
			buffer, HwChannels);
#else
		if (frame->channels != (int)HwChannels) {
			Debug(3, "audio: internal failure channels mismatch\n");
			fprintf(stderr, "AudioEnqueue: internal failure channels mismatch\n");
			return;
		}
		buffer = (void *)frame->data[0];
#endif
	}

	if (AudioCompression) {		// in place operation
		AudioCompressor(buffer, count);
	}
	if (AudioNormalize) {		// in place operation
		AudioNormalizer(buffer, count);
	}

	pthread_mutex_lock(&AudioRbMutex);
	n = RingBufferWrite(AudioRingBuffer, buffer, count);
	if (n != (size_t) count) {
		Error(_("audio: can't place %d samples in ring buffer\n"), count);
		fprintf(stderr, "AudioEnqueue: can't place %d samples in ring buffer\n", count);
	}
	PTS = frame->pts + ((frame->nb_samples * 1000 * 90) / frame->sample_rate);
	pthread_mutex_unlock(&AudioRbMutex);

	if (!AudioRunning) {		// check, if we can start the thread
		int skip;

		n = RingBufferUsedBytes(AudioRingBuffer);
		skip = AudioSkip;
		// FIXME: round to packet size

		Debug(3, "audio: start? %4zdms skip %dms\n", (n * 1000)
			/ (HwSampleRate * HwChannels * AudioBytesProSample),
			(skip * 1000) / (HwSampleRate * HwChannels * AudioBytesProSample));
#ifdef AV_SYNC_DEBUG
		fprintf(stderr, "AudioEnqueue: start? %4zdms skip %dms AudioSkip %dms\n",
			(n * 1000) / (HwSampleRate * HwChannels * AudioBytesProSample),
			(skip * 1000) / (HwSampleRate * HwChannels * AudioBytesProSample),
			(AudioSkip * 1000) / (HwSampleRate * HwChannels * AudioBytesProSample));
#endif
		if (skip) {
			if (n < (unsigned)skip) {
				skip = n;
			}
			AudioSkip -= skip;
			RingBufferReadAdvance(AudioRingBuffer, skip);
			n = RingBufferUsedBytes(AudioRingBuffer);
		}
		// forced start or enough video + audio buffered
		// for some exotic channels * 4 too small
		if ((AudioVideoIsReady && AudioStartThreshold < n) ||
			AudioStartThreshold * 4 < n) {
			// restart play-back
			// no lock needed, can wakeup next time
#ifdef AV_SYNC_DEBUG
			fprintf(stderr, "AudioEnqueue: start play-back Threshold %ums RingBuffer %zums AudioVideoIsReady %d\n",
				(AudioStartThreshold * 1000) / (HwSampleRate * HwChannels * AudioBytesProSample),
				(n * 1000) / (HwSampleRate * HwChannels * AudioBytesProSample),
				AudioVideoIsReady);
#endif
			AudioRunning = 1;
			pthread_cond_signal(&AudioStartCond);
		}
	}
	av_frame_free(&frame);
}

void AudioFilter(AVFrame *inframe)
{
	AVFrame *outframe = NULL;
	int err;

	if (!inframe) {
		goto get_frame;
		fprintf(stderr, "AudioFilter: NO inframe!!!\n");
	}

//	if (FilterInit && (inframe->sample_rate != filter_graph->sink_links[0]->sample_rate ||
//		inframe->channels != filter_graph->sink_links[0]->channels || Filterchanged)) {
	if (FilterInit && (inframe->sample_rate != filter_graph->sink_links[0]->sample_rate ||
		Filterchanged)) {

		avfilter_graph_free(&filter_graph);
		FilterInit = 0;
#ifdef DEBUG
		fprintf(stderr, "AudioFilter: Free the filter graph.\n");
#endif
	}

	if (!FilterInit) {
		AudioFilterInit(inframe);
	}

	if ((err = av_buffersrc_add_frame(abuffersrc_ctx, inframe)) < 0) {
		char errbuf[128];
		av_strerror(err, errbuf, sizeof(errbuf));
		fprintf(stderr, "AudioFilter: Error submitting the frame to the filter fmt %s channels %d %s\n",
			av_get_sample_fmt_name(inframe->format), inframe->channels, errbuf);
		av_frame_free(&inframe);
	}

get_frame:
	outframe  = av_frame_alloc();
	err = av_buffersink_get_frame(abuffersink_ctx, outframe);

//	if (!inframe && err != AVERROR_EOF) {		// Das passt noch nicht!!!
//		av_frame_free(&outframe);
//		fprintf(stderr, "AudioFilter: waiting for AVERROR_EOF\n");
//		goto get_frame;
//	}

	if (err == AVERROR(EAGAIN)) {
//		fprintf(stderr, "AudioFilter: Error filtering AVERROR(EAGAIN)\n");
		av_frame_free(&outframe);
	} else if (err == AVERROR_EOF) {
		fprintf(stderr, "AudioFilter: Error filtering AVERROR_EOF\n");
		av_frame_free(&outframe);
	} else if (err < 0) {
		fprintf(stderr, "AudioFilter: Error filtering the data\n");
		av_frame_free(&outframe);
	}

/*		fprintf(stderr, "AudioFilter IN: %s %dHz *%d NS %i linesize %d out %s %dHz *%d NS %i linesize %d count %d\n",
			av_get_sample_fmt_name(inframe->format), inframe->sample_rate,
			inframe->channels, inframe->nb_samples, inframe->linesize[0],
			av_get_sample_fmt_name(outframe->format), outframe->sample_rate,
			outframe->channels, outframe->nb_samples, outframe->linesize[0],
			count);*/
/*		fprintf(stderr, "AudioFilter OUT: PTS %" PRId64 " PTS %" PRId64 "ms %dms nb_samples %d sample_rate%d\n",
			outframe->pts, outframe->pts /90,
			outframe->nb_samples * 1000 / outframe->sample_rate,
			outframe->nb_samples, outframe->sample_rate);
*/
	if (outframe)
		AudioEnqueue(outframe);
}

/**
**	Video is ready.
**
**	@param pts	video presentation timestamp
*/
int AudioVideoReady(int64_t pts)
{
	int64_t audio_pts;
	int64_t used;
	int skip;

	if (AudioRunning) {
		fprintf(stderr, "AudioVideoReady: Audio is Running !!!???\n");
	}

	// no valid audio known
	if (!HwSampleRate || !HwChannels || PTS == AV_NOPTS_VALUE) {
		Debug(3, "audio: a/v start, no valid audio\n");
		fprintf(stderr, "AudioVideoReady: can't a/v start, HwSampleRate %d HwChannels %d PTS %s\n",
			HwSampleRate, HwChannels, PTS == AV_NOPTS_VALUE ? "n" : "y");
		return -1;
	}

	used = RingBufferUsedBytes(AudioRingBuffer);
	audio_pts = PTS - ((used * 90 * 1000) / (HwSampleRate *
		HwChannels * AudioBytesProSample));
	skip = pts - audio_pts - VideoAudioDelay;
#ifdef AV_SYNC_DEBUG
	Debug(3, "AudioVideoReady: a/v sync buf(%4llims) %s|%s = %dms %s\n",
		(used * 1000) / (HwSampleRate * HwChannels * AudioBytesProSample),
		PtsTimestamp2String(pts), PtsTimestamp2String(audio_pts),
		(int)(pts - audio_pts) / 90, AudioRunning ? "running" : "ready");

	fprintf(stderr, "AudioVideoReady: Ringbuffer %" PRId64 "ms V %s A %s Diff %dms %s\n",
		(used * 1000) / (HwSampleRate * HwChannels * AudioBytesProSample),
		PtsTimestamp2String(pts), PtsTimestamp2String(audio_pts),
		skip / 90, AudioRunning ? "running" : "ready");
#endif
	// guard against old PTS
//	if (skip > 0 && skip < 2000 * 90) {
	if (skip > 0) {
		skip = (((int64_t)skip * HwSampleRate) / (1000 * 90))
			* HwChannels * AudioBytesProSample;
		if ((unsigned)skip > used) {
			AudioSkip = skip - used;
			skip = used;
		}
		Debug(3, "audio: sync advance %dms %d/%lli\n",
			(skip * 1000) / (HwSampleRate * HwChannels *
			AudioBytesProSample), skip, used);
#ifdef AV_SYNC_DEBUG
		fprintf(stderr, "AudioVideoReady: sync advance skip %dms %d available %" PRId64 "\n",
			(skip * 1000) / (HwSampleRate * HwChannels *
			AudioBytesProSample), skip, used);
#endif
		RingBufferReadAdvance(AudioRingBuffer, skip);

		used = RingBufferUsedBytes(AudioRingBuffer);
	}

	// enough video + audio buffered
	if (AudioStartThreshold < used) {
		AudioRunning = 1;
#ifdef AV_SYNC_DEBUG
		fprintf(stderr, "AudioVideoReady: start play-back\n");
#endif
		pthread_cond_signal(&AudioStartCond);
	}
	AudioVideoIsReady = 1;
	return 0;
}

/**
**	Flush audio buffers.
*/
void AudioFlushBuffers(void)
{
//	fprintf(stderr, "AudioFlushBuffers: AudioFlushBuffers\n");

	if (AudioRunning)
		AlsaPlayerStop = 1;
	else if (PTS != AV_NOPTS_VALUE)
		AlsaFlushBuffers();

	if (FilterInit)
		Filterchanged = 1;
}

/**
**	Call back to play audio polled.
*/
void AudioPoller(void)
{
    // FIXME: write poller
//	fprintf(stderr, "FIXME: write audio poller!\n");
}

/**
**	Get free bytes in audio output.
*/
int AudioFreeBytes(void)
{
    return AudioRingBuffer ?
		RingBufferFreeBytes(AudioRingBuffer)
		: INT32_MAX;
}

/**
**	Get used bytes in audio output.
*/
int AudioUsedBytes(void)
{
    // FIXME: not correct, if multiple buffer are in use
    return AudioRingBuffer ?
	RingBufferUsedBytes(AudioRingBuffer) : 0;
}

/**
**	Get current audio clock.
**
**	@returns the audio clock in time stamps.
*/
int64_t AudioGetClock(void)
{
	if (!AudioRunning || !HwSampleRate ||
		!AlsaPCMHandle || PTS == AV_NOPTS_VALUE) {
//		printf("AudioGetClock: AV_NOPTS_VALUE! AudioRingFilled %d AudioRunning %s AV_NOPTS %s\n",
//			atomic_read(&AudioRingFilled), AudioRunning ? "y" : "n",
//			(AudioRing[AudioRingRead].PTS == AV_NOPTS_VALUE) ? "y" : "n");
		return AV_NOPTS_VALUE;
	}
	snd_pcm_sframes_t delay;
	int64_t pts;

	pthread_mutex_lock(&AudioRbMutex);
	// delay in frames in alsa + kernel buffers
	if (snd_pcm_delay(AlsaPCMHandle, &delay) < 0) {
		//Debug(3, "audio/alsa: no hw delay\n");
		printf("audio/alsa: no hw delay\n");
		delay = 0L;
	}

	if (delay < 0) {
		delay = 0L;
	}

	pts = ((int64_t) delay * 90 * 1000) / HwSampleRate;

	pts += ((int64_t) RingBufferUsedBytes(AudioRingBuffer)
		* 90 * 1000) / (HwSampleRate * HwChannels * AudioBytesProSample);
	pthread_mutex_unlock(&AudioRbMutex);

	return PTS - pts;
}

/**
**	Set mixer volume (0-1000)
**
**	@param volume	volume (0 .. 1000)
*/
void AudioSetVolume(int volume)
{
    AudioVolume = volume;
    AudioMute = !volume;
    // reduce loudness for stereo output
    if (AudioStereoDescent && InChannels == 2 && !Passthrough) {
	volume -= AudioStereoDescent;
	if (volume < 0) {
	    volume = 0;
	} else if (volume > 1000) {
	    volume = 1000;
	}
    }
    AudioAmplifier = volume;
    if (!AudioSoftVolume) {
	AlsaSetVolume(volume);
    }
}

/**
**	Play audio.
*/
void AudioPlay(void)
{
	if (!AudioPaused) {
		Debug(3, "audio: not paused, check the code\n");
#ifdef DEBUG
		fprintf(stderr, "AudioPlay: not paused, check the code\n");
#endif
		return;
	}
	Debug(3, "audio: resumed\n");
	AudioPaused = 0;
//    AudioEnqueue(NULL, 0, NULL);		// wakeup thread
}

/**
**	Pause audio.
*/
void AudioPause(void)
{
    if (AudioPaused) {
	Debug(3, "audio: already paused, check the code\n");
	return;
    }
    Debug(3, "audio: paused\n");
    AudioPaused = 1;
}

/**
**	Set audio buffer time.
**
**	PES audio packets have a max distance of 300 ms.
**	TS audio packet have a max distance of 100 ms.
**	The period size of the audio buffer is 24 ms.
**	With streamdev sometimes extra +100ms are needed.
*/
void AudioSetBufferTime(int delay)
{
    if (!delay) {
	delay = 600;
    }
    AudioBufferTime = delay;
}

/**
**	Enable/disable software volume.
**
**	@param onoff	-1 toggle, true turn on, false turn off
*/
void AudioSetSoftvol(int onoff)
{
    if (onoff < 0) {
	AudioSoftVolume ^= 1;
    } else {
	AudioSoftVolume = onoff;
    }
}

/**
**	Set normalize volume parameters.
**
**	@param onoff	-1 toggle, true turn on, false turn off
**	@param maxfac	max. factor of normalize /1000
*/
void AudioSetNormalize(int onoff, int maxfac)
{
    if (onoff < 0) {
	AudioNormalize ^= 1;
    } else {
	AudioNormalize = onoff;
    }
    AudioMaxNormalize = maxfac;
}

/**
**	Set volume compression parameters.
**
**	@param onoff	-1 toggle, true turn on, false turn off
**	@param maxfac	max. factor of compression /1000
*/
void AudioSetCompression(int onoff, int maxfac)
{
    if (onoff < 0) {
		AudioCompression ^= 1;
    } else {
		AudioCompression = onoff;
    }
    AudioMaxCompression = maxfac;
    if (!AudioCompressionFactor) {
		AudioCompressionFactor = 1000;
    }
    if (AudioCompressionFactor > AudioMaxCompression) {
		AudioCompressionFactor = AudioMaxCompression;
    }
}

/**
**	Set stereo loudness descent.
**
**	@param delta	value (/1000) to reduce stereo volume
*/
void AudioSetStereoDescent(int delta)
{
    AudioStereoDescent = delta;
    AudioSetVolume(AudioVolume);	// update channel delta
}

/**
**	Set pcm audio device.
**
**	@param device	name of pcm device (fe. "hw:0,9")
**
**	@note this is currently used to select alsa output module.
*/
void AudioSetDevice(const char *device)
{
    AudioPCMDevice = device;
}

/**
**	Set pass-through audio device.
**
**	@param device	name of pass-through device (fe. "hw:0,1")
**
**	@note this is currently usable with alsa only.
*/
void AudioSetPassthroughDevice(const char *device)
{
    AudioPassthroughDevice = device;
}

/**
**	Set pcm audio mixer channel.
**
**	@param channel	name of the mixer channel (fe. PCM or Master)
**
**	@note this is currently used to select alsa output module.
*/
void AudioSetChannel(const char *channel)
{
    AudioMixerChannel = channel;
}

/**
**	Set automatic AES flag handling.
**
**	@param onoff	turn setting AES flag on or off
*/
void AudioSetAutoAES(int onoff)
{
    if (onoff < 0) {
	AudioAppendAES ^= 1;
    } else {
	AudioAppendAES = onoff;
    }
}

/**
**	Initialize audio output module.
**
**	@todo FIXME: make audio output module selectable.
*/
void AudioInit(void)
{
    unsigned u;
    unsigned int freq;
    unsigned int chan;

    AudioDoingInit = 1;
    AudioRingInit();
    AlsaInit();
#ifdef SOUND_DEBUG
	snd_pcm_chmap_query_t **chmaps = snd_pcm_query_chmaps(AlsaPCMHandle);

	if (!chmaps) {
		fprintf(stderr, "AudioInit: No chmaps found!!!\n");
	} else {
		for (int i = 0; chmaps[i] != NULL; i++) {
			char aname[128];
			if (snd_pcm_chmap_print(&chmaps[i]->map, sizeof(aname), aname) <= 0)
				aname[0] = '\0';
			fprintf(stderr, "AudioInit: chmap %s %d %s found\n",
				aname, chmaps[i]->map.channels, snd_pcm_chmap_type_name(chmaps[i]->type));
		}
	}
	snd_pcm_free_chmaps(chmaps);
#endif
    //
    //	Check which channels/rates/formats are supported
    //	FIXME: we force 44.1Khz and 48Khz must be supported equal
    //	FIXME: should use bitmap of channels supported in RatesInHw
    //	FIXME: use loop over sample-rates
    freq = 44100;
    AudioRatesInHw[Audio44100] = 0;
    for (chan = 1; chan < 9; ++chan) {
		unsigned int tchan;
		unsigned int tfreq;

		tchan = chan;
		tfreq = freq;
		if (AlsaSetup(&tfreq, &tchan, 0)) {
			AudioChannelsInHw[chan] = 0;
		} else {
			AudioChannelsInHw[chan] = chan;
			AudioRatesInHw[Audio44100] |= (1 << chan);
		}
    }
    freq = 48000;
    AudioRatesInHw[Audio48000] = 0;
    for (chan = 1; chan < 9; ++chan) {
		unsigned int tchan;
		unsigned int tfreq;

		if (!AudioChannelsInHw[chan]) {
			continue;
		}
		tchan = chan;
		tfreq = freq;
		if (AlsaSetup(&tfreq, &tchan, 0)) {
			//AudioChannelsInHw[chan] = 0;
		} else {
			AudioChannelsInHw[chan] = chan;
			AudioRatesInHw[Audio48000] |= (1 << chan);
		}
    }
    freq = 192000;
    AudioRatesInHw[Audio192000] = 0;
	for (chan = 1; chan < 9; ++chan) {
		unsigned int tchan;
		unsigned int tfreq;

		if (!AudioChannelsInHw[chan]) {
			continue;
		}
		tchan = chan;
		tfreq = freq;
		if (AlsaSetup(&tfreq, &tchan, 0)) {
			//AudioChannelsInHw[chan] = 0;
		} else {
			AudioChannelsInHw[chan] = chan;
			AudioRatesInHw[Audio192000] |= (1 << chan);
		}
    }
    //	build channel support and conversion table
    for (u = 0; u < AudioRatesMax; ++u) {
	for (chan = 1; chan < 9; ++chan) {
	    AudioChannelMatrix[u][chan] = 0;
	    if (!AudioRatesInHw[u]) {	// rate unsupported
		continue;
	    }
	    if (AudioChannelsInHw[chan]) {
		AudioChannelMatrix[u][chan] = chan;
	    } else {
		switch (chan) {
		    case 1:
			if (AudioChannelsInHw[2]) {
			    AudioChannelMatrix[u][chan] = 2;
			}
			break;
		    case 2:
		    case 3:
			if (AudioChannelsInHw[4]) {
			    AudioChannelMatrix[u][chan] = 4;
			    break;
			}
		    case 4:
			if (AudioChannelsInHw[5]) {
			    AudioChannelMatrix[u][chan] = 5;
			    break;
			}
		    case 5:
			if (AudioChannelsInHw[6]) {
			    AudioChannelMatrix[u][chan] = 6;
			    break;
			}
		    case 6:
			if (AudioChannelsInHw[7]) {
			    AudioChannelMatrix[u][chan] = 7;
			    break;
			}
		    case 7:
			if (AudioChannelsInHw[8]) {
			    AudioChannelMatrix[u][chan] = 8;
			    break;
			}
		    case 8:
			if (AudioChannelsInHw[6]) {
			    AudioChannelMatrix[u][chan] = 6;
			    break;
			}
			if (AudioChannelsInHw[2]) {
			    AudioChannelMatrix[u][chan] = 2;
			    break;
			}
			if (AudioChannelsInHw[1]) {
			    AudioChannelMatrix[u][chan] = 1;
			    break;
			}
			break;
		}
	    }
	}
    }
    for (u = 0; u < AudioRatesMax; ++u) {
	Info(_("audio: %6dHz supports %d %d %d %d %d %d %d %d channels\n"),
	    AudioRatesTable[u], AudioChannelMatrix[u][1],
	    AudioChannelMatrix[u][2], AudioChannelMatrix[u][3],
	    AudioChannelMatrix[u][4], AudioChannelMatrix[u][5],
	    AudioChannelMatrix[u][6], AudioChannelMatrix[u][7],
	    AudioChannelMatrix[u][8]);
    }
	AudioInitThread();
    AudioDoingInit = 0;
}

/**
**	Cleanup audio output module.
*/
void AudioExit(void)
{
    Debug(3, "audio: %s\n", __FUNCTION__);

	AudioExitThread();
    AlsaExit();
    AudioRingExit();
    AudioRunning = 0;
    AudioPaused = 0;
}
