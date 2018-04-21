///
///	@file codec.c	@brief Codec functions
///
///	Copyright (c) 2009 - 2015 by Johns.  All Rights Reserved.
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
///	@defgroup Codec The codec module.
///
///		This module contains all decoder and codec functions.
///		It is uses ffmpeg (http://ffmpeg.org) as backend.
///

    /// compile with pass-through support (stable, AC-3, E-AC-3 only)
#define USE_PASSTHROUGH
    /// compile audio drift correction support (very experimental)
#define USE_AUDIO_DRIFT_CORRECTION
    /// compile AC-3 audio drift correction support (very experimental)
#define USE_AC3_DRIFT_CORRECTION

#include <unistd.h>
#include <libintl.h>
#define _(str) gettext(str)		///< gettext shortcut
#define _N(str) str			///< gettext_noop shortcut

#include <libavcodec/avcodec.h>
#include <libswresample/swresample.h>

#ifndef __USE_GNU
#define __USE_GNU
#endif
#include <pthread.h>

#ifdef MAIN_H
#include MAIN_H
#endif
#include "iatomic.h"
#include "misc.h"
#include "video.h"
#include "audio.h"
#include "codec.h"


//----------------------------------------------------------------------------
//	Global
//----------------------------------------------------------------------------

      ///
      ///	ffmpeg lock mutex
      ///
      ///	new ffmpeg dislikes simultanous open/close
      ///	this breaks our code, until this is fixed use lock.
      ///
static pthread_mutex_t CodecLockMutex;

    /// Flag prefer fast channel switch
char CodecUsePossibleDefectFrames;

//----------------------------------------------------------------------------
//	Video
//----------------------------------------------------------------------------

#if 0
///
///	Video decoder typedef.
///
//typedef struct _video_decoder_ Decoder;
#endif

///
///	Video decoder structure.
///
struct _video_decoder_
{
    VideoHwDecoder *HwDecoder;		///< video hardware decoder

    int GetFormatDone;			///< flag get format called!
    AVCodec *VideoCodec;		///< video codec
    AVCodecContext *VideoCtx;		///< video codec context
#ifdef FFMPEG_WORKAROUND_ARTIFACTS
    int FirstKeyFrame;			///< flag first frame
#endif
    AVFrame *Frame;			///< decoded video frame
};

//----------------------------------------------------------------------------
//	Call-backs
//----------------------------------------------------------------------------

/**
**	Callback to negotiate the PixelFormat.
**
**	@param video_ctx	codec context
**	@param fmt		is the list of formats which are supported by
**				the codec, it is terminated by -1 as 0 is a
**				valid format, the formats are ordered by
**				quality.
*/
static enum AVPixelFormat Codec_get_format(AVCodecContext * video_ctx,
    const enum AVPixelFormat *fmt)
{
    VideoDecoder *decoder;
    decoder = video_ctx->opaque;

    // bug in ffmpeg 1.1.1, called with zero width or height
    if (!video_ctx->width || !video_ctx->height) {
	Error("codec/video: ffmpeg buggy: width or height zero\n");
    }

    decoder->GetFormatDone = 1;
    return Video_get_format(decoder->HwDecoder, video_ctx, fmt);
}

/**
**	Video buffer management, get buffer for frame.
**
**	Called at the beginning of each frame to get a buffer for it.
**
**	@param video_ctx	Codec context
**	@param frame		Get buffer for this frame
*/
/*static int Codec_get_buffer(AVCodecContext * video_ctx, AVFrame * frame)
{
    VideoDecoder *decoder;

    decoder = video_ctx->opaque;
    if (!decoder->GetFormatDone) {	// get_format missing
	enum AVPixelFormat fmts[2];

	fprintf(stderr, "codec: buggy libav, use ffmpeg\n");
	Warning(_("codec: buggy libav, use ffmpeg\n"));
	fmts[0] = video_ctx->pix_fmt;
	fmts[1] = AV_PIX_FMT_NONE;
	Codec_get_format(video_ctx, fmts);
    }
    // VA-API:
    if (video_ctx->hwaccel_context) {
	unsigned surface;

	surface = VideoGetSurface(decoder->HwDecoder, video_ctx);

	//Debug(3, "codec: use surface %#010x\n", surface);

	// vaapi needs both fields set
	frame->data[0] = (void *)(size_t) surface;
	frame->data[3] = (void *)(size_t) surface;

	return 0;
    }
    //Debug(3, "codec: fallback to default get_buffer\n");
    return avcodec_default_get_buffer(video_ctx, frame);
}*/

/*static int Codec_get_buffer2(AVCodecContext * video_ctx, AVFrame * frame)
{
    return avcodec_default_get_buffer2(video_ctx, frame, 0);
}*/

/**
**	Video buffer management, release buffer for frame.
**	Called to release buffers which were allocated with get_buffer.
**
**	@param video_ctx	Codec context
**	@param frame		Release buffer for this frame
*/
/*static void Codec_release_buffer(AVCodecContext * video_ctx, AVFrame * frame)
{
    // VA-API
    if (video_ctx->hwaccel_context) {
	VideoDecoder *decoder;
	unsigned surface;

	decoder = video_ctx->opaque;
	surface = (unsigned)(size_t) frame->data[3];

	//Debug(3, "codec: release surface %#010x\n", surface);
	VideoReleaseSurface(decoder->HwDecoder, surface);

	frame->data[0] = NULL;
	frame->data[3] = NULL;

	return;
    }
    //Debug(3, "codec: fallback to default release_buffer\n");
    return avcodec_default_release_buffer(video_ctx, frame);
}*/

//----------------------------------------------------------------------------
//	Test
//----------------------------------------------------------------------------

/**
**	Allocate a new video decoder context.
**
**	@param hw_decoder	video hardware decoder
**
**	@returns private decoder pointer for video decoder.
*/
VideoDecoder *CodecVideoNewDecoder(VideoHwDecoder * hw_decoder)
{
    VideoDecoder *decoder;

    if (!(decoder = calloc(1, sizeof(*decoder)))) {
	Fatal(_("codec: can't allocate vodeo decoder\n"));
    }
    decoder->HwDecoder = hw_decoder;

    return decoder;
}

/**
**	Deallocate a video decoder context.
**
**	@param decoder	private video decoder
*/
void CodecVideoDelDecoder(VideoDecoder * decoder)
{
    free(decoder);
}

/**
**	Open video decoder.
**
**	@param decoder	private video decoder
**	@param codec_id	video codec id
*/
void CodecVideoOpen(VideoDecoder * decoder, int codec_id)
{

	if (codec_id == AV_CODEC_ID_MPEG2VIDEO) {
		if (!(decoder->VideoCodec = avcodec_find_decoder(codec_id))) {
			Fatal(_("codec: codec ID %#06x not found\n"), codec_id);
			fprintf(stderr, "codec: codec ID %#06x not found\n", codec_id);
		}
//		fprintf(stderr, "codec_name: %s codec_id: %#06x\n", avcodec_get_name(codec_id), codec_id);
	} else {
		char *codec_name = (char *) malloc(1 + sizeof(avcodec_get_name(codec_id)) + 7);
		strcpy(codec_name, avcodec_get_name(codec_id));
		strcat(codec_name, "_rkmpp");
		if (!(decoder->VideoCodec = avcodec_find_decoder_by_name(codec_name)))
			fprintf(stderr, "The HW video_codec is not present in libavcodec\n");
//		fprintf(stderr, "codec_name: %s codec_id: %#06x\n", codec_name, codec_id);
		free(codec_name);
//		av_log_set_level(AV_LOG_DEBUG);
//		av_log_set_level(AV_LOG_ERROR );
	}

    if (!(decoder->VideoCtx = avcodec_alloc_context3(decoder->VideoCodec))) {
		Fatal(_("codec: can't allocate video codec context\n"));
		fprintf(stderr, "codec: can't allocate video codec context\n");
    }

	decoder->VideoCtx->get_format = Codec_get_format;
    decoder->VideoCtx->opaque = decoder;	// our structure

    pthread_mutex_lock(&CodecLockMutex);
    // open codec
    if (avcodec_open2(decoder->VideoCtx, decoder->VideoCodec, NULL) < 0) {
		Fatal(_("codec: can't open video codec!\n"));
		fprintf(stderr, "codec: can't open video codec!\n");
    }
    pthread_mutex_unlock(&CodecLockMutex);
}

/**
**	Close video decoder.
**
**	@param video_decoder	private video decoder
*/
void CodecVideoClose(VideoDecoder * video_decoder)
{
	if (video_decoder->VideoCodec->capabilities & AV_CODEC_CAP_DELAY) {
		AVPacket avpkt;

		av_init_packet(&avpkt);
		avpkt.data = NULL;
		avpkt.size = 0;

		CodecVideoDecode(video_decoder, &avpkt);
		av_packet_unref(&avpkt);
	}

    if (video_decoder->VideoCtx) {
	pthread_mutex_lock(&CodecLockMutex);
	avcodec_close(video_decoder->VideoCtx);
	avcodec_free_context(&video_decoder->VideoCtx);
	av_freep(&video_decoder->VideoCtx);
	pthread_mutex_unlock(&CodecLockMutex);
    }
}

#if 0

/**
**	Display pts...
**
**	ffmpeg-0.9 pts always AV_NOPTS_VALUE
**	ffmpeg-0.9 pkt_pts nice monotonic (only with HD)
**	ffmpeg-0.9 pkt_dts wild jumping -160 - 340 ms
*/
void DisplayPts(AVCodecContext * video_ctx, AVFrame * frame)
{
    int ms_delay;
    int64_t pts;
    static int64_t last_pts;

    pts = frame->pkt_pts;
    if (pts == (int64_t) AV_NOPTS_VALUE) {
	printf("*");
    }
    ms_delay = (1000 * video_ctx->time_base.num) / video_ctx->time_base.den;
    ms_delay += frame->repeat_pict * ms_delay / 2;
    printf("codec: PTS %s%s %" PRId64 " %d %d/%d %dms\n",
	frame->repeat_pict ? "r" : " ", frame->interlaced_frame ? "I" : " ",
	pts, (int)(pts - last_pts) / 90, video_ctx->time_base.num,
	video_ctx->time_base.den, ms_delay);

    if (pts != (int64_t) AV_NOPTS_VALUE) {
	last_pts = pts;
    }
}

#endif

/**
**	Decode a video packet.
**
**	@param decoder	video decoder data
**	@param avpkt	video packet
*/
int CodecVideoDecode(VideoDecoder * decoder, const AVPacket * avpkt)
{
    AVCodecContext *video_ctx;
    AVFrame *frame;
    int ret_in, ret_out;
//    int used;
    int got_frame;
    int cap_delay = 0;

    if (!(decoder->Frame = av_frame_alloc())) {
	Fatal(_("codec: can't allocate decoder frame\n"));
    }
    decoder->Frame->format = AV_PIX_FMT_NONE;

    video_ctx = decoder->VideoCtx;
    frame = decoder->Frame;
    if (avpkt->data == NULL)
		cap_delay = 1;

	ret_in = avcodec_send_packet(video_ctx, avpkt);
//	if (ret_in == AVERROR(EAGAIN))
//		fprintf(stderr, "CodecVideoDecode: Error sending a packet for decoding AVERROR(EAGAIN)\n");
	if (ret_in == AVERROR(ENOMEM))
		fprintf(stderr, "CodecVideoDecode: Error sending a packet for decoding AVERROR(ENOMEM)\n");
	if (ret_in == AVERROR(EINVAL))
		fprintf(stderr, "CodecVideoDecode: Error sending a packet for decoding AVERROR(EINVAL)\n");

	ret_out = avcodec_receive_frame(video_ctx, frame);
	if (ret_out == 0) {
		got_frame = 1;
	}
//	if (ret_out == AVERROR(EAGAIN))
//		fprintf(stderr, "CodecVideoDecode: Error receive frame AVERROR(EAGAIN)\n");
//	if (ret_out == AVERROR_EOF)
//		fprintf(stderr, "CodecVideoDecode: Error receive frame AVERROR_EOF\n");
	if (ret_out == AVERROR(EINVAL))
		fprintf(stderr, "CodecVideoDecode: Error receive frame AVERROR(EINVAL)\n");

//		fprintf(stderr, "CodecVideoDecode cap_delay %i frame %i x %i ctx %i x %i frame %p\n",
//			cap_delay, frame->width, frame->height,
//			video_ctx->width, video_ctx->height, frame);

    if (got_frame && !frame->width == 0 && !cap_delay && frame->width == video_ctx->width) {
		VideoRenderFrame(decoder->HwDecoder, video_ctx, frame);
	} else {
//		fprintf(stderr, "CodecVideoDecode cap_delay %i frame %i x %i ctx %i x %i frame %p\n",
//			cap_delay, frame->width, frame->height,
//			video_ctx->width, video_ctx->height, frame);
		av_frame_free(&frame);
    }
	if (ret_in == AVERROR(EAGAIN))
		return 0;
	return 1;
}

/**
**	Flush the video decoder.
**
**	@param decoder	video decoder data
*/
void CodecVideoFlushBuffers(VideoDecoder * decoder)
{
    if (decoder->VideoCtx) {
	avcodec_flush_buffers(decoder->VideoCtx);
    }
}

//----------------------------------------------------------------------------
//	Audio
//----------------------------------------------------------------------------

#if 0
///
///	Audio decoder typedef.
///
typedef struct _audio_decoder_ AudioDecoder;
#endif

///
///	Audio decoder structure.
///
struct _audio_decoder_
{
    AVCodec *AudioCodec;		///< audio codec
    AVCodecContext *AudioCtx;		///< audio codec context

    char Passthrough;			///< current pass-through flags
    int SampleRate;			///< current stream sample rate
    int Channels;			///< current stream channels

    int HwSampleRate;			///< hw sample rate
    int HwChannels;			///< hw channels

    AVFrame *Frame;			///< decoded audio frame buffer
    SwrContext *Resample;		///< ffmpeg software resample context

    uint16_t Spdif[24576 / 2];		///< SPDIF output buffer
    int SpdifIndex;			///< index into SPDIF output buffer
    int SpdifCount;			///< SPDIF repeat counter

    int64_t LastDelay;			///< last delay
    struct timespec LastTime;		///< last time
    int64_t LastPTS;			///< last PTS

    int Drift;				///< accumulated audio drift
    int DriftCorr;			///< audio drift correction value
    int DriftFrac;			///< audio drift fraction for ac3
};

///
///	IEC Data type enumeration.
///
enum IEC61937
{
    IEC61937_AC3 = 0x01,		///< AC-3 data
    // FIXME: more data types
    IEC61937_EAC3 = 0x15,		///< E-AC-3 data
};

#ifdef USE_AUDIO_DRIFT_CORRECTION
#define CORRECT_PCM	1		///< do PCM audio-drift correction
#define CORRECT_AC3	2		///< do AC-3 audio-drift correction
static char CodecAudioDrift;		///< flag: enable audio-drift correction
#else
static const int CodecAudioDrift = 0;
#endif
#ifdef USE_PASSTHROUGH
    ///
    /// Pass-through flags: CodecPCM, CodecAC3, CodecEAC3, ...
    ///
static char CodecPassthrough;
#else
static const int CodecPassthrough = 0;
#endif
static char CodecDownmix;		///< enable AC-3 decoder downmix

/**
**	Allocate a new audio decoder context.
**
**	@returns private decoder pointer for audio decoder.
*/
AudioDecoder *CodecAudioNewDecoder(void)
{
    AudioDecoder *audio_decoder;

    if (!(audio_decoder = calloc(1, sizeof(*audio_decoder)))) {
	Fatal(_("codec: can't allocate audio decoder\n"));
    }
    if (!(audio_decoder->Frame = av_frame_alloc())) {
	Fatal(_("codec: can't allocate audio decoder frame buffer\n"));
    }

    return audio_decoder;
}

/**
**	Deallocate an audio decoder context.
**
**	@param decoder	private audio decoder
*/
void CodecAudioDelDecoder(AudioDecoder * decoder)
{
    av_frame_free(&decoder->Frame);	// callee does checks
    free(decoder);
}

/**
**	Open audio decoder.
**
**	@param audio_decoder	private audio decoder
**	@param codec_id	audio	codec id
*/
void CodecAudioOpen(AudioDecoder * audio_decoder, int codec_id)
{
    AVCodec *audio_codec;

    Debug(3, "codec: using audio codec ID %#06x (%s)\n", codec_id,
	avcodec_get_name(codec_id));

    if (!(audio_codec = avcodec_find_decoder(codec_id))) {
	Fatal(_("codec: codec ID %#06x not found\n"), codec_id);
	// FIXME: errors aren't fatal
    }
    audio_decoder->AudioCodec = audio_codec;

    if (!(audio_decoder->AudioCtx = avcodec_alloc_context3(audio_codec))) {
	Fatal(_("codec: can't allocate audio codec context\n"));
    }

    if (CodecDownmix) {
	audio_decoder->AudioCtx->request_channel_layout =
	    AV_CH_LAYOUT_STEREO_DOWNMIX;
    }
    pthread_mutex_lock(&CodecLockMutex);
    // open codec
    if (1) {
	AVDictionary *av_dict;

	av_dict = NULL;
	// FIXME: import settings
	//av_dict_set(&av_dict, "dmix_mode", "0", 0);
	//av_dict_set(&av_dict, "ltrt_cmixlev", "1.414", 0);
	//av_dict_set(&av_dict, "loro_cmixlev", "1.414", 0);
	if (avcodec_open2(audio_decoder->AudioCtx, audio_codec, &av_dict) < 0) {
	    pthread_mutex_unlock(&CodecLockMutex);
	    Fatal(_("codec: can't open audio codec\n"));
	}
	av_dict_free(&av_dict);
    }
    pthread_mutex_unlock(&CodecLockMutex);
    Debug(3, "codec: audio '%s'\n", audio_decoder->AudioCodec->long_name);

    if (audio_codec->capabilities & AV_CODEC_CAP_TRUNCATED) {
	Debug(3, "codec: audio can use truncated packets\n");
	// we send only complete frames
	// audio_decoder->AudioCtx->flags |= CODEC_FLAG_TRUNCATED;
    }
    audio_decoder->SampleRate = 0;
    audio_decoder->Channels = 0;
    audio_decoder->HwSampleRate = 0;
    audio_decoder->HwChannels = 0;
    audio_decoder->LastDelay = 0;
}

/**
**	Close audio decoder.
**
**	@param audio_decoder	private audio decoder
*/
void CodecAudioClose(AudioDecoder * audio_decoder)
{
    // FIXME: output any buffered data
    if (audio_decoder->Resample) {
	swr_free(&audio_decoder->Resample);
    }
    if (audio_decoder->AudioCtx) {
	pthread_mutex_lock(&CodecLockMutex);
	avcodec_close(audio_decoder->AudioCtx);
	av_freep(&audio_decoder->AudioCtx);
	pthread_mutex_unlock(&CodecLockMutex);
    }
}

/**
**	Set audio drift correction.
**
**	@param mask	enable mask (PCM, AC-3)
*/
void CodecSetAudioDrift(int mask)
{
#ifdef USE_AUDIO_DRIFT_CORRECTION
    CodecAudioDrift = mask & (CORRECT_PCM | CORRECT_AC3);
#endif
    (void)mask;
}

/**
**	Set audio pass-through.
**
**	@param mask	enable mask (PCM, AC-3, E-AC-3)
*/
void CodecSetAudioPassthrough(int mask)
{
#ifdef USE_PASSTHROUGH
    CodecPassthrough = mask & (CodecPCM | CodecAC3 | CodecEAC3);
#endif
    (void)mask;
}

/**
**	Set audio downmix.
**
**	@param onoff	enable/disable downmix.
*/
void CodecSetAudioDownmix(int onoff)
{
    if (onoff == -1) {
	CodecDownmix ^= 1;
	return;
    }
    CodecDownmix = onoff;
}

/**
**	Reorder audio frame.
**
**	ffmpeg L  R  C	Ls Rs		-> alsa L R  Ls Rs C
**	ffmpeg L  R  C	LFE Ls Rs	-> alsa L R  Ls Rs C  LFE
**	ffmpeg L  R  C	LFE Ls Rs Rl Rr	-> alsa L R  Ls Rs C  LFE Rl Rr
**
**	@param buf[IN,OUT]	sample buffer
**	@param size		size of sample buffer in bytes
**	@param channels		number of channels interleaved in sample buffer
*/
static void CodecReorderAudioFrame(int16_t * buf, int size, int channels)
{
    int i;
    int c;
    int ls;
    int rs;
    int lfe;

    switch (channels) {
	case 5:
	    size /= 2;
	    for (i = 0; i < size; i += 5) {
		c = buf[i + 2];
		ls = buf[i + 3];
		rs = buf[i + 4];
		buf[i + 2] = ls;
		buf[i + 3] = rs;
		buf[i + 4] = c;
	    }
	    break;
	case 6:
	    size /= 2;
	    for (i = 0; i < size; i += 6) {
		c = buf[i + 2];
		lfe = buf[i + 3];
		ls = buf[i + 4];
		rs = buf[i + 5];
		buf[i + 2] = ls;
		buf[i + 3] = rs;
		buf[i + 4] = c;
		buf[i + 5] = lfe;
	    }
	    break;
	case 8:
	    size /= 2;
	    for (i = 0; i < size; i += 8) {
		c = buf[i + 2];
		lfe = buf[i + 3];
		ls = buf[i + 4];
		rs = buf[i + 5];
		buf[i + 2] = ls;
		buf[i + 3] = rs;
		buf[i + 4] = c;
		buf[i + 5] = lfe;
	    }
	    break;
    }
}

/**
**	Handle audio format changes helper.
**
**	@param audio_decoder	audio decoder data
**	@param[out] passthrough	pass-through output
*/
static int CodecAudioUpdateHelper(AudioDecoder * audio_decoder,
    int *passthrough)
{
    const AVCodecContext *audio_ctx;
    int err;

    audio_ctx = audio_decoder->AudioCtx;
    Debug(3, "codec/audio: format change %s %dHz *%d channels%s%s%s%s%s\n",
	av_get_sample_fmt_name(audio_ctx->sample_fmt), audio_ctx->sample_rate,
	audio_ctx->channels, CodecPassthrough & CodecPCM ? " PCM" : "",
	CodecPassthrough & CodecMPA ? " MPA" : "",
	CodecPassthrough & CodecAC3 ? " AC-3" : "",
	CodecPassthrough & CodecEAC3 ? " E-AC-3" : "",
	CodecPassthrough ? " pass-through" : "");

    *passthrough = 0;
    audio_decoder->SampleRate = audio_ctx->sample_rate;
    audio_decoder->HwSampleRate = audio_ctx->sample_rate;
    audio_decoder->Channels = audio_ctx->channels;
    audio_decoder->HwChannels = audio_ctx->channels;
    audio_decoder->Passthrough = CodecPassthrough;

    // SPDIF/HDMI pass-through
    if ((CodecPassthrough & CodecAC3 && audio_ctx->codec_id == AV_CODEC_ID_AC3)
	|| (CodecPassthrough & CodecEAC3
	    && audio_ctx->codec_id == AV_CODEC_ID_EAC3)) {
	if (audio_ctx->codec_id == AV_CODEC_ID_EAC3) {
	    // E-AC-3 over HDMI some receivers need HBR
	    audio_decoder->HwSampleRate *= 4;
	}
	audio_decoder->HwChannels = 2;
	audio_decoder->SpdifIndex = 0;	// reset buffer
	audio_decoder->SpdifCount = 0;
	*passthrough = 1;
    }
    // channels/sample-rate not support?
    if ((err =
	    AudioSetup(&audio_decoder->HwSampleRate,
		&audio_decoder->HwChannels, *passthrough))) {

	// try E-AC-3 none HBR
	audio_decoder->HwSampleRate /= 4;
	if (audio_ctx->codec_id != AV_CODEC_ID_EAC3
	    || (err =
		AudioSetup(&audio_decoder->HwSampleRate,
		    &audio_decoder->HwChannels, *passthrough))) {

	    Debug(3, "codec/audio: audio setup error\n");
	    // FIXME: handle errors
	    audio_decoder->HwChannels = 0;
	    audio_decoder->HwSampleRate = 0;
	    return err;
	}
    }

    Debug(3, "codec/audio: resample %s %dHz *%d -> %s %dHz *%d\n",
	av_get_sample_fmt_name(audio_ctx->sample_fmt), audio_ctx->sample_rate,
	audio_ctx->channels, av_get_sample_fmt_name(AV_SAMPLE_FMT_S16),
	audio_decoder->HwSampleRate, audio_decoder->HwChannels);

    return 0;
}

/**
**	Audio pass-through decoder helper.
**
**	@param audio_decoder	audio decoder data
**	@param avpkt		undecoded audio packet
*/
static int CodecAudioPassthroughHelper(AudioDecoder * audio_decoder,
    const AVPacket * avpkt)
{
#ifdef USE_PASSTHROUGH
    const AVCodecContext *audio_ctx;

    audio_ctx = audio_decoder->AudioCtx;
    // SPDIF/HDMI passthrough
    if (CodecPassthrough & CodecAC3 && audio_ctx->codec_id == AV_CODEC_ID_AC3) {
	uint16_t *spdif;
	int spdif_sz;

	spdif = audio_decoder->Spdif;
	spdif_sz = 6144;

#ifdef USE_AC3_DRIFT_CORRECTION
	// FIXME: this works with some TVs/AVReceivers
	// FIXME: write burst size drift correction, which should work with all
	if (CodecAudioDrift & CORRECT_AC3) {
	    int x;

	    x = (audio_decoder->DriftFrac +
		(audio_decoder->DriftCorr * spdif_sz)) / (10 *
		audio_decoder->HwSampleRate * 100);
	    audio_decoder->DriftFrac =
		(audio_decoder->DriftFrac +
		(audio_decoder->DriftCorr * spdif_sz)) % (10 *
		audio_decoder->HwSampleRate * 100);
	    // round to word border
	    x *= audio_decoder->HwChannels * 4;
	    if (x < -64) {		// limit correction
		x = -64;
	    } else if (x > 64) {
		x = 64;
	    }
	    spdif_sz += x;
	}
#endif

	// build SPDIF header and append A52 audio to it
	// avpkt is the original data
	if (spdif_sz < avpkt->size + 8) {
	    Error(_("codec/audio: decoded data smaller than encoded\n"));
	    return -1;
	}
	spdif[0] = htole16(0xF872);	// iec 61937 sync word
	spdif[1] = htole16(0x4E1F);
	spdif[2] = htole16(IEC61937_AC3 | (avpkt->data[5] & 0x07) << 8);
	spdif[3] = htole16(avpkt->size * 8);
	// copy original data for output
	// FIXME: not 100% sure, if endian is correct on not intel hardware
	swab(avpkt->data, spdif + 4, avpkt->size);
	// FIXME: don't need to clear always
	memset(spdif + 4 + avpkt->size / 2, 0, spdif_sz - 8 - avpkt->size);
	// don't play with the ac-3 samples
	AudioEnqueue(spdif, spdif_sz);
	return 1;
    }
    if (CodecPassthrough & CodecEAC3
	&& audio_ctx->codec_id == AV_CODEC_ID_EAC3) {
	uint16_t *spdif;
	int spdif_sz;
	int repeat;

	// build SPDIF header and append A52 audio to it
	// avpkt is the original data
	spdif = audio_decoder->Spdif;
	spdif_sz = 24576;		// 4 * 6144
	if (audio_decoder->HwSampleRate == 48000) {
	    spdif_sz = 6144;
	}
	if (spdif_sz < audio_decoder->SpdifIndex + avpkt->size + 8) {
	    Error(_("codec/audio: decoded data smaller than encoded\n"));
	    return -1;
	}
	// check if we must pack multiple packets
	repeat = 1;
	if ((avpkt->data[4] & 0xc0) != 0xc0) {	// fscod
	    static const uint8_t eac3_repeat[4] = { 6, 3, 2, 1 };

	    // fscod2
	    repeat = eac3_repeat[(avpkt->data[4] & 0x30) >> 4];
	}
	// fprintf(stderr, "repeat %d %d\n", repeat, avpkt->size);

	// copy original data for output
	// pack upto repeat EAC-3 pakets into one IEC 61937 burst
	// FIXME: not 100% sure, if endian is correct on not intel hardware
	swab(avpkt->data, spdif + 4 + audio_decoder->SpdifIndex, avpkt->size);
	audio_decoder->SpdifIndex += avpkt->size;
	if (++audio_decoder->SpdifCount < repeat) {
	    return 1;
	}

	spdif[0] = htole16(0xF872);	// iec 61937 sync word
	spdif[1] = htole16(0x4E1F);
	spdif[2] = htole16(IEC61937_EAC3);
	spdif[3] = htole16(audio_decoder->SpdifIndex * 8);
	memset(spdif + 4 + audio_decoder->SpdifIndex / 2, 0,
	    spdif_sz - 8 - audio_decoder->SpdifIndex);

	// don't play with the eac-3 samples
	AudioEnqueue(spdif, spdif_sz);

	audio_decoder->SpdifIndex = 0;
	audio_decoder->SpdifCount = 0;
	return 1;
    }
#endif
    return 0;
}


/**
**	Set/update audio pts clock.
**
**	@param audio_decoder	audio decoder data
**	@param pts		presentation timestamp
*/
static void CodecAudioSetClock(AudioDecoder * audio_decoder, int64_t pts)
{
#ifdef USE_AUDIO_DRIFT_CORRECTION
    struct timespec nowtime;
    int64_t delay;
    int64_t tim_diff;
    int64_t pts_diff;
    int drift;
    int corr;

    AudioSetClock(pts);

    delay = AudioGetDelay();
    if (!delay) {
	return;
    }
    clock_gettime(CLOCK_MONOTONIC, &nowtime);
    if (!audio_decoder->LastDelay) {
	audio_decoder->LastTime = nowtime;
	audio_decoder->LastPTS = pts;
	audio_decoder->LastDelay = delay;
	audio_decoder->Drift = 0;
	audio_decoder->DriftFrac = 0;
	Debug(3, "codec/audio: inital drift delay %" PRId64 "ms\n",
	    delay / 90);
	return;
    }
    // collect over some time
    pts_diff = pts - audio_decoder->LastPTS;
    if (pts_diff < 10 * 1000 * 90) {
	return;
    }

    tim_diff = (nowtime.tv_sec - audio_decoder->LastTime.tv_sec)
	* 1000 * 1000 * 1000 + (nowtime.tv_nsec -
	audio_decoder->LastTime.tv_nsec);

    drift =
	(tim_diff * 90) / (1000 * 1000) - pts_diff + delay -
	audio_decoder->LastDelay;

    // adjust rounding error
    nowtime.tv_nsec -= nowtime.tv_nsec % (1000 * 1000 / 90);
    audio_decoder->LastTime = nowtime;
    audio_decoder->LastPTS = pts;
    audio_decoder->LastDelay = delay;

    if (0) {
	Debug(3,
	    "codec/audio: interval P:%5" PRId64 "ms T:%5" PRId64 "ms D:%4"
	    PRId64 "ms %f %d\n", pts_diff / 90, tim_diff / (1000 * 1000),
	    delay / 90, drift / 90.0, audio_decoder->DriftCorr);
    }
    // underruns and av_resample have the same time :(((
    if (abs(drift) > 10 * 90) {
	// drift too big, pts changed?
	Debug(3, "codec/audio: drift(%6d) %3dms reset\n",
	    audio_decoder->DriftCorr, drift / 90);
	audio_decoder->LastDelay = 0;
#ifdef DEBUG
	corr = 0;			// keep gcc happy
#endif
    } else {

	drift += audio_decoder->Drift;
	audio_decoder->Drift = drift;
	corr = (10 * audio_decoder->HwSampleRate * drift) / (90 * 1000);
	// SPDIF/HDMI passthrough
	if ((CodecAudioDrift & CORRECT_AC3) && (!(CodecPassthrough & CodecAC3)
		|| audio_decoder->AudioCtx->codec_id != AV_CODEC_ID_AC3)
	    && (!(CodecPassthrough & CodecEAC3)
		|| audio_decoder->AudioCtx->codec_id != AV_CODEC_ID_EAC3)) {
	    audio_decoder->DriftCorr = -corr;
	}

	if (audio_decoder->DriftCorr < -20000) {	// limit correction
	    audio_decoder->DriftCorr = -20000;
	} else if (audio_decoder->DriftCorr > 20000) {
	    audio_decoder->DriftCorr = 20000;
	}
    }

    if (audio_decoder->Resample && audio_decoder->DriftCorr) {
	int distance;

	// try workaround for buggy ffmpeg 0.10
	if (abs(audio_decoder->DriftCorr) < 2000) {
	    distance = (pts_diff * audio_decoder->HwSampleRate) / (900 * 1000);
	} else {
	    distance = (pts_diff * audio_decoder->HwSampleRate) / (90 * 1000);
	}
	if (swr_set_compensation(audio_decoder->Resample,
		audio_decoder->DriftCorr / 10, distance)) {
	    Debug(3, "codec/audio: swr_set_compensation failed\n");
	}
    }
    if (1) {
	static int c;

	if (!(c++ % 10)) {
	    Debug(3, "codec/audio: drift(%6d) %8dus %5d\n",
		audio_decoder->DriftCorr, drift * 1000 / 90, corr);
	}
    }
#else
    AudioSetClock(pts);
#endif
}

/**
**	Handle audio format changes.
**
**	@param audio_decoder	audio decoder data
*/
static void CodecAudioUpdateFormat(AudioDecoder * audio_decoder)
{
    int passthrough;
    const AVCodecContext *audio_ctx;

    if (CodecAudioUpdateHelper(audio_decoder, &passthrough)) {
	// FIXME: handle swresample format conversions.
	return;
    }
    if (passthrough) {			// pass-through no conversion allowed
	return;
    }

    audio_ctx = audio_decoder->AudioCtx;

#ifdef DEBUG
    if (audio_ctx->sample_fmt == AV_SAMPLE_FMT_S16
	&& audio_ctx->sample_rate == audio_decoder->HwSampleRate
	&& !CodecAudioDrift) {
	// FIXME: use Resample only, when it is needed!
	fprintf(stderr, "no resample needed\n");
    }
#endif

    audio_decoder->Resample =
	swr_alloc_set_opts(audio_decoder->Resample, audio_ctx->channel_layout,
	AV_SAMPLE_FMT_S16, audio_decoder->HwSampleRate,
	audio_ctx->channel_layout, audio_ctx->sample_fmt,
	audio_ctx->sample_rate, 0, NULL);
    if (audio_decoder->Resample) {
	swr_init(audio_decoder->Resample);
    } else {
	Error(_("codec/audio: can't setup resample\n"));
    }
}

/**
**	Decode an audio packet.
**
**	PTS must be handled self.
**
**	@note the caller has not aligned avpkt and not cleared the end.
**
**	@param audio_decoder	audio decoder data
**	@param avpkt		audio packet
*/
void CodecAudioDecode(AudioDecoder * audio_decoder, const AVPacket * avpkt)
{
    AVCodecContext *audio_ctx;
    AVFrame *frame;
    int got_frame;
    int n;

    audio_ctx = audio_decoder->AudioCtx;

    // FIXME: don't need to decode pass-through codecs
    frame = audio_decoder->Frame;
    av_frame_unref(frame);

    got_frame = 0;
    n = avcodec_decode_audio4(audio_ctx, frame, &got_frame,
	(AVPacket *) avpkt);

    if (n != avpkt->size) {
	if (n == AVERROR(EAGAIN)) {
	    Error(_("codec/audio: latm\n"));
	    return;
	}
	if (n < 0) {			// no audio frame could be decompressed
	    Error(_("codec/audio: bad audio frame\n"));
	    return;
	}
	Error(_("codec/audio: error more than one frame data\n"));
    }
    if (!got_frame) {
	Error(_("codec/audio: no frame\n"));
	return;
    }
    // update audio clock
    if (avpkt->pts != (int64_t) AV_NOPTS_VALUE) {
	CodecAudioSetClock(audio_decoder, avpkt->pts);
    }
    // format change
    if (audio_decoder->Passthrough != CodecPassthrough
	|| audio_decoder->SampleRate != audio_ctx->sample_rate
	|| audio_decoder->Channels != audio_ctx->channels) {
	CodecAudioUpdateFormat(audio_decoder);
    }

    if (!audio_decoder->HwSampleRate || !audio_decoder->HwChannels) {
	return;				// unsupported sample format
    }

    if (CodecAudioPassthroughHelper(audio_decoder, avpkt)) {
	return;
    }

    if (0) {
	char strbuf[32];
	int data_sz;
	int plane_sz;

	data_sz =
	    av_samples_get_buffer_size(&plane_sz, audio_ctx->channels,
	    frame->nb_samples, audio_ctx->sample_fmt, 1);
	fprintf(stderr, "codec/audio: sample_fmt %s\n",
	    av_get_sample_fmt_name(audio_ctx->sample_fmt));
	av_get_channel_layout_string(strbuf, 32, audio_ctx->channels,
	    audio_ctx->channel_layout);
	fprintf(stderr, "codec/audio: layout %s\n", strbuf);
	fprintf(stderr,
	    "codec/audio: channels %d samples %d plane %d data %d\n",
	    audio_ctx->channels, frame->nb_samples, plane_sz, data_sz);
    }

//	fprintf(stderr, "CodecAudioDecode: avpkt->pts %"PRIu64"\n", avpkt->pts);

    if (audio_decoder->Resample) {
	uint8_t outbuf[8192 * 2 * 8];
	uint8_t *out[1];

	out[0] = outbuf;
	n = swr_convert(audio_decoder->Resample, out,
	    sizeof(outbuf) / (2 * audio_decoder->HwChannels),
	    (const uint8_t **)frame->extended_data, frame->nb_samples);
	if (n > 0) {
	    if (!(audio_decoder->Passthrough & CodecPCM)) {
		CodecReorderAudioFrame((int16_t *) outbuf,
		    n * 2 * audio_decoder->HwChannels,
		    audio_decoder->HwChannels);
	    }
	    AudioEnqueue(outbuf, n * 2 * audio_decoder->HwChannels);
	}
	return;
    }

#ifdef DEBUG
    // should be never reached
    fprintf(stderr, "oops\n");
#endif
}


/**
**	Flush the audio decoder.
**
**	@param decoder	audio decoder data
*/
void CodecAudioFlushBuffers(AudioDecoder * decoder)
{
    avcodec_flush_buffers(decoder->AudioCtx);
}

//----------------------------------------------------------------------------
//	Codec
//----------------------------------------------------------------------------

/**
**	Empty log callback
*/
static void CodecNoopCallback( __attribute__ ((unused))
    void *ptr, __attribute__ ((unused))
    int level, __attribute__ ((unused))
    const char *fmt, __attribute__ ((unused)) va_list vl)
{
}

/**
**	Codec init
*/
void CodecInit(void)
{
    pthread_mutex_init(&CodecLockMutex, NULL);
#ifndef DEBUG
    // disable display ffmpeg error messages
    av_log_set_callback(CodecNoopCallback);
#else
    (void)CodecNoopCallback;
#endif
    avcodec_register_all();		// register all formats and codecs
}

/**
**	Codec exit.
*/
void CodecExit(void)
{
    pthread_mutex_destroy(&CodecLockMutex);
}
