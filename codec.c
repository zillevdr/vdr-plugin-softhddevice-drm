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
//#define USE_AUDIO_DRIFT_CORRECTION
    /// compile AC-3 audio drift correction support (very experimental)
//#define USE_AC3_DRIFT_CORRECTION

#include <unistd.h>
#include <libintl.h>
#define _(str) gettext(str)		///< gettext shortcut
#define _N(str) str			///< gettext_noop shortcut

#include <libavcodec/avcodec.h>

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
		// strlen statt sizeof???
		char *codec_name = (char *) malloc(1 + sizeof(avcodec_get_name(codec_id)) + 7);
		strcpy(codec_name, avcodec_get_name(codec_id));
		strcat(codec_name, "_rkmpp");
//		strcat(codec_name, "_v4l2m2m");
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

//	decoder->VideoCtx->codec_id = codec_id;
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

    AVFrame *Frame;			///< decoded audio frame buffer
    int64_t last_pts;			///< last PTS
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
}

/**
**	Close audio decoder.
**
**	@param audio_decoder	private audio decoder
*/
void CodecAudioClose(AudioDecoder * audio_decoder)
{
    // FIXME: output any buffered data
    if (audio_decoder->AudioCtx) {
	pthread_mutex_lock(&CodecLockMutex);
	avcodec_close(audio_decoder->AudioCtx);
	av_freep(&audio_decoder->AudioCtx);
	pthread_mutex_unlock(&CodecLockMutex);
    }
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
    if (frame->pts != (int64_t) AV_NOPTS_VALUE) {
		audio_decoder->last_pts = frame->pts;
    } else {
		frame->pts = audio_decoder->last_pts + 
			(int64_t)(frame->nb_samples * 1000 * 90 / frame->sample_rate);
		audio_decoder->last_pts = frame->pts;
	}

	AudioEnqueue(NULL, 0, frame);
	return;
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

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58,18,100)
    avcodec_register_all();		// register all formats and codecs
#endif
}

/**
**	Codec exit.
*/
void CodecExit(void)
{
    pthread_mutex_destroy(&CodecLockMutex);
}
