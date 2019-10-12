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

#include <unistd.h>
#include <libintl.h>
#define _(str) gettext(str)		///< gettext shortcut
#define _N(str) str			///< gettext_noop shortcut

#include <libavcodec/avcodec.h>

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

//----------------------------------------------------------------------------
//	Video
//----------------------------------------------------------------------------

///
///	Video decoder structure.
///
struct _video_decoder_
{
    VideoRender *Render;		///< video hardware decoder

    AVCodecContext *VideoCtx;		///< video codec context
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
**				valid format, the formats are ordered by quality.
*/
static enum AVPixelFormat Codec_get_format(AVCodecContext * video_ctx,
		const enum AVPixelFormat *fmt)
{
	VideoDecoder *decoder;
	decoder = video_ctx->opaque;

	return Video_get_format(decoder->Render, video_ctx, fmt);
}

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
VideoDecoder *CodecVideoNewDecoder(VideoRender * render)
{
    VideoDecoder *decoder;

    if (!(decoder = calloc(1, sizeof(*decoder)))) {
		Fatal(_("codec: can't allocate vodeo decoder\n"));
    }
    decoder->Render = render;

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
	AVCodec * codec;

	if (!(codec = avcodec_find_decoder_by_name(VideoGetDecoderName(
		avcodec_get_name(codec_id)))))
		fprintf(stderr, "[CodecVideoOpen] The video codec %s is not present in libavcodec\n",
			VideoGetDecoderName(avcodec_get_name(codec_id)));
//	fprintf(stderr, "[CodecVideoOpen] The video codec %s is used\n",
//		VideoGetDecoderName(avcodec_get_name(codec_id)));

	decoder->VideoCtx = avcodec_alloc_context3(codec);
	if (!decoder->VideoCtx) {
		fprintf(stderr, "[CodecVideoOpen]: can't open video codec!\n");
	}

	if (decoder->VideoCtx->codec != codec) {
		fprintf(stderr, "[CodecVideoOpen]: VideoCtx->codec != codec!\n");
		decoder->VideoCtx->codec = codec;
	}
	decoder->VideoCtx->codec_id = codec_id;
	decoder->VideoCtx->get_format = Codec_get_format;
	decoder->VideoCtx->opaque = decoder;

	decoder->VideoCtx->flags |= AV_CODEC_FLAG_BITEXACT;
//	decoder->VideoCtx->flags2 |= AV_CODEC_FLAG2_FAST;
//	decoder->VideoCtx->flags |= AV_CODEC_FLAG_TRUNCATED;
//	if (codec->capabilities & AV_CODEC_CAP_DR1)
//		fprintf(stderr, "[CodecVideoOpen] AV_CODEC_CAP_DR1 => get_buffer()\n");
	if (codec->capabilities & AV_CODEC_CAP_FRAME_THREADS ||
		AV_CODEC_CAP_SLICE_THREADS) {
		decoder->VideoCtx->thread_count = 4;
//		fprintf(stderr, "[CodecVideoOpen] codec use threads\n");
	}
	if (codec->capabilities & AV_CODEC_CAP_SLICE_THREADS){
		decoder->VideoCtx->thread_type = FF_THREAD_SLICE;
//		fprintf(stderr, "[CodecVideoOpen] codec use THREAD_SLICE threads\n");
	}

//	pthread_mutex_lock(&CodecLockMutex);
	if (avcodec_open2(decoder->VideoCtx, decoder->VideoCtx->codec, NULL) < 0)
		fprintf(stderr, "[CodecVideoOpen] Error opening the decoder: ");
//	pthread_mutex_unlock(&CodecLockMutex);
}

/**
**	Close video decoder.
**
**	@param decoder	private video decoder
*/
void CodecVideoClose(VideoDecoder * decoder)
{
#ifdef DEBUG
		fprintf(stderr, "[CodecVideoClose]\n");
#endif
	if (decoder->VideoCtx) {
		avcodec_close(decoder->VideoCtx);
		avcodec_free_context(&decoder->VideoCtx);
		av_freep(&decoder->VideoCtx);
		decoder->VideoCtx = NULL;
	}
}

/**
**	Decode a video packet.
**
**	@param decoder	video decoder data
**	@param avpkt	video packet
*/
int CodecVideoSendPacket(VideoDecoder * decoder, const AVPacket * avpkt)
{
	int ret;

#if 0
	if (!decoder->VideoCtx->extradata_size) {
		AVBSFContext *bsf_ctx;
		const AVBitStreamFilter *f;
		int extradata_size;
		uint8_t *extradata;

		f = av_bsf_get_by_name("extract_extradata");
		if (!f)
			fprintf(stderr, "extradata av_bsf_get_by_name failed!\n");

		if (av_bsf_alloc(f, &bsf_ctx) < 0)
			fprintf(stderr, "extradata av_bsf_alloc failed!\n");

		bsf_ctx->par_in->codec_id = decoder->VideoCtx->codec_id;

		if (av_bsf_init(bsf_ctx) < 0)
			fprintf(stderr, "extradata av_bsf_init failed!\n");

		if (av_bsf_send_packet(bsf_ctx, avpkt) < 0)
			fprintf(stderr, "extradata av_bsf_send_packet failed!\n");

		if (av_bsf_receive_packet(bsf_ctx, avpkt) < 0)
			fprintf(stderr, "extradata av_bsf_send_packet failed!\n");

		extradata = av_packet_get_side_data(avpkt, AV_PKT_DATA_NEW_EXTRADATA,
			&extradata_size);

		decoder->VideoCtx->extradata = av_mallocz(extradata_size + AV_INPUT_BUFFER_PADDING_SIZE);
		memcpy(decoder->VideoCtx->extradata, extradata, extradata_size);
		decoder->VideoCtx->extradata_size = extradata_size;

		av_bsf_free(&bsf_ctx);

		fprintf(stderr, "extradata %p %d\n", decoder->VideoCtx->extradata, decoder->VideoCtx->extradata_size);
	}
#endif

	if (!avpkt->size) {
#ifdef DEBUG
		fprintf(stderr, "CodecVideoSendPacket: !avpkt->size pts %s\n",
			PtsTimestamp2String(avpkt->pts));
#endif
		return 0;
	}

	ret = avcodec_send_packet(decoder->VideoCtx, avpkt);

	if (ret == AVERROR(ENOMEM))
		fprintf(stderr, "CodecVideoSendPacket: Error sending a packet for decoding AVERROR(ENOMEM)\n");
	if (ret == AVERROR(EINVAL))
		fprintf(stderr, "CodecVideoSendPacket: Error sending a packet for decoding AVERROR(EINVAL)\n");

	if (ret == AVERROR(EAGAIN))
		return 1;
	return 0;
}

void CodecVideoReceiveFrame(VideoDecoder * decoder)
{
	int ret;

	if (!(decoder->Frame = av_frame_alloc())) {
		Fatal(_("codec: can't allocate decoder frame\n"));
	}

	ret = avcodec_receive_frame(decoder->VideoCtx, decoder->Frame);

	if (!ret) {
		VideoRenderFrame(decoder->Render, decoder->VideoCtx, decoder->Frame);
	} else {
		av_frame_free(&decoder->Frame);
	}

	if (ret == AVERROR(EINVAL))
		fprintf(stderr, "CodecVideoReceiveFrame: Error receive frame AVERROR(EINVAL)\n");
}

/**
**	Flush the video decoder.
**
**	@param decoder	video decoder data
*/
void CodecVideoFlushBuffers(VideoDecoder * decoder)
{
#ifdef DEBUG
	fprintf(stderr, "CodecVideoFlushBuffers: \n");
#endif
	if (decoder->VideoCtx) {
		avcodec_flush_buffers(decoder->VideoCtx);
	}
}

//----------------------------------------------------------------------------
//	Audio
//----------------------------------------------------------------------------

///
///	Audio decoder structure.
///
struct _audio_decoder_
{
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
	audio_decoder->AudioCtx = NULL;

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
	AVCodec *codec;

	if (codec_id == AV_CODEC_ID_AC3) {
		if (!(codec = avcodec_find_decoder_by_name("ac3_fixed"))) {
			Fatal(_("codec: codec ac3_fixed ID %#06x not found\n"), codec_id);
		}
	} else {
		if (!(codec = avcodec_find_decoder(codec_id))) {
			Fatal(_("codec: codec %s ID %#06x not found\n"),
				avcodec_get_name(codec_id), codec_id);
			// FIXME: errors aren't fatal
		}
	}

	if (!(audio_decoder->AudioCtx = avcodec_alloc_context3(codec))) {
		Fatal(_("codec: can't allocate audio codec context\n"));
	}

//	if (CodecDownmix) {
#ifdef DEBUG
		fprintf(stderr, "CodecAudioOpen: CodecDownmix to AV_CH_LAYOUT_STEREO\n");
#endif
		audio_decoder->AudioCtx->request_channel_layout =
			AV_CH_LAYOUT_STEREO;
//	}

	// open codec
	if (avcodec_open2(audio_decoder->AudioCtx, audio_decoder->AudioCtx->codec, NULL) < 0) {
		Fatal(_("codec: can't open audio codec\n"));
	}
#ifdef DEBUG
	Debug(3, "codec: audio '%s'\n", audio_decoder->AudioCtx->codec->long_name);
	fprintf(stderr,"CodecAudioOpen: audio '%s'\n", audio_decoder->AudioCtx->codec->long_name);
#endif
	if (audio_decoder->AudioCtx->codec->capabilities & AV_CODEC_CAP_TRUNCATED) {
		Debug(3, "codec: audio can use truncated packets\n");
		fprintf(stderr, "CodecAudioOpen: audio can use truncated packets\n");
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
#ifdef DEBUG
		fprintf(stderr, "[CodecAudioClose]\n");
#endif
	if (audio_decoder->AudioCtx) {
		avcodec_close(audio_decoder->AudioCtx);
		av_freep(&audio_decoder->AudioCtx);
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

//	fprintf(stderr, "CodecAudioDecode: \n");

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

#ifdef AV_SYNC_DEBUG
	// Control PTS is possible
	if (audio_decoder->last_pts == (int64_t) AV_NOPTS_VALUE &&
		frame->pts == (int64_t) AV_NOPTS_VALUE) {
		fprintf(stderr, "CodecAudioDecode: NO VALID PTS\n");
	}
	if (frame->pts != (int64_t) AV_NOPTS_VALUE &&
		frame->pts != audio_decoder->last_pts + 
			(int64_t)(frame->nb_samples * 1000 * 90 / frame->sample_rate)) {
		fprintf(stderr, "CodecAudioDecode: frame->pts %s last_pts + samples %s\n",
			PtsTimestamp2String(frame->pts), PtsTimestamp2String(audio_decoder->last_pts + 
			(int64_t)(frame->nb_samples * 1000 * 90 / frame->sample_rate)));
	}
#endif
	// update audio clock
	if (frame->pts != (int64_t) AV_NOPTS_VALUE) {
		audio_decoder->last_pts = frame->pts;
	} else if (audio_decoder->last_pts != (int64_t) AV_NOPTS_VALUE) {
		frame->pts = audio_decoder->last_pts + 
			(int64_t)(frame->nb_samples * 1000 * 90 / frame->sample_rate);
		audio_decoder->last_pts = frame->pts;
	}

	AudioFilter(frame);
	return;
}


/**
**	Flush the audio decoder.
**
**	@param decoder	audio decoder data
*/
void CodecAudioFlushBuffers(AudioDecoder * decoder)
{
#ifdef DEBUG
	fprintf(stderr, "CodecAudioFlushBuffers: \n");
#endif
	if (decoder->AudioCtx) {
		avcodec_flush_buffers(decoder->AudioCtx);
	}
	decoder->last_pts = AV_NOPTS_VALUE;
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
#ifndef DEBUG
    // disable display ffmpeg error messages
    av_log_set_callback(CodecNoopCallback);
#else
    (void)CodecNoopCallback;
//		av_log_set_level(AV_LOG_DEBUG);
//		av_log_set_level(AV_LOG_ERROR );
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
}
