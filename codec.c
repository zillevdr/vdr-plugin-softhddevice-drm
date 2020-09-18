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
#include <pthread.h>

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

static pthread_mutex_t CodecLockMutex;

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
void CodecVideoOpen(VideoDecoder * decoder, int codec_id, AVCodecParameters * Par,
		AVRational * timebase)
{
	AVCodec * codec;
	enum AVHWDeviceType type = 0;
	static AVBufferRef *hw_device_ctx = NULL;

	if (VideoCodecMode(decoder->Render) == 1) {
		if (!(codec = avcodec_find_decoder_by_name(VideoGetDecoderName(
			avcodec_get_name(codec_id)))))

			fprintf(stderr, "CodecVideoOpen: The video codec %s is not present in libavcodec\n",
				VideoGetDecoderName(avcodec_get_name(codec_id)));
	} else {

		if (!(codec = avcodec_find_decoder(codec_id)))
			fprintf(stderr, "CodecVideoOpen: The video codec %s is not present in libavcodec\n",
				avcodec_get_name(codec_id));

		if (!(VideoCodecMode(decoder->Render) == 2 && codec_id == AV_CODEC_ID_MPEG2VIDEO)) {
			for (int n = 0; ; n++) {
				const AVCodecHWConfig *cfg = avcodec_get_hw_config(codec, n);
				if (!cfg)
					break;
				if (cfg->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX &&
					cfg->device_type == AV_HWDEVICE_TYPE_DRM) {

#ifdef CODEC_DEBUG
					fprintf(stderr, "CodecVideoOpen: HW codec %s gefunden\n",
						av_hwdevice_get_type_name(cfg->device_type));
#endif
					type = cfg->device_type;
					break;
				}
			}
		}
	}
#ifdef CODEC_DEBUG
	fprintf(stderr, "CodecVideoOpen: Codec %s found\n", codec->long_name);
#endif
	decoder->VideoCtx = avcodec_alloc_context3(codec);
	if (!decoder->VideoCtx) {
		fprintf(stderr, "CodecVideoOpen: can't open video codec!\n");
	}

	decoder->VideoCtx->codec_id = codec_id;
	decoder->VideoCtx->get_format = Codec_get_format;
	decoder->VideoCtx->opaque = decoder;

//	decoder->VideoCtx->flags |= AV_CODEC_FLAG_BITEXACT;
//	decoder->VideoCtx->flags2 |= AV_CODEC_FLAG2_FAST;
//	decoder->VideoCtx->flags |= AV_CODEC_FLAG_TRUNCATED;
//	if (codec->capabilities & AV_CODEC_CAP_DR1)
//		fprintf(stderr, "[CodecVideoOpen] AV_CODEC_CAP_DR1 => get_buffer()\n");
	if (codec->capabilities & AV_CODEC_CAP_FRAME_THREADS ||
		AV_CODEC_CAP_SLICE_THREADS) {
		decoder->VideoCtx->thread_count = 4;
#ifdef CODEC_DEBUG
		fprintf(stderr, "CodecVideoOpen: decoder use %d threads\n",
			decoder->VideoCtx->thread_count);
#endif
	}
	if (codec->capabilities & AV_CODEC_CAP_SLICE_THREADS){
		decoder->VideoCtx->thread_type = FF_THREAD_SLICE;
#ifdef CODEC_DEBUG
		fprintf(stderr, "CodecVideoOpen: decoder use THREAD_SLICE threads\n");
#endif
	}

	if (type) {
		if (av_hwdevice_ctx_create(&hw_device_ctx, type, NULL, NULL, 0) < 0)
			fprintf(stderr, "CodecVideoOpen: Error init the HW decoder\n");
		else
			decoder->VideoCtx->hw_device_ctx = av_buffer_ref(hw_device_ctx);
	}

	if (Par) {
		if ((avcodec_parameters_to_context(decoder->VideoCtx, Par)) < 0)
			fprintf(stderr, "CodecVideoOpen: insert parameters to context failed!\n");
	}
	if (timebase) {
		decoder->VideoCtx->pkt_timebase.num = timebase->num;
		decoder->VideoCtx->pkt_timebase.den = timebase->den;
	}

	if (avcodec_open2(decoder->VideoCtx, decoder->VideoCtx->codec, NULL) < 0)
		fprintf(stderr, "CodecVideoOpen: Error opening the decoder\n");
}

/**
**	Close video decoder.
**
**	@param decoder	private video decoder
*/
void CodecVideoClose(VideoDecoder * decoder)
{
#ifdef DEBUG
	fprintf(stderr, "CodecVideoClose: VideoCtx %p\n", decoder->VideoCtx);
#endif
	pthread_mutex_lock(&CodecLockMutex);
	if (decoder->VideoCtx) {
		avcodec_free_context(&decoder->VideoCtx);
	}
	pthread_mutex_unlock(&CodecLockMutex);
}

/**
**	Decode a video packet.
**
**	@param decoder	video decoder data
**	@param avpkt	video packet
**
**	@returns 1 if packet must send again.
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

int CodecVideoReceiveFrame(VideoDecoder * decoder, int no_deint)
{
	int ret;

	if (!(decoder->Frame = av_frame_alloc())) {
		Fatal(_("CodecVideoReceiveFrame: can't allocate decoder frame\n"));
	}

	ret = avcodec_receive_frame(decoder->VideoCtx, decoder->Frame);

	if (!ret) {
		if (no_deint) {
			decoder->Frame->interlaced_frame = 0;
#ifdef STILL_DEBUG
			fprintf(stderr, "CodecVideoReceiveFrame: interlaced_frame = 0\n");
#endif
		}
		VideoRenderFrame(decoder->Render, decoder->VideoCtx, decoder->Frame);
	} else {
		av_frame_free(&decoder->Frame);
#ifdef DEBUG
		fprintf(stderr, "CodecVideoReceiveFrame: av_frame_free ret: %s\n",
			av_err2str(ret));
#endif
	}

#ifdef DEBUG
	if (ret == AVERROR(EINVAL))
		fprintf(stderr, "CodecVideoReceiveFrame: Error receive frame AVERROR(EINVAL)\n");
#endif

	if (ret == AVERROR(EAGAIN))
		return 1;
	return 0;
}

/**
**	Flush the video decoder.
**
**	@param decoder	video decoder data
*/
void CodecVideoFlushBuffers(VideoDecoder * decoder)
{
#ifdef DEBUG
	fprintf(stderr, "CodecVideoFlushBuffers: VideoCtx %p\n", decoder->VideoCtx);
#endif
	pthread_mutex_lock(&CodecLockMutex);
	if (decoder->VideoCtx) {
		avcodec_flush_buffers(decoder->VideoCtx);
	}
	pthread_mutex_unlock(&CodecLockMutex);
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
void CodecAudioOpen(AudioDecoder * audio_decoder, int codec_id,
		AVCodecParameters *Par, AVRational * timebase)
{
	AVCodec *codec;

	if (codec_id == AV_CODEC_ID_AC3) {
		if (!(codec = avcodec_find_decoder_by_name("ac3_fixed"))) {
			Fatal(_("codec: codec ac3_fixed ID %#06x not found\n"), codec_id);
		}
	} else if (codec_id == AV_CODEC_ID_AAC) {	// ???
		if (!(codec = avcodec_find_decoder_by_name("aac_fixed"))) {
			Fatal(_("codec: codec aac_fixed ID %#06x not found\n"), codec_id);
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

	if (CodecDownmix) {
#ifdef CODEC_DEBUG
		fprintf(stderr, "CodecAudioOpen: CodecDownmix to AV_CH_LAYOUT_STEREO CodecDownmix %d\n",
			CodecDownmix);
#endif
		audio_decoder->AudioCtx->request_channel_layout = AV_CH_LAYOUT_STEREO;
	}

	audio_decoder->AudioCtx->pkt_timebase.num = timebase->num;
	audio_decoder->AudioCtx->pkt_timebase.den = timebase->den;

	if (Par) {
		if ((avcodec_parameters_to_context(audio_decoder->AudioCtx, Par)) < 0)
			fprintf(stderr, "CodecAudioOpen: insert parameters to context failed!\n");
	}

	// open codec
	if (avcodec_open2(audio_decoder->AudioCtx, audio_decoder->AudioCtx->codec, NULL) < 0) {
		Fatal(_("codec: can't open audio codec\n"));
	}
#ifdef CODEC_DEBUG
	Debug(3, "CodecAudioOpen: Codec %s found\n", audio_decoder->AudioCtx->codec->long_name);
	fprintf(stderr,"CodecAudioOpen: Codec %s found\n", audio_decoder->AudioCtx->codec->long_name);
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
	fprintf(stderr, "CodecAudioClose\n");
#endif
	if (audio_decoder->AudioCtx) {
		avcodec_free_context(&audio_decoder->AudioCtx);
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
**
**	@retval	-1	error, send packet again
**	@retval	1	error, send packet again with new configuration
*/
int CodecAudioDecode(AudioDecoder * audio_decoder, const AVPacket * avpkt)
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
			return -1;
		}
		if (n < 0) {			// no audio frame could be decompressed
			Error(_("codec/audio: bad audio frame\n"));
			return 0;	// ???
		}
		Error(_("codec/audio: error more than one frame data\n"));
	}
	if (!got_frame) {
		Error(_("codec/audio: no frame\n"));
		return 0;	// ???
	}

#ifdef AV_SYNC_DEBUG
	// Control PTS is possible
	if (audio_decoder->last_pts == (int64_t) AV_NOPTS_VALUE &&
		frame->pts == (int64_t) AV_NOPTS_VALUE) {
		fprintf(stderr, "CodecAudioDecode: NO VALID PTS\n");
	}
	if (frame->pts != (int64_t) AV_NOPTS_VALUE &&
		frame->pts != audio_decoder->last_pts + 
			(int64_t)(frame->nb_samples * audio_ctx->pkt_timebase.den / frame->sample_rate)) {
//		fprintf(stderr, "CodecAudioDecode: frame->pts %s last_pts + samples %s\n",
//			PtsTimestamp2String(frame->pts), PtsTimestamp2String(audio_decoder->last_pts + 
//			(int64_t)(frame->nb_samples * audio_ctx->pkt_timebase.den / frame->sample_rate)));
	}
#endif
	// update audio clock
	if (frame->pts != (int64_t) AV_NOPTS_VALUE) {
		audio_decoder->last_pts = frame->pts;
	} else if (audio_decoder->last_pts != (int64_t) AV_NOPTS_VALUE) {
		frame->pts = audio_decoder->last_pts + 
			(int64_t)(frame->nb_samples * audio_ctx->pkt_timebase.den / frame->sample_rate);
		audio_decoder->last_pts = frame->pts;
	}

	if (AudioFilter(frame, audio_decoder->AudioCtx))
		return 1;
	return 0;
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
	pthread_mutex_init(&CodecLockMutex, NULL);
}

/**
**	Codec exit.
*/
void CodecExit(void)
{
	pthread_mutex_destroy(&CodecLockMutex);
}
