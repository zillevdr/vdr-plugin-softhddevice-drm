///
///	@file video.c	@brief Video module
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
///	@defgroup Video The video module.
///
///	This module contains all video rendering functions.
///

#ifndef __USE_GNU
#define __USE_GNU
#endif

#define USE_VIDEO_THREAD		///< run decoder in an own thread

#include <stdbool.h>
#include <unistd.h>

#include <libintl.h>
#define _(str) gettext(str)		///< gettext shortcut
#define _N(str) str			///< gettext_noop shortcut

#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <libavcodec/avcodec.h>

#include <bcm_host.h>
#include <interface/mmal/mmal.h>
#include <interface/mmal/util/mmal_default_components.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_util_params.h>
#include <interface/mmal/mmal_parameters_video.h>

#include "iatomic.h"			// portable atomic_t
#include "misc.h"
#include "video.h"
#include "audio.h"

//----------------------------------------------------------------------------
//	Defines
//----------------------------------------------------------------------------

#define VIDEO_SURFACES_MAX	4	///< video output surfaces for queue

//----------------------------------------------------------------------------
//	Variables
//----------------------------------------------------------------------------

signed char VideoHardwareDecoder = -1;	///< flag use hardware decoder

int VideoAudioDelay;
int SWDeinterlacer;
int hdr;

extern void AudioVideoReady(int64_t);	///< tell audio video is ready

static pthread_t VideoThread;		///< video decode thread
static pthread_cond_t VideoWakeupCond;	///< wakeup condition variable
static pthread_mutex_t VideoDeintMutex;	///< video condition mutex
static pthread_mutex_t VideoLockMutex;	///< video lock mutex

//----------------------------------------------------------------------------
//	Common Functions
//----------------------------------------------------------------------------

static void VideoThreadExit(void);	///< exit/kill video thread


//----------------------------------------------------------------------------
//	MMAL
//----------------------------------------------------------------------------

typedef struct _Mmal_decoder_
{
    enum AVPixelFormat PixFmt;		///< ffmpeg frame pixfmt
//    int Interlaced;			///< ffmpeg interlaced flag

//    int CropX;				///< video crop x
//    int CropY;				///< video crop y
//    int CropWidth;			///< video crop width
//    int CropHeight;			///< video crop height


    int TrickSpeed;			///< current trick speed
    int TrickCounter;			///< current trick speed counter
    VideoStream *Stream;		///< video stream
    int Closing;			///< flag about closing current stream
    int64_t PTS;			///< video PTS clock

    int StartCounter;			///< counter for video start
    int FramesDuped;			///< number of frames duplicated
    int FramesMissed;			///< number of frames missed
    int FramesDropped;			///< number of frames dropped
    int FrameCounter;			///< number of frames decoded
//    int FramesDisplayed;		///< number of frames displayed
} MmalDecoder;

static MmalDecoder *MmalDecoders[1];	///< open decoder streams
static int MmalDecoderN;		///< number of decoder streams

struct data_t {
	MMAL_COMPONENT_T *vout;
	MMAL_POOL_T *vout_input_pool;
	MMAL_QUEUE_T *vout_queue;

	MMAL_COMPONENT_T *deint;
	MMAL_POOL_T *deint_input_pool;

	DISPMANX_RESOURCE_HANDLE_T resource;
	DISPMANX_DISPLAY_HANDLE_T display;
	DISPMANX_ELEMENT_HANDLE_T element;
	DISPMANX_UPDATE_HANDLE_T update;
	DISPMANX_MODEINFO_T info;
	uint32_t vc_image_ptr;
	void *osd_buf;
	int osd_width;
	int osd_height;
	int osd_x;
	int osd_y;

	uint32_t width;
	int32_t par_num;
	int32_t par_den;
	int interlaced;
	unsigned buffers_in_queue;
//	unsigned buffers_deint_in;
	unsigned buffers_deint_out;
	unsigned buffers;
};

static struct data_t *data_list = NULL;
static void MmalDisplayFrame(void);


//----------------------------------------------------------------------------
//	Helper functions
//----------------------------------------------------------------------------

static void buffer_worker()
{
	MMAL_STATUS_T status;
	MMAL_BUFFER_HEADER_T *buffer;
    struct data_t *data = data_list;

    if (data->buffers_deint_out < 3){
		buffer = mmal_queue_get(data->vout_input_pool->queue);
		mmal_buffer_header_reset(buffer);
		buffer->cmd = 0;
		data->buffers_deint_out++;
		status = mmal_port_send_buffer(data->deint->output[0], buffer);
		if(status != MMAL_SUCCESS)
			fprintf(stderr, "Failed send buffer to deinterlacer output port (%d, %s)\n",
				status, mmal_status_to_string(status));
	}
}


void vsync_callback(__attribute__ ((unused)) DISPMANX_UPDATE_HANDLE_T update,
					__attribute__ ((unused)) void *arg)
{
    struct data_t *data = data_list;

    MmalDisplayFrame();
    if(data->interlaced == 1 && data->buffers > 0){
		buffer_worker();
	}
}

//Renderer
static void vout_control_port_cb(__attribute__ ((unused)) MMAL_PORT_T *port,
								MMAL_BUFFER_HEADER_T *buffer)
{
	MMAL_STATUS_T status;
	AVFrame * frame = (AVFrame *)buffer->user_data;

	if (buffer->cmd == MMAL_EVENT_ERROR) {
		status = *(uint32_t *)buffer->data;
		fprintf(stderr, "Vout MMAL error %"PRIx32" \"%s\"\n",
			status, mmal_status_to_string(status));
	}

	if(frame){
		av_frame_free(&frame);
	}
	mmal_buffer_header_release(buffer);
}

static void vout_input_port_cb(__attribute__ ((unused)) MMAL_PORT_T *port,
							  MMAL_BUFFER_HEADER_T *buffer)
{
	AVFrame * frame = (AVFrame *)buffer->user_data;

	if(frame){
		av_frame_free(&frame);
	}
	mmal_buffer_header_release(buffer);
}

// Deinterlacer
static void deint_control_port_cb(__attribute__ ((unused))MMAL_PORT_T *port,
									MMAL_BUFFER_HEADER_T *buffer)
{
	mmal_buffer_header_release(buffer);
}

static void deint_input_port_cb(__attribute__ ((unused))MMAL_PORT_T *port,
									MMAL_BUFFER_HEADER_T *buffer)
{
//    struct data_t *data = data_list;
	AVFrame * frame = (AVFrame *)buffer->user_data;

	if(frame)
		av_frame_free(&frame);
	mmal_buffer_header_release(buffer);
//	data->buffers_deint_in--;
}

static void deint_output_port_cb(__attribute__ ((unused))MMAL_PORT_T *port,
									MMAL_BUFFER_HEADER_T *buffer)
{
    struct data_t *data = data_list;

	if (buffer->cmd == 0) {
		if (buffer->length > 0) {
			buffer->user_data = NULL;
			// Correct PTS MMAL use microseconds
			buffer->pts = (buffer->pts * 90 / 1000);
			mmal_queue_put(data->vout_queue, buffer);
            data->buffers_deint_out--;
			data->buffers_in_queue++;
		} else {
			mmal_buffer_header_release(buffer);
		}
	} else {
		mmal_buffer_header_release(buffer);
	}
}

static void* pool_allocator_alloc(void *context, uint32_t size)
{
	return mmal_port_payload_alloc((MMAL_PORT_T *)context, size);
}

static void pool_allocator_free(void *context, void *mem)
{
	mmal_port_payload_free((MMAL_PORT_T *)context, (uint8_t *)mem);
}


static void MmalClose(void)
{
	struct data_t *data = data_list;
	MMAL_STATUS_T status;

	if (data->interlaced) {
		if (data->deint->control && data->deint->control->is_enabled) {
			status = mmal_port_disable(data->deint->control);
			if (status != MMAL_SUCCESS) {
				fprintf(stderr, "Failed to disable control port deint (status=%"PRIx32" %s)\n",
					status, mmal_status_to_string(status));
			}
		}
		if (data->deint->input[0] && data->deint->input[0]->is_enabled) {
			status = mmal_port_disable(data->deint->input[0]);
			if (status != MMAL_SUCCESS) {
				fprintf(stderr, "Failed to disable input port deint (status=%"PRIx32" %s)\n",
					status, mmal_status_to_string(status));
			}
		}
		if (data->deint->output[0] && data->deint->output[0]->is_enabled) {
			status = mmal_port_disable(data->deint->output[0]);
			if (status != MMAL_SUCCESS) {
				fprintf(stderr, "Failed to disable output port deint (status=%"PRIx32" %s)\n",
					status, mmal_status_to_string(status));
			}
		}

		if (data->deint && data->deint->is_enabled) {
			status = mmal_component_disable(data->deint);
			if (status != MMAL_SUCCESS) {
				fprintf(stderr, "Failed to disable component deint (status=%"PRIx32" %s)\n",
					status, mmal_status_to_string(status));
			}
		}
		if (data->deint) {
			status = mmal_component_destroy(data->deint);
			if (status != MMAL_SUCCESS) {
				fprintf(stderr, "Failed to destroy component deint (status=%"PRIx32" %s)\n",
					status, mmal_status_to_string(status));
			}
		}

		if (data->deint_input_pool) {
			mmal_pool_destroy(data->deint_input_pool);
		}
	}

	if (data->vout->control && data->vout->control->is_enabled) {
		status = mmal_port_disable(data->vout->control);
		if (status != MMAL_SUCCESS) {
			fprintf(stderr, "Failed to disable control port (status=%"PRIx32" %s)\n",
				status, mmal_status_to_string(status));
		}
	}
	if (data->vout->input[0] && data->vout->input[0]->is_enabled) {
		status = mmal_port_disable(data->vout->input[0]);
		if (status != MMAL_SUCCESS) {
			fprintf(stderr, "Failed to disable input port (status=%"PRIx32" %s)\n",
				status, mmal_status_to_string(status));
		}
	}

	if (data->vout && data->vout->is_enabled) {
		status = mmal_component_disable(data->vout);
		if (status != MMAL_SUCCESS) {
			fprintf(stderr, "Failed to disable Componente vout (status=%"PRIx32" %s)\n",
				status, mmal_status_to_string(status));
		}
	}
	if (data->vout) {
		status = mmal_component_destroy(data->vout);
		if (status != MMAL_SUCCESS) {
			fprintf(stderr, "Failed to destroy Componente vout (status=%"PRIx32" %s)\n",
				status, mmal_status_to_string(status));
		}
	}

	if (data->vout_input_pool) {
		mmal_pool_destroy(data->vout_input_pool);
	}
	if (data->vout_queue) {
		mmal_queue_destroy(data->vout_queue);
	}

	data->interlaced = 0;
	data->buffers_in_queue = 0;
//	data->buffers_deint_in = 0;
	data->buffers_deint_out = 0;
	data->buffers = 0;
}


static void MmalChangeResolution(const AVCodecContext * video_ctx, int interlaced_frame)
{
	struct data_t *data = data_list;
	MMAL_STATUS_T status;

	if(data->width != 0){
		MmalClose();
	}

	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &data->vout);
	if (status != MMAL_SUCCESS)
		fprintf(stderr, "Failed to create MMAL render component %s (status=%"PRIx32" %s)\n",
		    MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, status, mmal_status_to_string(status));

	status = mmal_port_enable(data->vout->control, vout_control_port_cb);
	if (status != MMAL_SUCCESS)
		fprintf(stderr, "Failed to enable render control port %s (%x, %s)\n",
		    data->vout->control->name, status, mmal_status_to_string(status));

	data->vout->input[0]->format->type = MMAL_ES_TYPE_VIDEO;
	data->vout->input[0]->format->encoding = MMAL_ENCODING_OPAQUE;
	data->vout->input[0]->format->es->video.width = video_ctx->width;
	data->vout->input[0]->format->es->video.height = video_ctx->height;
	data->vout->input[0]->format->es->video.crop.x = 0;
	data->vout->input[0]->format->es->video.crop.y = 0;
	data->vout->input[0]->format->es->video.crop.width = video_ctx->width;
	data->vout->input[0]->format->es->video.crop.height = video_ctx->height;
	data->vout->input[0]->format->es->video.frame_rate.num = video_ctx->framerate.num;
	data->vout->input[0]->format->es->video.frame_rate.den = video_ctx->framerate.den;
	data->vout->input[0]->format->es->video.par.num = video_ctx->sample_aspect_ratio.num;
	data->vout->input[0]->format->es->video.par.den = video_ctx->sample_aspect_ratio.den;
	//data->vout->input[0]->format->es->video.color_space
	//data->vout->input[0]->format->bitrate
	data->vout->input[0]->format->flags = MMAL_ES_FORMAT_FLAG_FRAMED;
	status = mmal_port_format_commit(data->vout->input[0]);
	if (status != MMAL_SUCCESS)
		fprintf(stderr, "Failed to commit format for render input port %s (status=%"PRIx32" %s)\n",
		   data->vout->input[0]->name, status, mmal_status_to_string(status));

    // Set PARAMETER_DISPLAYREGION
/*    MMAL_DISPLAYREGION_T param;
    param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
    param.hdr.size = sizeof(MMAL_DISPLAYREGION_T);
    param.layer = 2;
//    param.fullscreen = 1;
//    param.mode = MMAL_DISPLAY_MODE_FILL;
    param.mode = MMAL_DISPLAY_MODE_LETTERBOX;
//    param.display_num = 0;//0 typically being a directly connected LCD display
//    param.set = MMAL_DISPLAY_SET_LAYER|MMAL_DISPLAY_SET_NUM|MMAL_DISPLAY_SET_FULLSCREEN|MMAL_DISPLAY_SET_MODE;
    param.set = MMAL_DISPLAY_SET_LAYER|MMAL_DISPLAY_SET_MODE;
    status = mmal_port_parameter_set(data->vout->input[0], &param.hdr);
    if(status != MMAL_SUCCESS && status != MMAL_ENOSYS)
        fprintf(stderr, "cannot set vout Component (%d): %m\n", status);
*/
	data->vout->input[0]->buffer_size = data->vout->input[0]->buffer_size_recommended;
	data->vout->input[0]->buffer_num = 50;
	status = mmal_port_enable(data->vout->input[0], vout_input_port_cb);
	if (status != MMAL_SUCCESS)
		fprintf(stderr, "Failed to enable renderer input port %s (status=%"PRIx32" %s)\n",
		    data->vout->input[0]->name, status, mmal_status_to_string(status));

	status = mmal_component_enable(data->vout);
	if (status != MMAL_SUCCESS)
		fprintf(stderr, "Failed to enable renderer component %s (status=%"PRIx32" %s)\n",
		    data->vout->name, status, mmal_status_to_string(status));

	data->vout_input_pool = mmal_pool_create_with_allocator(data->vout->input[0]->buffer_num,
	   data->vout->input[0]->buffer_size, data->vout->input[0], pool_allocator_alloc, pool_allocator_free);
	if(!data->vout_input_pool)
		printf("Failed to create pool for vout input port (%d, %s)\n",
			status, mmal_status_to_string(status));

	data->vout_queue = mmal_queue_create();

	if(interlaced_frame == 1){
		data->interlaced = 1;
		// Deinterlacer
		status = mmal_component_create("vc.ril.image_fx", &data->deint);
		if(status != MMAL_SUCCESS)
			printf("Failed to create deinterlace component vc.ril.image_fx (%x, %s)\n",
				status, mmal_status_to_string(status));

		// Param 3 is set 0 (full frame rate) Param 4 is set 1 (QPU usage)
		MMAL_PARAMETER_IMAGEFX_PARAMETERS_T imfx_param = {{MMAL_PARAMETER_IMAGE_EFFECT_PARAMETERS,
			sizeof(imfx_param)}, MMAL_PARAM_IMAGEFX_DEINTERLACE_ADV, 4, {3, 0, 0, 1 }};
		status = mmal_port_parameter_set(data->deint->output[0], &imfx_param.hdr);
		if (status != MMAL_SUCCESS)
			printf("Failed to configure MMAL component vc.ril.image_fx (status=%"PRIx32" %s)",
					status, mmal_status_to_string(status));

		status = mmal_port_enable(data->deint->control, deint_control_port_cb);
		if(status != MMAL_SUCCESS)
			printf("Failed to enable deinterlace control port %s (%x, %s)\n",
				data->vout->control->name, status, mmal_status_to_string(status));

		mmal_format_copy(data->deint->input[0]->format, data->vout->input[0]->format);
		status = mmal_port_format_commit(data->deint->input[0]);
		if (status != MMAL_SUCCESS)
			printf("Failed to commit deinterlace intput format (status=%"PRIx32" %s)\n",
				status, mmal_status_to_string(status));

		status = mmal_port_parameter_set_uint32(data->deint->input[0],
			MMAL_PARAMETER_EXTRA_BUFFERS, 5);
		if (status != MMAL_SUCCESS)
			printf("Failed to set MMAL_PARAMETER_EXTRA_BUFFERS on input deinterlacer port (status=%"PRIx32" %s)\n",
					status, mmal_status_to_string(status));

		data->deint->input[0]->buffer_num = data->deint->input[0]->buffer_num_recommended;
		data->deint->input[0]->buffer_size = data->deint->input[0]->buffer_size_min;
		status = mmal_port_enable(data->deint->input[0], deint_input_port_cb);
		if(status != MMAL_SUCCESS)
			printf("Failed to enable deinterlace input port %s (%d, %s)\n",
				data->deint->input[0]->name, status, mmal_status_to_string(status));

		mmal_format_copy(data->deint->output[0]->format, data->vout->input[0]->format);
		status = mmal_port_format_commit(data->deint->output[0]);
		if (status != MMAL_SUCCESS)
			printf("Failed to commit deinterlace output format (status=%"PRIx32" %s)\n",
				status, mmal_status_to_string(status));

		status = mmal_component_enable(data->deint);
		if(status != MMAL_SUCCESS)
			printf("Failed to enable deinterlace component %s (%d, %s)\n",
				data->deint->name, status, mmal_status_to_string(status));

		data->deint->output[0]->buffer_num = data->deint->output[0]->buffer_num_recommended;
		data->deint->output[0]->buffer_size = data->deint->output[0]->buffer_size_min;
		status = mmal_port_enable(data->deint->output[0], deint_output_port_cb);
		if(status != MMAL_SUCCESS)
			printf("Failed to enable deinterlacer output port %s (%d, %s)\n",
				data->deint->output[0]->name, status, mmal_status_to_string(status));

		data->deint_input_pool = mmal_pool_create_with_allocator(data->deint->input[0]->buffer_num,
			data->deint->input[0]->buffer_size, data->deint->input[0], pool_allocator_alloc, pool_allocator_free);
		if(!data->deint_input_pool)
			printf("Failed to create pool for deinterlacer input port (%d, %s)\n",
			status, mmal_status_to_string(status));
	}

	// start vc_dispmanx_vsync_callback
//	if(vc_dispmanx_vsync_callback(data->display, vsync_callback, NULL))
//		fprintf(stderr, "Error: cannot open dispmanx vsync callback\n");

	data->width = video_ctx->width;
	data->par_num = video_ctx->sample_aspect_ratio.num;
	data->par_den = video_ctx->sample_aspect_ratio.den;
//	set_latency_target(MMAL_TRUE);
}


///
///	Display a video frame.
///
static void MmalDisplayFrame(void)
{
	MMAL_BUFFER_HEADER_T *buffer, *rbuffer;
	MMAL_STATUS_T status;
	MmalDecoder *decoder;
	decoder = MmalDecoders[0];
	AVFrame *frame;
	struct data_t *data = data_list;

	if(data->buffers_in_queue == 0 ){
		if(decoder->StartCounter > 0)
			decoder->FramesMissed++;
		return;
	}

dequeue:
	buffer = mmal_queue_get(data->vout_queue);
	// Debug segfault
	if (buffer == NULL){
		syslog(LOG_INFO, "MmalDisplayFrame: buffer are NULL!!! buffers in queue: %i buffers: %i\n",
			data->buffers_in_queue, data->buffers);
		return;
	}

	frame = (AVFrame *)buffer->user_data;
	data->buffers_in_queue--;
	data->buffers--;
	if(decoder->Closing > -5){
		if(frame)
			av_frame_free(&frame);
		mmal_buffer_header_release(buffer);
		if(data->buffers_in_queue > 0)
			goto dequeue;
		return;
	}

	int64_t audio_clock = AudioGetClock();
	int64_t video_clock = decoder->PTS = buffer->pts;
	int diff = video_clock - audio_clock - VideoAudioDelay;

	if(diff > 55 * 90 && decoder->FrameCounter % 2 == 0 && !decoder->TrickSpeed){
		decoder->FramesDuped++;
		rbuffer = mmal_queue_get(data->vout_input_pool->queue);
		memcpy(rbuffer->data, buffer->data, buffer->length);
		rbuffer->length = buffer->length;
		rbuffer->user_data = NULL;
		mmal_queue_put_back(data->vout_queue, buffer);
		data->buffers_in_queue++;
		data->buffers++;
		buffer = rbuffer;
	}
	if (diff < -25 * 90 && data->buffers_in_queue > 1 && !decoder->TrickSpeed) {
		decoder->FramesDropped++;
		if(frame)
			av_frame_free(&frame);
		mmal_buffer_header_release(buffer);
		goto dequeue;
	}

	//Debug
//	uint32_t newtime = GetMsTicks();
/*	fprintf(stderr, "vor Dec %3d buffers %2d queue %d deint_in %d deint_out %i Diff %5i Dup %i Drop %i Miss %i Closing %i TSpeed %i TCount %i\n",
		VideoGetBuffers(decoder->Stream), data->buffers,
		data->buffers_in_queue, data->buffers_deint_in, data->buffers_deint_out,
		diff, decoder->FramesDuped, decoder->FramesDropped, decoder->FramesMissed,
		decoder->Closing, decoder->TrickSpeed, decoder->TrickCounter);*/
//	data->mytime = newtime;

	if(decoder->StartCounter == 0)
		AudioVideoReady(video_clock);

	// HDMI phase
//	usleep(9000); 

	buffer->pts = 1;
	buffer->cmd = 0;
	status = mmal_port_send_buffer(data->vout->input[0], buffer);
	if(status != MMAL_SUCCESS)
		fprintf(stderr, "Failed send buffer to renderer input port (%d, %s)\n",
		    status, mmal_status_to_string(status));

//	decoder->FramesDisplayed++;
	decoder->StartCounter++;
	decoder->FrameCounter++;

}


///
///	Handle a MMAL display.
///
static void MmalDisplayHandlerThread(void)
{
    int err;
    MmalDecoder *decoder;
    struct data_t *data = data_list;

    if (!(decoder = MmalDecoders[0])) {	// no stream available
	fprintf(stderr, "MmalDisplayHandlerThread: no stream available\n");
	return;
    }

    if (data->buffers < 7) {
		err = VideoDecodeInput(decoder->Stream);
    } else {
		err = VideoPollInput(decoder->Stream);
    }
    if (err) {
		// FIXME
		usleep(10 * 1000);		// nothing buffered
		if (err == -1 && decoder->Closing) {
			decoder->Closing--;
			if (!decoder->Closing) {
				Debug(3, "video/mmal: closing eof\n");
				decoder->Closing = -1;
			}
		}
    }
}


//----------------------------------------------------------------------------
//	OSD
//----------------------------------------------------------------------------

///
///	Clear the OSD.
///
///
void VideoOsdClear(void)
{
	struct data_t *data = data_list;
	int ret;

	if(data->osd_buf != NULL){
		data->update = vc_dispmanx_update_start( 10 );
		assert( data->update );
		ret = vc_dispmanx_element_remove( data->update, data->element );
		assert( ret == 0 );
		ret = vc_dispmanx_update_submit_sync( data->update );
		assert( ret == 0 );
		ret = vc_dispmanx_resource_delete( data->resource );
		assert( ret == 0 );
		free(data->osd_buf);
		data->osd_buf = NULL;
	}
}

///
///	Draw an OSD ARGB image.
///
///	@param xi	x-coordinate in argb image
///	@param yi	y-coordinate in argb image
///	@paran height	height in pixel in argb image
///	@paran width	width in pixel in argb image
///	@param pitch	pitch of argb image
///	@param argb	32bit ARGB image data
///	@param x	x-coordinate on screen of argb image
///	@param y	y-coordinate on screen of argb image
///
void VideoOsdDrawARGB(__attribute__ ((unused)) int xi, __attribute__ ((unused)) int yi,
		int width, int height, int pitch, const uint8_t * argb, int x, int y)
{
	struct data_t *data = data_list;
	VC_DISPMANX_ALPHA_T alpha = {DISPMANX_FLAGS_ALPHA_FROM_SOURCE, 0, 0};
	VC_IMAGE_TYPE_T type = VC_IMAGE_ARGB8888;
	VC_RECT_T src_rect, dst_rect, rect;
	int ret, i;

	if (data->osd_buf == NULL) {
		int h = FFALIGN(height, 16);
		int w = FFALIGN(width, 32);
		data->osd_buf = calloc(h, w * 4);
		data->osd_width = w;
		data->osd_height = h;
		data->osd_x = x;
		data->osd_y = y;

		for (i = 0; i < height; ++i) {
			memcpy(data->osd_buf + i * data->osd_width * 4, argb + i * width * 4, width * 4);
		}

		data->resource = vc_dispmanx_resource_create( type, w, h,
			&data->vc_image_ptr );
		assert(data->resource);

		vc_dispmanx_rect_set(&dst_rect, 0, 0, w, h);

		ret = vc_dispmanx_resource_write_data(data->resource, type,
			data->osd_width * 4, data->osd_buf, &dst_rect);
		assert(ret == 0);

		data->update = vc_dispmanx_update_start(10);
		assert(data->update);

		vc_dispmanx_rect_set(&src_rect, 0, 0, w << 16, h << 16);

		vc_dispmanx_rect_set(&dst_rect, data->osd_x, data->osd_y, w, h);

		data->element = vc_dispmanx_element_add(data->update, data->display,
			2000, &dst_rect, data->resource, &src_rect, DISPMANX_PROTECTION_NONE,
			&alpha, NULL, VC_IMAGE_ROT0);

		ret = vc_dispmanx_update_submit_sync(data->update);
		assert(ret == 0);

	} else {

		for (i = 0; i < height; ++i) {
			memcpy(data->osd_buf + (y - data->osd_y) * data->osd_width * 4 + 
				data->osd_width * 4 * i + (x - data->osd_x) * 4,
					argb + i * pitch, (size_t)pitch);
		}
		vc_dispmanx_rect_set(&dst_rect, 0, 0, data->osd_width, data->osd_height);

		ret = vc_dispmanx_resource_write_data(data->resource, type,
			data->osd_width * 4, data->osd_buf, &dst_rect);
		assert(ret == 0);

		data->update = vc_dispmanx_update_start(10);
		assert(data->update);

		vc_dispmanx_rect_set(&rect, x - data->osd_x, y - data->osd_y, data->osd_width, data->osd_height);

		ret = vc_dispmanx_element_modified(data->update, data->element, &rect);
		assert(ret == 0);

		ret = vc_dispmanx_update_submit_sync(data->update);
		assert(ret == 0);
	}
}


//----------------------------------------------------------------------------
//	Thread
//----------------------------------------------------------------------------

///
///	Video render thread.
///
static void *VideoDisplayHandlerThread(void *dummy)
{
    Debug(3, "video: display thread started\n");

    for (;;) {
		pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
		pthread_testcancel();
		pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
		MmalDisplayHandlerThread();
    }

    return dummy;
}

///
///	Initialize video threads.
///
static void VideoThreadInit(void)
{
//	fprintf(stderr, "[video.c]: VideoThreadInit\n");
    pthread_mutex_init(&VideoDeintMutex, NULL);
    pthread_mutex_init(&VideoLockMutex, NULL);
    pthread_cond_init(&VideoWakeupCond, NULL);
    pthread_create(&VideoThread, NULL, VideoDisplayHandlerThread, NULL);
    pthread_setname_np(VideoThread, "softhddev video");

//	pthread_create(&presentation_thread_id, NULL, DrmDisplayFrame, NULL);
}

///
///	Exit and cleanup video threads.
///
static void VideoThreadExit(void)
{
    if (VideoThread) {
		void *retval;

		Debug(3, "video: video thread canceled\n");

		// FIXME: can't cancel locked
		if (pthread_cancel(VideoThread)) {
			Error(_("video: can't queue cancel video display thread\n"));
			fprintf(stderr, "VideoThreadExit: can't queue cancel video display thread\n");
		}
		if (pthread_join(VideoThread, &retval) || retval != PTHREAD_CANCELED) {
			Error(_("video: can't cancel video display thread\n"));
			fprintf(stderr, "VideoThreadExit: can't cancel video display thread\n");
		}
		VideoThread = 0;
		pthread_cond_destroy(&VideoWakeupCond);
		pthread_mutex_destroy(&VideoDeintMutex);
		pthread_mutex_destroy(&VideoLockMutex);
    }
}

///
///	Video display wakeup.
///
///	New video arrived, wakeup video thread.
///
void VideoDisplayWakeup(void)
{
    if (!VideoThread) {			// start video thread, if needed
//		fprintf(stderr, "VideoDisplayWakeup: (!VideoThread)\n");
		VideoThreadInit();
    }
}

//----------------------------------------------------------------------------
//	Video API
//----------------------------------------------------------------------------

///
///	Allocate new video hw decoder.
///
///	@param stream	video stream
///
///	@returns a new initialized video hardware decoder.
///
VideoHwDecoder *VideoNewHwDecoder(VideoStream * stream)
{
	MmalDecoder *decoder;

	if (MmalDecoderN == 1) {
		Fatal(_("video/MMAL: out of decoders\n"));
	}
	if (!(decoder = calloc(1, sizeof(*decoder)))) {
		Error(_("video/MMAL: out of memory\n"));
		return NULL;
	}

	decoder->Closing = -300 - 1;
	decoder->PixFmt = AV_PIX_FMT_NONE;
	decoder->PTS = AV_NOPTS_VALUE;
	decoder->Stream = stream;
	MmalDecoders[MmalDecoderN++] = decoder;

	return decoder;
}

///
///	Destroy a video hw decoder.
///
///	@param hw_decoder	video hardware decoder
///
void VideoDelHwDecoder(VideoHwDecoder * decoder)
{
	decoder = MmalDecoders[0];

	free(decoder);
	return;
}

///
///	Callback to negotiate the PixelFormat.
///
///	@param hw_decoder	video hardware decoder
///	@param video_ctx	ffmpeg video codec context
///	@param fmt		is the list of formats which are supported by
///				the codec, it is terminated by -1 as 0 is a
///				valid format, the formats are ordered by
///				quality.
///
enum AVPixelFormat Video_get_format(__attribute__ ((unused))VideoHwDecoder * decoder,
		__attribute__ ((unused)) AVCodecContext * video_ctx, const enum AVPixelFormat *fmt)
{
	while (*fmt != AV_PIX_FMT_NONE) {
		if (*fmt == AV_PIX_FMT_MMAL)
			return AV_PIX_FMT_MMAL;
		fmt++;
	}
	fprintf(stderr, "The MMAL pixel format not offered\n");
	return AV_PIX_FMT_NONE;
}


///
///	Display a ffmpeg frame
///
///	@param hw_decoder	video hardware decoder
///	@param video_ctx	ffmpeg video codec context
///	@param frame		frame to display
///
void VideoRenderFrame(VideoHwDecoder * decoder,
		const AVCodecContext * video_ctx, AVFrame * frame)
{
	MMAL_BUFFER_HEADER_T *buffer, *qbuffer;
//	MMAL_ES_FORMAT_T *format;
	MMAL_STATUS_T status;
	struct data_t *data = data_list;
//	format = (MMAL_ES_FORMAT_T *)frame->data[2];

    // fill no frame to output queue
	if(decoder->Closing == 1){
		av_frame_free(&frame);
		fprintf(stderr, "VideoRenderFrame: Closing %d\n", decoder->Closing);
		return;
	}

	// if resolution changed
	if(video_ctx->width != (int)data->width)
		MmalChangeResolution(video_ctx, frame->interlaced_frame);

	if(data->par_num != video_ctx->sample_aspect_ratio.num ||
		data->par_den != video_ctx->sample_aspect_ratio.den) {
		data->vout->input[0]->format->es->video.par.num = video_ctx->sample_aspect_ratio.num;
		data->vout->input[0]->format->es->video.par.den = video_ctx->sample_aspect_ratio.den;
		status = mmal_port_format_commit(data->vout->input[0]);
		if (status != MMAL_SUCCESS){
			fprintf(stderr, "Failed to commit format for render input port %s (status=%"PRIx32" %s)\n",
			data->vout->input[0]->name, status, mmal_status_to_string(status));
		} else {
			data->par_num = video_ctx->sample_aspect_ratio.num;
			data->par_den = video_ctx->sample_aspect_ratio.den;
		}
	}

	// can always use vout_input_pool?
	buffer = (MMAL_BUFFER_HEADER_T *)frame->data[3];

	if (buffer == NULL){
		syslog(LOG_INFO, "VideoRenderFrame: buffer are NULL!!!\n");
		return;
	}

	if (data->interlaced == 0) {
		qbuffer = mmal_queue_get(data->vout_input_pool->queue);
	} else {
		qbuffer = mmal_queue_get(data->deint_input_pool->queue);
	}

//	mmal_buffer_header_reset(buffer);
//	buffer->cmd = 0;
	memcpy(qbuffer->data, buffer->data, buffer->length);
	qbuffer->length = buffer->length;
	qbuffer->user_data = frame;

	if (qbuffer == NULL){
		syslog(LOG_INFO, "VideoRenderFrame: qbuffer are NULL!!!\n");
		return;
	}

	// fill frame to output queue
	if (data->interlaced == 0) {
		qbuffer->pts = buffer->pts;
		mmal_queue_put(data->vout_queue, qbuffer);
		data->buffers_in_queue++;
		data->buffers++;
	} else {
		// MMAL use microseconds
		qbuffer->pts = buffer->pts / 90 * 1000;
		mmal_port_send_buffer(data->deint->input[0], qbuffer);
//		data->buffers_deint_in++;
		data->buffers++;
		data->buffers++;
	}
}

///
///	Set closing stream flag.
///
///	@param hw_decoder	video hardware decoder
///
void VideoSetClosing(VideoHwDecoder * decoder, int closing)
{
	decoder->Closing = closing;
}

///
///	Reset start of frame counter.
///
///	@param hw_decoder	video hardware decoder
///
void VideoResetStart(VideoHwDecoder * decoder)
{
    Debug(3, "video: reset start\n");
    decoder->StartCounter = 0;
    decoder->FramesDuped = 0;
    decoder->FramesDropped = 0;
}

///
///	Set trick play speed.
///
///	@param hw_decoder	video hardware decoder
///	@param speed		trick speed (0 = normal)
///
void VideoSetTrickSpeed(VideoHwDecoder * decoder, int speed)
{
	Debug(3, "video: set trick-speed %d\n", speed);
//	fprintf(stderr, "video: set trick-speed %d\n", speed);
	decoder->TrickSpeed = speed;
	decoder->TrickCounter = speed;
//	if (speed) {
//		decoder->Closing = 0;
//	}
}

///
///	Grab full screen image.
///
///	@param size[out]	size of allocated image
///	@param width[in,out]	width of image
///	@param height[in,out]	height of image
///
uint8_t *VideoGrab(int *size, int *width, int *height, int write_header)
{
    Debug(3, "video: grab\n");

#ifdef USE_GRAB
    if (DrmGrabOutput) {
	uint8_t *data;
	uint8_t *rgb;
	char buf[64];
	int i;
	int n;
	int scale_width;
	int scale_height;
	int x;
	int y;
	double src_x;
	double src_y;
	double scale_x;
	double scale_y;

	scale_width = *width;
	scale_height = *height;
	n = 0;
	data = DrmGrabOutput(size, width, height);
	if (data == NULL)
	    return NULL;

	if (scale_width <= 0) {
	    scale_width = *width;
	}
	if (scale_height <= 0) {
	    scale_height = *height;
	}
	// hardware didn't scale for us, use simple software scaler
	if (scale_width != *width && scale_height != *height) {
	    if (write_header) {
		n = snprintf(buf, sizeof(buf), "P6\n%d\n%d\n255\n",
		    scale_width, scale_height);
	    }
	    rgb = malloc(scale_width * scale_height * 3 + n);
	    if (!rgb) {
		Error(_("video: out of memory\n"));
		free(data);
		return NULL;
	    }
	    *size = scale_width * scale_height * 3 + n;
	    memcpy(rgb, buf, n);	// header

	    scale_x = (double)*width / scale_width;
	    scale_y = (double)*height / scale_height;

	    src_y = 0.0;
	    for (y = 0; y < scale_height; y++) {
		int o;

		src_x = 0.0;
		o = (int)src_y **width;

		for (x = 0; x < scale_width; x++) {
		    i = 4 * (o + (int)src_x);

		    rgb[n + (x + y * scale_width) * 3 + 0] = data[i + 2];
		    rgb[n + (x + y * scale_width) * 3 + 1] = data[i + 1];
		    rgb[n + (x + y * scale_width) * 3 + 2] = data[i + 0];

		    src_x += scale_x;
		}

		src_y += scale_y;
	    }

	    *width = scale_width;
	    *height = scale_height;

	    // grabed image of correct size convert BGRA -> RGB
	} else {
	    if (write_header) {
		n = snprintf(buf, sizeof(buf), "P6\n%d\n%d\n255\n", *width,
		    *height);
	    }
	    rgb = malloc(*width * *height * 3 + n);
	    if (!rgb) {
		Error(_("video: out of memory\n"));
		free(data);
		return NULL;
	    }
	    memcpy(rgb, buf, n);	// header

	    for (i = 0; i < *size / 4; ++i) {	// convert bgra -> rgb
		rgb[n + i * 3 + 0] = data[i * 4 + 2];
		rgb[n + i * 3 + 1] = data[i * 4 + 1];
		rgb[n + i * 3 + 2] = data[i * 4 + 0];
	    }

	    *size = *width * *height * 3 + n;
	}
	free(data);

	return rgb;
    } else
#endif
    {
	Warning(_("softhddev: grab unsupported\n"));
    }

    (void)size;
    (void)width;
    (void)height;
    (void)write_header;
    return NULL;
}

///
///	Grab image service.
///
///	@param size[out]	size of allocated image
///	@param width[in,out]	width of image
///	@param height[in,out]	height of image
///
uint8_t *VideoGrabService(int *size, int *width, int *height)
{
    Debug(3, "video: grab service\n");
	Warning(_("softhddev: grab unsupported\n"));

    (void)size;
    (void)width;
    (void)height;
    return NULL;
}

///
///	Get decoder statistics.
///
///	@param hw_decoder	video hardware decoder
///	@param[out] missed	missed frames
///	@param[out] duped	duped frames
///	@param[out] dropped	dropped frames
///	@param[out] count	number of decoded frames
///
void VideoGetStats(VideoHwDecoder * decoder, int *missed, int *duped,
    int *dropped, int *counter)
{
    *missed = decoder->FramesDuped;
    *duped = decoder->FramesDuped;
    *dropped = decoder->FramesDropped;
    *counter = decoder->StartCounter;
}

///
///	Get screen size.
///
///	@param hw_decoder	video hardware decoder
///	@param[out] width	video stream width
///	@param[out] height	video stream height
///	@param[out] aspect_num	video stream aspect numerator
///	@param[out] aspect_den	video stream aspect denominator
///
void VideoGetScreenSize(__attribute__ ((unused)) VideoHwDecoder * hw_decoder, int *width, int *height,
    int *aspect_num, int *aspect_den)
{
    *width = 1920;
    *height = 1080;			// unknown default
    *aspect_num = 16;
    *aspect_den = 9;
}

//----------------------------------------------------------------------------
//	Setup
//----------------------------------------------------------------------------
///
///	Set screen size.
///
///	@param width	screen width
///
void VideoSetScreenSize(char *size)
{
	if (!strcasecmp("hdr", size))
		hdr = 1;
}

///
///	Set audio delay.
///
///	@param ms	delay in ms
///
void VideoSetAudioDelay(int ms)
{
//	fprintf(stderr, "VideoSetAudioDelay %i\n", ms);
    VideoAudioDelay = ms * 90;
}

///
///	Initialize video output module.
///
void VideoInit(void)
{
	struct data_t *data;

	// allocate mem for dev_list
	data = (struct data_t *) malloc(sizeof(struct data_t));
	memset(data, 0, sizeof(struct data_t));

	data->width = 0;
	data->par_num = 0;
	data->par_den = 0;
//	OsdConfigWidth = 1920; // default
//	OsdConfigHeight = 1080;

	bcm_host_init();
	data->display = vc_dispmanx_display_open(0);
	if (data->display == DISPMANX_NO_HANDLE)
		fprintf(stderr, "VideoInit: Error cannot open dispmanx display\n");
	// start vc_dispmanx_vsync_callback
	if(vc_dispmanx_vsync_callback(data->display, vsync_callback, NULL))
		fprintf(stderr, "VideoInit: Error cannot open dispmanx vsync callback\n");

	data_list = data;
}

///
///	Cleanup video output module.
///
void VideoExit(void)
{
	struct data_t *data = data_list;

	VideoThreadExit();

	if(vc_dispmanx_vsync_callback(data->display, NULL, NULL))
		fprintf(stderr, "Error: cannot close dispmanx vsync callback\n");

/*	vc_dispmanx_display_close(data->display);
	bcm_host_deinit();*/

//	set_latency_target(MMAL_FALSE);
}

const char *VideoGetDecoderName(const char *codec_name)
{
	if (!(strcmp("mpeg2video", codec_name)))
		return "mpeg2_mmal";

	if (!(strcmp("h264", codec_name)))
		return "h264_mmal";

	return codec_name;
}
