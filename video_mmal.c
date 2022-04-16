///
///	@file video.c	@brief Video module
///
///	Copyright (c) 2009 - 2015 by Johns.  All Rights Reserved.
///	Copyright (c) 2018 -2019 by zille.  All Rights Reserved.
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
#include "softhddev.h"

//----------------------------------------------------------------------------
//	Defines
//----------------------------------------------------------------------------


//----------------------------------------------------------------------------
//	Variables
//----------------------------------------------------------------------------
int VideoAudioDelay;

static pthread_t VideoThread;		///< video decode thread

//----------------------------------------------------------------------------
//	Common Functions
//----------------------------------------------------------------------------


//----------------------------------------------------------------------------
//	MMAL
//----------------------------------------------------------------------------

struct _Mmal_Render_
{
    int TrickSpeed;			///< current trick speed
    int TrickCounter;			///< current trick speed counter
    VideoStream *Stream;		///< video stream
    int Closing;			///< flag about closing current stream

    int StartCounter;			///< counter for video start
    int FramesDuped;			///< number of frames duplicated
    int FramesDropped;			///< number of frames dropped
    int FrameCounter;			///< number of frames decoded
	AVRational *timebase;		///< pointer to AVCodecContext pkts_timebase
	int64_t pts;

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
	unsigned buffers_deint_out;
	unsigned buffers;
};

static void MmalDisplayFrame(VideoRender * render);


//----------------------------------------------------------------------------
//	Helper functions
//----------------------------------------------------------------------------

static void buffer_worker(VideoRender * render)
{
	MMAL_STATUS_T status;
	MMAL_BUFFER_HEADER_T *buffer;

    if (render->buffers_deint_out < 3){
		buffer = mmal_queue_get(render->vout_input_pool->queue);
		mmal_buffer_header_reset(buffer);
		buffer->cmd = 0;
		render->buffers_deint_out++;
		status = mmal_port_send_buffer(render->deint->output[0], buffer);
		if(status != MMAL_SUCCESS)
			fprintf(stderr, "Failed send buffer to deinterlacer output port (%d, %s)\n",
				status, mmal_status_to_string(status));
	}
}

void vsync_callback(__attribute__ ((unused)) DISPMANX_UPDATE_HANDLE_T update,
					void *arg)
{
	VideoRender * render = (VideoRender *)arg;

	MmalDisplayFrame(render);
	if(render->interlaced == 1 && render->buffers > 0){
		buffer_worker(render);
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
	AVFrame * frame = (AVFrame *)buffer->user_data;

	if(frame)
		av_frame_free(&frame);
	mmal_buffer_header_release(buffer);
}

static void deint_output_port_cb(MMAL_PORT_T *port,
									MMAL_BUFFER_HEADER_T *buffer)
{
	VideoRender *render;
	render = (VideoRender *)port->userdata;

	if (buffer->cmd == 0 && !render->Closing) {
		if (buffer->length > 0) {
			buffer->user_data = NULL;
			// Correct PTS MMAL use microseconds
			buffer->pts = buffer->pts / av_q2d(*render->timebase);
			mmal_queue_put(render->vout_queue, buffer);
            render->buffers_deint_out--;
			render->buffers_in_queue++;
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

static void MmalClose(VideoRender * render)
{
	MMAL_STATUS_T status;

	if (render->interlaced) {
		if (render->deint->control && render->deint->control->is_enabled) {
			status = mmal_port_disable(render->deint->control);
			if (status != MMAL_SUCCESS) {
				fprintf(stderr, "Failed to disable control port deint (status=%"PRIx32" %s)\n",
					status, mmal_status_to_string(status));
			}
		}
		if (render->deint->input[0] && render->deint->input[0]->is_enabled) {
			status = mmal_port_disable(render->deint->input[0]);
			if (status != MMAL_SUCCESS) {
				fprintf(stderr, "Failed to disable input port deint (status=%"PRIx32" %s)\n",
					status, mmal_status_to_string(status));
			}
		}
		if (render->deint->output[0] && render->deint->output[0]->is_enabled) {
			status = mmal_port_disable(render->deint->output[0]);
			if (status != MMAL_SUCCESS) {
				fprintf(stderr, "Failed to disable output port deint (status=%"PRIx32" %s)\n",
					status, mmal_status_to_string(status));
			}
		}

		if (render->deint && render->deint->is_enabled) {
			status = mmal_component_disable(render->deint);
			if (status != MMAL_SUCCESS) {
				fprintf(stderr, "Failed to disable component deint (status=%"PRIx32" %s)\n",
					status, mmal_status_to_string(status));
			}
		}
		if (render->deint) {
			status = mmal_component_destroy(render->deint);
			if (status != MMAL_SUCCESS) {
				fprintf(stderr, "Failed to destroy component deint (status=%"PRIx32" %s)\n",
					status, mmal_status_to_string(status));
			}
		}

		if (render->deint_input_pool) {
			mmal_pool_destroy(render->deint_input_pool);
		}
	}

	if (render->vout->control && render->vout->control->is_enabled) {
		status = mmal_port_disable(render->vout->control);
		if (status != MMAL_SUCCESS) {
			fprintf(stderr, "Failed to disable control port (status=%"PRIx32" %s)\n",
				status, mmal_status_to_string(status));
		}
	}
	if (render->vout->input[0] && render->vout->input[0]->is_enabled) {
		status = mmal_port_disable(render->vout->input[0]);
		if (status != MMAL_SUCCESS) {
			fprintf(stderr, "Failed to disable input port (status=%"PRIx32" %s)\n",
				status, mmal_status_to_string(status));
		}
	}

	if (render->vout && render->vout->is_enabled) {
		status = mmal_component_disable(render->vout);
		if (status != MMAL_SUCCESS) {
			fprintf(stderr, "Failed to disable Componente vout (status=%"PRIx32" %s)\n",
				status, mmal_status_to_string(status));
		}
	}
	if (render->vout) {
		status = mmal_component_destroy(render->vout);
		if (status != MMAL_SUCCESS) {
			fprintf(stderr, "Failed to destroy Componente vout (status=%"PRIx32" %s)\n",
				status, mmal_status_to_string(status));
		}
	}

	if (render->vout_input_pool) {
		mmal_pool_destroy(render->vout_input_pool);
	}
	if (render->vout_queue) {
		mmal_queue_destroy(render->vout_queue);
	}

	render->interlaced = 0;
	render->buffers_in_queue = 0;
	render->buffers_deint_out = 0;
	render->buffers = 0;
	render->width = 0;
}

static void MmalChangeResolution(VideoRender * render,
		const AVCodecContext * video_ctx, int interlaced_frame)
{
	MMAL_STATUS_T status;

	if(render->width != 0){
		MmalClose(render);
	}

	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &render->vout);
	if (status != MMAL_SUCCESS)
		fprintf(stderr, "Failed to create MMAL render component %s (status=%"PRIx32" %s)\n",
		    MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, status, mmal_status_to_string(status));

	status = mmal_port_enable(render->vout->control, vout_control_port_cb);
	if (status != MMAL_SUCCESS)
		fprintf(stderr, "Failed to enable render control port %s (%x, %s)\n",
		    render->vout->control->name, status, mmal_status_to_string(status));

	render->vout->input[0]->format->type = MMAL_ES_TYPE_VIDEO;
	render->vout->input[0]->format->encoding = MMAL_ENCODING_OPAQUE;
	render->vout->input[0]->format->es->video.width = video_ctx->width;
	render->vout->input[0]->format->es->video.height = video_ctx->height;
	render->vout->input[0]->format->es->video.crop.x = 0;
	render->vout->input[0]->format->es->video.crop.y = 0;
	render->vout->input[0]->format->es->video.crop.width = video_ctx->width;
	render->vout->input[0]->format->es->video.crop.height = video_ctx->height;
	render->vout->input[0]->format->es->video.frame_rate.num = video_ctx->framerate.num;
	render->vout->input[0]->format->es->video.frame_rate.den = video_ctx->framerate.den;
	render->vout->input[0]->format->es->video.par.num = video_ctx->sample_aspect_ratio.num;
	render->vout->input[0]->format->es->video.par.den = video_ctx->sample_aspect_ratio.den;
	//render->vout->input[0]->format->es->video.color_space
	//render->vout->input[0]->format->bitrate
	render->vout->input[0]->format->flags = MMAL_ES_FORMAT_FLAG_FRAMED;
	status = mmal_port_format_commit(render->vout->input[0]);
	if (status != MMAL_SUCCESS)
		fprintf(stderr, "Failed to commit format for render input port %s (status=%"PRIx32" %s)\n",
		   render->vout->input[0]->name, status, mmal_status_to_string(status));

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
    status = mmal_port_parameter_set(render->vout->input[0], &param.hdr);
    if(status != MMAL_SUCCESS && status != MMAL_ENOSYS)
        fprintf(stderr, "cannot set vout Component (%d): %m\n", status);
*/
	render->vout->input[0]->buffer_size = render->vout->input[0]->buffer_size_recommended;
	render->vout->input[0]->buffer_num = 50;
	status = mmal_port_enable(render->vout->input[0], vout_input_port_cb);
	if (status != MMAL_SUCCESS)
		fprintf(stderr, "Failed to enable renderer input port %s (status=%"PRIx32" %s)\n",
		    render->vout->input[0]->name, status, mmal_status_to_string(status));

	status = mmal_component_enable(render->vout);
	if (status != MMAL_SUCCESS)
		fprintf(stderr, "Failed to enable renderer component %s (status=%"PRIx32" %s)\n",
			render->vout->name, status, mmal_status_to_string(status));

	render->vout_input_pool = mmal_pool_create_with_allocator(render->vout->input[0]->buffer_num,
		render->vout->input[0]->buffer_size, render->vout->input[0], pool_allocator_alloc, pool_allocator_free);
	if(!render->vout_input_pool)
		printf("Failed to create pool for vout input port (%d, %s)\n",
			status, mmal_status_to_string(status));

	render->vout_queue = mmal_queue_create();

	if(interlaced_frame == 1){
		render->interlaced = 1;
		// Deinterlacer
		status = mmal_component_create("vc.ril.image_fx", &render->deint);
		if(status != MMAL_SUCCESS)
			printf("Failed to create deinterlace component vc.ril.image_fx (%x, %s)\n",
				status, mmal_status_to_string(status));

		// Param 3 is set 0 (full frame rate) Param 4 is set 1 (QPU usage)
		MMAL_PARAMETER_IMAGEFX_PARAMETERS_T imfx_param = {{MMAL_PARAMETER_IMAGE_EFFECT_PARAMETERS,
			sizeof(imfx_param)}, MMAL_PARAM_IMAGEFX_DEINTERLACE_ADV, 4, {3, 0, 0, 1 }};
		status = mmal_port_parameter_set(render->deint->output[0], &imfx_param.hdr);
		if (status != MMAL_SUCCESS)
			printf("Failed to configure MMAL component vc.ril.image_fx (status=%"PRIx32" %s)",
					status, mmal_status_to_string(status));

		status = mmal_port_enable(render->deint->control, deint_control_port_cb);
		if(status != MMAL_SUCCESS)
			printf("Failed to enable deinterlace control port %s (%x, %s)\n",
				render->vout->control->name, status, mmal_status_to_string(status));

		mmal_format_copy(render->deint->input[0]->format, render->vout->input[0]->format);
		status = mmal_port_format_commit(render->deint->input[0]);
		if (status != MMAL_SUCCESS)
			printf("Failed to commit deinterlace intput format (status=%"PRIx32" %s)\n",
				status, mmal_status_to_string(status));

		status = mmal_port_parameter_set_uint32(render->deint->input[0],
			MMAL_PARAMETER_EXTRA_BUFFERS, 5);
		if (status != MMAL_SUCCESS)
			printf("Failed to set MMAL_PARAMETER_EXTRA_BUFFERS on input deinterlacer port (status=%"PRIx32" %s)\n",
					status, mmal_status_to_string(status));

		render->deint->input[0]->buffer_num = render->deint->input[0]->buffer_num_recommended;
		render->deint->input[0]->buffer_size = render->deint->input[0]->buffer_size_min;
		status = mmal_port_enable(render->deint->input[0], deint_input_port_cb);
		if(status != MMAL_SUCCESS)
			printf("Failed to enable deinterlace input port %s (%d, %s)\n",
				render->deint->input[0]->name, status, mmal_status_to_string(status));

		mmal_format_copy(render->deint->output[0]->format, render->vout->input[0]->format);
		status = mmal_port_format_commit(render->deint->output[0]);
		if (status != MMAL_SUCCESS)
			printf("Failed to commit deinterlace output format (status=%"PRIx32" %s)\n",
				status, mmal_status_to_string(status));

		status = mmal_component_enable(render->deint);
		if(status != MMAL_SUCCESS)
			printf("Failed to enable deinterlace component %s (%d, %s)\n",
				render->deint->name, status, mmal_status_to_string(status));

		render->deint->output[0]->buffer_num = render->deint->output[0]->buffer_num_recommended;
		render->deint->output[0]->buffer_size = render->deint->output[0]->buffer_size_min;
		render->deint->output[0]->userdata = (struct MMAL_PORT_USERDATA_T *)render;
		status = mmal_port_enable(render->deint->output[0], deint_output_port_cb);
		if(status != MMAL_SUCCESS)
			printf("Failed to enable deinterlacer output port %s (%d, %s)\n",
				render->deint->output[0]->name, status, mmal_status_to_string(status));

		render->deint_input_pool = mmal_pool_create_with_allocator(render->deint->input[0]->buffer_num,
			render->deint->input[0]->buffer_size, render->deint->input[0], pool_allocator_alloc, pool_allocator_free);
		if(!render->deint_input_pool)
			printf("Failed to create pool for deinterlacer input port (%d, %s)\n",
			status, mmal_status_to_string(status));
	}

	render->width = video_ctx->width;
	render->par_num = video_ctx->sample_aspect_ratio.num;
	render->par_den = video_ctx->sample_aspect_ratio.den;
}

///
///	Display a video frame.
///
static void MmalDisplayFrame(VideoRender * render)
{
	MMAL_BUFFER_HEADER_T *buffer, *rbuffer;
	MMAL_STATUS_T status;
	AVFrame *frame;

	if(render->buffers_in_queue == 0 ){
		return;
	}

dequeue:
	buffer = mmal_queue_get(render->vout_queue);
	// Debug segfault
	if (buffer == NULL){
		syslog(LOG_INFO, "MmalDisplayFrame: buffer are NULL!!! buffers in queue: %i buffers: %i\n",
			render->buffers_in_queue, render->buffers);
		return;
	}

	frame = (AVFrame *)buffer->user_data;
	render->buffers_in_queue--;
	render->buffers--;
	if(render->Closing){
		if(frame)
			av_frame_free(&frame);
		mmal_buffer_header_release(buffer);
		if(render->buffers_in_queue > 0) {
			goto dequeue;
		} else {
			MmalClose(render);
		}
		render->Closing = 0;
		return;
	}

	int64_t audio_clock = AudioGetClock();
	int64_t video_clock = buffer->pts * 1000 * av_q2d(*render->timebase);
	render->pts = buffer->pts;
	int diff = video_clock - audio_clock - VideoAudioDelay;

	if (render->TrickCounter) {
		render->TrickCounter--;
		goto dupe;
	}
	if (render->TrickSpeed && !render->TrickCounter) {
		render->TrickCounter = render->TrickSpeed;
	}


	if(diff > 35 && render->FrameCounter % 2 == 0 && !render->TrickSpeed){
		render->FramesDuped++;
#ifdef AV_SYNC_DEBUG
		fprintf(stderr, "FrameDuped Pkts %d deint %d Frames %d AudioUsedBytes %d audio %s video %s Delay %dms diff %dms\n",
			VideoGetPackets(render->Stream), render->buffers_deint_out,
			render->buffers_in_queue, AudioUsedBytes(), Timestamp2String(audio_clock),
			Timestamp2String(video_clock), VideoAudioDelay, diff);
#endif
dupe:
		rbuffer = mmal_queue_get(render->vout_input_pool->queue);
		memcpy(rbuffer->data, buffer->data, buffer->length);
		rbuffer->length = buffer->length;
		rbuffer->user_data = NULL;
		mmal_queue_put_back(render->vout_queue, buffer);
		render->buffers_in_queue++;
		render->buffers++;
		buffer = rbuffer;
	}
	if (diff < -5 && render->buffers_in_queue > 1 && !render->TrickSpeed) {
		render->FramesDropped++;
#ifdef AV_SYNC_DEBUG
		fprintf(stderr, "FrameDropped Pkts %d deint %d Frames %d AudioUsedBytes %d audio %s video %s Delay %dms diff %dms\n",
			VideoGetPackets(render->Stream), render->buffers_deint_out,
			render->buffers_in_queue, AudioUsedBytes(), Timestamp2String(audio_clock),
			Timestamp2String(video_clock), VideoAudioDelay, diff);
#endif
		if(frame)
			av_frame_free(&frame);
		mmal_buffer_header_release(buffer);
		goto dequeue;
	}

	//Debug
//	uint32_t newtime = GetMsTicks();
/*	fprintf(stderr, "vor Dec %3d buffers %2d queue %d deint_in %d deint_out %i Diff %5i Dup %i Drop %i Closing %i TSpeed %i TCount %i\n",
		VideoGetPackets(render->Stream), render->buffers,
		render->buffers_in_queue, render->buffers_deint_in, render->buffers_deint_out,
		diff, render->FramesDuped, render->FramesDropped,
		render->Closing, render->TrickSpeed, render->TrickCounter);*/
//	render->mytime = newtime;

	if(render->StartCounter == 0)
		AudioVideoReady(video_clock);

	// HDMI phase
//	usleep(9000); 

	buffer->pts = 1;
	buffer->cmd = 0;
	status = mmal_port_send_buffer(render->vout->input[0], buffer);
	if(status != MMAL_SUCCESS)
		fprintf(stderr, "Failed send buffer to renderer input port (%d, %s)\n",
			status, mmal_status_to_string(status));

	render->StartCounter++;
	render->FrameCounter++;
}

//----------------------------------------------------------------------------
//	OSD
//----------------------------------------------------------------------------

///
///	Clear the OSD.
///
///
void VideoOsdClear(VideoRender * render)
{
	int ret;

	if(render->osd_buf != NULL){
		render->update = vc_dispmanx_update_start( 10 );
		assert( render->update );
		ret = vc_dispmanx_element_remove( render->update, render->element );
		assert( ret == 0 );
		ret = vc_dispmanx_update_submit_sync( render->update );
		assert( ret == 0 );
		ret = vc_dispmanx_resource_delete( render->resource );
		assert( ret == 0 );
		free(render->osd_buf);
		render->osd_buf = NULL;
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
void VideoOsdDrawARGB(VideoRender * render, __attribute__ ((unused)) int xi,
		__attribute__ ((unused)) int yi, int width, int height, int pitch,
		const uint8_t * argb, int x, int y)
{
	VC_DISPMANX_ALPHA_T alpha = {DISPMANX_FLAGS_ALPHA_FROM_SOURCE, 0, 0};
	VC_IMAGE_TYPE_T type = VC_IMAGE_ARGB8888;
	VC_RECT_T src_rect, dst_rect, rect;
	int ret, i;
	int h = FFALIGN(height, 16);
	int w = FFALIGN(width, 32);

	if (render->osd_buf == NULL) {
		render->osd_buf = calloc(h, w * 4);
		render->osd_width = w;
		render->osd_height = h;
		render->osd_x = x;
		render->osd_y = y;

		for (i = 0; i < height; ++i) {
			memcpy(render->osd_buf + i * w * 4, argb + i * width * 4, width * 4);
		}

		render->resource = vc_dispmanx_resource_create( type, w, h,
			&render->vc_image_ptr );
		assert(render->resource);

		vc_dispmanx_rect_set(&dst_rect, 0, 0, w, h);

		ret = vc_dispmanx_resource_write_data(render->resource, type,
			w * 4, render->osd_buf, &dst_rect);
		assert(ret == 0);

		render->update = vc_dispmanx_update_start(10);
		assert(render->update);

		vc_dispmanx_rect_set(&src_rect, 0, 0, w << 16, h << 16);

		vc_dispmanx_rect_set(&dst_rect, x, y, w, h);

		render->element = vc_dispmanx_element_add(render->update, render->display,
			2000, &dst_rect, render->resource, &src_rect, DISPMANX_PROTECTION_NONE,
			&alpha, NULL, VC_IMAGE_ROT0);

		ret = vc_dispmanx_update_submit_sync(render->update);
		assert(ret == 0);

	} else {

		for (i = 0; i < height; ++i) {
			memcpy(render->osd_buf + (y - render->osd_y) * render->osd_width * 4 + 
				render->osd_width * 4 * i + (x - render->osd_x) * 4,
					argb + i * pitch, (size_t)pitch);
		}
		vc_dispmanx_rect_set(&dst_rect, 0, 0, render->osd_width, render->osd_height);

		ret = vc_dispmanx_resource_write_data(render->resource, type,
			render->osd_width * 4, render->osd_buf, &dst_rect);
		assert(ret == 0);

		render->update = vc_dispmanx_update_start(10);
		assert(render->update);

		vc_dispmanx_rect_set(&rect, x - render->osd_x, y - render->osd_y, render->osd_width, render->osd_height);

		ret = vc_dispmanx_element_modified(render->update, render->element, &rect);
		assert(ret == 0);

		ret = vc_dispmanx_update_submit_sync(render->update);
		assert(ret == 0);
	}
}

//----------------------------------------------------------------------------
//	Thread
//----------------------------------------------------------------------------

///
///	Video render thread.
///
static void *DisplayHandlerThread(void *arg)
{
	VideoRender * render = (VideoRender *)arg;

	Debug(3, "video: display thread started\n");

	pthread_setcancelstate (PTHREAD_CANCEL_ENABLE, NULL);
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);

	for (;;) {
		pthread_testcancel();

		if (render->buffers < 7) {
			if (VideoDecodeInput(render->Stream))
				usleep(10000);

		} else {
			usleep(10000);
		}
	}
	pthread_exit((void *)pthread_self());
}

///
///	Exit and cleanup video threads.
///
void VideoThreadExit(void)
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
	}
}

///
///	Video display wakeup.
///
///	New video arrived, wakeup video thread.
///
void VideoThreadWakeup(VideoRender * render)
{
	render->Closing = 0;

	if (!VideoThread) {
		pthread_create(&VideoThread, NULL, DisplayHandlerThread, render);
		pthread_setname_np(VideoThread, "softhddev video");
	}
}

//----------------------------------------------------------------------------
//	Video API
//----------------------------------------------------------------------------

///
///	Allocate new video hw render.
///
///	@param stream	video stream
///
///	@returns a new initialized video hardware render.
///
VideoRender *VideoNewRender(VideoStream * stream)
{
	VideoRender *render;

	if (!(render = calloc(1, sizeof(*render)))) {
		Error(_("video/MMAL: out of memory\n"));
		return NULL;
	}

	render->Closing = 0;
	render->Stream = stream;

	return render;
}

///
///	Destroy a video hw render.
///
///	@param hw_render	video hardware render
///
void VideoDelRender(VideoRender * render)
{
	free(render);
	return;
}

///
///	Callback to negotiate the PixelFormat.
///
///	@param hw_render	video hardware render
///	@param video_ctx	ffmpeg video codec context
///	@param fmt		is the list of formats which are supported by
///				the codec, it is terminated by -1 as 0 is a
///				valid format, the formats are ordered by
///				quality.
///
enum AVPixelFormat Video_get_format(__attribute__ ((unused))VideoRender * render,
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
///	@param hw_render	video hardware render
///	@param video_ctx	ffmpeg video codec context
///	@param frame		frame to display
///
void VideoRenderFrame(VideoRender * render,
		AVCodecContext * video_ctx, AVFrame * frame)
{
	MMAL_BUFFER_HEADER_T *buffer, *qbuffer;
	MMAL_STATUS_T status;

	if (!render->StartCounter) {
		render->timebase = &video_ctx->pkt_timebase;
	}

	if(render->Closing){
		av_frame_free(&frame);
		fprintf(stderr, "VideoRenderFrame: Closing %d\n", render->Closing);
		return;
	}

	if(video_ctx->width != (int)render->width)
		MmalChangeResolution(render, video_ctx, frame->interlaced_frame);

	if(render->par_num != video_ctx->sample_aspect_ratio.num ||
		render->par_den != video_ctx->sample_aspect_ratio.den) {
		render->vout->input[0]->format->es->video.par.num = video_ctx->sample_aspect_ratio.num;
		render->vout->input[0]->format->es->video.par.den = video_ctx->sample_aspect_ratio.den;
		status = mmal_port_format_commit(render->vout->input[0]);
		if (status != MMAL_SUCCESS){
			fprintf(stderr, "Failed to commit format for render input port %s (status=%"PRIx32" %s)\n",
			render->vout->input[0]->name, status, mmal_status_to_string(status));
		} else {
			render->par_num = video_ctx->sample_aspect_ratio.num;
			render->par_den = video_ctx->sample_aspect_ratio.den;
		}
	}

	buffer = (MMAL_BUFFER_HEADER_T *)frame->data[3];
	if (buffer == NULL){
		syslog(LOG_INFO, "VideoRenderFrame: buffer are NULL!!!\n");
		return;
	}

	if (render->interlaced == 0) {
		qbuffer = mmal_queue_get(render->vout_input_pool->queue);
	} else {
		qbuffer = mmal_queue_get(render->deint_input_pool->queue);
	}

	memcpy(qbuffer->data, buffer->data, buffer->length);
	qbuffer->length = buffer->length;
	qbuffer->user_data = frame;

	if (qbuffer == NULL){
		syslog(LOG_INFO, "VideoRenderFrame: qbuffer are NULL!!!\n");
		return;
	}

	// fill frame to output queue
	if (render->interlaced == 0) {
		qbuffer->pts = buffer->pts;
		mmal_queue_put(render->vout_queue, qbuffer);
		render->buffers_in_queue++;
		render->buffers++;
	} else {
		// MMAL use microseconds
		qbuffer->pts = buffer->pts * av_q2d(*render->timebase);
		mmal_port_send_buffer(render->deint->input[0], qbuffer);
		render->buffers++;
		render->buffers++;
	}
}

///
///	Get video clock.
///
///	@param hw_decoder	video hardware decoder
///
///	@note this isn't monoton, decoding reorders frames, setter keeps it
///	monotonic
///
int64_t VideoGetClock(const VideoRender * render)
{
	return render->pts;
}

///
///	Set closing stream flag.
///
///	@param hw_render	video hardware render
///
void VideoSetClosing(VideoRender * render)
{
	render->Closing = 1;

	render->StartCounter = 0;
	render->FramesDuped = 0;
	render->FramesDropped = 0;
}

/**
**	Pause video.
*/
void VideoPause( __attribute__ ((unused)) VideoRender * render)
{
#ifdef DEBUG
	fprintf(stderr, "VideoPause:\n");
#endif
}

///
///	Set trick play speed.
///
///	@param hw_render	video hardware render
///	@param speed		trick speed (0 = normal)
///
void VideoSetTrickSpeed(VideoRender * render, int speed)
{
	Debug(3, "video: set trick-speed %d\n", speed);
	render->TrickSpeed = speed;
	render->TrickCounter = speed;
	if (speed) {
		render->Closing = 0;
	}
}

/**
**	Play video.
*/
void VideoPlay(VideoRender * render)
{
#ifdef DEBUG
	fprintf(stderr, "VideoPlay: Closing %d\n", render->Closing);
#endif
	if (render->TrickSpeed) {
		render->TrickSpeed = 0;
	}

	VideoThreadWakeup(render);
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
    Debug(3, "video: no grab service\n");

    (void)write_header;
    (void)size;
    (void)width;
    (void)height;
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
    Debug(3, "video: no grab service\n");
	Warning(_("softhddev: grab unsupported\n"));

    (void)size;
    (void)width;
    (void)height;
    return NULL;
}

///
///	Get render statistics.
///
///	@param hw_render	video hardware render
///	@param[out] duped	duped frames
///	@param[out] dropped	dropped frames
///	@param[out] count	number of decoded frames
///
void VideoGetStats(VideoRender * render, int *duped,
    int *dropped, int *counter)
{
    *duped = render->FramesDuped;
    *dropped = render->FramesDropped;
    *counter = render->StartCounter;
}

///
///	Get screen size.
///
///	@param[out] width	video stream width
///	@param[out] height	video stream height
///	@param[out] aspect_num	video stream aspect numerator
///	@param[out] aspect_den	video stream aspect denominator
///
void VideoGetScreenSize(__attribute__ ((unused)) VideoRender * render,
		int *width, int *height, double *pixel_aspect)
{
	*width = 1920;
	*height = 1080;			// unknown default
	*pixel_aspect = (double)16 / (double)9;
}

//----------------------------------------------------------------------------
//	Setup
//----------------------------------------------------------------------------
///
///	Set audio delay.
///
///	@param ms	delay in ms
///
void VideoSetAudioDelay(int ms)
{
//	fprintf(stderr, "VideoSetAudioDelay %i\n", ms);
    VideoAudioDelay = ms;
}

///
///	Initialize video output module.
///
void VideoInit(VideoRender * render)
{
	render->width = 0;
	render->par_num = 0;
	render->par_den = 0;

	bcm_host_init();
	render->display = vc_dispmanx_display_open(0);
	if (render->display == DISPMANX_NO_HANDLE)
		fprintf(stderr, "VideoInit: Error cannot open dispmanx display\n");

	if(vc_dispmanx_vsync_callback(render->display, vsync_callback, render))
		fprintf(stderr, "VideoInit: Error cannot open dispmanx vsync callback\n");
}

///
///	Cleanup video output module.
///
void VideoExit(VideoRender * render)
{
	VideoThreadExit();

	if(vc_dispmanx_vsync_callback(render->display, NULL, NULL))
		fprintf(stderr, "Error: cannot close dispmanx vsync callback\n");
}

const char *VideoGetDecoderName(const char *codec_name)
{
	if (!(strcmp("mpeg2video", codec_name)))
		return "mpeg2_mmal";

	if (!(strcmp("h264", codec_name)))
		return "h264_mmal";

	return codec_name;
}

int VideoCodecMode( __attribute__ ((unused)) VideoRender * render)
{
	return 1;
}
