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

#include <stdbool.h>
#include <unistd.h>

#include <libintl.h>
#define _(str) gettext(str)		///< gettext shortcut
#define _N(str) str			///< gettext_noop shortcut

#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext_drm.h>
#include <libavutil/time.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>

#include "iatomic.h"			// portable atomic_t
#include "misc.h"
#include "video.h"
#include "audio.h"
#include "softhddev.h"


//----------------------------------------------------------------------------
//	Defines
//----------------------------------------------------------------------------

#define VIDEO_SURFACES_MAX	4	///< video output surfaces for queue

//----------------------------------------------------------------------------
//	Variables
//----------------------------------------------------------------------------
signed char VideoHardwareDecoder = -1;	///< Not used!!!

int VideoAudioDelay;
int SWDeinterlacer;
int hdr;

static pthread_t DecodeThread;		///< video decode thread
static pthread_mutex_t VideoLockMutex;	///< video lock mutex
static pthread_mutex_t VideoDeintMutex;	///< video condition mutex

static pthread_t DisplayThread;

static pthread_t FilterThread;

static pthread_cond_t cond;
static pthread_mutex_t cond_mutex;

//----------------------------------------------------------------------------
//	Common Functions
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//	DRM
//----------------------------------------------------------------------------

struct drm_buf {
	uint32_t x, y, width, height, size, pitch[3], handle[3], offset[3], fb_id;
	uint8_t *plane[3];
	uint32_t pix_fmt;
	int fd_prime;
	AVFrame *frame;
};

struct _Drm_Render_
{
	AVFrame  *FramesDeintRb[VIDEO_SURFACES_MAX];
	int FramesDeintWrite;			///< write pointer
	int FramesDeintRead;			///< read pointer
	atomic_t FramesDeintFilled;		///< how many of the buffer is used

	AVFrame  *FramesRb[VIDEO_SURFACES_MAX];
	int FramesWrite;			///< write pointer
	int FramesRead;			///< read pointer
	atomic_t FramesFilled;		///< how many of the buffer is used

	VideoStream *Stream;		///< video stream
	int TrickSpeed;			///< current trick speed
	int TrickCounter;			///< current trick speed counter
	int Closing;			///< flag about closing current stream
	int Deint_Close;

	int StartCounter;			///< counter for video start
	int FramesDuped;			///< number of frames duplicated
	int FramesDropped;			///< number of frames dropped

	AVFilterGraph *filter_graph;
	AVFilterContext *buffersrc_ctx, *buffersink_ctx;

	int fd_drm;
	drmModeModeInfo mode;
	drmModeCrtc *saved_crtc;
	drmEventContext ev;
	struct drm_buf *act_buf;
	struct drm_buf bufs[36];
	struct drm_buf buf_osd;
	struct drm_buf buf_black;
	int use_zpos;
	uint64_t zpos_overlay;
	uint64_t zpos_primary;
	uint32_t connector_id, crtc_id, video_plane, osd_plane, front_buf, act_fb_id;
	int second_field;
	AVFrame *lastframe;
	int prime_buffers;
};

//----------------------------------------------------------------------------
//	Helper functions
//----------------------------------------------------------------------------

static void ThreadExitHandler(void * arg)
{
	VideoRender * render = (VideoRender *)arg;

	avfilter_graph_free(&render->filter_graph);
	FilterThread = 0;
}

static uint64_t GetPropertyValue(int fd_drm, uint32_t objectID,
						uint32_t objectType, const char *propName)
{
	uint32_t i;
	int found = 0;
	uint64_t value = 0;
	drmModePropertyPtr Prop;
	drmModeObjectPropertiesPtr objectProps =
		drmModeObjectGetProperties(fd_drm, objectID, objectType);

	for (i = 0; i < objectProps->count_props; i++) {
		if ((Prop = drmModeGetProperty(fd_drm, objectProps->props[i])) == NULL)
			fprintf(stderr, "GetPropertyValue: Unable to query property.\n");

		if (strcmp(propName, Prop->name) == 0) {
			value = objectProps->prop_values[i];
			found = 1;
		}

		drmModeFreeProperty(Prop);

		if (found)
			break;
	}

	drmModeFreeObjectProperties(objectProps);

#ifdef DRM_DEBUG
	if (!found)
		fprintf(stderr, "GetPropertyValue: Unable to find value for property \'%s\'.\n",
			propName);
#endif
	return value;
}

static int SetPropertyRequest(drmModeAtomicReqPtr ModeReq, int fd_drm,
					uint32_t objectID, uint32_t objectType,
					const char *propName, uint32_t value)
{
	uint32_t i;
	uint64_t id = 0;
	drmModePropertyPtr Prop;
	drmModeObjectPropertiesPtr objectProps =
		drmModeObjectGetProperties(fd_drm, objectID, objectType);

	for (i = 0; i < objectProps->count_props; i++) {
		if ((Prop = drmModeGetProperty(fd_drm, objectProps->props[i])) == NULL)
			fprintf(stderr, "SetPropertyRequest: Unable to query property.\n");

		if (strcmp(propName, Prop->name) == 0) {
			id = Prop->prop_id;
			drmModeFreeProperty(Prop);
			break;
		}

		drmModeFreeProperty(Prop);
	}

	drmModeFreeObjectProperties(objectProps);

	if (id == 0)
		fprintf(stderr, "SetPropertyRequest: Unable to find value for property \'%s\'.\n",
			propName);

	return drmModeAtomicAddProperty(ModeReq, objectID, id, value);
}

///
/// If primary plane support only rgb and overlay plane nv12
/// must the zpos change. At the end it must change back.
/// @param backward		if set change to origin.
///
void ChangePlanes(VideoRender * render, int back)
{
	drmModeAtomicReqPtr ModeReq;
	const uint32_t flags = DRM_MODE_ATOMIC_ALLOW_MODESET;
	uint64_t zpos_video;
	uint64_t zpos_osd;

	if (!(ModeReq = drmModeAtomicAlloc()))
		fprintf(stderr, "ChangePlanes: cannot allocate atomic request (%d): %m\n", errno);

	if (back) {
		zpos_video = render->zpos_overlay;
		zpos_osd = render->zpos_primary;
	} else {
		zpos_video = render->zpos_primary;
		zpos_osd = render->zpos_overlay;
	}
	SetPropertyRequest(ModeReq, render->fd_drm, render->video_plane,
			DRM_MODE_OBJECT_PLANE, "zpos", zpos_video);
	SetPropertyRequest(ModeReq, render->fd_drm, render->osd_plane,
			DRM_MODE_OBJECT_PLANE, "zpos", zpos_osd);

	if (drmModeAtomicCommit(render->fd_drm, ModeReq, flags, NULL) != 0)
		fprintf(stderr, "ChangePlanes: cannot change planes (%d): %m\n", errno);

	drmModeAtomicFree(ModeReq);
}

void SetCrtc(VideoRender * render, drmModeAtomicReqPtr ModeReq,
				uint32_t plane_id)
{
	SetPropertyRequest(ModeReq, render->fd_drm, plane_id,
						DRM_MODE_OBJECT_PLANE, "CRTC_X", 0);
	SetPropertyRequest(ModeReq, render->fd_drm, plane_id,
						DRM_MODE_OBJECT_PLANE, "CRTC_Y", 0);
	SetPropertyRequest(ModeReq, render->fd_drm, plane_id,
						DRM_MODE_OBJECT_PLANE, "CRTC_W", render->mode.hdisplay);
	SetPropertyRequest(ModeReq, render->fd_drm, plane_id,
						DRM_MODE_OBJECT_PLANE, "CRTC_H", render->mode.vdisplay);
}

void SetSrc(VideoRender * render, drmModeAtomicReqPtr ModeReq,
				uint32_t plane_id, struct drm_buf *buf)
{
	SetPropertyRequest(ModeReq, render->fd_drm, plane_id,
						DRM_MODE_OBJECT_PLANE, "SRC_X", 0);
	SetPropertyRequest(ModeReq, render->fd_drm, plane_id,
						DRM_MODE_OBJECT_PLANE, "SRC_Y", 0);
	SetPropertyRequest(ModeReq, render->fd_drm, plane_id,
						DRM_MODE_OBJECT_PLANE, "SRC_W", buf->width << 16);
	SetPropertyRequest(ModeReq, render->fd_drm, plane_id,
						DRM_MODE_OBJECT_PLANE, "SRC_H", buf->height << 16);
}

void SetBuf(VideoRender * render, struct drm_buf *buf, uint32_t plane_id)
{
	drmModeAtomicReqPtr ModeReq;
	const uint32_t flags = DRM_MODE_ATOMIC_ALLOW_MODESET;

//	fprintf(stderr, "Set atomic buf prime_buffers %2i fd_prime %"PRIu32" Handle %"PRIu32" fb_id %3i %i x %i\n",
//		render->prime_buffers, buf->fd_prime, buf->handle[0], buf->fb_id, buf->width, buf->height);

	if (!(ModeReq = drmModeAtomicAlloc()))
		fprintf(stderr, "SetBuf: cannot allocate atomic request (%d): %m\n", errno);

	SetSrc(render, ModeReq, plane_id, buf);
	SetPropertyRequest(ModeReq, render->fd_drm, plane_id,
						DRM_MODE_OBJECT_PLANE, "FB_ID", buf->fb_id);

	if (drmModeAtomicCommit(render->fd_drm, ModeReq, flags, NULL) != 0)
		fprintf(stderr, "SetBuf: cannot set atomic buf %i width %i height %i fb_id %i (%d): %m\n",
			render->prime_buffers, buf->width, buf->height, buf->fb_id, errno);

	drmModeAtomicFree(ModeReq);
}

static int FindDevice(VideoRender * render)
{
//	drmVersion *version;
	drmModeRes *resources;
	drmModeConnector *connector;
	drmModeEncoder *encoder = 0;
	drmModeModeInfo *mode;
	drmModePlane *plane;
	drmModePlaneRes *plane_res;
	uint32_t j, k;
	uint64_t has_dumb;
	uint64_t has_prime;
	int i;

	render->fd_drm = open("/dev/dri/card0", O_RDWR);
	if (render->fd_drm < 0) {
		fprintf(stderr, "FindDevice: cannot open /dev/dri/card0: %m\n");
		return -errno;
	}

//	version = drmGetVersion(render->fd_drm);
//	fprintf(stderr, "FindDevice: open /dev/dri/card0: %i %s\n", version->name_len, version->name);

	// check capability
	if (drmGetCap(render->fd_drm, DRM_CAP_DUMB_BUFFER, &has_dumb) < 0 || has_dumb == 0)
		fprintf(stderr, "FindDevice: drmGetCap DRM_CAP_DUMB_BUFFER failed or doesn't have dumb buffer\n");

	if (drmSetClientCap(render->fd_drm, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) != 0)
		fprintf(stderr, "FindDevice: DRM_CLIENT_CAP_UNIVERSAL_PLANES not available.\n");

	if (drmSetClientCap(render->fd_drm, DRM_CLIENT_CAP_ATOMIC, 1) != 0)
		fprintf(stderr, "FindDevice: DRM_CLIENT_CAP_ATOMIC not available.\n");

	if (drmGetCap(render->fd_drm, DRM_CAP_PRIME, &has_prime) < 0)
		fprintf(stderr, "FindDevice: DRM_CAP_PRIME not available.\n");

	if (drmGetCap(render->fd_drm, DRM_PRIME_CAP_EXPORT, &has_prime) < 0)
		fprintf(stderr, "FindDevice: DRM_PRIME_CAP_EXPORT not available.\n");

	if (drmGetCap(render->fd_drm, DRM_PRIME_CAP_IMPORT, &has_prime) < 0)
		fprintf(stderr, "FindDevice: DRM_PRIME_CAP_IMPORT not available.\n");

	if ((resources = drmModeGetResources(render->fd_drm)) == NULL){
		fprintf(stderr, "FindDevice: cannot retrieve DRM resources (%d): %m\n",	errno);
		return -errno;
	}

#ifdef DRM_DEBUG
	Info(_("[FindDevice] DRM have %i connectors, %i crtcs, %i encoders\n"),
		resources->count_connectors, resources->count_crtcs,
		resources->count_encoders);
#endif

	// find all available connectors
	for (i = 0; i < resources->count_connectors; i++) {
		connector = drmModeGetConnector(render->fd_drm, resources->connectors[i]);
		if (!connector) {
			fprintf(stderr, "FindDevice: cannot retrieve DRM connector (%d): %m\n", errno);
		return -errno;
		}

		if (connector != NULL && connector->connection == DRM_MODE_CONNECTED && connector->count_modes > 0) {
			render->connector_id = connector->connector_id;

			// FIXME: use default encoder/crtc pair
			if ((encoder = drmModeGetEncoder(render->fd_drm, connector->encoder_id)) == NULL){
				fprintf(stderr, "FindDevice: cannot retrieve encoder (%d): %m\n", errno);
				return -errno;
			}
			render->crtc_id = encoder->crtc_id;
		}
		    // search Modes for HD, HDready and SD
		for (i = 0; i < connector->count_modes; i++) {
			mode = &connector->modes[i];
			// Mode HD
			if(mode->hdisplay == 1920 && mode->vdisplay == 1080 && mode->vrefresh == 50
				&& !(mode->flags & DRM_MODE_FLAG_INTERLACE) && !hdr) {
				memcpy(&render->mode, &connector->modes[i], sizeof(drmModeModeInfo));
			}
			// Mode HDready
			if(mode->hdisplay == 1280 && mode->vdisplay == 720 && mode->vrefresh == 50
				&& !(mode->flags & DRM_MODE_FLAG_INTERLACE) && hdr) {
				memcpy(&render->mode, &connector->modes[i], sizeof(drmModeModeInfo));
			}
		}
		drmModeFreeConnector(connector);
	}

	// find first plane
	if ((plane_res = drmModeGetPlaneResources(render->fd_drm)) == NULL)
		fprintf(stderr, "FindDevice: cannot retrieve PlaneResources (%d): %m\n", errno);

	for (j = 0; j < plane_res->count_planes; j++) {
		plane = drmModeGetPlane(render->fd_drm, plane_res->planes[j]);

		if (plane == NULL)
			fprintf(stderr, "FindDevice: cannot query DRM-KMS plane %d\n", j);

		for (i = 0; i < resources->count_crtcs; i++) {
			if (plane->possible_crtcs & (1 << i))
				break;
		}

		uint64_t type = GetPropertyValue(render->fd_drm, plane_res->planes[j],
							DRM_MODE_OBJECT_PLANE, "type");
		uint64_t zpos = GetPropertyValue(render->fd_drm, plane_res->planes[j],
							DRM_MODE_OBJECT_PLANE, "zpos");

#ifdef DRM_DEBUG // If more then 2 crtcs this must rewriten!!!
		Info(_("[FindDevice] Plane id %i crtc_id %i possible_crtcs %i possible CRTC %i type %s\n"),
			plane->plane_id, plane->crtc_id, plane->possible_crtcs, resources->crtcs[i],
			(type == DRM_PLANE_TYPE_PRIMARY) ? "primary plane" :
			(type == DRM_PLANE_TYPE_OVERLAY) ? "overlay plane" :
			(type == DRM_PLANE_TYPE_CURSOR) ? "cursor plane" : "No plane type");
#endif

		// test pixel format and plane caps
		for (k = 0; k < plane->count_formats; k++) {
			if (encoder->possible_crtcs & plane->possible_crtcs) {
				switch (plane->formats[k]) {
					case DRM_FORMAT_NV12:
						if (!render->video_plane) {
							if (type != DRM_PLANE_TYPE_PRIMARY) {
								render->use_zpos = 1;
								render->zpos_overlay = zpos;
							}
							render->video_plane = plane->plane_id;
							if (plane->plane_id == render->osd_plane)
								render->osd_plane = 0;
						}
						break;
					case DRM_FORMAT_ARGB8888:
						if (!render->osd_plane) {
							if (type != DRM_PLANE_TYPE_OVERLAY)
								render->zpos_primary = zpos;
							render->osd_plane = plane->plane_id;
						}
						break;
					default:
						break;
				}
			}
		}
		drmModeFreePlane(plane);
	}

	drmModeFreePlaneResources(plane_res);
	drmModeFreeEncoder(encoder);
	drmModeFreeResources(resources);

#ifdef DRM_DEBUG
	Info(_("[FindDevice] DRM setup CRTC: %i video_plane: %i osd_plane %i\n"),
		render->crtc_id, render->video_plane, render->osd_plane);
#endif

	return 0;
}

static int SetupFB(VideoRender * render, struct drm_buf *buf,
			AVDRMFrameDescriptor *primedata)
{
	struct drm_mode_create_dumb creq;

	if (primedata) {
		uint32_t prime_handle;

		buf->pix_fmt = primedata->layers[0].format;

		if (drmPrimeFDToHandle(render->fd_drm, primedata->objects[0].fd, &prime_handle))
			fprintf(stderr, "SetupFB: Failed to retrieve the Prime Handle %i size %i (%d): %m\n",
				primedata->objects[0].fd, primedata->objects[0].size, errno);

		buf->handle[0] = buf->handle[1] = prime_handle;
		buf->pitch[0] = primedata->layers[0].planes[0].pitch;
		buf->offset[0] = primedata->layers[0].planes[0].offset;
		buf->pitch[1] = primedata->layers[0].planes[1].pitch;
		buf->offset[1] = primedata->layers[0].planes[1].offset;
	} else {
		memset(&creq, 0, sizeof(struct drm_mode_create_dumb));
		creq.width = buf->width;
		creq.height = buf->height;
		// 32 bpp for ARGB, 8 bpp for YUV420 and NV12
		if (buf->pix_fmt == DRM_FORMAT_ARGB8888)
			creq.bpp = 32;
		else
			creq.bpp = 12;

		if (drmIoctl(render->fd_drm, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0){
			fprintf(stderr, "SetupFB: cannot create dumb buffer (%d): %m\n", errno);
			return -errno;
		}

		buf->size = creq.size;
		buf->handle[2] = buf->handle[1] = buf->handle[0] = creq.handle;

		if (buf->pix_fmt == DRM_FORMAT_YUV420) {
			buf->pitch[0] = buf->width;
			buf->pitch[2] = buf->pitch[1] = buf->pitch[0] / 2;

			buf->offset[0] = 0;
			buf->offset[1] = buf->pitch[0] * buf->height;
			buf->offset[2] = buf->offset[1] + buf->pitch[1] * buf->height / 2;
		}

		if (buf->pix_fmt == DRM_FORMAT_NV12) {
			buf->pitch[1] = buf->pitch[0] = buf->width;

			buf->offset[0] = 0;
			buf->offset[1] = buf->pitch[0] * buf->height;
		}

		if (buf->pix_fmt == DRM_FORMAT_ARGB8888) {
			buf->pitch[0] = creq.pitch;

			buf->offset[0] = 0;
		}
	}

	if (drmModeAddFB2(render->fd_drm, buf->width, buf->height,
		buf->pix_fmt, buf->handle, buf->pitch, buf->offset, &buf->fb_id, 0)) {
		fprintf(stderr, "SetupFB: cannot create framebuffer (%d): %m\n", errno);
		return -errno;
	}

//	fprintf(stderr, "SetupFB prime_buffers %i Handle %"PRIu32" fb_id %i %i x %i\n",
//		render->prime_buffers, buf->handle[0], buf->fb_id, buf->width, buf->height);

	if (primedata)
		return 0;

	struct drm_mode_map_dumb mreq;
	memset(&mreq, 0, sizeof(struct drm_mode_map_dumb));
	mreq.handle = buf->handle[0];

	if (drmIoctl(render->fd_drm, DRM_IOCTL_MODE_MAP_DUMB, &mreq)){
		fprintf(stderr, "SetupFB: cannot map dumb buffer (%d): %m\n", errno);
		return -errno;
	}

	buf->plane[0] = mmap(0, creq.size, PROT_READ | PROT_WRITE, MAP_SHARED, render->fd_drm, mreq.offset);
	if (buf->plane[0] == MAP_FAILED) {
		fprintf(stderr, "SetupFB: cannot mmap dumb buffer (%d): %m\n", errno);
		return -errno;
	}
	buf->plane[1] = buf->plane[0] + buf->offset[1];
	buf->plane[2] = buf->plane[0] + buf->offset[2];

	return 0;
}

/*static void Drm_page_flip_event( __attribute__ ((unused)) int fd,
					__attribute__ ((unused)) unsigned int frame,
					__attribute__ ((unused)) unsigned int sec,
					__attribute__ ((unused)) unsigned int usec,
					__attribute__ ((unused)) void *data)
{
}*/

static void DestroyFB(int fd_drm, struct drm_buf *buf)
{
	struct drm_mode_destroy_dumb dreq;

//	fprintf(stderr, "DestroyFB: destroy FB.\n");

	if (buf->plane[0] != 0) {
		munmap(buf->plane[0], buf->size);
	}

	if (drmModeRmFB(fd_drm, buf->fb_id) < 0)
		fprintf(stderr, "DestroyFB: cannot remake FB (%d): %m\n", errno);

	if (buf->plane[0] != 0) {
		memset(&dreq, 0, sizeof(dreq));
		dreq.handle = buf->handle[0];
		if (drmIoctl(fd_drm, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq) < 0)
			fprintf(stderr, "DestroyFB: cannot destroy dumb buffer (%d): %m\n", errno);
	}
	buf->width = 0;
	buf->height = 0;
	buf->fb_id = 0;
	buf->plane[0] = 0;
	buf->size = 0;
	buf->fd_prime = 0;
}

///
/// Clean DRM
///
static void CleanDisplayThread(VideoRender * render, AVFrame *frame)
{
	int i;

	if (render->lastframe) {
		av_frame_free(&render->lastframe);
	}

dequeue:
	if (frame) {
		av_frame_free(&frame);
		pthread_mutex_lock(&VideoLockMutex);
		render->FramesRead = (render->FramesRead + 1) % VIDEO_SURFACES_MAX;
		atomic_dec(&render->FramesFilled);
		pthread_mutex_unlock(&VideoLockMutex);
	}
	if (atomic_read(&render->FramesFilled)) {
		pthread_mutex_lock(&VideoLockMutex);
		frame = render->FramesRb[render->FramesRead];
		pthread_mutex_unlock(&VideoLockMutex);
		goto dequeue;
	}

	// Destroy FBs
	if (render->prime_buffers) {
		for (i = 0; i < render->prime_buffers; ++i) {
			DestroyFB(render->fd_drm, &render->bufs[i]);
		}
		render->prime_buffers = 0;
		render->front_buf = 0;
	}

	pthread_mutex_lock(&cond_mutex);
	pthread_cond_signal(&cond);
	pthread_mutex_unlock(&cond_mutex);

	render->Closing = 0;
//	fprintf(stderr, "CleanDisplayThread: DRM cleaned.\n");
}

///
///	Draw a video frame.
///
static void Frame2Display(VideoRender * render)
{
	struct drm_buf *buf = 0;
	AVFrame *frame;
	AVDRMFrameDescriptor *primedata = NULL;
	int64_t audio_clock;
	int i, j;

/*	fprintf(stderr, "Frame2Display: Buffers Pkts %d Deint %d Video %d\n",
		VideoGetPackets(render->Stream),
		atomic_read(&render->FramesDeintFilled),
		atomic_read(&render->FramesFilled));*/

	if (render->Closing) {
closing:
		// set a black FB
		SetBuf(render, &render->buf_black, render->video_plane);
		buf = &render->buf_black;
		goto page_flip;
	}

dequeue:
	while ((atomic_read(&render->FramesFilled)) == 0 ) {
		fprintf(stderr, "Frame2Display: Kein Frame in der Queue!!!\n");
		usleep(20000);
	}

	if (atomic_read(&render->FramesFilled)) {
		pthread_mutex_lock(&VideoLockMutex);
		frame = render->FramesRb[render->FramesRead];
		pthread_mutex_unlock(&VideoLockMutex);

		if (atomic_read(&render->FramesDeintFilled) < VIDEO_SURFACES_MAX &&
			atomic_read(&render->FramesFilled) < VIDEO_SURFACES_MAX) {

			pthread_mutex_lock(&cond_mutex);
			pthread_cond_signal(&cond);
			pthread_mutex_unlock(&cond_mutex);
		}
	} else {
		frame = NULL;
		fprintf(stderr, "Frame2Display: frame = NULL Kein Frame in der Queue!!!\n");
	}

	if (frame->format == AV_PIX_FMT_DRM_PRIME) {
		// frame in prime fd
		primedata = (AVDRMFrameDescriptor *)frame->data[0];
		// search or made fd / FB combination
		for (i = 0; i < render->prime_buffers; i++) {
			if (render->bufs[i].fd_prime == primedata->objects[0].fd) {
				buf = &render->bufs[i];
				break;
			}
		}
		if (buf == 0) {
			buf = &render->bufs[render->prime_buffers];
			buf->width = (uint32_t)frame->width;
			buf->height = (uint32_t)frame->height;
			buf->fd_prime = primedata->objects[0].fd;

			if (SetupFB(render, buf, primedata))
				fprintf(stderr, "Frame2Display: SetupFB FB %i x %i failed\n", buf->width, buf->height);
			if (render->prime_buffers == 0) {
				SetBuf(render, buf, render->video_plane);
				render->act_fb_id = buf->fb_id;
			}
			render->prime_buffers++;
		}
	} else {
		// frame in frame data
		buf = &render->bufs[render->front_buf];
		if (buf->fb_id == 0) {
			buf->width = (uint32_t)frame->width;
			if (frame->interlaced_frame == 1)
				buf->height = (uint32_t)frame->height / 2;
			else buf->height = (uint32_t)frame->height;

			if (SetupFB(render, buf, NULL))
				fprintf(stderr, "Frame2Display: SetupFB FB %i x %i failed\n", buf->width, buf->height);
			if (render->prime_buffers == 0) {
				SetBuf(render, buf, render->video_plane);
				render->act_fb_id = buf->fb_id;
			}
			render->prime_buffers++;
		}

		// Copy YUV420 to NV12 and deinterlace at once
		for (i = 0; i < frame->height; ++i)
			if (((i + frame->top_field_first) % 2 == 0 && render->second_field == 1) ||
				((i + frame->top_field_first + 1) % 2 == 0 && render->second_field == 0) ||
				frame->interlaced_frame == 0)
					memcpy(buf->plane[0] + i / (frame->interlaced_frame + 1) * frame->width,
						frame->data[0] + i * frame->linesize[0], frame->width);

		for (i = 0; i < frame->height / 2; ++i) {
			if (((i + frame->top_field_first) % 2 == 0 && render->second_field == 1) ||
				((i + frame->top_field_first + 1) % 2 == 0 && render->second_field == 0) ||
				frame->interlaced_frame == 0)
				for (j = 0; j < frame->width; ++j) {
					if (j % 2 == 0)
						memcpy(buf->plane[1] + i / (frame->interlaced_frame + 1) * frame->width + j,
							frame->data[1] + i * frame->linesize[2] + j / 2, 1 );
					else memcpy(buf->plane[1] + i / (frame->interlaced_frame + 1) * frame->width + j,
							frame->data[2] + i * frame->linesize[1] + (j +1) / 2, 1 );
				}
		}

		if (frame->interlaced_frame == 1) {
			if (render->second_field == 0) {
				render->second_field = 1;
			} else {
				render->second_field = 0;
				frame->pts += 1800;
			}
		}
	}

	if(!render->StartCounter && !render->Closing) {
//		fprintf(stderr, "Frame2Display: AudioVideoReady\n");
		AudioVideoReady(frame->pts);
	}

audioclock:
	audio_clock = AudioGetClock();
	if (audio_clock == (int64_t) AV_NOPTS_VALUE && !render->TrickSpeed) {

		if (render->Closing)
			goto closing;

		usleep(20000);
		goto audioclock;
	}
	int diff = frame->pts - audio_clock - VideoAudioDelay;

	if(diff > 55 * 90 && !render->TrickSpeed) {
		render->FramesDuped++;

//		fprintf(stderr, "Frame2Display: FramesDuped Timstamp %s audio %s video %s diff %d %d\n",
//			Timestamp2String(av_gettime()), Timestamp2String(audio_clock),
//			Timestamp2String(frame->pts), diff, diff /90);

		if (render->Closing)
			goto closing;

		usleep(20000);
		goto audioclock;
	}

	if (diff < -25 * 90 && !render->TrickSpeed) {
		render->FramesDropped++;

//		fprintf(stderr, "Frame2Display: FramesDropped Timstamp %s audio %s video %s diff %d %d\n",
//			Timestamp2String(av_gettime()), Timestamp2String(audio_clock),
//			Timestamp2String(frame->pts), diff, diff /90);

		if (render->Closing)
			goto closing;

		av_frame_free(&frame);
		pthread_mutex_lock(&VideoLockMutex);
		render->FramesRead = (render->FramesRead + 1) % VIDEO_SURFACES_MAX;
		atomic_dec(&render->FramesFilled);
		pthread_mutex_unlock(&VideoLockMutex);
		if (render->Closing == 0) {
			render->StartCounter++;
		}
		goto dequeue;
	}

	if (frame->interlaced_frame == 0 || render->second_field == 0) {
		buf->frame = frame;
		pthread_mutex_lock(&VideoLockMutex);
		render->FramesRead = (render->FramesRead + 1) % VIDEO_SURFACES_MAX;
		atomic_dec(&render->FramesFilled);
		pthread_mutex_unlock(&VideoLockMutex);
	} else
		buf->frame = NULL;

	render->StartCounter++;

page_flip:
	render->act_fb_id = buf->fb_id;
	render->act_buf = buf;

	drmModeAtomicReqPtr ModeReq;
	const uint32_t flags = DRM_MODE_PAGE_FLIP_EVENT;
	if (!(ModeReq = drmModeAtomicAlloc()))
		fprintf(stderr, "Frame2Display: cannot allocate atomic request (%d): %m\n", errno);

	SetPropertyRequest(ModeReq, render->fd_drm, render->video_plane,
					DRM_MODE_OBJECT_PLANE, "FB_ID", buf->fb_id);
	if (drmModeAtomicCommit(render->fd_drm, ModeReq, flags, NULL) != 0)
		fprintf(stderr, "Frame2Display: cannot page flip to FB %i (%d): %m\n",
			buf->fb_id, errno);

	drmModeAtomicFree(ModeReq);
	render->front_buf ^= 1;
}

///
///	Display a video frame.
///
static void *DisplayHandlerThread(void * arg)
{
	VideoRender * render = (VideoRender *)arg;

	pthread_setcancelstate (PTHREAD_CANCEL_ENABLE, NULL);
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);

	while ((atomic_read(&render->FramesFilled)) < 2 ){
		usleep(20000);
	}

	while (1) {
		pthread_testcancel();

		if (atomic_read(&render->FramesFilled)) {
			Frame2Display(render);

			if (drmHandleEvent(render->fd_drm, &render->ev) != 0)
				fprintf(stderr, "DisplayHandlerThread: drmHandleEvent failed!\n");

			if (render->lastframe) {
				av_frame_free(&render->lastframe);
			}
			render->lastframe = render->act_buf->frame;

		} else {
			usleep(20000);
		}

		if (render->buf_black.fb_id == render->act_fb_id &&
			render->Closing) {

			CleanDisplayThread(render, NULL);
		}
	}
	pthread_exit((void *)pthread_self());
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
	if (render->use_zpos) {
		ChangePlanes(render, 1);
		memset((void *)render->buf_osd.plane[0], 0,
			(size_t)(render->buf_osd.pitch[0] * render->buf_osd.height));
	} else {
		if (drmModeSetPlane(render->fd_drm, render->osd_plane, render->crtc_id, 0, 0,
			0, 0, render->buf_osd.width, render->buf_osd.height, 0, 0, 0 << 16, 0 << 16))
				fprintf(stderr, "VideoOsdClear: failed to clear plane: (%d): %m\n", (errno));
		render->buf_osd.x = 0;
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
	int i;

	if (render->use_zpos) {
		ChangePlanes(render, 0);
	} else {
		if (render->buf_osd.x == 0){
			if (drmModeSetPlane(render->fd_drm, render->osd_plane, render->crtc_id, render->buf_osd.fb_id,
				0, x, y, width, height, 0, 0, width << 16, height << 16))
					fprintf(stderr, "VideoOsdDrawARGB: failed to enable plane: (%d): %m\n", (errno));
			render->buf_osd.x = x;
			render->buf_osd.y = y;
		}
	}

	for (i = 0; i < height; ++i) {
		memcpy(render->buf_osd.plane[0] + (x - render->buf_osd.x) * 4 + (i + y - render->buf_osd.y)
		   * render->buf_osd.pitch[0], argb + i * pitch, (size_t)pitch);
	}
//	fprintf(stderr, "DrmOsdDrawARGB width: %i height: %i pitch: %i x: %i y: %i xi: %i yi: %i diff_y: %i diff_x: %i\n",
//	   width, height, pitch, x, y, xi, yi, y - render->buf_osd.y, x - render->buf_osd.x);
}

//----------------------------------------------------------------------------
//	Thread
//----------------------------------------------------------------------------

///
///	Video render thread.
///
static void *DecodeHandlerThread(void *arg)
{
	VideoRender * render = (VideoRender *)arg;
	int ret;

	Debug(3, "video: display thread started\n");

	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);

	for (;;) {
		pthread_testcancel();

		// manage fill frame output ring buffer
		if (atomic_read(&render->FramesDeintFilled) < VIDEO_SURFACES_MAX &&
			atomic_read(&render->FramesFilled) < VIDEO_SURFACES_MAX) {

			ret = VideoDecodeInput(render->Stream);
		} else {
			pthread_mutex_lock(&cond_mutex);
			pthread_cond_wait(&cond, &cond_mutex);
			pthread_mutex_unlock(&cond_mutex);

			ret = VideoDecodeInput(render->Stream);
		}
		if (ret) {
			usleep(20000);
		}
	}
	pthread_exit((void *)pthread_self());
}

///
///	Exit and cleanup video threads.
///
void VideoThreadExit(void)
{
	void *retval;

	if (DecodeThread) {
		Debug(3, "video: video thread canceled\n");
		// FIXME: can't cancel locked
		if (pthread_cancel(DecodeThread)) {
			Error(_("video: can't queue cancel video display thread\n"));
			fprintf(stderr, "VideoThreadExit: can't queue cancel video display thread\n");
		}
		if (pthread_join(DecodeThread, &retval) || retval != PTHREAD_CANCELED) {
			Error(_("video: can't cancel video display thread\n"));
			fprintf(stderr, "VideoThreadExit: can't cancel video display thread\n");
		}
		DecodeThread = 0;
//		fprintf(stderr, "VideoThreadExit: DecodeThread cleaned.\n");
		pthread_mutex_destroy(&cond_mutex);
		pthread_mutex_destroy(&VideoDeintMutex);
		pthread_mutex_destroy(&VideoLockMutex);
	}

	if (DisplayThread) {
		if (pthread_cancel(DisplayThread)) {
			Error(_("video: can't cancel DisplayHandlerThread thread\n"));
			fprintf(stderr, "VideoThreadExit: can't cancel DisplayHandlerThread thread\n");
		}
		if (pthread_join(DisplayThread, &retval) || retval != PTHREAD_CANCELED) {
			Error(_("video: can't cancel video display thread\n"));
			fprintf(stderr, "VideoThreadExit: can't cancel video display thread\n");
		}
		DisplayThread = 0;
//		fprintf(stderr, "VideoThreadExit: DisplayThread cleaned.\n");
	}
}

///
///	Video display wakeup.
///
///	New video arrived, wakeup video thread.
///
void VideoThreadWakeup(VideoRender * render)
{
	if (!DecodeThread) {
//		fprintf(stderr, "VideoThreadWakeup: (!DecodeThread)\n");

		pthread_cond_init(&cond,NULL);
		pthread_mutex_init(&cond_mutex, NULL);
		pthread_mutex_init(&VideoDeintMutex, NULL);
		pthread_mutex_init(&VideoLockMutex, NULL);
		pthread_create(&DecodeThread, NULL, DecodeHandlerThread, render);
		pthread_setname_np(DecodeThread, "softhddev video");
	}

	if (!DisplayThread) {
		pthread_create(&DisplayThread, NULL, DisplayHandlerThread, render);
//		fprintf(stderr, "VideoThreadWakeup: DisplayThread started\n");
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
//	fprintf(stderr, "VideoNewRender\n");
	if (!(render = calloc(1, sizeof(*render)))) {
		Error(_("video/DRM: out of memory\n"));
		return NULL;
	}
	atomic_set(&render->FramesFilled, 0);
	atomic_set(&render->FramesDeintFilled, 0);
	render->Stream = stream;
	render->Closing = 0;

	return render;
}

///
///	Destroy a video render.
///
///	@param render	video render
///
void VideoDelRender(VideoRender * render)
{
//	fprintf(stderr, "VideoDelRender\n");
    if (render) {
#ifdef DEBUG
		if (!pthread_equal(pthread_self(), DecodeThread)) {
			Debug(3, "video: should only be called from inside the thread\n");
		}
#endif
		free(render);
		return;
    }
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
    AVCodecContext * video_ctx, const enum AVPixelFormat *fmt)
{
	while (*fmt != AV_PIX_FMT_NONE) {
		if (*fmt == AV_PIX_FMT_YUV420P && video_ctx->codec_id == AV_CODEC_ID_MPEG2VIDEO) {
//			fprintf(stderr, "Video_get_format: AV_PIX_FMT_YUV420P Codecname: %s\n",
//				video_ctx->codec->name);
			return AV_PIX_FMT_YUV420P;
		}
		if (*fmt == AV_PIX_FMT_NV12 && video_ctx->codec_id == AV_CODEC_ID_H264) {
//			fprintf(stderr, "Video_get_format: AV_PIX_FMT_NV12 Codecname: %s\n",
//				video_ctx->codec->name);
			return AV_PIX_FMT_NV12;
		}
		if (*fmt == AV_PIX_FMT_NV12 && video_ctx->codec_id == AV_CODEC_ID_HEVC) {
//			fprintf(stderr, "Video_get_format: AV_PIX_FMT_NV12 Codecname: %s\n",
//				video_ctx->codec->name);
			return AV_PIX_FMT_NV12;
		fmt++;
		}
	}
	fprintf(stderr, "Video_get_format: No pixel format found!\n");

	return avcodec_default_get_format(video_ctx, fmt);
}

/**
**	Filter thread.
*/
static void *FilterHandlerThread(void * arg)
{
	VideoRender * render = (VideoRender *)arg;
	AVFrame *frame = 0;
	int ret = 0;
	int thread_close = 0;

	while (1) {
		while ((atomic_read(&render->FramesDeintFilled)) == 0 && !render->Deint_Close) {
			usleep(10000);
		}

getinframe:
		if (atomic_read(&render->FramesDeintFilled)) {
			pthread_mutex_lock(&VideoDeintMutex);
			frame = render->FramesDeintRb[render->FramesDeintRead];
			render->FramesDeintRead = (render->FramesDeintRead + 1) % VIDEO_SURFACES_MAX;
			atomic_dec(&render->FramesDeintFilled);
			pthread_mutex_unlock(&VideoDeintMutex);

			if (atomic_read(&render->FramesDeintFilled) < VIDEO_SURFACES_MAX &&
				atomic_read(&render->FramesFilled) < VIDEO_SURFACES_MAX) {

				pthread_mutex_lock(&cond_mutex);
				pthread_cond_signal(&cond);
				pthread_mutex_unlock(&cond_mutex);
			}
		} else {
			frame = NULL;
		}

		if (render->Deint_Close) {
			thread_close = 1;
			if (frame)
				av_frame_free(&frame);
			if (atomic_read(&render->FramesDeintFilled)) {
				goto getinframe;
			}
			frame = NULL;
		}

		if (av_buffersrc_add_frame_flags(render->buffersrc_ctx,
			frame, AV_BUFFERSRC_FLAG_KEEP_REF) < 0) {
			fprintf(stderr, "FilterHandlerThread: can't send_packet.\n");
		} else {
			av_frame_free(&frame);
		}

		while (1) {
			AVFrame *filt_frame = av_frame_alloc();
getoutframe:
			ret = av_buffersink_get_frame(render->buffersink_ctx, filt_frame);

			if (ret == AVERROR(EAGAIN)) {
				av_frame_free(&filt_frame);
				break;
			}
			if (ret == AVERROR_EOF) {
				av_frame_free(&filt_frame);
				goto closing;
			}
			if (thread_close) {
				goto getoutframe;
			}
			filt_frame->pts = filt_frame->pts / 2;
fillframe:
			if (render->Deint_Close) {
				av_frame_free(&filt_frame);
				break;
			}
			if (atomic_read(&render->FramesFilled) < VIDEO_SURFACES_MAX) {
				pthread_mutex_lock(&VideoLockMutex);
				render->FramesRb[render->FramesWrite] = filt_frame;
				render->FramesWrite = (render->FramesWrite + 1) % VIDEO_SURFACES_MAX;
				atomic_inc(&render->FramesFilled);
				pthread_mutex_unlock(&VideoLockMutex);
			} else {
				usleep(10000);
				goto fillframe;
			}
		}
	}
closing:
	avfilter_graph_free(&render->filter_graph);
	render->Deint_Close = 0;

	pthread_cleanup_push(ThreadExitHandler, render);
	pthread_cleanup_pop(1);
	pthread_exit((void *)pthread_self());
}

/**
**	Filter init.
*/
void InitFilter(VideoRender * render, const AVCodecContext * video_ctx,
		AVFrame * frame)
{
	char args[512];
	const AVFilter *buffersrc  = avfilter_get_by_name("buffer");
	const AVFilter *buffersink = avfilter_get_by_name("buffersink");
	AVFilterInOut *outputs = avfilter_inout_alloc();
	AVFilterInOut *inputs  = avfilter_inout_alloc();
	render->filter_graph = avfilter_graph_alloc();

//	const char *filter_descr = "yadif=1:-1:0";
	const char *filter_descr = "bwdif=1:-1:0";

#if LIBAVFILTER_VERSION_INT < AV_VERSION_INT(7,16,100)
	avfilter_register_all();
#endif

	snprintf(args, sizeof(args),
		"video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
		video_ctx->width, video_ctx->height, frame->format,
		video_ctx->time_base.num, video_ctx->time_base.den,
		video_ctx->sample_aspect_ratio.num, video_ctx->sample_aspect_ratio.den);

	if (avfilter_graph_create_filter(&render->buffersrc_ctx, buffersrc, "in",
		args, NULL, render->filter_graph) < 0)
			fprintf(stderr, "InitFilter: Cannot create buffer source\n");

	AVBufferSinkParams *params = av_buffersink_params_alloc();
	enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_NV12, AV_PIX_FMT_NONE };
	params->pixel_fmts = pix_fmts;

	if (avfilter_graph_create_filter(&render->buffersink_ctx, buffersink, "out",
		NULL, params, render->filter_graph) < 0)
			fprintf(stderr, "InitFilter: Cannot create buffer sink\n");
	av_free(params);

	outputs->name       = av_strdup("in");
	outputs->filter_ctx = render->buffersrc_ctx;
	outputs->pad_idx    = 0;
	outputs->next       = NULL;

	inputs->name       = av_strdup("out");
	inputs->filter_ctx = render->buffersink_ctx;
	inputs->pad_idx    = 0;
	inputs->next       = NULL;

	if ((avfilter_graph_parse_ptr(render->filter_graph, filter_descr,
		&inputs, &outputs, NULL)) < 0)
			fprintf(stderr, "InitFilter: avfilter_graph_parse_ptr failed\n");

	if ((avfilter_graph_config(render->filter_graph, NULL)) < 0)
			fprintf(stderr, "InitFilter: avfilter_graph_config failed\n");

	avfilter_inout_free(&inputs);
	avfilter_inout_free(&outputs);
}

///
///	Display a ffmpeg frame
///
///	@param hw_render	video hardware render
///	@param video_ctx	ffmpeg video codec context
///	@param frame		frame to display
///
void VideoRenderFrame(VideoRender * render,
    const AVCodecContext * video_ctx, AVFrame * frame)
{
	if (render->Closing) {
		av_frame_free(&frame);
		return;
	}

	if (frame->interlaced_frame && SWDeinterlacer) {
		if (!FilterThread) {
			InitFilter(render, video_ctx, frame);
			pthread_create(&FilterThread, NULL, FilterHandlerThread, render);
			pthread_setname_np(FilterThread, "softhddev deint");
		}

		pthread_mutex_lock(&VideoDeintMutex);
		render->FramesDeintRb[render->FramesDeintWrite] = frame;
		render->FramesDeintWrite = (render->FramesDeintWrite + 1) % VIDEO_SURFACES_MAX;
		atomic_inc(&render->FramesDeintFilled);
		pthread_mutex_unlock(&VideoDeintMutex);
	} else {
		pthread_mutex_lock(&VideoLockMutex);
		render->FramesRb[render->FramesWrite] = frame;
		render->FramesWrite = (render->FramesWrite + 1) % VIDEO_SURFACES_MAX;
		atomic_inc(&render->FramesFilled);
		pthread_mutex_unlock(&VideoLockMutex);
	}
}

///
///	Set closing stream flag.
///
///	@param hw_render	video hardware render
///
void VideoSetClosing(VideoRender * render, int closing)
{
	Debug(3, "video: set closing\n");

	if (DisplayThread)
		render->Closing = closing;
	if (FilterThread)
		render->Deint_Close = 1;
//	fprintf(stderr, "VideoSetClosing %i %i\n",
//		render->Closing, render->Deint_Close);
}

///
///	Reset start of frame counter.
///
///	@param hw_render	video hardware render
///
void VideoResetStart(VideoRender * render)
{
    Debug(3, "video: reset start\n");
    render->StartCounter = 0;
    render->FramesDuped = 0;
    render->FramesDropped = 0;
//	fprintf(stderr, "DrmResetStart: StartCounter %i\n", render->StartCounter);
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
//	fprintf(stderr, "video: set trick-speed %d\n", speed);
	render->TrickSpeed = speed;
	render->TrickCounter = speed;
	if (speed) {
		render->Closing = 0;
    }
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
///	@param[out] missed	missed frames
///	@param[out] duped	duped frames
///	@param[out] dropped	dropped frames
///	@param[out] count	number of decoded frames
///
void VideoGetStats(VideoRender * render, int *missed, int *duped,
    int *dropped, int *counter)
{
    *missed = render->FramesDuped;
    *duped = render->FramesDuped;
    *dropped = render->FramesDropped;
    *counter = render->StartCounter;
}

//----------------------------------------------------------------------------
//	Setup
//----------------------------------------------------------------------------

///
///	Get screen size.
///
///	@param[out] width	video stream width
///	@param[out] height	video stream height
///	@param[out] aspect_num	video stream aspect numerator
///	@param[out] aspect_den	video stream aspect denominator
///
void VideoGetScreenSize(VideoRender * render, int *width, int *height,
		double *pixel_aspect)
{
	*width = render->mode.hdisplay;
	*height = render->mode.vdisplay;
	*pixel_aspect = (double)16 / (double)9;
}

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
///	Set use sw deinterlacer.
///
void VideoSetSWDeinterlacer(VideoRender * render, int deint)
{
//	fprintf(stderr, "VideoSetSWDeinterlacer: deint %d\n", deint);

	if (SWDeinterlacer != deint && render) {
		SWDeinterlacer = deint;
		render->Closing = 1;
		render->Deint_Close = 1;
		sleep(1);
		render->Closing = 0;
	} else {
		SWDeinterlacer = deint;
	}
}

///
///	Initialize video output module.
///
void VideoInit(VideoRender * render)
{

	if (FindDevice(render)){
		fprintf(stderr, "VideoInit: FindDevice() failed\n");
	}
	render->bufs[0].width = render->bufs[1].width = 0;
	render->bufs[0].height = render->bufs[1].height = 0;
	render->bufs[0].pix_fmt = render->bufs[1].pix_fmt = DRM_FORMAT_NV12;
	render->buf_osd.pix_fmt = DRM_FORMAT_ARGB8888;

	// osd FB
    render->buf_osd.x = 0;
    render->buf_osd.width = render->mode.hdisplay;
    render->buf_osd.height = render->mode.vdisplay;
    if (SetupFB(render, &render->buf_osd, NULL)){
	    fprintf(stderr, "VideoOsdInit: SetupFB FB OSD failed\n");
    }
//    VideoOsdClear();

	// black fb
	render->buf_black.pix_fmt = DRM_FORMAT_NV12;
	render->buf_black.width = 720;
	render->buf_black.height = 576;
	if (SetupFB(render, &render->buf_black, NULL))
		fprintf(stderr, "VideoInit: SetupFB black FB %i x %i failed\n",
			render->buf_black.width, render->buf_black.height);
	unsigned int i;
	for (i = 0; i < render->buf_black.width * render->buf_black.height; ++i) {
		render->buf_black.plane[0][i] = 0x10;
		if (i < render->buf_black.width * render->buf_black.height / 2)
		render->buf_black.plane[1][i] = 0x80;
	}

	// save actual modesetting
	render->saved_crtc = drmModeGetCrtc(render->fd_drm, render->crtc_id);

	drmModeAtomicReqPtr ModeReq;
	const uint32_t flags = DRM_MODE_ATOMIC_ALLOW_MODESET;
	uint32_t modeID = 0;
	uint32_t prime_plane;
	uint32_t overlay_plane;

	if (render->use_zpos) {
		prime_plane = render->osd_plane;
		overlay_plane = render->video_plane;
	} else {
		prime_plane = render->video_plane;
		overlay_plane = render->osd_plane;
	}

	if (drmModeCreatePropertyBlob(render->fd_drm, &render->mode, sizeof(render->mode), &modeID) != 0)
		fprintf(stderr, "Failed to create mode property.\n");
	if (!(ModeReq = drmModeAtomicAlloc()))
		fprintf(stderr, "cannot allocate atomic request (%d): %m\n", errno);

	SetPropertyRequest(ModeReq, render->fd_drm, render->crtc_id,
						DRM_MODE_OBJECT_CRTC, "MODE_ID", modeID);
	SetPropertyRequest(ModeReq, render->fd_drm, render->connector_id,
						DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID", render->crtc_id);
	SetPropertyRequest(ModeReq, render->fd_drm, render->crtc_id,
						DRM_MODE_OBJECT_CRTC, "ACTIVE", 1);
	SetCrtc(render, ModeReq, prime_plane);

	if (render->use_zpos) {
		// Primary plane
		SetSrc(render, ModeReq, prime_plane, &render->buf_osd);
		SetPropertyRequest(ModeReq, render->fd_drm, prime_plane,
						DRM_MODE_OBJECT_PLANE, "FB_ID", render->buf_osd.fb_id);
		// Black Buffer
		SetCrtc(render, ModeReq, overlay_plane);
		SetPropertyRequest(ModeReq, render->fd_drm, overlay_plane,
						DRM_MODE_OBJECT_PLANE, "CRTC_ID", render->crtc_id);
		SetSrc(render, ModeReq, overlay_plane, &render->buf_black);
		SetPropertyRequest(ModeReq, render->fd_drm, overlay_plane,
						DRM_MODE_OBJECT_PLANE, "FB_ID", render->buf_black.fb_id);
	} else {
		// Black Buffer
		SetSrc(render, ModeReq, prime_plane, &render->buf_black);
		SetPropertyRequest(ModeReq, render->fd_drm, prime_plane,
						DRM_MODE_OBJECT_PLANE, "FB_ID", render->buf_black.fb_id);
	}
	if (drmModeAtomicCommit(render->fd_drm, ModeReq, flags, NULL) != 0)
		fprintf(stderr, "cannot set atomic mode (%d): %m\n", errno);

	drmModeAtomicFree(ModeReq);

	// init variables page flip
//    if (render->ev.page_flip_handler != Drm_page_flip_event) {
		memset(&render->ev, 0, sizeof(render->ev));
//		render->ev.version = DRM_EVENT_CONTEXT_VERSION;
		render->ev.version = 2;
//		render->ev.page_flip_handler = Drm_page_flip_event;
//	}
}

///
///	Cleanup video output module.
///
void VideoExit(VideoRender * render)
{
	VideoThreadExit();

	if (render) {
		// restore saved CRTC configuration
		if (render->saved_crtc){
			drmModeSetCrtc(render->fd_drm, render->saved_crtc->crtc_id, render->saved_crtc->buffer_id,
				render->saved_crtc->x, render->saved_crtc->y, &render->connector_id, 1, &render->saved_crtc->mode);
			drmModeFreeCrtc(render->saved_crtc);
		}

		DestroyFB(render->fd_drm, &render->buf_black);
		DestroyFB(render->fd_drm, &render->buf_osd);
	}
}

const char *VideoGetDecoderName(const char *codec_name)
{
	if (!(strcmp("mpeg2video", codec_name)))
		return "mpeg2video";

	if (!(strcmp("h264", codec_name)))
		return "h264_rkmpp";

	if (!(strcmp("hevc", codec_name)))
		return "hevc_rkmpp";

	return codec_name;
}
