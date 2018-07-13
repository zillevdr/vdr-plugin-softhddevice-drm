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
#ifndef HAVE_PTHREAD_NAME
    /// only available with newer glibc
#define pthread_setname_np(thread, name)
#endif

#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext_drm.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>

#include "iatomic.h"			// portable atomic_t
#include "misc.h"
#include "video.h"
#include "audio.h"

//----------------------------------------------------------------------------
//	Defines
//----------------------------------------------------------------------------

#define CODEC_SURFACES_MAX	31	///< maximal of surfaces
#define VIDEO_SURFACES_MAX	4	///< video output surfaces for queue

//----------------------------------------------------------------------------
//	Variables
//----------------------------------------------------------------------------

signed char VideoHardwareDecoder = -1;	///< flag use hardware decoder

    /// Default audio/video delay
int VideoAudioDelay;
int SWDeinterlacer;

#ifdef DEBUG
extern uint32_t VideoSwitch;		///< ticks for channel switch
#endif
extern void AudioVideoReady(int64_t);	///< tell audio video is ready

static pthread_t VideoThread;		///< video decode thread
static pthread_cond_t VideoWakeupCond;	///< wakeup condition variable
static pthread_mutex_t VideoDeintMutex;	///< video condition mutex
static pthread_mutex_t VideoLockMutex;	///< video lock mutex

//----------------------------------------------------------------------------
//	Common Functions
//----------------------------------------------------------------------------

static void VideoThreadLock(void);	///< lock video thread
static void VideoThreadUnlock(void);	///< unlock video thread
static void VideoThreadExit(void);	///< exit/kill video thread


//----------------------------------------------------------------------------
//	DRM
//----------------------------------------------------------------------------

typedef struct _Drm_decoder_ DrmDecoder;

struct _Drm_decoder_
{
//    enum AVPixelFormat PixFmt;		///< ffmpeg frame pixfmt
    AVFrame  *SurfacesRb[CODEC_SURFACES_MAX];
    int SurfaceWrite;			///< write pointer
    int SurfaceRead;			///< read pointer
    atomic_t SurfacesFilled;		///< how many of the buffer is used

    AVFrame  *FramesRb[CODEC_SURFACES_MAX];
    int FramesWrite;			///< write pointer
    int FramesRead;			///< read pointer
    atomic_t FramesFilled;		///< how many of the buffer is used

    int TrickSpeed;			///< current trick speed
    int TrickCounter;			///< current trick speed counter
    VideoStream *Stream;		///< video stream
    int Closing;			///< flag about closing current stream
    int64_t PTS;			///< video PTS clock

    int StartCounter;			///< counter for video start
//    int FramesMissed;			///< number of frames missed
    int FramesDuped;			///< number of frames duplicated
    int FramesDropped;			///< number of frames dropped
//    int FrameCounter;			///< number of frames decoded
//    int FramesDisplayed;		///< number of frames displayed

    AVFilterGraph *filter_graph;
    AVFilterContext *buffersrc_ctx, *buffersink_ctx;
    int FilterInit;
};

static DrmDecoder *DrmDecoders[1];	///< open decoder streams

//----------------------------------------------------------------------------
//	Helper structures and functions
//----------------------------------------------------------------------------

struct drm_buf {
	uint32_t x, y, width, height, size, pitch[3], handle[3], offset[3], fb_id;
	uint8_t *plane[3];
	uint32_t pix_fmt;
	int fd_prime;
	AVFrame *frame;
};

struct data_priv {
	int fd_drm;
	drmModeModeInfo mode_sd;
	drmModeModeInfo mode_hdr;
	drmModeModeInfo mode_hd;
	drmModeCrtc *saved_crtc;
	drmEventContext ev;   ///< event context page flip
	struct drm_buf bufs[36];
    struct drm_buf buf_osd;
    struct drm_buf buf_black;
	uint32_t connector_id, crtc_id, plane_id, osd_plane_id, front_buf, act_fb_id;
	bool pflip_pending, cleanup;
    int second_field;
    int prime_buffers;
};

static struct data_priv *d_priv = NULL;
static pthread_t presentation_thread_id;
static pthread_t deinterlacer_thread_id;
static void DrmFrame2Drm(void);

static uint64_t DrmGetPropertyValue(int fd_drm, uint32_t objectID,
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
			fprintf(stderr, "Unable to query property.\n");

		if (strcmp(propName, Prop->name) == 0) {
			value = objectProps->prop_values[i];
			found = 1;
		}

		drmModeFreeProperty(Prop);

		if (found)
			break;
	}

	drmModeFreeObjectProperties(objectProps);

	if (!found)
		fprintf(stderr, "Unable to find value for property \'%s\'.\n", propName);

	return value;
}

static int DrmSetPropertyRequest(drmModeAtomicReqPtr ModeReq, int fd_drm,
					uint32_t objectID, uint32_t objectType,
					const char *propName, uint32_t value)
{
	uint32_t i;
	int found = 0;
	uint64_t id = 0;
	drmModePropertyPtr Prop;
	drmModeObjectPropertiesPtr objectProps =
		drmModeObjectGetProperties(fd_drm, objectID, objectType);

	for (i = 0; i < objectProps->count_props; i++) {
		if ((Prop = drmModeGetProperty(fd_drm, objectProps->props[i])) == NULL)
			fprintf(stderr, "Unable to query property.\n");

		if (strcmp(propName, Prop->name) == 0) {
			id = Prop->prop_id;
			found = 1;
		}

		drmModeFreeProperty(Prop);

		if (found)
			break;
	}

	drmModeFreeObjectProperties(objectProps);

	if (id == 0)
		fprintf(stderr, "Unable to find value for property \'%s\'.\n", propName);

	return drmModeAtomicAddProperty(ModeReq, objectID, id, value);
}

void DrmSetMode(drmModeModeInfo mode)
{
	struct data_priv *priv = d_priv;
	drmModeAtomicReqPtr ModeReq;
	const uint32_t flags = DRM_MODE_ATOMIC_ALLOW_MODESET;
	uint32_t modeID = 0;

#ifdef DRM_DEBUG
	Info(_("[softhddev]: Setting mode  %ix%i@%i crtc_id %i plane_id %i connector_id %i\n"),
		mode.hdisplay, mode.vdisplay, mode.vrefresh, priv->crtc_id, priv->plane_id, priv->connector_id);
#endif

	if (!(ModeReq = drmModeAtomicAlloc()))
		fprintf(stderr, "cannot allocate atomic request (%d): %m\n", errno);

	if (drmModeCreatePropertyBlob(priv->fd_drm, &mode, sizeof(mode), &modeID) != 0)
		fprintf(stderr, "Failed to create mode property.\n");
	DrmSetPropertyRequest(ModeReq, priv->fd_drm, priv->crtc_id,
						DRM_MODE_OBJECT_CRTC, "MODE_ID", modeID);
	DrmSetPropertyRequest(ModeReq, priv->fd_drm, priv->crtc_id,
						DRM_MODE_OBJECT_CRTC, "ACTIVE", 1);

	DrmSetPropertyRequest(ModeReq, priv->fd_drm, priv->connector_id,
						DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID", priv->crtc_id);

	DrmSetPropertyRequest(ModeReq, priv->fd_drm, priv->plane_id,
						DRM_MODE_OBJECT_PLANE, "CRTC_X", 0);
	DrmSetPropertyRequest(ModeReq, priv->fd_drm, priv->plane_id,
						DRM_MODE_OBJECT_PLANE, "CRTC_Y", 0);
	DrmSetPropertyRequest(ModeReq, priv->fd_drm, priv->plane_id,
						DRM_MODE_OBJECT_PLANE, "CRTC_W", mode.hdisplay);
	DrmSetPropertyRequest(ModeReq, priv->fd_drm, priv->plane_id,
						DRM_MODE_OBJECT_PLANE, "CRTC_H", mode.vdisplay);

	if (drmModeAtomicCommit(priv->fd_drm, ModeReq, flags, NULL) != 0)
		fprintf(stderr, "Cannot set atomic mode %i x %i (%d): %m\n",
			mode.hdisplay, mode.vdisplay, errno);

	drmModeAtomicFree(ModeReq);
}

void DrmSetBuf(struct drm_buf *buf)
{
	struct data_priv *priv = d_priv;
	drmModeAtomicReqPtr ModeReq;
	const uint32_t flags = DRM_MODE_ATOMIC_ALLOW_MODESET;

//	fprintf(stderr, "Set atomic buf prime_buffers %2i fd_prime %"PRIu32" Handle %"PRIu32" fb_id %3i %i x %i\n",
//		priv->prime_buffers, buf->fd_prime, buf->handle[0], buf->fb_id, buf->width, buf->height);

	if (!(ModeReq = drmModeAtomicAlloc()))
		fprintf(stderr, "cannot allocate atomic request (%d): %m\n", errno);

	DrmSetPropertyRequest(ModeReq, priv->fd_drm, priv->plane_id,
						DRM_MODE_OBJECT_PLANE, "SRC_X", 0);
	DrmSetPropertyRequest(ModeReq, priv->fd_drm, priv->plane_id,
						DRM_MODE_OBJECT_PLANE, "SRC_Y", 0);
	DrmSetPropertyRequest(ModeReq, priv->fd_drm, priv->plane_id,
						DRM_MODE_OBJECT_PLANE, "SRC_W", buf->width << 16);
	DrmSetPropertyRequest(ModeReq, priv->fd_drm, priv->plane_id,
						DRM_MODE_OBJECT_PLANE, "SRC_H", buf->height << 16);

	DrmSetPropertyRequest(ModeReq, priv->fd_drm, priv->plane_id,
						DRM_MODE_OBJECT_PLANE, "FB_ID", buf->fb_id);

	if (drmModeAtomicCommit(priv->fd_drm, ModeReq, flags, NULL) != 0)
		fprintf(stderr, "cannot set atomic buf %i width %i height %i fb_id %i (%d): %m\n",
			priv->prime_buffers, buf->width, buf->height, buf->fb_id, errno);

	drmModeAtomicFree(ModeReq);
}

static int Drm_find_dev()
{
	struct data_priv *priv;
//	drmVersion *version;
	drmModeRes *resources;
	drmModeConnector *connector;
	drmModeEncoder *encoder = 0;
	drmModeModeInfo *mode;
	drmModePlane *plane;
	drmModePlaneRes *plane_res;
	int i, fd_drm;
	uint32_t j;
	uint64_t has_dumb;
	uint64_t has_prime;

//	fd_drm = drmOpen("imx-drm", NULL);
	fd_drm = open("/dev/dri/card0", O_RDWR);
	if (fd_drm < 0) {
		fprintf(stderr, "cannot open /dev/dri/card0: %m\n");
		return -errno;
	}

//	version = drmGetVersion(fd_drm);
//	fprintf(stderr, "open /dev/dri/card0: %i %s\n", version->name_len, version->name);

	// allocate mem for d_priv
	priv = (struct data_priv *) malloc(sizeof(struct data_priv));
	memset(priv, 0, sizeof(struct data_priv));
	priv->fd_drm = fd_drm;

	// check capability
	if (drmGetCap(fd_drm, DRM_CAP_DUMB_BUFFER, &has_dumb) < 0 || has_dumb == 0)
		fprintf(stderr, "drmGetCap DRM_CAP_DUMB_BUFFER failed or doesn't have dumb buffer\n");

	if (drmSetClientCap(fd_drm, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) != 0)
		fprintf(stderr, "DRM_CLIENT_CAP_UNIVERSAL_PLANES not available.\n");

	if (drmSetClientCap(fd_drm, DRM_CLIENT_CAP_ATOMIC, 1) != 0)
		fprintf(stderr, "DRM_CLIENT_CAP_ATOMIC not available.\n");

	if (drmGetCap(fd_drm, DRM_CAP_PRIME, &has_prime) < 0)
		fprintf(stderr, "DRM_CAP_PRIME not available.\n");

	if (drmGetCap(fd_drm, DRM_PRIME_CAP_EXPORT, &has_prime) < 0)
		fprintf(stderr, "DRM_PRIME_CAP_EXPORT not available.\n");

	if (drmGetCap(fd_drm, DRM_PRIME_CAP_IMPORT, &has_prime) < 0)
		fprintf(stderr, "DRM_PRIME_CAP_IMPORT not available.\n");

	if ((resources = drmModeGetResources(fd_drm)) == NULL){
		fprintf(stderr, "cannot retrieve DRM resources (%d): %m\n",	errno);
		return -errno;
	}

#ifdef DRM_DEBUG
	Info(_("[softhddev] DRM have %i connectors, %i crtcs, %i encoders\n"),
		resources->count_connectors, resources->count_crtcs,
		resources->count_encoders);
#endif

	// find all available connectors
	for (i = 0; i < resources->count_connectors; i++) {
		connector = drmModeGetConnector(fd_drm, resources->connectors[i]);
		if (!connector) {
			fprintf(stderr, "cannot retrieve DRM connector (%d): %m\n", errno);
		return -errno;
		}

		if (connector != NULL && connector->connection == DRM_MODE_CONNECTED && connector->count_modes > 0) {
			priv->connector_id = connector->connector_id;

			// FIXME: use default encoder/crtc pair
			if ((encoder = drmModeGetEncoder(priv->fd_drm, connector->encoder_id)) == NULL){
				fprintf(stderr, "cannot retrieve encoder (%d): %m\n", errno);
				return -errno;
			}
			priv->crtc_id = encoder->crtc_id;
		}
		    // search Modes for HD, HDready and SD
		for (i = 0; i < connector->count_modes; i++) {
			mode = &connector->modes[i];
			// Mode HD
			if(mode->hdisplay == 1920 && mode->vdisplay == 1080 && mode->vrefresh == 50
				&& !(mode->flags & DRM_MODE_FLAG_INTERLACE)) {
				memcpy(&priv->mode_hd, &connector->modes[i], sizeof(drmModeModeInfo));
			}
			// Mode HDready
			if(mode->hdisplay == 1280 && mode->vdisplay == 720 && mode->vrefresh == 50
				&& !(mode->flags & DRM_MODE_FLAG_INTERLACE)) {
				memcpy(&priv->mode_hdr, &connector->modes[i], sizeof(drmModeModeInfo));
			}
			// Mode SD
			if(mode->hdisplay == 720 && mode->vdisplay == 576 && mode->vrefresh == 50
				&& !(mode->flags & DRM_MODE_FLAG_INTERLACE)) {
				memcpy(&priv->mode_sd, &connector->modes[i], sizeof(drmModeModeInfo));
			}
		}
		drmModeFreeConnector(connector);
	}

	// find first plane
	if ((plane_res = drmModeGetPlaneResources(fd_drm)) == NULL)
		fprintf(stderr, "cannot retrieve PlaneResources (%d): %m\n", errno);

	for (j = 0; j < plane_res->count_planes; j++) {
		plane = drmModeGetPlane(fd_drm, plane_res->planes[j]);

		if (plane == NULL)
			fprintf(stderr, "cannot query DRM-KMS plane %d\n", j);

		for (i = 0; i < resources->count_crtcs; i++) {
			if (plane->possible_crtcs & (1 << i))
				break;
		}

		uint64_t type = DrmGetPropertyValue(fd_drm, plane_res->planes[j],
							DRM_MODE_OBJECT_PLANE, "type");

#ifdef DRM_DEBUG // If more then 2 crtcs this must rewriten!!!
		Info(_("[softhddev] Plane id %i crtc_id %i possible_crtcs %i possible CRTC %i type %s\n"),
			plane->plane_id, plane->crtc_id, plane->possible_crtcs, resources->crtcs[i],
			(type == DRM_PLANE_TYPE_PRIMARY) ? "primary plane" :
			(type == DRM_PLANE_TYPE_OVERLAY) ? "overlay plane" :
			(type == DRM_PLANE_TYPE_CURSOR) ? "cursor plane" : "No plane type");
#endif

		if (type == DRM_PLANE_TYPE_PRIMARY && plane->crtc_id == priv->crtc_id) {
			priv->plane_id = plane->plane_id;
		}
		if (type == DRM_PLANE_TYPE_OVERLAY && (encoder->possible_crtcs & plane->possible_crtcs)) {
			priv->osd_plane_id = plane->plane_id;
			break;
		}

		// test pixel format
/*		for (k = 0; k < plane->count_formats; k++) {
			switch (plane->formats[k]) {
				case DRM_FORMAT_YUV420:
					priv->pix_fmt = plane->formats[k];
				case DRM_FORMAT_NV12:
					priv->pix_fmt = plane->formats[k];
				case DRM_FORMAT_ARGB8888:
				break;
			}
		}*/

		drmModeFreePlane(plane);
	}

	drmModeFreeResources(resources);
	drmModeFreeEncoder(encoder);
	drmModeFreePlaneResources(plane_res);

#ifdef DRM_DEBUG
	Info(_("[softhddev] DRM setup CRTC: %i plane_id: %i osd_plane_id %i\n"),
		priv->crtc_id, priv->plane_id, priv->osd_plane_id);
#endif

	d_priv = priv;

	return 0;
}

static int DrmSetupFB(struct drm_buf *buf, AVDRMFrameDescriptor *primedata)
{
	struct data_priv *priv = d_priv;
	struct drm_mode_create_dumb creq;

	if (primedata) {
		uint32_t prime_handle;

		buf->pix_fmt = primedata->layers[0].format;

		if (drmPrimeFDToHandle(priv->fd_drm, primedata->objects[0].fd, &prime_handle))
			fprintf(stderr, "Failed to retrieve the Prime Handle %i size %i (%d): %m\n",
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

		if (drmIoctl(priv->fd_drm, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0){
			fprintf(stderr, "cannot create dumb buffer (%d): %m\n", errno);
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

	if (drmModeAddFB2(priv->fd_drm, buf->width, buf->height,
		buf->pix_fmt, buf->handle, buf->pitch, buf->offset, &buf->fb_id, 0)) {
		fprintf(stderr, "cannot create framebuffer (%d): %m\n", errno);
		return -errno;
	}

//	fprintf(stderr, "DrmSetupFB prime_buffers %i Handle %"PRIu32" fb_id %i %i x %i\n",
//		priv->prime_buffers, buf->handle[0], buf->fb_id, buf->width, buf->height);

	if (primedata)
		return 0;

	struct drm_mode_map_dumb mreq;
	memset(&mreq, 0, sizeof(struct drm_mode_map_dumb));
	mreq.handle = buf->handle[0];

	if (drmIoctl(priv->fd_drm, DRM_IOCTL_MODE_MAP_DUMB, &mreq)){
		fprintf(stderr, "cannot map dumb buffer (%d): %m\n", errno);
		return -errno;
	}

	buf->plane[0] = mmap(0, creq.size, PROT_READ | PROT_WRITE, MAP_SHARED, priv->fd_drm, mreq.offset);
	if (buf->plane[0] == MAP_FAILED) {
		fprintf(stderr, "cannot mmap dumb buffer (%d): %m\n", errno);
		return -errno;
	}
	buf->plane[1] = buf->plane[0] + buf->offset[1];
	buf->plane[2] = buf->plane[0] + buf->offset[2];

	return 0;
}

static void Drm_page_flip_event( __attribute__ ((unused)) int fd,
                    __attribute__ ((unused)) unsigned int frame,
				    __attribute__ ((unused)) unsigned int sec,
				    __attribute__ ((unused)) unsigned int usec,
				    void *data)
{
	struct data_priv *priv = d_priv;
	struct drm_buf *buf = data;

	priv->pflip_pending = false;
	DrmFrame2Drm();

	// 20ms is the frame on screen, then it can free for next use
	if (buf->frame) {
		usleep(20000);
		av_frame_free(&buf->frame);
	}
}

static void DrmDestroyFB(int fd_drm, struct drm_buf *buf)
{
	struct drm_mode_destroy_dumb dreq;

	if (buf->plane[0] != 0)
		munmap(buf->plane[0], buf->size);

	drmModeRmFB(fd_drm, buf->fb_id);

	if (buf->plane[0] != 0) {
		memset(&dreq, 0, sizeof(dreq));
		dreq.handle = buf->handle[0];
		drmIoctl(fd_drm, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
	}
	buf->width = 0;
	buf->height = 0;
	buf->fb_id = 0;
	buf->plane[0] = 0;
	buf->size = 0;
	buf->fd_prime = 0;
}


///
///	Display a video frame.
///
static void *DrmDisplayFrame()
{
	struct data_priv *priv = d_priv;
	DrmDecoder *decoder;

	// Thread setcancelstate
	pthread_setcancelstate (PTHREAD_CANCEL_ENABLE, NULL);

	decoder = DrmDecoders[0];

    //	Render videos into output
	while ((atomic_read(&decoder->SurfacesFilled)) < 2 ){
		usleep(15000);
	}
	DrmFrame2Drm();
	while (1){
		if (drmHandleEvent(priv->fd_drm, &priv->ev) != 0)
			fprintf(stderr, "DrmDisplayFrame: drmHandleEvent failed!\n");

		// Destroy black FB
		if (decoder->StartCounter == 2 && priv->buf_black.fb_id != 0) {
			DrmDestroyFB(priv->fd_drm, &priv->buf_black);
		}
	}
	return 0;
}


///
///	Draw a video frame.
///
static void DrmFrame2Drm(void)
{
    int i, j;
	struct data_priv *priv = d_priv;
	struct drm_buf *buf = 0;
	DrmDecoder *decoder;
	decoder = DrmDecoders[0];
	AVFrame *frame;
	AVDRMFrameDescriptor *primedata = NULL;
	int64_t audio_clock;
	uint32_t fb_id;

dequeue:
	fb_id = DrmGetPropertyValue(priv->fd_drm, priv->plane_id,
						DRM_MODE_OBJECT_PLANE, "FB_ID");

	// Destroy FBs
	if (priv->prime_buffers && priv->buf_black.fb_id == fb_id) {
		for (i = 0; i < priv->prime_buffers; ++i) {
			DrmDestroyFB(priv->fd_drm, &priv->bufs[i]);
		}
		priv->prime_buffers = 0;
		priv->front_buf = 0;
	}

	while ((atomic_read(&decoder->SurfacesFilled)) == 0 ) {
		if (decoder->Closing && priv->prime_buffers)
			break;
		usleep(20000);
	}

	if (atomic_read(&decoder->SurfacesFilled)) {
		pthread_mutex_lock(&VideoLockMutex);
		frame = decoder->SurfacesRb[decoder->SurfaceRead];
		pthread_mutex_unlock(&VideoLockMutex);
	} else
		frame = NULL;

	if (decoder->Closing) {
closing:
		if (frame) {
			av_frame_free(&frame);
			pthread_mutex_lock(&VideoLockMutex);
			decoder->SurfaceRead = (decoder->SurfaceRead + 1) % VIDEO_SURFACES_MAX;
			atomic_dec(&decoder->SurfacesFilled);
			pthread_mutex_unlock(&VideoLockMutex);
		}

		// set a black FB
		if (!(priv->buf_black.fb_id)) {
			priv->buf_black.width = 720;
			priv->buf_black.height = 576;
			if (DrmSetupFB(&priv->buf_black, NULL))
				fprintf(stderr, "DrmFrame2Drm: DrmSetupFB black FB %i x %i failed\n",
					priv->buf_black.width, priv->buf_black.height);
			DrmSetBuf(&priv->buf_black);
			buf = &priv->buf_black;
			goto page_flip;
		}
		if (priv->buf_black.fb_id && priv->buf_black.fb_id != fb_id) {
			DrmSetBuf(&priv->buf_black);
			buf = &priv->buf_black;
			goto page_flip;
		}
		goto dequeue;
	}

	if (frame->format == AV_PIX_FMT_DRM_PRIME) {
		// frame in prime fd
		primedata = (AVDRMFrameDescriptor *)frame->data[0];
		// search or made fd / FB combination
		for (i = 0; i < priv->prime_buffers; i++) {
			if (priv->bufs[i].fd_prime == primedata->objects[0].fd) {
				buf = &priv->bufs[i];
				break;
			}
		}
		if (buf == 0) {
			buf = &priv->bufs[priv->prime_buffers];
			buf->width = (uint32_t)frame->width;
			buf->height = (uint32_t)frame->height;
			buf->fd_prime = primedata->objects[0].fd;

			if (DrmSetupFB(buf, primedata))
				fprintf(stderr, "DrmFrame2Drm: DrmSetupFB FB %i x %i failed\n", buf->width, buf->height);

			if (priv->prime_buffers == 0)
				DrmSetBuf(buf);
			priv->prime_buffers++;
		}
	} else {
		// frame in frame data
		buf = &priv->bufs[priv->front_buf];
		if (buf->fb_id == 0) {
			buf->width = (uint32_t)frame->width;
			if (frame->interlaced_frame == 1)
				buf->height = (uint32_t)frame->height / 2;
			else buf->height = (uint32_t)frame->height;

			if (DrmSetupFB(buf, NULL))
				fprintf(stderr, "DrmFrame2Drm: DrmSetupFB FB %i x %i failed\n", buf->width, buf->height);

			if (priv->prime_buffers == 0)
				DrmSetBuf(buf);
			priv->prime_buffers++;
		}

		// Copy YUV420 to NV12 and deinterlace at once
		for (i = 0; i < frame->height; ++i)
			if (((i + frame->top_field_first) % 2 == 0 && priv->second_field == 1) ||
				((i + frame->top_field_first + 1) % 2 == 0 && priv->second_field == 0) ||
				frame->interlaced_frame == 0)
					memcpy(buf->plane[0] + i / (frame->interlaced_frame + 1) * frame->width,
						frame->data[0] + i * frame->linesize[0], frame->width);

		for (i = 0; i < frame->height / 2; ++i) {
			if (((i + frame->top_field_first) % 2 == 0 && priv->second_field == 1) ||
				((i + frame->top_field_first + 1) % 2 == 0 && priv->second_field == 0) ||
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
			if (priv->second_field == 0) {
				priv->second_field = 1;
			} else {
				priv->second_field = 0;
				frame->pts += 1800;
			}
		}
	}

	if(decoder->StartCounter == 0 && decoder->Closing == 0)
		AudioVideoReady(frame->pts);

audioclock:
	audio_clock = AudioGetClock();
	if (audio_clock == (int64_t) AV_NOPTS_VALUE && !decoder->TrickSpeed) {

		if (decoder->Closing)
			goto closing;

		usleep(20000);
		goto audioclock;
	}

	int diff = frame->pts - audio_clock - VideoAudioDelay;
	decoder->PTS = frame->pts;

	if(diff > 55 * 90 && !decoder->TrickSpeed) {
		decoder->FramesDuped++;

		if (decoder->Closing)
			goto closing;

		usleep(20000);
		goto audioclock;
	}

	if (diff < -25 * 90 && !decoder->TrickSpeed) {
		decoder->FramesDropped++;

		if (decoder->Closing)
			goto closing;

		av_frame_free(&frame);
		pthread_mutex_lock(&VideoLockMutex);
		decoder->SurfaceRead = (decoder->SurfaceRead + 1) % VIDEO_SURFACES_MAX;
		atomic_dec(&decoder->SurfacesFilled);
		pthread_mutex_unlock(&VideoLockMutex);
		if (decoder->Closing == 0) {
			decoder->StartCounter++;
		}
		goto dequeue;
	}

	buf->frame = NULL;
	if (frame->interlaced_frame == 0 || priv->second_field == 0) {
		buf->frame = frame;
		pthread_mutex_lock(&VideoLockMutex);
		decoder->SurfaceRead = (decoder->SurfaceRead + 1) % VIDEO_SURFACES_MAX;
		atomic_dec(&decoder->SurfacesFilled);
		pthread_mutex_unlock(&VideoLockMutex);
	}

	decoder->StartCounter++;

page_flip:
	priv->act_fb_id = buf->fb_id;
	if (drmModePageFlip(priv->fd_drm, priv->crtc_id, buf->fb_id,
			DRM_MODE_PAGE_FLIP_EVENT, buf)) {
		fprintf(stderr, "cannot flip CRTC for connector %u fb_id %i width %i (%d): %m\n",
			priv->connector_id, buf->fb_id, frame->width, errno);
	} else {
		priv->front_buf ^= 1;
		priv->pflip_pending = true;
	}
}

///
///	Handle a DRM display.
///
static void DrmDisplayHandlerThread(void)
{
    int err;
    DrmDecoder *decoder;

    if (!(decoder = DrmDecoders[0])) {	// no stream available
	fprintf(stderr, "DrmDisplayHandlerThread: no stream available\n");
	return;
    }
    // manage fill frame output ring buffer
    if (atomic_read(&decoder->SurfacesFilled) < VIDEO_SURFACES_MAX - 1 &&
		atomic_read(&decoder->FramesFilled) < VIDEO_SURFACES_MAX - 1) {

		// FIXME: hot polling
		// fetch+decode or reopen
		err = VideoDecodeInput(decoder->Stream);
    } else {
		err = VideoPollInput(decoder->Stream);
    }
    if (err) {
		// FIXME: sleep on wakeup
		usleep(10000);		// nothing buffered
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
	struct data_priv *priv = d_priv;

    VideoThreadLock();

    if (drmModeSetPlane(priv->fd_drm, priv->osd_plane_id, priv->crtc_id,
	    0, 0, 0, 0, 0, 0, 0, 0, 0 << 16, 0 << 16))
            fprintf(stderr, "failed to clear plane: (%d): %m\n", (errno));
    priv->buf_osd.x = 0;

    VideoThreadUnlock();
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
	struct data_priv *priv = d_priv;
	int i;

    VideoThreadLock();

	if (priv->buf_osd.x == 0){
		if (drmModeSetPlane(priv->fd_drm, priv->osd_plane_id, priv->crtc_id, priv->buf_osd.fb_id,
            0, x, y, width, height, 0, 0, width << 16, height << 16))
                fprintf(stderr, "failed to enable plane: (%d): %m\n", (errno));
        priv->buf_osd.x = x;
        priv->buf_osd.y = y;
    }

	for (i = 0; i < height; ++i) {
		memcpy(priv->buf_osd.plane[0] + (x - priv->buf_osd.x) * 4 + (i + y - priv->buf_osd.y)
		   * priv->buf_osd.pitch[0], argb + i * pitch, (size_t)pitch);
	}
//	fprintf(stderr, "DrmOsdDrawARGB width: %i height: %i pitch: %i x: %i y: %i xi: %i yi: %i diff_y: %i diff_x: %i\n",
//	   width, height, pitch, x, y, xi, yi, y - priv->buf_osd.y, x - priv->buf_osd.x);

    VideoThreadUnlock();
}

///
///	Get OSD size.
///
///	@param[out] width	OSD width
///	@param[out] height	OSD height
///
void VideoGetOsdSize(int *width, int *height)
{
	struct data_priv *priv = d_priv;

    *width = 1920;
    *height = 1080;			// unknown default

    if (priv->mode_hd.hdisplay && priv->mode_hd.vdisplay) {
	*width = priv->mode_hd.hdisplay;
	*height = priv->mode_hd.vdisplay;
    }
}

///
///	Setup osd.
///
///	FIXME: looking for BGRA, but this fourcc isn't supported by the
///	drawing functions yet.
///
void VideoOsdInit(void)
{
	struct data_priv *priv = d_priv;

    VideoThreadLock();

    priv->buf_osd.x = 0;
    priv->buf_osd.width = priv->mode_hd.hdisplay;
    priv->buf_osd.height = priv->mode_hd.vdisplay;

    if (DrmSetupFB(&priv->buf_osd, NULL)){
	    fprintf(stderr, "VideoOsdInit: DrmSetupFB FB OSD failed\n");
    }

    VideoThreadUnlock();
    VideoOsdClear();
}

///
///	Cleanup OSD.
///
void VideoOsdExit(void)
{
	struct data_priv *priv = d_priv;

    VideoThreadLock();

	if (priv)
		DrmDestroyFB(priv->fd_drm, &priv->buf_osd);

    VideoThreadUnlock();
}

//----------------------------------------------------------------------------
//	Thread
//----------------------------------------------------------------------------

///
///	Lock video thread.
///
static void VideoThreadLock(void)
{
    if (VideoThread) {
		if (pthread_mutex_lock(&VideoLockMutex)) {
			Error(_("video: can't lock thread\n"));
		}
    }
}

///
///	Unlock video thread.
///
static void VideoThreadUnlock(void)
{
    if (VideoThread) {
		if (pthread_mutex_unlock(&VideoLockMutex)) {
			Error(_("video: can't unlock thread\n"));
		}
    }
}

///
///	Video render thread.
///
static void *VideoDisplayHandlerThread(void *dummy)
{
    Debug(3, "video: display thread started\n");

    for (;;) {
		// fix dead-lock with VdpauExit
		pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
		pthread_testcancel();
		pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
		DrmDisplayHandlerThread();
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

	pthread_create(&presentation_thread_id, NULL, DrmDisplayFrame, NULL);
}

///
///	Exit and cleanup video threads.
///
static void VideoThreadExit(void)
{
    if (VideoThread) {
		void *retval;

		Debug(3, "video: video thread canceled\n");

		//VideoThreadLock();
		// FIXME: can't cancel locked
		if (pthread_cancel(VideoThread)) {
			Error(_("video: can't queue cancel video display thread\n"));
			fprintf(stderr, "VideoThreadExit: can't queue cancel video display thread\n");
		}
		// DrmDisplayFrame loop end
		if (pthread_cancel(presentation_thread_id)) {
			Error(_("video: can't cancel DrmDisplayFrame thread\n"));
			fprintf(stderr, "VideoThreadExit: can't cancel DrmDisplayFrame thread\n");
		}
		//VideoThreadUnlock();
		if (pthread_join(VideoThread, &retval) || retval != PTHREAD_CANCELED) {
			Error(_("video: can't cancel video display thread\n"));
			fprintf(stderr, "VideoThreadExit: can't cancel video display thread\n");
		}
		VideoThread = 0;
		pthread_cond_destroy(&VideoWakeupCond);
		pthread_mutex_destroy(&VideoLockMutex);
		pthread_mutex_destroy(&VideoDeintMutex);
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
    DrmDecoder *decoder;
//	fprintf(stderr, "VideoNewHwDecoder\n");

    VideoThreadLock();
    if (!(decoder = calloc(1, sizeof(*decoder)))) {
		Error(_("video/DRM: out of memory\n"));
		return NULL;
    }
    // setup video surface ring buffer
    atomic_set(&decoder->SurfacesFilled, 0);
//    decoder->PixFmt = AV_PIX_FMT_NONE;
    decoder->Stream = stream;
//	decoder->SyncOnAudio = 1;
    decoder->Closing = 0;
    decoder->PTS = AV_NOPTS_VALUE;
    DrmDecoders[0] = decoder;
    VideoThreadUnlock();

    return decoder;
}

///
///	Destroy a video hw decoder.
///
///	@param hw_decoder	video hardware decoder
///
void VideoDelHwDecoder(VideoHwDecoder * decoder)
{
	fprintf(stderr, "VideoDelHwDecoder\n");
    if (decoder) {
#ifdef DEBUG
		if (!pthread_equal(pthread_self(), VideoThread)) {
			Debug(3, "video: should only be called from inside the thread\n");
		}
#endif

		if (DrmDecoders[0] == decoder) {
			DrmDecoders[0] = NULL;
			free(decoder);
			return;
		}
		Error(_("video/DRM: decoder not in decoder list.\n"));
    }
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
static void *VideoFilterThread(__attribute__ ((unused))void * dummy)
{
	DrmDecoder *decoder;
	decoder = DrmDecoders[0];
	AVFrame *frame = 0;
	int ret = 0;

	while (1) {
		while ((atomic_read(&decoder->FramesFilled)) == 0 ) {
			usleep(20000);
		}

getframe:
		pthread_mutex_lock(&VideoDeintMutex);
		frame = decoder->FramesRb[decoder->FramesRead];
		decoder->FramesRead = (decoder->FramesRead + 1) % VIDEO_SURFACES_MAX;
		atomic_dec(&decoder->FramesFilled);
		pthread_mutex_unlock(&VideoDeintMutex);

		if (decoder->Closing) {
			av_frame_free(&frame);
			if (atomic_read(&decoder->FramesFilled)) {
				goto getframe;
			}
			frame = NULL;
		}

		if (av_buffersrc_add_frame_flags(decoder->buffersrc_ctx,
			frame, AV_BUFFERSRC_FLAG_KEEP_REF) < 0) {
			fprintf(stderr, "VideoFilterThread can't send_packet.\n");
		} else {
			av_frame_free(&frame);
		}

		while (1) {
			AVFrame *filt_frame = av_frame_alloc();

			ret = av_buffersink_get_frame(decoder->buffersink_ctx, filt_frame);

			if (ret == AVERROR(EAGAIN)) {
				av_frame_free(&filt_frame);
				break;
			}
			if (ret == AVERROR_EOF) {
				av_frame_free(&filt_frame);
				goto closing;
			}

			filt_frame->pts = filt_frame->pts / 2;
fillframe:
			if (atomic_read(&decoder->SurfacesFilled) < VIDEO_SURFACES_MAX - 1) {
				pthread_mutex_lock(&VideoLockMutex);
				decoder->SurfacesRb[decoder->SurfaceWrite] = filt_frame;
				decoder->SurfaceWrite = (decoder->SurfaceWrite + 1) % VIDEO_SURFACES_MAX;
				atomic_inc(&decoder->SurfacesFilled);
				pthread_mutex_unlock(&VideoLockMutex);
			} else {
				usleep(20000);
				goto fillframe;
			}
		}
	}
closing:
	avfilter_graph_free(&decoder->filter_graph);
	decoder->FilterInit = 0;
	return 0;
}

/**
**	Filter init.
*/
void VideoFilterInit(const AVCodecContext * video_ctx, AVFrame * frame)
{
	DrmDecoder *decoder;
	decoder = DrmDecoders[0];
	char args[512];
	const AVFilter *buffersrc  = avfilter_get_by_name("buffer");
	const AVFilter *buffersink = avfilter_get_by_name("buffersink");
	AVFilterInOut *outputs = avfilter_inout_alloc();
	AVFilterInOut *inputs  = avfilter_inout_alloc();
	decoder->filter_graph = avfilter_graph_alloc();

//	const char *filter_descr = "yadif=1:-1:0";
//	const char *filter_descr = "bwdif=1:-1:0,format=nv12";
	const char *filter_descr = "bwdif=1:-1:0";

#if LIBAVFILTER_VERSION_INT < AV_VERSION_INT(7,16,100)
	avfilter_register_all();
#endif

	snprintf(args, sizeof(args),
		"video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
		video_ctx->width, video_ctx->height, frame->format,
		video_ctx->time_base.num, video_ctx->time_base.den,
		video_ctx->sample_aspect_ratio.num, video_ctx->sample_aspect_ratio.den);

	if (avfilter_graph_create_filter(&decoder->buffersrc_ctx, buffersrc, "in",
		args, NULL, decoder->filter_graph) < 0)
			fprintf(stderr, "VideoFilterInit: Cannot create buffer source\n");

	AVBufferSinkParams *params = av_buffersink_params_alloc();
	enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_NV12, AV_PIX_FMT_NONE };
	params->pixel_fmts = pix_fmts;

	if (avfilter_graph_create_filter(&decoder->buffersink_ctx, buffersink, "out",
		NULL, params, decoder->filter_graph) < 0)
			fprintf(stderr, "VideoFilterInit: Cannot create buffer sink\n");
	av_free(params);

	outputs->name       = av_strdup("in");
	outputs->filter_ctx = decoder->buffersrc_ctx;
	outputs->pad_idx    = 0;
	outputs->next       = NULL;

	inputs->name       = av_strdup("out");
	inputs->filter_ctx = decoder->buffersink_ctx;
	inputs->pad_idx    = 0;
	inputs->next       = NULL;

	if ((avfilter_graph_parse_ptr(decoder->filter_graph, filter_descr,
		&inputs, &outputs, NULL)) < 0)
			fprintf(stderr, "VideoFilterInit: avfilter_graph_parse_ptr failed\n");

	if ((avfilter_graph_config(decoder->filter_graph, NULL)) < 0)
			fprintf(stderr, "VideoFilterInit: avfilter_graph_config failed\n");

	avfilter_inout_free(&inputs);
	avfilter_inout_free(&outputs);
	decoder->FilterInit = 1;
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
	if (decoder->Closing) {
		av_frame_free(&frame);
		return;
	}

	if (frame->interlaced_frame && SWDeinterlacer) {
		if (!decoder->FilterInit) {
			VideoFilterInit(video_ctx, frame);
			pthread_create(&deinterlacer_thread_id, NULL, VideoFilterThread, NULL);
			pthread_setname_np(deinterlacer_thread_id, "softhddev deint");
			atomic_set(&decoder->FramesFilled, 0);
		}

		pthread_mutex_lock(&VideoDeintMutex);
		decoder->FramesRb[decoder->FramesWrite] = frame;
		decoder->FramesWrite = (decoder->FramesWrite + 1) % VIDEO_SURFACES_MAX;
		atomic_inc(&decoder->FramesFilled);
		pthread_mutex_unlock(&VideoDeintMutex);
	} else {
		pthread_mutex_lock(&VideoLockMutex);
		decoder->SurfacesRb[decoder->SurfaceWrite] = frame;
		decoder->SurfaceWrite = (decoder->SurfaceWrite + 1) % VIDEO_SURFACES_MAX;
		atomic_inc(&decoder->SurfacesFilled);
		pthread_mutex_unlock(&VideoLockMutex);
	}
}


///
///	Set video clock.
///
///	@param hw_decoder	video hardware decoder
///	@param pts		audio presentation timestamp
///
void VideoSetClock(VideoHwDecoder * decoder, int64_t pts)
{
    Debug(3, "video: set clock %s\n", Timestamp2String(pts));
//	fprintf(stderr, "VideoSetClock: pts %"PRId64"\n", pts);
    if (decoder) {
		decoder->PTS = pts;
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
int64_t VideoGetClock(const VideoHwDecoder * decoder)
{
	fprintf(stderr, "VideoGetClock\n");
    if (decoder) {
		if (decoder->PTS == (int64_t) AV_NOPTS_VALUE) {
//		fprintf(stderr, "DrmGetClock decoder->PTS == AV_NOPTS_VALUE\n");
		return AV_NOPTS_VALUE;
		}
    return decoder->PTS - 20 * 90 * (atomic_read(&decoder->SurfacesFilled));
    }
    return AV_NOPTS_VALUE;
}

///
///	Set closing stream flag.
///
///	@param hw_decoder	video hardware decoder
///
void VideoSetClosing(VideoHwDecoder * decoder, int closing)
{
    Debug(3, "video: set closing\n");
    decoder->Closing = closing;
//	fprintf(stderr, "DrmSetClosing %i\n", decoder->Closing);
    // clear clock to avoid further sync
    VideoSetClock(decoder, AV_NOPTS_VALUE);
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
//	fprintf(stderr, "DrmResetStart: StartCounter %i\n", decoder->StartCounter);
    // clear clock to trigger new video stream
    VideoSetClock(decoder, AV_NOPTS_VALUE);
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
    if (speed) {
		decoder->Closing = 0;
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
///	Get decoder video stream size.
///
///	@param hw_decoder	video hardware decoder
///	@param[out] width	video stream width
///	@param[out] height	video stream height
///	@param[out] aspect_num	video stream aspect numerator
///	@param[out] aspect_den	video stream aspect denominator
///
void VideoGetVideoSize(__attribute__ ((unused)) VideoHwDecoder * hw_decoder, int *width, int *height,
    int *aspect_num, int *aspect_den)
{
	fprintf(stderr, "VideoGetVideoSize\n");
    *width = 1920;
    *height = 1080;
    *aspect_num = 16;
    *aspect_den = 9;
    // FIXME: test to check if working, than make module function
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
    VideoAudioDelay = ms * 90;
}

///
///	Set use sw deinterlacer.
///
void VideoSetSWDeinterlacer(int deint)
{
	DrmDecoder *decoder;
	decoder = DrmDecoders[0];

	if (SWDeinterlacer != deint) {
		if (decoder) {
			decoder->Closing = 1;
			SWDeinterlacer = deint;
			sleep(1);
			decoder->Closing = 0;
		} else {
			SWDeinterlacer = deint;
		}
	}
}

///
///	Initialize video output module.
///
void VideoInit(void)
{
	struct data_priv *priv;

	if (Drm_find_dev()){
		fprintf(stderr, "drm_find_dev() failed\n");
	}
	priv = d_priv;
	priv->bufs[0].width = priv->bufs[1].width = 0;
	priv->bufs[0].height = priv->bufs[1].height = 0;
	priv->bufs[0].pix_fmt = priv->bufs[1].pix_fmt = DRM_FORMAT_NV12;
	priv->buf_osd.pix_fmt = DRM_FORMAT_ARGB8888;
	priv->buf_black.width = 0;
	priv->buf_black.pix_fmt = DRM_FORMAT_ARGB8888;

	// save actual modesetting for connector + CRTC
	priv->saved_crtc = drmModeGetCrtc(priv->fd_drm, priv->crtc_id);

	DrmSetMode(priv->mode_hd);

	// init variables page flip
    if (priv->ev.page_flip_handler != Drm_page_flip_event) {
		memset(&priv->ev, 0, sizeof(priv->ev));
//		priv->ev.version = DRM_EVENT_CONTEXT_VERSION;
		priv->ev.version = 2;
		priv->ev.page_flip_handler = Drm_page_flip_event;
	}

    // FIXME: make it configurable from gui
    if (getenv("NO_MPEG_HW")) {
		VideoHardwareDecoder = 1;
    }
    if (getenv("NO_HW")) {
		VideoHardwareDecoder = 0;
    }
}

///
///	Cleanup video output module.
///
void VideoExit(void)
{
    VideoThreadExit();
	drmEventContext ev;
	struct data_priv *priv = d_priv;

	if (priv) {
		// init variables
		memset(&ev, 0, sizeof(ev));
		ev.version = DRM_EVENT_CONTEXT_VERSION;
		ev.page_flip_handler = Drm_page_flip_event;

		// if a pageflip is pending, wait for it to complete
		priv->cleanup = true;
		while (priv->pflip_pending) 
			if (drmHandleEvent(priv->fd_drm, &ev))
				break;

		// restore saved CRTC configuration
		if (priv->saved_crtc){
			drmModeSetCrtc(priv->fd_drm, priv->saved_crtc->crtc_id, priv->saved_crtc->buffer_id,
				priv->saved_crtc->x, priv->saved_crtc->y, &priv->connector_id, 1, &priv->saved_crtc->mode);
			drmModeFreeCrtc(priv->saved_crtc);
		}

		// free allocated memory
		free(priv);
	}
}
