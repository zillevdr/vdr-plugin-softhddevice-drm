///
///	@file video.h	@brief Video module header file
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

/// @addtogroup Video
/// @{

//----------------------------------------------------------------------------
//	Typedefs
//----------------------------------------------------------------------------
#ifdef MMAL
    /// Video hardware decoder typedef
typedef struct _Mmal_Render_ VideoRender;
#else
    /// Video hardware decoder typedef
typedef struct _Drm_Render_ VideoRender;
#endif
    /// Video output stream typedef
typedef struct __video_stream__ VideoStream; 		// in softhddev.h ?

//----------------------------------------------------------------------------
//	Variables
//----------------------------------------------------------------------------

extern signed char VideoHardwareDecoder;	///< flag use hardware decoder
extern int VideoAudioDelay;		///< audio/video delay

//----------------------------------------------------------------------------
//	Prototypes
//----------------------------------------------------------------------------

    /// Allocate new video hardware decoder.
extern VideoRender *VideoNewRender(VideoStream *);

    /// Deallocate video hardware decoder.
extern void VideoDelRender(VideoRender *);

    /// Callback to negotiate the PixelFormat.
extern enum AVPixelFormat Video_get_format(VideoRender *, AVCodecContext *,
    const enum AVPixelFormat *);

    /// Render a ffmpeg frame.
extern void VideoRenderFrame(VideoRender *, const AVCodecContext *,
    AVFrame *);

    /// Wakeup display handler.
extern void VideoDisplayWakeup(VideoRender *);

    /// Set audio delay.
extern void VideoSetAudioDelay(int);

#ifndef MMAL
    /// Set use sw deinterlacer.
extern void VideoSetSWDeinterlacer(VideoRender * ,int);
#endif
    /// Clear OSD.
extern void VideoOsdClear(VideoRender *);

    /// Draw an OSD ARGB image.
extern void VideoOsdDrawARGB(VideoRender *, int, int, int,
		int, int, const uint8_t *, int, int);

    /// Set closing flag.
extern void VideoSetClosing(VideoRender *, int);

    /// Reset start of frame counter
extern void VideoResetStart(VideoRender *);

    /// Set trick play speed.
extern void VideoSetTrickSpeed(VideoRender *, int);

    /// Grab screen.
extern uint8_t *VideoGrab(int *, int *, int *, int);

    /// Grab screen raw.
extern uint8_t *VideoGrabService(int *, int *, int *);

    /// Get decoder statistics.
extern void VideoGetStats(VideoRender *, int *, int *, int *, int *);

    /// Get screen size
extern void VideoGetScreenSize(VideoRender * render, int *, int *, double *);

    /// Set screen size
extern void VideoSetScreenSize(char *);

extern void VideoInit(VideoRender *);	///< Setup video module.
extern void VideoExit(VideoRender *);		///< Cleanup and exit video module.

    /// Poll video input buffers.
extern int VideoPollInput(VideoStream *);

    /// Decode video input buffers.
extern int VideoDecodeInput(VideoStream *);

    /// Get number of input buffers.
extern int VideoGetBuffers(const VideoStream *);

extern const char * VideoGetDecoderName(const char *);

/// @}
