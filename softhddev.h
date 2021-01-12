///
///	@file softhddev.h	@brief software HD device plugin header file.
///
///	Copyright (c) 2011 - 2015 by Johns.  All Rights Reserved.
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

#ifdef __cplusplus
extern "C"
{
#endif

//----------------------------------------------------------------------------
//	Typedefs
//----------------------------------------------------------------------------
    /// Video output stream typedef
    typedef struct __video_stream__ VideoStream; 		// in softhddev.h ?

//----------------------------------------------------------------------------
//	Prototypes
//----------------------------------------------------------------------------
    /// C plugin close osd
    extern void OsdClose(void);
    /// C plugin draw osd pixmap
    extern void OsdDrawARGB(int, int, int, int, int, const uint8_t *, int,
		int);

    /// C plugin play media file
    extern void SetAudioCodec(int, AVCodecParameters *, AVRational *);
    extern void SetVideoCodec(int, AVCodecParameters *, AVRational *);
    extern int PlayAudioPkts(AVPacket *);
    extern int PlayVideoPkts(AVPacket *);

    /// C plugin play audio packet
    extern int PlayAudio(const uint8_t *, int, uint8_t);
    /// C plugin set audio volume
    extern void SetVolumeDevice(int);
    /// C plugin reset channel id (restarts audio)
    extern void ResetChannelId(void);

    /// C plugin play video packet
    extern int PlayVideo(const uint8_t *, int);
    /// Decode video input buffers.
    extern int VideoDecodeInput(VideoStream *);
    /// Get number of input buffers.
    extern int VideoGetPackets(const VideoStream *);
    /// C plugin grab an image
    extern uint8_t *GrabImage(int *, int, int, int, int);

    /// C plugin set play mode
    extern int SetPlayMode(int);
    /// C plugin set trick speed
    extern void TrickSpeed(int);
    /// C plugin clears all video and audio data from the device
    extern void Clear(void);
    /// C plugin sets the device into play mode
    extern void Play(void);
    /// C plugin sets the device into "freeze frame" mode
    extern void Freeze(void);
    /// C plugin mute audio
    extern void Mute(void);
    /// C plugin display I-frame as a still picture.
    extern void StillPicture(const uint8_t *, int);
    /// C plugin poll if ready
    extern int Poll(int);
    /// C plugin flush output buffers
    extern int Flush(int);
    /// C plugin get current system time counter
    extern int64_t GetSTC(void);
    /// C plugin get video stream size and aspect
    extern void GetScreenSize(int *, int *, double *);
    /// C plugin command line help
    extern const char *CommandLineHelp(void);
    /// C plugin process the command line arguments
    extern int ProcessArgs(int, char *const[]);

    /// C plugin exit + cleanup
    extern void SoftHdDeviceExit(void);
    /// C plugin start code
    extern int Start(void);
    /// C plugin stop code
    extern void Stop(void);
    /// C plugin house keeping

    /// Get decoder statistics
    extern void GetStats(int *, int *, int *);
    /// Get parsed width and height
    extern void ParseResolutionH264(int *, int *);

#ifdef __cplusplus
}
#endif
