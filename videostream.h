/**
 * @file videostream.h
 * Videostream class header file
 *
 * @copyright (c) 2011 - 2015 by Johns.  All Rights Reserved.
 * @copyright (c) 2025 by Andreas Baierl. All Rights Reserved.
 *
 * @license{AGPLv3
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.}
 */

#ifndef __VIDEOSTREAM_H
#define __VIDEOSTREAM_H

#include <atomic>
#include <functional>
#include <set>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
}

#include "queue.h"
#include "threads.h"

#define VIDEO_BUFFER_SIZE (512 * 1024)  ///< video PES buffer default size
#define VIDEO_PACKET_MAX 192            ///< max number of video packets held in the buffer

// Hardware quirks, that are set depending on the hardware used
#define QUIRK_NO_HW_DEINT               1 << 0     ///< set, if no hw deinterlacer available
#define QUIRK_CODEC_FLUSH_WORKAROUND    1 << 1     ///< set, if we have to close and reopen the codec instead of avcodec_flush_buffers (rpi)
#define QUIRK_CODEC_NEEDS_EXT_INIT      1 << 2     ///< set, if codec needs some infos for init (coded_width and coded_height)
#define QUIRK_CODEC_SKIP_FIRST_FRAMES   1 << 3     ///< set, if codec should skip first I-Frames
#define QUIRK_CODEC_SKIP_NUM_FRAMES     2          ///< skip QUIRK_CODEC_SKIP_NUM_FRAMES, in case QUIRK_CODEC_SKIP_FIRST_FRAMES is set
#define QUIRK_CODEC_DISABLE_MPEG_HW     1 << 4     ///< set, if disable mpeg hardware decoder
#define QUIRK_CODEC_DISABLE_H264_HW     1 << 5     ///< set, if disable h264 hardware decoder

class cSoftHdConfig;
class cVideoDecoder;
class cVideoRender;

/**
 * cVideoStream - Video stream class
 */
class cVideoStream
{
public:
	virtual ~cVideoStream(void);

	void DecodeInput(void);
	bool IsInterlaced(void) { return false; };

	void Open(AVCodecID, AVCodecParameters * = nullptr, AVRational = { .num = 1, .den = 90000 });
	void Exit(void);
	void ClearVdrCoreToDecoderQueue(void);
	void FlushDecoder(void);
	void CloseDecoder(void);
	bool PushAvPacket(AVPacket *avpkt);
	void Flush(void);

	// getters and setters
	cVideoDecoder *Decoder(void) { return m_pDecoder; };
	void StartDecoder();
	size_t GetAvPacketsFilled(void) { return m_packets.Size(); };
	bool IsInputBufferFull(void) { return m_packets.Size() >= VIDEO_PACKET_MAX; };
	enum AVCodecID GetCodecId(void) { return m_codecId; };
	void ResetTrickSpeedFramesSentCounter(void) { m_sentTrickPkts = 0; };
	bool HasInputPts(void) { return m_inputPts != AV_NOPTS_VALUE; }
	int64_t GetInputPtsMs(void);
	int64_t GetInputPts(void) { return m_inputPts; };
	void ResetInputPts(void) { m_inputPts = AV_NOPTS_VALUE; };
	void GetVideoSize(int *, int *, double *);

	// decoding thread
	void ExitDecodingThread(void);
	void DecodingThreadHalt(void) { m_pDecodingThread->Halt(); };
	void DecodingThreadResume(void) { m_pDecodingThread->Resume(); };

	// Filter
	void CancelFilterThread(void);
	void ResetFilterThreadNeededCheck() { m_checkFilterThreadNeeded = true; };

	virtual void SetDeinterlacerDeactivated(bool deactivate) { m_deinterlacerDeactivated = deactivate; };
	bool IsDeinterlacerDeactivated(void) { return m_deinterlacerDeactivated; };
	int HardwareQuirks(void) { return m_hardwareQuirks; };
	void DisableDeint(bool disable) { m_userDisabledDeinterlacer = disable; };
	void SetStartDecodingWithIFrame(bool enable) { m_startDecodingWithIFrame = enable; };
	void SetParseH264Dimensions(bool enable) { m_parseH264Dimensions = enable; };
	void SetDecoderFallbackToSwNumPkts(int numPackets) { m_decoderFallbackToSwNumPkts = numPackets; };

protected:
	cVideoStream(cVideoRender *, cQueue<cDrmBuffer> *, cSoftHdConfig *, bool, std::function<void(AVFrame *)>);

private:
	cSoftHdConfig *m_pConfig;           ///< plugin config
	cVideoDecoder *m_pDecoder;          ///< video decoder
	cVideoRender *m_pRender;            ///< video renderer
	cFilterThread *m_pFilterThread;     ///< pointer to deinterlace filter thread
	const char *m_identifier;           ///< identifier string for logging
	std::string m_filterThreadName;     ///< filter thread name string (persists for object lifetime)
	std::string m_decodingThreadName;   ///< decoding thread name string (persists for object lifetime)
	std::function<void(AVFrame *)> m_frameOutput;   ///< function to output the frame
	cQueue<cDrmBuffer> *m_pDrmBufferQueue;          ///< pointer to renderer's DRM buffer queue

	bool m_checkFilterThreadNeeded;                 ///< set, if we have to check, if filter thread is needed at start of playback
	int m_hardwareQuirks;                           ///< hardware specific quirks
	bool m_userDisabledDeinterlacer = false;        ///< set, if the user configured the deinterlace to be disabled
	bool m_deinterlacerDeactivated;                 ///< set, if the deinterlacer should be disabled temporarily (trickspeed, stillpicture, pip)
	bool m_startDecodingWithIFrame = false;         ///< wait for an I-Frame to start h264 decoding
	bool m_parseH264Dimensions = false;             ///< parse width and height when starting an h264 stream
	int m_decoderFallbackToSwNumPkts = 22;          ///< fallback to sw decoder if hw decoder fails after the given number of packets sent

	cQueue<AVPacket> m_packets{VIDEO_PACKET_MAX};   ///< AVPackets queue

	enum AVCodecID m_codecId = AV_CODEC_ID_NONE;    ///< current codec id
	AVCodecParameters *m_pPar = nullptr;            ///< current codec parameters
	std::atomic<struct AVRational> m_timebase;      ///< current codec timebase
	int m_trickpkts;                       ///< how many avpkt does the decoder need in trickspeed mode?
	int m_sentTrickPkts = 0;               ///< how many avpkt have been sent to the decoder in trickspeed mode?
	volatile bool m_newStream = false;     ///< flag for new stream
	bool m_interlaced;                     ///< flag for interlaced stream

	cDecodingThread *m_pDecodingThread;    ///< pointer to decoding thread
	int64_t m_inputPts = AV_NOPTS_VALUE;   ///< PTS of the first packet in the input buffer
	std::vector<std::string> m_naluTypesAtStart;
	int m_numIFrames = 0;
	bool m_logFrames = false;
	std::set<int> m_dpbFrames;
	int m_maxFrameNum = 1;
	int m_log2MaxFrameNumMinus4 = -4;

	void RenderFrame(AVFrame *);
	void CheckForcingFrameDecode(void);
	void OpenDecoder(void);
};

/**
 * cMainVideoStream - Main video stream class
 */
class cMainVideoStream : public cVideoStream
{
public:
	cMainVideoStream(cVideoRender *render, cQueue<cDrmBuffer> *buf, cSoftHdConfig *config, std::function<void(AVFrame *)> fn)
		: cVideoStream(render, buf, config, false, fn) {};
};

/**
 * cPipVideoStream - Pip video stream class
 */
class cPipVideoStream : public cVideoStream
{
public:
	cPipVideoStream(cVideoRender *render, cQueue<cDrmBuffer> *buf, cSoftHdConfig *config, std::function<void(AVFrame *)> fn)
		: cVideoStream(render, buf, config, true, fn) {};
	void SetDeinterlacerDeactivated(bool) override {}; // deinterlacing is permanently disabled
};

#endif
