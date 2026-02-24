/**
 * @file videostream.cpp
 * Videostream class
 *
 * This file defines cVideoStream, which is repsonsible for
 * handling the video stream.
 *
 * @copyright (c) 2011 - 2015 by Johns.  All Rights Reserved.
 * @copyright (c) 2018 - 2019 by zille.  All Rights Reserved.
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

#include <functional>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/timestamp.h>
}

#include "codec_video.h"
#include "config.h"
#include "h264parser.h"
#include "logger.h"
#include "misc.h"
#include "queue.h"
#include "threads.h"
#include "videostream.h"
#include "videorender.h"

/**
 * Helper function to read a line from a given file
 *
 * @param[out] buf           pointer to the data
 * @param[out] size          size of the data at buf
 * @param[in] file           the filepointer to be read on
 *
 * @returns the number of characters read
 */
static size_t ReadLineFromFile(char *buf, size_t size, const char * file)
{
	FILE *fd = NULL;
	size_t character;

	fd = fopen(file, "r");
	if (fd == NULL) {
		LOGERROR("videostream: %s: Can't open %s", __FUNCTION__, file);
		return 0;
	}

	character = getline(&buf, &size, fd);

	fclose(fd);

	return character;
}

/**
 * Helper function to find out which platform we are on
 *
 * @returns the hardware quirks of the device
 */
static int ReadHWPlatform(void)
{
	char *txt_buf;
	char *read_ptr;
	size_t bufsize = 128;
	size_t read_size;

	txt_buf = (char *) calloc(bufsize, sizeof(char));
	int hardwareQuirks = 0;

	read_size = ReadLineFromFile(txt_buf, bufsize, "/sys/firmware/devicetree/base/compatible");
	if (!read_size) {
		free((void *)txt_buf);
		return 0;
	}

	read_ptr = txt_buf;
	// be aware: device tree string can contain \x0 bytes, so every C-string function
	// thinks, we already reached the string's terminating null bytes
	// so copy the string into a temporary string without the "\0"
	char *_txt_buf = (char *) calloc(bufsize, sizeof(char));
	char *_read_ptr = _txt_buf;
	for (size_t i = 0; i < bufsize; i++) {
		if (memcmp(read_ptr, "\0", sizeof(char))) {
			memcpy(_read_ptr, read_ptr, sizeof(char));
			_read_ptr++;
		}
		read_ptr++;
	}

	read_ptr = txt_buf;
	LOGDEBUG2(L_DRM, "videostream: %s: found \"%s\", set hardware quirks", __FUNCTION__, _txt_buf);

	while(read_size) {
		if (strstr(read_ptr, "bcm2836")) {
			LOGDEBUG2(L_DRM, "videostream: %s: bcm2836 (Raspberry Pi 2 Model B) found", __FUNCTION__);
			hardwareQuirks |= QUIRK_CODEC_FLUSH_WORKAROUND;
			break;
		}
		if (strstr(read_ptr, "bcm2837")) {
			LOGDEBUG2(L_DRM, "videostream: %s: bcm2837 (Raspberry Pi 2 Model B v1.2/ 3 Model B, Raspberry Pi 3 Compute Module 3) found", __FUNCTION__);
			hardwareQuirks |= QUIRK_CODEC_FLUSH_WORKAROUND;
			break;
		}
		if (strstr(read_ptr, "bcm2711")) {
			LOGDEBUG2(L_DRM, "videostream: %s: bcm2711 (Raspberry Pi 4 Model B, Compute Module 4, Pi 400) found", __FUNCTION__);
			hardwareQuirks |= QUIRK_CODEC_FLUSH_WORKAROUND;
			break;
		}
		if (strstr(read_ptr, "bcm2712")) {
			LOGDEBUG2(L_DRM, "videostream: %s: bcm2712 (Raspberry Pi 5, Compute Module 5, Pi 500) found", __FUNCTION__);
			hardwareQuirks |= QUIRK_CODEC_FLUSH_WORKAROUND;
			break;
		}
		if (strstr(read_ptr, "amlogic")) {
			LOGDEBUG2(L_DRM, "videostream: %s: amlogic found, disable HW deinterlacer", __FUNCTION__);
			hardwareQuirks |= QUIRK_CODEC_NEEDS_EXT_INIT
			               |  QUIRK_CODEC_SKIP_FIRST_FRAMES
			               |  QUIRK_NO_HW_DEINT;
			break;
		}

		read_size -= (strlen(read_ptr) + 1);
		read_ptr = (char *)&read_ptr[(strlen(read_ptr) + 1)];
	}
	free((void *)_txt_buf);
	free((void *)txt_buf);

	return hardwareQuirks;
}

/*****************************************************************************
 * cVideoStream class
 ****************************************************************************/

/**
 * cVideoStream constructor
 */
cVideoStream::cVideoStream(cVideoRender *render, cQueue<cDrmBuffer> *drmBufferQueue, cSoftHdConfig *config, bool isPipStream, std::function<void(AVFrame *)> frameOutput)
	: m_pConfig(config),
	  m_pDecoder(nullptr),
	  m_pRender(render),
	  m_identifier(isPipStream ? "PIP" : "main"),
	  m_frameOutput(frameOutput),
	  m_pDrmBufferQueue(drmBufferQueue),
	  m_userDisabledDeinterlacer(config->ConfigDisableDeint),
	  m_deinterlacerDeactivated(isPipStream ? true : false),
	  m_startDecodingWithIFrame(config->ConfigDecoderNeedsIFrame),
	  m_parseH264Dimensions(config->ConfigParseH264Dimensions)
{
	m_filterThreadName = "shd " + std::string(m_identifier) + " filter";
	m_pFilterThread = new cFilterThread(render, m_pDrmBufferQueue, m_filterThreadName.c_str(), frameOutput);
	m_hardwareQuirks = ReadHWPlatform();
	m_decoderFallbackToSwNumPkts = config->ConfigDecoderFallbackToSw ? config->ConfigDecoderFallbackToSwNumPkts : 0;

	LOGDEBUG("videostream %s: %s", __FUNCTION__, m_identifier);
	if (m_decoderFallbackToSwNumPkts)
		LOGDEBUG2(L_CODEC, "videostream %s: %s: fallback to sw decoder after %d packets sent", __FUNCTION__, m_identifier, m_decoderFallbackToSwNumPkts);
}

/**
 * cVideoStream destructor
 */
cVideoStream::~cVideoStream(void)
{
	LOGDEBUG("videostream %s:", __FUNCTION__);

	if (m_pFilterThread)
		delete m_pFilterThread;
}

/**
 * Flushes the video stream by finalizing any pending data.
 *
 * This function completes processing of any remaining PES fragments in the fragmentation
 * buffer, then pushes a nullptr packet to the queue to signal a flush operation to the decoder.
 */
void cVideoStream::Flush(void)
{
	m_packets.Push(nullptr);
}

/**
 * Pushes a pre-assembled AVPacket directly to the processing queue.
 *
 * This function bypasses the PES fragmentation/reassembly mechanism and directly
 * pushes an already-complete AVPacket to the m_packets queue for decoding. Used
 * when packets are received from sources that don't require fragmentation handling.
 *
 * @param avpkt    The AVPacket to push to the queue
 * @return         true if the packet was successfully pushed, false otherwise
 */
bool cVideoStream::PushAvPacket(AVPacket *avpkt)
{
	if (avpkt->pts != AV_NOPTS_VALUE)
		m_inputPts = avpkt->pts;

	return m_packets.Push(avpkt);
}

int64_t cVideoStream::GetInputPtsMs(void)
{
	return m_inputPts * 1000 * av_q2d(m_timebase);
}

/**
 * Exit video stream
 */
void cVideoStream::Exit(void)
{
	LOGDEBUG("videostream %s: %s:", m_identifier, __FUNCTION__);

	ExitDecodingThread();

	if (m_pDecoder) {
		m_pDecoder->Close();
		delete(m_pDecoder);
		m_pDecoder = nullptr;
	}

	ClearVdrCoreToDecoderQueue();
}

/**
 * Clears all video stream data, which is buffered to be decoded
 */
void cVideoStream::ClearVdrCoreToDecoderQueue(void)
{
	LOGDEBUG("videostream %s: %s: packets %d", m_identifier, __FUNCTION__, m_packets.Size());

	while (!m_packets.IsEmpty()) {
		AVPacket *avpkt = m_packets.Pop();
		av_packet_free(&avpkt);
	}

	m_inputPts = AV_NOPTS_VALUE;
}

/**
 * Start the decoder
 */
void cVideoStream::StartDecoder()
{
	LOGDEBUG2(L_CODEC, "videostream %s: %s", m_identifier, __FUNCTION__);

	m_pDecoder = new cVideoDecoder(m_hardwareQuirks, m_identifier);

	m_decodingThreadName = "shd " + std::string(m_identifier) + " decode";
	m_pDecodingThread = new cDecodingThread(this, m_decodingThreadName.c_str());
}

/**
 * Close the decoder
 */
void cVideoStream::CloseDecoder(void)
{
	LOGDEBUG2(L_CODEC, "videostream %s: %s", m_identifier, __FUNCTION__);

	m_codecId = AV_CODEC_ID_NONE;
	m_pDecoder->Close();
	m_pPar = nullptr;
	m_numIFrames = 0;
	m_naluTypesAtStart.clear();
}

/**
 * Flush the decoder
 *
 * Some hardware (RPI) needs a reopen workaround (close/open) here, because
 * hardware doesn't do the hardware flush right.
 */
void cVideoStream::FlushDecoder(void)
{
	LOGDEBUG2(L_CODEC, "videostream %s: %s", m_identifier, __FUNCTION__);

	if ((m_hardwareQuirks & QUIRK_CODEC_FLUSH_WORKAROUND) && m_pDecoder->IsHardwareDecoder()) {
		if (m_pDecoder->ReopenCodec(m_codecId, m_pPar, m_timebase, 0))
			LOGFATAL("videostream %s: %s: Could not reopen the decoder (flush)!", m_identifier, __FUNCTION__);
	} else {
		m_pDecoder->FlushBuffers();
	}
}

/**
 * Check, if we need to force the decoder to decode the frame (force a decoder drain)
 *
 * Get the number of packets we need to have in the buffer while in interlaced
 * trickspeed mode, in order to get a decoded frameout of the decoder.
 *
 * In a normal interlaced h264 stream we need to force decoding after sending 2 packets in
 * backwards trickspeed to get a decoded frame, in an mpeg2 stream 1 packet is enough.
 *
 * This minPkts magic guarantees, that we don't drain the decoder too early, but exactly after
 * the right amount of packets was sent in trickspeed mode.
 */
void cVideoStream::CheckForcingFrameDecode(void)
{
	int minPkts = m_interlaced ? m_trickpkts : 1;

	if (!m_pRender->IsForwardTrickspeed()) {
		m_sentTrickPkts++;
		if (m_sentTrickPkts >= minPkts) {
			m_pDecoder->SendPacket(NULL);
			m_sentTrickPkts = 0;
		}
	}
}

/**
 * Open the decoder including an H.264 parsing if needed
 */
void cVideoStream::OpenDecoder(void)
{
	int width = 0;
	int height = 0;

	bool needsParsing = m_startDecodingWithIFrame ||
	                    m_parseH264Dimensions ||
	                   (m_hardwareQuirks & QUIRK_CODEC_NEEDS_EXT_INIT);

	if (needsParsing && m_codecId == AV_CODEC_ID_H264) {
		cH264Parser h264Packet(m_packets.Peek());
		if (!h264Packet.HasSPS()) {
			LOGDEBUG2(L_CODEC, "videostream %s: %s: No SPS in the packet!", m_identifier, __FUNCTION__);
			h264Packet.PrintStreamData();
		}

		// start decoding with an I-Frame only
		if (!h264Packet.IsIFrame() && m_startDecodingWithIFrame) {
			h264Packet.PrintNalUnits();
			LOGDEBUG2(L_CODEC, "videostream %s: %s: Skip h264 packet, no I-Frame!", m_identifier, __FUNCTION__);
			AVPacket *avpkt = m_packets.Pop();
			av_packet_free(&avpkt);
			return;
		}

		// amlogic h264 decoder needs width an height for correct decoder open
		if ((m_hardwareQuirks & QUIRK_CODEC_NEEDS_EXT_INIT) || m_parseH264Dimensions) {
			width = h264Packet.GetWidth();
			height = h264Packet.GetHeight();
			LOGDEBUG2(L_CODEC, "videostream %s: %s: Parsed width %d height %d", m_identifier, __FUNCTION__, width, height);
		}
	}

	if (m_pDecoder->Open(m_codecId, m_pPar, m_timebase, false, width, height))
		LOGFATAL("videostream %s: %s: Could not open the decoder!", m_identifier, __FUNCTION__);

	m_pConfig->CurrentDecoderType = m_pDecoder->IsHardwareDecoder() ? "hardware" : "software";
	m_pConfig->CurrentDecoderName = m_pDecoder->Name();
	m_newStream = false;
	m_logFrames = m_pConfig->ConfigParseH264StreamStart;
}

/**
 * Decodes a reassembled codec packet.
 */
void cVideoStream::DecodeInput(void)
{
	AVFrame *frame = nullptr;
	int ret = 0;

	if (m_codecId == AV_CODEC_ID_NONE || m_packets.IsEmpty() || m_pDrmBufferQueue->IsFull() || m_pFilterThread->IsInputBufferFull())
		return;

	if (m_newStream)
		OpenDecoder();

	// send packet to decoder
	AVPacket *avpkt = m_packets.Peek();

	ret = m_pDecoder->SendPacket(avpkt);

	// log nal units of the frames until the second I Frame arrives
	if (!ret && m_codecId == AV_CODEC_ID_H264 && m_logFrames) {
		if (m_numIFrames < 2) {
			cH264Parser h264Packet(m_packets.Peek());
			if (h264Packet.IsIFrame())
				m_numIFrames++;
			m_naluTypesAtStart.push_back(h264Packet.GetNalUnitString());
		} else {
			LOGDEBUG("videostream %s: %s: parsed H.264 stream:", m_identifier, __FUNCTION__);
			for (std::size_t i = 0; i < m_naluTypesAtStart.size(); i++) {
				LOGDEBUG("[H264] (%02d) %s", i, m_naluTypesAtStart[i].c_str());
			}
			m_logFrames = false;
		}
	}

	if (ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
		avpkt = m_packets.Pop();
		av_packet_free(&avpkt);
	}

	if (!ret && m_pRender->IsTrickSpeed())
		CheckForcingFrameDecode();

	// receive frame from decoder
	ret = m_pDecoder->ReceiveFrame(&frame);
	if (ret == 0) {
		RenderFrame(frame);
	} else if (ret == AVERROR_EOF) {
		FlushDecoder();
		m_sentTrickPkts = 0;
	}

	if (m_pDecoder->IsHardwareDecoder() && !m_pDecoder->GetFramesReceived()) {
		// log maximum number of packets needed for the hw decoder to deliver a frame
		if (m_pConfig->GetDecoderNeedsMaxPackets() < m_pDecoder->GetPacketsSent())
			m_pConfig->SetDecoderNeedsMaxPackets(m_pDecoder->GetPacketsSent());

		// fallback to software decoder if configured and hw decoder fails
		if (m_decoderFallbackToSwNumPkts && m_pDecoder->GetPacketsSent() > m_decoderFallbackToSwNumPkts) {
			LOGWARNING("videostream %s: %s: Could not decode frame after %d packets sent, fallback to software decoder!", m_identifier, __FUNCTION__, m_decoderFallbackToSwNumPkts);
			if (m_pDecoder->ReopenCodec(m_codecId, m_pPar, m_timebase, m_decoderFallbackToSwNumPkts))
				LOGFATAL("videostream %s: %s: Could not reopen the decoder (sw fallback)!", m_identifier, __FUNCTION__);
		}
	}
}

/**
 * Get video size and aspect ratio
 *
 * @param[out] width            video stream width
 * @param[out] height           video stream height
 * @param[out] aspect_ratio     video stream aspect ratio (is currently width/ height)
 */
void cVideoStream::GetVideoSize(int *width, int *height, double *aspect_ratio)
{
	AVCodecContext *videoCtx = m_pDecoder->GetContext();

	if (m_pDecoder && videoCtx) {
		*width = videoCtx->coded_width;
		*height = videoCtx->coded_height;
		*aspect_ratio = *width / (double)*height;
	} else {
		*width = 0;
		*height = 0;
		*aspect_ratio = 1.0;
	}
}

/**
 * Open a video codec
 *
 * @param codecId       video codec id
 * @param par           video codec parameters
 * @param timebase      timebase
 */
void cVideoStream::Open(AVCodecID codecId, AVCodecParameters *par, AVRational timebase) {
	m_newStream = true;
	m_trickpkts = codecId == AV_CODEC_ID_MPEG2VIDEO ? 1 : 2;
	m_timebase = timebase;
	m_codecId = codecId;
	m_pPar = par;
}

/*****************************************************************************
 * Thread
 ****************************************************************************/

/**
 * Stop decoding thread
 */
void cVideoStream::ExitDecodingThread(void)
{
	LOGDEBUG("videostream %s: %s", m_identifier, __FUNCTION__);

	if (m_pDecodingThread->Active())
		m_pDecodingThread->Stop();

	if (m_pDecodingThread)
		delete m_pDecodingThread;
}

/**
 * Stop filter thread
 */
void cVideoStream::CancelFilterThread(void) {
	if (m_pFilterThread->Active())
		m_pFilterThread->Stop();

	m_checkFilterThreadNeeded = true;
	SetDeinterlacerDeactivated(false);
}

/**
 * Render a frame
 *
 * Frames either go through the filter thread or directly into the render buffer.
 *
 * @param videoCtx      ffmpeg video codec context
 * @param frame         frame to render
 */
void cVideoStream::RenderFrame(AVFrame * frame)
{
	if (frame->decode_error_flags || frame->flags & AV_FRAME_FLAG_CORRUPT)
		LOGWARNING("videostream: %s: %s: error_flag or FRAME_FLAG_CORRUPT", m_identifier, __FUNCTION__);

	// Filter thread will only be started, if the lambda function returns true
	if (m_checkFilterThreadNeeded) {
		m_timebase = m_pDecoder->GetContext()->pkt_timebase;

		// Enable the deinterlacer only if:
		// - The user did not disable the deinterlacer
		// - The deinterlacer is not temporarily deactivated (trickspeed and still picture)
		// - A hardware quirk does not forbid using the deinterlacer
		// - It is an interlaced stream, determined by:
		//   - The codec is different from HEVC (always progressive)
		//   - The framerate is lower or equal to 30fps
		//   - Or, if the frame's interlaced flag is set
		// We cannot solely rely on the frame's interlaced flag, because the deinterlacer shall also be enabled with mixed progressive/interlaced streams (e.g. TV station "ProSieben").

		m_interlaced =
			(m_pDecoder->GetContext()->codec_id != AV_CODEC_ID_HEVC &&
			m_pDecoder->GetContext()->framerate.num > 0 &&
			av_q2d(m_pDecoder->GetContext()->framerate) < 30.1) || isInterlacedFrame(frame); // account for rounding errors when comparing double

		bool useDeinterlacer =
			!m_userDisabledDeinterlacer &&
			!m_deinterlacerDeactivated &&
			!(m_hardwareQuirks & QUIRK_NO_HW_DEINT) &&
			m_interlaced;

		if (m_userDisabledDeinterlacer)
			LOGDEBUG("videostream: %s: %s: deinterlacer disabled by user configuration", m_identifier, __FUNCTION__);

		// Use the filter thread if:
		// - AV_PIX_FMT_YUV420P, interlaced -> software deinterlacer (bwdif filter)
		// - AV_PIX_FMT_YUV420P, progressive -> scale filter to get NV12 frames
		// - AV_PIX_FMT_DRM_PRIME, interlaced, hw deinterlacer available -> hw deinterlacer
		if (frame->format == AV_PIX_FMT_YUV420P || (frame->format == AV_PIX_FMT_DRM_PRIME && useDeinterlacer))
			m_pFilterThread->InitAndStart(m_pDecoder->GetContext(), frame, useDeinterlacer);

		m_checkFilterThreadNeeded = false;
	}

	if (m_pFilterThread->Active())
		m_pFilterThread->PushFrame(frame);
	else {
		// AV_PIX_FMT_DRM_PRIME, interlaced, hw deinterlacer not available
		// AV_PIX_FMT_DRM_PRIME, progressive
		// -> put the frame directly into render buffer
		if (!m_pFilterThread->GetNumFramesToFilter())
			m_frameOutput(frame);
	}
}
