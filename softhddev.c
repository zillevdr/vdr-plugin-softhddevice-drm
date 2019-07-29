///
///	@file softhddev.c	@brief A software HD device plugin for VDR.
///
///	Copyright (c) 2011 - 2015 by Johns.  All Rights Reserved.
///	Copyright (c) 2018 - 2019 by zille.  All Rights Reserved.
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

#define noDUMP_TRICKSPEED		///< dump raw trickspeed packets

#ifndef __USE_GNU
#define __USE_GNU
#endif

#include <unistd.h>

#include <libintl.h>
#define _(str) gettext(str)		///< gettext shortcut
#define _N(str) str			///< gettext_noop shortcut

#include <pthread.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>

#include "iatomic.h"			// portable atomic_t
#include "misc.h"
#include "softhddev.h"
#include "audio.h"
#include "video.h"
#include "codec.h"

#ifdef DEBUG
static int DumpH264(const uint8_t * data, int size);
static void DumpMpeg(const uint8_t * data, int size);
#endif

//////////////////////////////////////////////////////////////////////////////
//	Variables
//////////////////////////////////////////////////////////////////////////////

extern int ConfigAudioBufferTime;	///< config size ms of audio buffer
static char ConfigStillDecoder;		///< hw/sw decoder for still picture

static pthread_mutex_t SuspendLockMutex;	///< suspend lock mutex

static volatile char StreamFreezed;	///< stream freezed

//////////////////////////////////////////////////////////////////////////////
//	Audio
//////////////////////////////////////////////////////////////////////////////

static volatile char NewAudioStream;	///< new audio stream
static volatile char SkipAudio;		///< skip audio stream
static AudioDecoder *MyAudioDecoder;	///< audio decoder
static enum AVCodecID AudioCodecID;	///< current codec id
static int AudioChannelID;		///< current audio channel id
//static VideoStream *AudioSyncStream;	///< video stream for audio/video sync

    /// Minimum free space in audio buffer 8 packets for 8 channels
#define AUDIO_MIN_BUFFER_FREE (3072 * 8 * 8)
#define AUDIO_BUFFER_SIZE (512 * 1024)	///< audio PES buffer default size
static AVPacket AudioAvPkt[1];		///< audio a/v packet

//////////////////////////////////////////////////////////////////////////////
//	Audio codec parser
//////////////////////////////////////////////////////////////////////////////

///
///	Mpeg bitrate table.
///
///	BitRateTable[Version][Layer][Index]
///
static const uint16_t BitRateTable[2][4][16] = {
    // MPEG Version 1
    {{},
	{0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448,
	    0},
	{0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384, 0},
	{0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 0}},
    // MPEG Version 2 & 2.5
    {{},
	{0, 32, 48, 56, 64, 80, 96, 112, 128, 144, 160, 176, 192, 224, 256, 0},
	{0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, 0},
	{0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, 0}
	}
};

///
///	Mpeg samperate table.
///
static const uint16_t SampleRateTable[4] = {
    44100, 48000, 32000, 0
};

///
///	Fast check for Mpeg audio.
///
///	4 bytes 0xFFExxxxx Mpeg audio
///
static inline int FastMpegCheck(const uint8_t * p)
{
    if (p[0] != 0xFF) {			// 11bit frame sync
	return 0;
    }
    if ((p[1] & 0xE0) != 0xE0) {
	return 0;
    }
    if ((p[1] & 0x18) == 0x08) {	// version ID - 01 reserved
	return 0;
    }
    if (!(p[1] & 0x06)) {		// layer description - 00 reserved
	return 0;
    }
    if ((p[2] & 0xF0) == 0xF0) {	// bitrate index - 1111 reserved
	return 0;
    }
    if ((p[2] & 0x0C) == 0x0C) {	// sampling rate index - 11 reserved
	return 0;
    }
    return 1;
}

///
///	Check for Mpeg audio.
///
///	0xFFEx already checked.
///
///	@param data	incomplete PES packet
///	@param size	number of bytes
///
///	@retval <0	possible mpeg audio, but need more data
///	@retval 0	no valid mpeg audio
///	@retval >0	valid mpeg audio
///
///	From: http://www.mpgedit.org/mpgedit/mpeg_format/mpeghdr.htm
///
///	AAAAAAAA AAABBCCD EEEEFFGH IIJJKLMM
///
///	o a 11x Frame sync
///	o b 2x	Mpeg audio version (2.5, reserved, 2, 1)
///	o c 2x	Layer (reserved, III, II, I)
///	o e 2x	BitRate index
///	o f 2x	SampleRate index (4100, 48000, 32000, 0)
///	o g 1x	Paddding bit
///	o ..	Doesn't care
///
///	frame length:
///	Layer I:
///		FrameLengthInBytes = (12 * BitRate / SampleRate + Padding) * 4
///	Layer II & III:
///		FrameLengthInBytes = 144 * BitRate / SampleRate + Padding
///
static int MpegCheck(const uint8_t * data, int size)
{
    int mpeg2;
    int mpeg25;
    int layer;
    int bit_rate_index;
    int sample_rate_index;
    int padding;
    int bit_rate;
    int sample_rate;
    int frame_size;

    mpeg2 = !(data[1] & 0x08) && (data[1] & 0x10);
    mpeg25 = !(data[1] & 0x08) && !(data[1] & 0x10);
    layer = 4 - ((data[1] >> 1) & 0x03);
    bit_rate_index = (data[2] >> 4) & 0x0F;
    sample_rate_index = (data[2] >> 2) & 0x03;
    padding = (data[2] >> 1) & 0x01;

    sample_rate = SampleRateTable[sample_rate_index];
    if (!sample_rate) {			// no valid sample rate try next
	// moved into fast check
	abort();
	return 0;
    }
    sample_rate >>= mpeg2;		// mpeg 2 half rate
    sample_rate >>= mpeg25;		// mpeg 2.5 quarter rate

    bit_rate = BitRateTable[mpeg2 | mpeg25][layer][bit_rate_index];
    if (!bit_rate) {			// no valid bit-rate try next
	// FIXME: move into fast check?
	return 0;
    }
    bit_rate *= 1000;
    switch (layer) {
	case 1:
	    frame_size = (12 * bit_rate) / sample_rate;
	    frame_size = (frame_size + padding) * 4;
	    break;
	case 2:
	case 3:
	default:
	    frame_size = (144 * bit_rate) / sample_rate;
	    frame_size = frame_size + padding;
	    break;
    }
    if (0) {
	Debug(3,
	    "pesdemux: mpeg%s layer%d bitrate=%d samplerate=%d %d bytes\n",
	    mpeg25 ? "2.5" : mpeg2 ? "2" : "1", layer, bit_rate, sample_rate,
	    frame_size);
    }

    if (frame_size + 4 > size) {
	return -frame_size - 4;
    }
    // check if after this frame a new mpeg frame starts
    if (FastMpegCheck(data + frame_size)) {
	return frame_size;
    }

    return 0;
}

///
///	Fast check for AAC LATM audio.
///
///	3 bytes 0x56Exxx AAC LATM audio
///
static inline int FastLatmCheck(const uint8_t * p)
{
    if (p[0] != 0x56) {			// 11bit sync
	return 0;
    }
    if ((p[1] & 0xE0) != 0xE0) {
	return 0;
    }
    return 1;
}

///
///	Check for AAC LATM audio.
///
///	0x56Exxx already checked.
///
///	@param data	incomplete PES packet
///	@param size	number of bytes
///
///	@retval <0	possible AAC LATM audio, but need more data
///	@retval 0	no valid AAC LATM audio
///	@retval >0	valid AAC LATM audio
///
static int LatmCheck(const uint8_t * data, int size)
{
    int frame_size;

    // 13 bit frame size without header
    frame_size = ((data[1] & 0x1F) << 8) + data[2];
    frame_size += 3;

    if (frame_size + 2 > size) {
	return -frame_size - 2;
    }
    // check if after this frame a new AAC LATM frame starts
    if (FastLatmCheck(data + frame_size)) {
	return frame_size;
    }

    return 0;
}

///
///	Possible AC-3 frame sizes.
///
///	from ATSC A/52 table 5.18 frame size code table.
///
const uint16_t Ac3FrameSizeTable[38][3] = {
    {64, 69, 96}, {64, 70, 96}, {80, 87, 120}, {80, 88, 120},
    {96, 104, 144}, {96, 105, 144}, {112, 121, 168}, {112, 122, 168},
    {128, 139, 192}, {128, 140, 192}, {160, 174, 240}, {160, 175, 240},
    {192, 208, 288}, {192, 209, 288}, {224, 243, 336}, {224, 244, 336},
    {256, 278, 384}, {256, 279, 384}, {320, 348, 480}, {320, 349, 480},
    {384, 417, 576}, {384, 418, 576}, {448, 487, 672}, {448, 488, 672},
    {512, 557, 768}, {512, 558, 768}, {640, 696, 960}, {640, 697, 960},
    {768, 835, 1152}, {768, 836, 1152}, {896, 975, 1344}, {896, 976, 1344},
    {1024, 1114, 1536}, {1024, 1115, 1536}, {1152, 1253, 1728},
    {1152, 1254, 1728}, {1280, 1393, 1920}, {1280, 1394, 1920},
};

///
///	Fast check for (E-)AC-3 audio.
///
///	5 bytes 0x0B77xxxxxx AC-3 audio
///
static inline int FastAc3Check(const uint8_t * p)
{
    if (p[0] != 0x0B) {			// 16bit sync
	return 0;
    }
    if (p[1] != 0x77) {
	return 0;
    }
    return 1;
}

///
///	Check for (E-)AC-3 audio.
///
///	0x0B77xxxxxx already checked.
///
///	@param data	incomplete PES packet
///	@param size	number of bytes
///
///	@retval <0	possible AC-3 audio, but need more data
///	@retval 0	no valid AC-3 audio
///	@retval >0	valid AC-3 audio
///
///	o AC-3 Header
///	AAAAAAAA AAAAAAAA BBBBBBBB BBBBBBBB CCDDDDDD EEEEEFFF
///
///	o a 16x Frame sync, always 0x0B77
///	o b 16x CRC 16
///	o c 2x	Samplerate
///	o d 6x	Framesize code
///	o e 5x	Bitstream ID
///	o f 3x	Bitstream mode
///
///	o E-AC-3 Header
///	AAAAAAAA AAAAAAAA BBCCCDDD DDDDDDDD EEFFGGGH IIIII...
///
///	o a 16x Frame sync, always 0x0B77
///	o b 2x	Frame type
///	o c 3x	Sub stream ID
///	o d 10x Framesize - 1 in words
///	o e 2x	Framesize code
///	o f 2x	Framesize code 2
///
static int Ac3Check(const uint8_t * data, int size)
{
    int frame_size;

    if (size < 5) {			// need 5 bytes to see if AC-3/E-AC-3
	return -5;
    }

    if (data[5] > (10 << 3)) {		// E-AC-3
	if ((data[4] & 0xF0) == 0xF0) {	// invalid fscod fscod2
	    return 0;
	}
	frame_size = ((data[2] & 0x03) << 8) + data[3] + 1;
	frame_size *= 2;
    } else {				// AC-3
	int fscod;
	int frmsizcod;

	// crc1 crc1 fscod|frmsizcod
	fscod = data[4] >> 6;
	if (fscod == 0x03) {		// invalid sample rate
	    return 0;
	}
	frmsizcod = data[4] & 0x3F;
	if (frmsizcod > 37) {		// invalid frame size
	    return 0;
	}
	// invalid is checked above
	frame_size = Ac3FrameSizeTable[frmsizcod][fscod] * 2;
    }

    if (frame_size + 5 > size) {
	return -frame_size - 5;
    }
    // FIXME: relaxed checks if codec is already detected
    // check if after this frame a new AC-3 frame starts
    if (FastAc3Check(data + frame_size)) {
	return frame_size;
    }

    return 0;
}

///
///	Fast check for ADTS Audio Data Transport Stream.
///
///	7/9 bytes 0xFFFxxxxxxxxxxx(xxxx)  ADTS audio
///
static inline int FastAdtsCheck(const uint8_t * p)
{
    if (p[0] != 0xFF) {			// 12bit sync
	return 0;
    }
    if ((p[1] & 0xF6) != 0xF0) {	// sync + layer must be 0
	return 0;
    }
    if ((p[2] & 0x3C) == 0x3C) {	// sampling frequency index != 15
	return 0;
    }
    return 1;
}

///
///	Check for ADTS Audio Data Transport Stream.
///
///	0xFFF already checked.
///
///	@param data	incomplete PES packet
///	@param size	number of bytes
///
///	@retval <0	possible ADTS audio, but need more data
///	@retval 0	no valid ADTS audio
///	@retval >0	valid AC-3 audio
///
///	AAAAAAAA AAAABCCD EEFFFFGH HHIJKLMM MMMMMMMM MMMOOOOO OOOOOOPP
///	(QQQQQQQQ QQQQQQQ)
///
///	o A*12	syncword 0xFFF
///	o B*1	MPEG Version: 0 for MPEG-4, 1 for MPEG-2
///	o C*2	layer: always 0
///	o ..
///	o F*4	sampling frequency index (15 is invalid)
///	o ..
///	o M*13	frame length
///
static int AdtsCheck(const uint8_t * data, int size)
{
    int frame_size;

    if (size < 6) {
	return -6;
    }

    frame_size = (data[3] & 0x03) << 11;
    frame_size |= (data[4] & 0xFF) << 3;
    frame_size |= (data[5] & 0xE0) >> 5;

    if (frame_size + 3 > size) {
	return -frame_size - 3;
    }
    // check if after this frame a new ADTS frame starts
    if (FastAdtsCheck(data + frame_size)) {
	return frame_size;
    }

    return 0;
}

//////////////////////////////////////////////////////////////////////////////
//	PES Demux
//////////////////////////////////////////////////////////////////////////////

///
///	PES type.
///
enum
{
    PES_PROG_STREAM_MAP = 0xBC,
    PES_PRIVATE_STREAM1 = 0xBD,
    PES_PADDING_STREAM = 0xBE,		///< filler, padding stream
    PES_PRIVATE_STREAM2 = 0xBF,
    PES_AUDIO_STREAM_S = 0xC0,
    PES_AUDIO_STREAM_E = 0xDF,
    PES_VIDEO_STREAM_S = 0xE0,
    PES_VIDEO_STREAM_E = 0xEF,
    PES_ECM_STREAM = 0xF0,
    PES_EMM_STREAM = 0xF1,
    PES_DSM_CC_STREAM = 0xF2,
    PES_ISO13522_STREAM = 0xF3,
    PES_TYPE_E_STREAM = 0xF8,		///< ITU-T rec. h.222.1 type E stream
    PES_PROG_STREAM_DIR = 0xFF,
};

/**
**	Play audio packet.
**
**	@param data	data of exactly one complete PES packet
**	@param size	size of PES packet
**	@param id	PES packet type
*/
int PlayAudio(const uint8_t * data, int size, uint8_t id)
{
    int n;
    const uint8_t *p;

    // channel switch: SetAudioChannelDevice: SetDigitalAudioDevice:

    if (SkipAudio || !MyAudioDecoder) {	// skip audio
	return size;
    }
    if (StreamFreezed) {		// stream freezed
	return 0;
    }
    if (NewAudioStream) {
	// this clears the audio ringbuffer indirect, open and setup does it
	CodecAudioClose(MyAudioDecoder);
	AudioFlushBuffers();
	AudioSetBufferTime(ConfigAudioBufferTime);
	AudioCodecID = AV_CODEC_ID_NONE;
	AudioChannelID = -1;
	NewAudioStream = 0;
    }
    // hard limit buffer full: don't overrun audio buffers on replay
    if (AudioFreeBytes() < AUDIO_MIN_BUFFER_FREE) {
	return 0;
    }
    // PES header 0x00 0x00 0x01 ID
    // ID 0xBD 0xC0-0xCF
    // must be a PES start code
    if (size < 9 || !data || data[0] || data[1] || data[2] != 0x01) {
	Error(_("[softhddev] invalid PES audio packet\n"));
	return size;
    }
    n = data[8];			// header size

    if (size < 9 + n + 4) {		// wrong size
	if (size == 9 + n) {
	    Warning(_("[softhddev] empty audio packet\n"));
	} else {
	    Error(_("[softhddev] invalid audio packet %d bytes\n"), size);
	}
	return size;
    }

    if (data[7] & 0x80 && n >= 5) {
	AudioAvPkt->pts =
	    (int64_t) (data[9] & 0x0E) << 29 | data[10] << 22 | (data[11] &
	    0xFE) << 14 | data[12] << 7 | (data[13] & 0xFE) >> 1;
	//Debug(3, "audio: pts %#012" PRIx64 "\n", AudioAvPkt->pts);
    }
    if (0) {				// dts is unused
	if (data[7] & 0x40) {
	    AudioAvPkt->dts =
		(int64_t) (data[14] & 0x0E) << 29 | data[15] << 22 | (data[16]
		& 0xFE) << 14 | data[17] << 7 | (data[18] & 0xFE) >> 1;
	    Debug(3, "audio: dts %#012" PRIx64 "\n", AudioAvPkt->dts);
	}
    }

    p = data + 9 + n;
    n = size - 9 - n;			// skip pes header
    if (n + AudioAvPkt->stream_index > AudioAvPkt->size) {
	Fatal(_("[softhddev] audio buffer too small\n"));
	AudioAvPkt->stream_index = 0;
    }

    if (AudioChannelID != id) {		// id changed audio track changed
	AudioChannelID = id;
	AudioCodecID = AV_CODEC_ID_NONE;
	Debug(3, "audio/demux: new channel id\n");
    }
    // Private stream + LPCM ID
    if ((id & 0xF0) == 0xA0) {
	if (n < 7) {
	    Error(_("[softhddev] invalid LPCM audio packet %d bytes\n"), size);
	    return size;
	}
	if (AudioCodecID != AV_CODEC_ID_PCM_DVD) {
	    static int samplerates[] = { 48000, 96000, 44100, 32000 };
	    int samplerate;
	    int channels;
	    int bits_per_sample;

	    Debug(3, "[softhddev]%s: LPCM %d sr:%d bits:%d chan:%d\n",
		__FUNCTION__, id, p[5] >> 4, (((p[5] >> 6) & 0x3) + 4) * 4,
		(p[5] & 0x7) + 1);
	    CodecAudioClose(MyAudioDecoder);

	    bits_per_sample = (((p[5] >> 6) & 0x3) + 4) * 4;
	    if (bits_per_sample != 16) {
		Error(_
		    ("[softhddev] LPCM %d bits per sample aren't supported\n"),
		    bits_per_sample);
		// FIXME: handle unsupported formats.
	    }
	    samplerate = samplerates[p[5] >> 4];
	    channels = (p[5] & 0x7) + 1;

	    // FIXME: ConfigAudioBufferTime + x
	    AudioSetBufferTime(400);
	    AudioSetup(&samplerate, &channels, 0);
	    if (samplerate != samplerates[p[5] >> 4]) {
		Error(_("[softhddev] LPCM %d sample-rate is unsupported\n"),
		    samplerates[p[5] >> 4]);
		// FIXME: support resample
	    }
	    if (channels != (p[5] & 0x7) + 1) {
		Error(_("[softhddev] LPCM %d channels are unsupported\n"),
		    (p[5] & 0x7) + 1);
		// FIXME: support resample
	    }
	    //CodecAudioOpen(MyAudioDecoder, AV_CODEC_ID_PCM_DVD);
	    AudioCodecID = AV_CODEC_ID_PCM_DVD;
	}

	if (AudioAvPkt->pts != (int64_t) AV_NOPTS_VALUE) {
//	    AudioSetClock(AudioAvPkt->pts);
	    AudioAvPkt->pts = AV_NOPTS_VALUE;
	}
	swab(p + 7, AudioAvPkt->data, n - 7);
	AudioEnqueue(AudioAvPkt->data, n - 7, NULL);

	return size;
    }
    // DVD track header
    if ((id & 0xF0) == 0x80 && (p[0] & 0xF0) == 0x80) {
	p += 4;
	n -= 4;				// skip track header
	if (AudioCodecID == AV_CODEC_ID_NONE) {
	    // FIXME: ConfigAudioBufferTime + x
	    AudioSetBufferTime(400);
	}
    }
    // append new packet, to partial old data
    memcpy(AudioAvPkt->data + AudioAvPkt->stream_index, p, n);
    AudioAvPkt->stream_index += n;

    n = AudioAvPkt->stream_index;
    p = AudioAvPkt->data;
    while (n >= 5) {
	int r;
	unsigned codec_id;

	// 4 bytes 0xFFExxxxx Mpeg audio
	// 3 bytes 0x56Exxx AAC LATM audio
	// 5 bytes 0x0B77xxxxxx AC-3 audio
	// 6 bytes 0x0B77xxxxxxxx E-AC-3 audio
	// 7/9 bytes 0xFFFxxxxxxxxxxx ADTS audio
	// PCM audio can't be found
	r = 0;
	codec_id = AV_CODEC_ID_NONE;	// keep compiler happy
	if (id != 0xbd && FastMpegCheck(p)) {
	    r = MpegCheck(p, n);
	    codec_id = AV_CODEC_ID_MP2;
	}
	if (id != 0xbd && !r && FastLatmCheck(p)) {
	    r = LatmCheck(p, n);
	    codec_id = AV_CODEC_ID_AAC_LATM;
	}
	if ((id == 0xbd || (id & 0xF0) == 0x80) && !r && FastAc3Check(p)) {
	    r = Ac3Check(p, n);
	    codec_id = AV_CODEC_ID_AC3;
	    if (r > 0 && p[5] > (10 << 3)) {
		codec_id = AV_CODEC_ID_EAC3;
	    }
	    /* faster ac3 detection at end of pes packet (no improvemnts)
	       if (AudioCodecID == codec_id && -r - 2 == n) {
	       r = n;
	       }
	     */
	}
	if (id != 0xbd && !r && FastAdtsCheck(p)) {
	    r = AdtsCheck(p, n);
	    codec_id = AV_CODEC_ID_AAC;
	}
	if (r < 0) {			// need more bytes
	    break;
	}
	if (r > 0) {
	    AVPacket avpkt[1];

	    // new codec id, close and open new
	    if (AudioCodecID != codec_id) {
		CodecAudioClose(MyAudioDecoder);
		CodecAudioOpen(MyAudioDecoder, codec_id);
		AudioCodecID = codec_id;
	    }
	    av_init_packet(avpkt);
	    avpkt->data = (void *)p;
	    avpkt->size = r;
	    avpkt->pts = AudioAvPkt->pts;
	    avpkt->dts = AudioAvPkt->dts;
	    // FIXME: not aligned for ffmpeg
	    CodecAudioDecode(MyAudioDecoder, avpkt);
	    AudioAvPkt->pts = AV_NOPTS_VALUE;
	    AudioAvPkt->dts = AV_NOPTS_VALUE;
	    p += r;
	    n -= r;
	    continue;
	}
	++p;
	--n;
    }

    // copy remaining bytes to start of packet
    if (n) {
	memmove(AudioAvPkt->data, p, n);
    }
    AudioAvPkt->stream_index = n;

    return size;
}

/**
**	Set volume of audio device.
**
**	@param volume	VDR volume (0 .. 255)
*/
void SetVolumeDevice(int volume)
{
    AudioSetVolume((volume * 1000) / 255);
}

/**
***	Resets channel ID (restarts audio).
**/
void ResetChannelId(void)
{
    AudioChannelID = -1;
    Debug(3, "audio/demux: reset channel id\n");
}

//////////////////////////////////////////////////////////////////////////////
//	Video
//////////////////////////////////////////////////////////////////////////////

#define VIDEO_BUFFER_SIZE (512 * 1024)	///< video PES buffer default size
#define VIDEO_PACKET_MAX 192		///< max number of video packets

/**
**	Video output stream device structure.	Parser, decoder, display.
*/
struct __video_stream__
{
    VideoRender *Render;		///< video hardware decoder
    VideoDecoder *Decoder;		///< video decoder
    pthread_mutex_t DecoderLockMutex;	///< video decoder lock mutex

    enum AVCodecID CodecID;		///< current codec id
    enum AVCodecID LastCodecID;		///< last codec id

    volatile char NewStream;		///< flag new video stream
    volatile char ClosingStream;	///< flag closing video stream
    volatile char SkipStream;		///< skip video stream
    volatile char Freezed;		///< stream freezed

    volatile char TrickSpeed;		///< current trick speed
//    volatile char Close;		///< command close video stream
    volatile char ClearBuffers;		///< command clear video buffers

    int InvalidPesCounter;		///< counter of invalid PES packets

    enum AVCodecID CodecIDRb[VIDEO_PACKET_MAX];	///< codec ids in ring buffer
    AVPacket PacketRb[VIDEO_PACKET_MAX];	///< PES packet ring buffer
    int StartCodeState;			///< last three bytes start code state

    int PacketWrite;			///< ring buffer write pointer
    int PacketRead;			///< ring buffer read pointer
    atomic_t PacketsFilled;		///< how many of the ring buffer is used
};

static VideoStream MyVideoStream[1];	///< normal video stream

//#ifdef DEBUG
uint32_t VideoSwitch;			///< debug video switch ticks
//static int VideoMaxPacketSize;		///< biggest used packet buffer
//#endif
//#define STILL_DEBUG 2
#ifdef STILL_DEBUG
static char InStillPicture;		///< flag still picture
#endif

//////////////////////////////////////////////////////////////////////////////

void PrintStreamData(const uint8_t *data, int size)
{
	fprintf(stderr, "[Data]: %02x %02x %02x %02x %02x %02x %02x %02x %02x "
		"%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x "
		"%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x size %d\n",
		data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8],
		data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17],
		data[18], data[19], data[20], data[21], data[22], data[23], data[24], data[25], data[26],
		data[27], data[28], data[29], data[30], data[31], data[32], data[33], data[34], size);
}

/**
**	Initialize video packet ringbuffer.
**
**	@param stream	video stream
*/
static void VideoPacketInit(VideoStream * stream)
{
    int i;

    for (i = 0; i < VIDEO_PACKET_MAX; ++i) {
	AVPacket *avpkt;

		avpkt = &stream->PacketRb[i];
		// build a clean ffmpeg av packet
		if (av_new_packet(avpkt, VIDEO_BUFFER_SIZE)) {
			Fatal(_("[softhddev] out of memory\n"));
		}
		avpkt->size = 0;
    }

//	fprintf(stderr, "VideoPacketInit: %i Packete erstellt.\n",
//		VIDEO_PACKET_MAX);
    atomic_set(&stream->PacketsFilled, 0);
    stream->PacketRead = stream->PacketWrite = 0;
}

/**
**	Cleanup video packet ringbuffer.
**
**	@param stream	video stream
*/
static void VideoPacketExit(VideoStream * stream)
{
    int i;

    atomic_set(&stream->PacketsFilled, 0);

    for (i = 0; i < VIDEO_PACKET_MAX; ++i) {
		av_packet_unref(&stream->PacketRb[i]);
    }
//	fprintf(stderr, "VideoPacketExit: %i Packete zerstört.\n",
//		VIDEO_PACKET_MAX);
}

/**
**	Place video data in packet ringbuffer.
**
**	@param stream	video stream
**	@param pts	presentation timestamp of pes packet
**	@param data	data of pes packet
**	@param size	size of pes packet
*/
static void VideoEnqueue(VideoStream * stream, int64_t pts, const void *data,
    int size)
{
	AVPacket *avpkt;
	int err;

//	PrintStreamData(data, size);

	avpkt = &stream->PacketRb[stream->PacketWrite];
	if (avpkt->pts == AV_NOPTS_VALUE) {		// add pts only for first added
		avpkt->pts = pts;
	}
	if (avpkt->size + size >= avpkt->buf->size) {
		Warning(_("video: packet buffer too small for %d\n"),
				avpkt->size + size);
		fprintf(stderr, "VideoEnqueue: vor grow avpkt->size %d size %d avpkt->buf->size %d\n",
			avpkt->size, size, avpkt->buf->size);
		// new + grow reserves AV_INPUT_BUFFER_PADDING_SIZE
		err = av_grow_packet(avpkt, avpkt->size + size - avpkt->buf->size +
			AV_INPUT_BUFFER_PADDING_SIZE);
		fprintf(stderr, "VideoEnqueue: nach grow avpkt->size %d size %d avpkt->buf->size %d err %i\n",
			avpkt->size, size, avpkt->buf->size, err);
		// FIXME: out of memory!
	}

	memcpy(avpkt->data + avpkt->size, data, size);

/*	if (pts == AV_NOPTS_VALUE)
		PrintStreamData(avpkt->data + avpkt->size -10, 1);
	else
		PrintStreamData(avpkt->data + size -10, 0);*/

	avpkt->size += size;
}

/**
**	Reset current packet.
**
**	@param stream	video stream
*/
static void VideoResetPacket(VideoStream * stream)
{
    AVPacket *avpkt;

    stream->StartCodeState = 0;		// reset start code state

    stream->CodecIDRb[stream->PacketWrite] = AV_CODEC_ID_NONE;
    avpkt = &stream->PacketRb[stream->PacketWrite];
	avpkt->size = 0;
    avpkt->pts = AV_NOPTS_VALUE;
    avpkt->dts = AV_NOPTS_VALUE;
}

/**
**	Finish current packet advance to next.
**
**	@param stream	video stream
**	@param codec_id	codec id of packet (MPEG/H264)
*/
static void VideoNextPacket(VideoStream * stream, int codec_id)
{
    AVPacket *avpkt;

    avpkt = &stream->PacketRb[stream->PacketWrite];
    if (!avpkt->size) {		// ignore empty packets
		if (codec_id != AV_CODEC_ID_NONE) {
			return;
		}
		Debug(3, "video: possible stream change loss\n");
    }

    if (atomic_read(&stream->PacketsFilled) >= VIDEO_PACKET_MAX - 1) {
		// no free slot available drop last packet
		Error(_("[VideoNextPacket] no empty slot in packet ringbuffer\n"));
		avpkt->size = 0;
		if (codec_id == AV_CODEC_ID_NONE) {
			Debug(3, "[VideoNextPacket] possible stream change loss\n");
		}
		return;
    }
    // clear area for decoder, always enough space allocated
    memset(avpkt->data + avpkt->size, 0, AV_INPUT_BUFFER_PADDING_SIZE);
//	memset(avpkt->data, 0, avpkt->buf->size);

    stream->CodecIDRb[stream->PacketWrite] = codec_id;
    //DumpH264(avpkt->data, avpkt->size);

    // advance packet write
    stream->PacketWrite = (stream->PacketWrite + 1) % VIDEO_PACKET_MAX;
    atomic_inc(&stream->PacketsFilled);

    // intialize next package to use
    VideoResetPacket(stream);
}


/**
**	Place mpeg video data in packet ringbuffer.
**
**	Some tv-stations sends mulitple pictures in a single PES packet.
**	Split the packet into single picture packets.
**	Nick/CC, Viva, MediaShop, Deutsches Music Fernsehen
**
**	FIXME: this code can be written much faster
**
**	@param stream	video stream
**	@param pts	presentation timestamp of pes packet
**	@param data	data of pes packet
**	@param size	size of pes packet
*/
static void VideoMpegEnqueue(VideoStream * stream, int64_t pts,
    const uint8_t * data, int size)
{
    static const char startcode[3] = { 0x00, 0x00, 0x01 };
    const uint8_t *p;
    int n;
    int first;

    // first scan
    first = !stream->PacketRb[stream->PacketWrite].stream_index;
    p = data;
    n = size;

#ifdef DEBUG
    if (n < 4) {
		// is a problem with the pes start code detection
		Error(_("[softhddev] too short PES video packet\n"));
		fprintf(stderr, "[softhddev] too short PES video packet\n");
    }
#endif

    switch (stream->StartCodeState) {	// prefix starting in last packet
	case 3:			// 0x00 0x00 0x01 seen
#ifdef DEBUG
	    fprintf(stderr, "last: %d\n", stream->StartCodeState);
#endif
	    if (!p[0] || p[0] == 0xb3) {
#ifdef DEBUG
		fprintf(stderr, "last: %d start\n", stream->StartCodeState);
#endif
		stream->PacketRb[stream->PacketWrite].stream_index -= 3;
		VideoNextPacket(stream, AV_CODEC_ID_MPEG2VIDEO);
		VideoEnqueue(stream, pts, startcode, 3);
		first = p[0] == 0xb3;
		p++;
		n--;
		pts = AV_NOPTS_VALUE;
	    }
	    break;
	case 2:			// 0x00 0x00 seen
#ifdef DEBUG
	    fprintf(stderr, "last: %d\n", stream->StartCodeState);
#endif
	    if (p[0] == 0x01 && (!p[1] || p[1] == 0xb3)) {
#ifdef DEBUG
		fprintf(stderr, "last: %d start\n", stream->StartCodeState);
#endif
		stream->PacketRb[stream->PacketWrite].stream_index -= 2;
		VideoNextPacket(stream, AV_CODEC_ID_MPEG2VIDEO);
		VideoEnqueue(stream, pts, startcode, 2);
		first = p[1] == 0xb3;
		p += 2;
		n -= 2;
		pts = AV_NOPTS_VALUE;
	    }
	    break;
	case 1:			// 0x00 seen
#ifdef DEBUG
	    fprintf(stderr, "last: %d\n", stream->StartCodeState);
#endif
	    if (!p[0] && p[1] == 0x01 && (!p[2] || p[2] == 0xb3)) {
#ifdef DEBUG
		fprintf(stderr, "last: %d start\n", stream->StartCodeState);
#endif
		stream->PacketRb[stream->PacketWrite].stream_index -= 1;
		VideoNextPacket(stream, AV_CODEC_ID_MPEG2VIDEO);
		VideoEnqueue(stream, pts, startcode, 1);
		first = p[2] == 0xb3;
		p += 3;
		n -= 3;
		pts = AV_NOPTS_VALUE;
	    }
	case 0:
	    break;
    }

    // b3 b4 b8 00 b5 ... 00 b5 ...

    while (n > 3) {
	if (0 && !p[0] && !p[1] && p[2] == 0x01) {
	    fprintf(stderr, " %02x", p[3]);
	}
	// scan for picture header 0x00000100
	// FIXME: not perfect, must split at 0xb3 also
	if (!p[0] && !p[1] && p[2] == 0x01 && !p[3]) {
	    if (first) {
		first = 0;
		n -= 4;
		p += 4;
		continue;
	    }
	    // packet has already an picture header
	    /*
	       fprintf(stderr, "\nfix:%9d,%02x%02x%02x %02x ", n,
	       p[0], p[1], p[2], p[3]);
	     */
	    // first packet goes only upto picture header
	    VideoEnqueue(stream, pts, data, p - data);
	    VideoNextPacket(stream, AV_CODEC_ID_MPEG2VIDEO);
#ifdef DEBUG
	    fprintf(stderr, "fix\r");
#endif
	    data = p;
	    size = n;

	    // time-stamp only valid for first packet
	    pts = AV_NOPTS_VALUE;
	    n -= 4;
	    p += 4;
	    continue;
	}
	--n;
	++p;
    }

    stream->StartCodeState = 0;
    switch (n) {			// handle packet border start code
	case 3:
	    if (!p[0] && !p[1] && p[2] == 0x01) {
		stream->StartCodeState = 3;
	    }
	    break;
	case 2:
	    if (!p[0] && !p[1]) {
		stream->StartCodeState = 2;
	    }
	    break;
	case 1:
	    if (!p[0]) {
		stream->StartCodeState = 1;
	    }
	    break;
	case 0:
	    break;
    }
    VideoEnqueue(stream, pts, data, size);
}


/**
**	Close video stream.
**
**	@param stream	video stream
**
*/
static void VideoStreamClose(VideoStream * stream)
{
//	fprintf(stderr, "VideoStreamClose:\n");

    stream->SkipStream = 1;
    if (stream->Decoder) {
		VideoDecoder *decoder;

		decoder = stream->Decoder;
		// FIXME: remove this lock for main stream close

		pthread_mutex_lock(&stream->DecoderLockMutex);
		CodecVideoClose(decoder);
		stream->Decoder = NULL;		// lock read thread
		pthread_mutex_unlock(&stream->DecoderLockMutex);

		CodecVideoDelDecoder(decoder);
    }
    if (stream->Render) {
		VideoDelRender(stream->Render);
		stream->Render = NULL;
	}
    VideoPacketExit(stream);

    stream->NewStream = 1;
    stream->InvalidPesCounter = 0;
}

/**
**	Decode from PES packet ringbuffer.
**
**	@param stream	video stream
**
**	@retval 0	packet decoded
**	@retval	1	stream paused
**	@retval	-1	empty stream
*/
int VideoDecodeInput(VideoStream * stream)
{
    int filled;
    AVPacket *avpkt;
    int ret = 0;

    if (!stream->Decoder) {		// closing
		fprintf(stderr, "VideoDecodeInput: no decoder!!!\n");
	return -1;
    }

    if (stream->ClearBuffers) {		// clear buffer request
//		fprintf(stderr, "VideoDecodeInput: ClearBuffers stream request\n");
		atomic_set(&stream->PacketsFilled, 0);
		stream->PacketRead = stream->PacketWrite;
		// FIXME: ->Decoder already checked
		if (stream->Decoder) {
			CodecVideoFlushBuffers(stream->Decoder);
			VideoResetStart(stream->Render);
		}
		stream->ClearBuffers = 0;
		return 1;
    }
    if (stream->Freezed) {		// stream freezed
		// clear is called during freezed
		return 1;
    }

    filled = atomic_read(&stream->PacketsFilled);
    if (!filled) {
//		fprintf(stderr, "VideoDecodeInput: Nix im Speicher zum decodieren\n");
		return -1;
    }

    //
    //	handle queued commands
    //
    avpkt = &stream->PacketRb[stream->PacketRead];
    switch (stream->CodecIDRb[stream->PacketRead]) {
	case AV_CODEC_ID_NONE:
	    stream->ClosingStream = 0;
	    ret = 1;
	    if (stream->LastCodecID != AV_CODEC_ID_NONE) {
//			VideoSetClosing(stream->Render, 0);
			stream->LastCodecID = AV_CODEC_ID_NONE;
			CodecVideoClose(stream->Decoder);
//			fprintf(stderr, "VideoDecodeInput: AV_CODEC_ID_NONE\n");
			goto skip;
	    }
	    // FIXME: look if more close are in the queue
	    // size can be zero
	    goto skip;
	case AV_CODEC_ID_MPEG2VIDEO:
	    if (stream->LastCodecID != AV_CODEC_ID_MPEG2VIDEO) {
			stream->LastCodecID = AV_CODEC_ID_MPEG2VIDEO;
			CodecVideoOpen(stream->Decoder, AV_CODEC_ID_MPEG2VIDEO);
//			fprintf(stderr, "VideoDecodeInput: AV_CODEC_ID_MPEG2VIDEO\n");
	    }
	    break;
	case AV_CODEC_ID_H264:
	    if (stream->LastCodecID != AV_CODEC_ID_H264) {
			stream->LastCodecID = AV_CODEC_ID_H264;
			CodecVideoOpen(stream->Decoder, AV_CODEC_ID_H264);
//			fprintf(stderr, "VideoDecodeInput: AV_CODEC_ID_H264\n");
	    }
	    break;
	case AV_CODEC_ID_HEVC:
	    if (stream->LastCodecID != AV_CODEC_ID_HEVC) {
			stream->LastCodecID = AV_CODEC_ID_HEVC;
			CodecVideoOpen(stream->Decoder, AV_CODEC_ID_HEVC);
//			fprintf(stderr, "VideoDecodeInput: AV_CODEC_ID_HEVC\n");
	    }
	    break;
	default:
	    break;
    }

    //DumpMpeg(avpkt->data, avpkt->size);
#ifdef STILL_DEBUG
    if (InStillPicture) {
		DumpMpeg(avpkt->data, avpkt->size);
    }
#endif
    // lock decoder against close
    pthread_mutex_lock(&stream->DecoderLockMutex);
	ret = CodecVideoDecode(stream->Decoder, avpkt);
    pthread_mutex_unlock(&stream->DecoderLockMutex);

  skip:
    // advance packet read
    if (ret) {
        stream->PacketRead = (stream->PacketRead + 1) % VIDEO_PACKET_MAX;
        atomic_dec(&stream->PacketsFilled);
	}

    return 0;
}

/**
**	Get number of video buffers.
**
**	@param stream	video stream
*/
int VideoGetPackets(const VideoStream * stream)
{
    return atomic_read(&stream->PacketsFilled);
}


#ifdef DEBUG

/**
**	Dump mpeg video packet.
**
**	Function to dump a mpeg packet, not needed.
*/
static void DumpMpeg(const uint8_t * data, int size)
{
    fprintf(stderr, "%8d: ", size);

    // b3 b4 b8 00 b5 ... 00 b5 ...

    while (size > 3) {
	if (!data[0] && !data[1] && data[2] == 0x01) {
	    fprintf(stderr, " %02x", data[3]);
	    size -= 4;
	    data += 4;
	    continue;
	}
	--size;
	++data;
    }
    fprintf(stderr, "\n");
}

/**
**	Dump h264 video packet.
**
**	Function to Dump a h264 packet, not needed.
*/
static int DumpH264(const uint8_t * data, int size)
{
    printf("H264:");
    do {
	if (size < 4) {
	    printf("\n");
	    return -1;
	}
	if (!data[0] && !data[1] && data[2] == 0x01) {
	    printf("%02x ", data[3]);
	}
	++data;
	--size;
    } while (size);
    printf("\n");

    return 0;
}

/**
**	Validate mpeg video packet.
**
**	Function to validate a mpeg packet, not needed.
*/
static int ValidateMpeg(const uint8_t * data, int size)
{
    int pes_l;

    do {
	if (size < 9) {
	    return -1;
	}
	if (data[0] || data[1] || data[2] != 0x01) {
	    printf("%02x: %02x %02x %02x %02x %02x\n", data[-1], data[0],
		data[1], data[2], data[3], data[4]);
	    return -1;
	}

	pes_l = (data[4] << 8) | data[5];
	if (!pes_l) {			// contains unknown length
	    return 1;
	}

	if (6 + pes_l > size) {
	    return -1;
	}

	data += 6 + pes_l;
	size -= 6 + pes_l;
    } while (size);

    return 0;
}
#endif

/**
**	Play video packet.
**
**	@param data		data of exactly one complete PES packet
**	@param size		size of PES packet
**
**	@return number of bytes used, 0 if internal buffer are full.
**
**	@note vdr sends incomplete packets, va-api h264 decoder only
**	supports complete packets.
**	We buffer here until we receive an complete PES Packet, which
**	is no problem, the audio is always far behind us.
**	cTsToPes::GetPes splits the packets.
**
**	@todo FIXME: combine the 5 ifs at start of the function
*/
int PlayVideo(const uint8_t * data, int size)
{
	VideoStream * stream = MyVideoStream;
    const uint8_t *check;
    int64_t pts;
    int n;
    int z;
    int l;

	if (!stream->Decoder) {
		return size;
	}
	if (stream->SkipStream) {		// skip video stream
		return size;
	}
	if (stream->Freezed) {		// stream freezed
		return 0;
	}
	if (stream->NewStream) {		// channel switched
		Debug(3, "video: new stream %dms\n", GetMsTicks() - VideoSwitch);
		if (atomic_read(&stream->PacketsFilled) >= VIDEO_PACKET_MAX - 1) {
			Debug(3, "video: new video stream lost\n");
			return 0;
		}
		VideoNextPacket(stream, AV_CODEC_ID_NONE);
		stream->CodecID = AV_CODEC_ID_NONE;
		stream->ClosingStream = 1;
		stream->NewStream = 0;
	}
    // must be a PES start code
    // FIXME: Valgrind-3.8.1 has a problem with this code
    if (size < 9 || !data || data[0] || data[1] || data[2] != 0x01) {
		if (!stream->InvalidPesCounter++) {
			Error(_("[softhddev] invalid PES video packet\n"));
			fprintf(stderr, "PlayVideo: invalid PES video packet %d  %02x %02x %02x\n",
				size, data[0], data[1], data[2]);
		}
		return size;
    }
    if (stream->InvalidPesCounter) {
		if (stream->InvalidPesCounter > 1) {
			Error(_("[softhddev] %d invalid PES video packet(s)\n"),
				stream->InvalidPesCounter);
		}
		stream->InvalidPesCounter = 0;
    }
    // 0xBE, filler, padding stream
    if (data[3] == PES_PADDING_STREAM) {	// from DVD plugin
		return size;
    }

    n = data[8];			// header size
    if (size <= 9 + n) {		// wrong size
		if (size == 9 + n) {
			Warning(_("[softhddev] empty video packet\n"));
		} else {
			Error(_("[softhddev] invalid video packet %d/%d bytes\n"),
				9 + n, size);
		}
		return size;
    }

    // hard limit buffer full: needed for replay
    if (atomic_read(&stream->PacketsFilled) >= VIDEO_PACKET_MAX - 10) {
//		fprintf(stderr, "PlayVideo: buffer full\n");
		return 0;
    }
    // get pts/dts
    pts = AV_NOPTS_VALUE;
    if (data[7] & 0x80) {
		pts = (int64_t) (data[9] & 0x0E) << 29 | data[10] << 22 | (data[11] &
			0xFE) << 14 | data[12] << 7 | (data[13] & 0xFE) >> 1;
    }

    check = data + 9 + n;
    l = size - 9 - n;
    z = 0;
    while (!*check) {			// count leading zeros
		if (l < 3) {
			Warning(_("[softhddev] empty video packet %d bytes\n"), size);
			z = 0;
			break;
		}
		--l;
		++check;
		++z;
    }

    // H264 NAL AUD Access Unit Delimiter (0x00) 0x00 0x00 0x01 0x09
    // and next start code
    if ((data[6] & 0xC0) == 0x80 && z >= 2 && check[0] == 0x01
		&& check[1] == 0x09 && !check[3] && !check[4]) {
		// old PES HDTV recording z == 2 -> stronger check!
		if (stream->CodecID == AV_CODEC_ID_H264) {
			// this should improve ffwd+frew, but produce crash in ffmpeg
			// with some streams
			if (stream->TrickSpeed && pts != (int64_t) AV_NOPTS_VALUE) {
				// H264 NAL End of Sequence
				static uint8_t seq_end_h264[] =
				{ 0x00, 0x00, 0x00, 0x01, 0x0A };

				// 1-5=SLICE 6=SEI 7=SPS 8=PPS
				// NAL SPS sequence parameter set
				if ((check[7] & 0x1F) == 0x07) {
					VideoNextPacket(stream, AV_CODEC_ID_H264);
					VideoEnqueue(stream, AV_NOPTS_VALUE, seq_end_h264,
					sizeof(seq_end_h264));
				}
			}
			VideoNextPacket(stream, AV_CODEC_ID_H264);
		} else {
			if (check[2] == 0x10) {
//				fprintf(stderr, "video: h264 detected\n");
				Debug(3, "video: h264 detected\n");
				stream->CodecID = AV_CODEC_ID_H264;
			} else {
				return size;
			}
		}

		// SKIP PES header (ffmpeg supports short start code)
		VideoEnqueue(stream, pts, check - 2, l + 2);
		return size;
    }

    // HEVC Codec
    if ((data[6] & 0xC0) == 0x80 && z >= 2 && check[0] == 0x01 && check[1] == 0x46) {
		// old PES HDTV recording z == 2 -> stronger check!

		if (stream->CodecID == AV_CODEC_ID_HEVC) {
			VideoNextPacket(stream, AV_CODEC_ID_HEVC);
		} else {
//            fprintf(stderr, "video: hevc detected\n");
            Debug(3, "video: hevc detected\n");
            stream->CodecID = AV_CODEC_ID_HEVC;
		}
		// SKIP PES header (ffmpeg supports short start code)
		VideoEnqueue(stream, pts, check - 2, l + 2);
		return size;
    }

    // PES start code 0x00 0x00 0x01 0x00|0xb3
    if (z > 1 && check[0] == 0x01 && (!check[1] || check[1] == 0xb3)) {
		if (stream->CodecID == AV_CODEC_ID_MPEG2VIDEO) {
			VideoNextPacket(stream, AV_CODEC_ID_MPEG2VIDEO);
		} else {
			if (check[1] == 0xb3) {
//				fprintf(stderr, "video: mpeg2 detected ID %02x\n", check[3]);
				Debug(3, "video: mpeg2 detected ID %02x\n", check[3]);
				stream->CodecID = AV_CODEC_ID_MPEG2VIDEO;
			} else {
				return size;
			}
		}
#ifdef noDEBUG				// pip pes packet has no lenght
		if (ValidateMpeg(data, size)) {
			Debug(3, "softhddev/video: invalid mpeg2 video packet\n");
		}
#endif
		// SKIP PES header, begin of start code
		VideoEnqueue(stream, pts, check - 2, l + 2);
		return size;
	}
	// this happens when vdr sends incomplete packets
	if (stream->CodecID == AV_CODEC_ID_NONE) {
		Debug(3, "video: not detected\n");
		return size;
	}

	// SKIP PES header
	VideoEnqueue(stream, pts, data + 9 + n, size - 9 - n);

	return size;
}


/**
**	Display the given I-frame as a still picture.
**
**	@param data	pes frame data
**	@param size	number of bytes in frame
*/
void StillPicture(const uint8_t * data, int size)
{
    static uint8_t seq_end_mpeg[] = { 0x00, 0x00, 0x01, 0xB7 };
    // H264 NAL End of Sequence
    static uint8_t seq_end_h264[] = { 0x00, 0x00, 0x00, 0x01, 0x0A };
    // H265 NAL End of Sequence
    //0x48 = end of seq   0x4a = end of stream
    static uint8_t seq_end_h265[] = { 0x00, 0x00, 0x00, 0x01, 0x48, 0x01 };
    int old_video_hardware_decoder;
    int i;

    // might be called in Suspended Mode
    if (!MyVideoStream->Decoder || MyVideoStream->SkipStream) {
	return;
    }
    // must be a PES start code
    if (size < 9 || !data || data[0] || data[1] || data[2] != 0x01) {
	Error(_("[softhddev] invalid still video packet\n"));
	return;
    }
#ifdef STILL_DEBUG
    InStillPicture = 1;
#endif
    VideoSetTrickSpeed(MyVideoStream->Render, 1);
    VideoResetPacket(MyVideoStream);
    old_video_hardware_decoder = VideoHardwareDecoder;
    // enable/disable hardware decoder for still picture
    if (VideoHardwareDecoder != ConfigStillDecoder) {
	VideoHardwareDecoder = ConfigStillDecoder;
	VideoNextPacket(MyVideoStream, AV_CODEC_ID_NONE);	// close last stream
    }

    if (MyVideoStream->CodecID == AV_CODEC_ID_NONE) {
	// FIXME: should detect codec, see PlayVideo
	Error(_("[softhddev] no codec known for still picture\n"));
    }
    // FIXME: can check video backend, if a frame was produced.
    // output for max reference frames
#ifdef STILL_DEBUG
    fprintf(stderr, "still-picture\n");
#endif

//int reps;
//    switch(MyVideoStream->CodecID) {
//        case AV_CODEC_ID_MPEG2VIDEO:
//            reps = 20;
//            break;

    for (i = 0; i < (MyVideoStream->CodecID == AV_CODEC_ID_MPEG2VIDEO ? 4 : 25);
//    for (i = 0; i < (MyVideoStream->CodecID == AV_CODEC_ID_HEVC ? 3 : 4);
	++i) {
	const uint8_t *split;
	int n;

	// FIXME: vdr pes recordings sends mixed audio/video
	if ((data[3] & 0xF0) == 0xE0) {	// PES packet
	    split = data;
	    n = size;
	    // split the I-frame into single pes packets
	    do {
		int len;

#ifdef DEBUG
		if (split[0] || split[1] || split[2] != 0x01) {
		    Error(_("[softhddev] invalid still video packet\n"));
		    break;
		}
#endif

		len = (split[4] << 8) + split[5];
		if (!len || len + 6 > n) {
		    if ((split[3] & 0xF0) == 0xE0) {
			// video only
			while (!PlayVideo(split, n)) {	// feed remaining bytes
			}
		    }
		    break;
		}
		if ((split[3] & 0xF0) == 0xE0) {
		    // video only
		    while (!PlayVideo(split, len + 6)) {	// feed it
		    }
		}
		split += 6 + len;
		n -= 6 + len;
	    } while (n > 6);

	    VideoNextPacket(MyVideoStream, MyVideoStream->CodecID);	// terminate last packet
	} else {			// ES packet
	    if (MyVideoStream->CodecID != AV_CODEC_ID_MPEG2VIDEO) {
		VideoNextPacket(MyVideoStream, AV_CODEC_ID_NONE);	// close last stream
		MyVideoStream->CodecID = AV_CODEC_ID_MPEG2VIDEO;
	    }
	    VideoEnqueue(MyVideoStream, AV_NOPTS_VALUE, data, size);
	}
	if (MyVideoStream->CodecID == AV_CODEC_ID_H264) {
	    VideoEnqueue(MyVideoStream, AV_NOPTS_VALUE, seq_end_h264,
		sizeof(seq_end_h264));
	} else if (MyVideoStream->CodecID == AV_CODEC_ID_HEVC) {
	    VideoEnqueue(MyVideoStream, AV_NOPTS_VALUE, seq_end_h265,
		sizeof(seq_end_h265));
	} else {
	    VideoEnqueue(MyVideoStream, AV_NOPTS_VALUE, seq_end_mpeg,
		sizeof(seq_end_mpeg));
	}
	VideoNextPacket(MyVideoStream, MyVideoStream->CodecID);	// terminate last packet
    }

    // wait for empty buffers
    for (i = 0; VideoGetPackets(MyVideoStream) && i < 30; ++i) {
	usleep(10 * 1000);
    }
    Debug(3, "[softhddev]%s: buffers %d %dms\n", __FUNCTION__,
	VideoGetPackets(MyVideoStream), i * 10);
#ifdef STILL_DEBUG
    InStillPicture = 0;
#endif
    if (VideoHardwareDecoder != old_video_hardware_decoder) {
	VideoHardwareDecoder = old_video_hardware_decoder;
	VideoNextPacket(MyVideoStream, AV_CODEC_ID_NONE);	// close last stream
    }
    VideoSetTrickSpeed(MyVideoStream->Render, 0);
}


    /// call VDR support function
extern uint8_t *CreateJpeg(uint8_t *, int *, int, int, int);

#if defined(USE_JPEG) && JPEG_LIB_VERSION >= 80

/**
**	Create a jpeg image in memory.
**
**	@param image		raw RGB image
**	@param raw_size		size of raw image
**	@param size[out]	size of jpeg image
**	@param quality		jpeg quality
**	@param width		number of horizontal pixels in image
**	@param height		number of vertical pixels in image
**
**	@returns allocated jpeg image.
*/
uint8_t *CreateJpeg(uint8_t * image, int raw_size, int *size, int quality,
    int width, int height)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_ptr[1];
    int row_stride;
    uint8_t *outbuf;
    long unsigned int outsize;

    outbuf = NULL;
    outsize = 0;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_mem_dest(&cinfo, &outbuf, &outsize);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = raw_size / height / width;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    row_stride = width * 3;
    while (cinfo.next_scanline < cinfo.image_height) {
	row_ptr[0] = &image[cinfo.next_scanline * row_stride];
	jpeg_write_scanlines(&cinfo, row_ptr, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    *size = outsize;

    return outbuf;
}

#endif

/**
**	Grabs the currently visible screen image.
**
**	@param size	size of the returned data
**	@param jpeg	flag true, create JPEG data
**	@param quality	JPEG quality
**	@param width	number of horizontal pixels in the frame
**	@param height	number of vertical pixels in the frame
*/
uint8_t *GrabImage(int *size, int jpeg, int quality, int width, int height)
{
    if (jpeg) {
	uint8_t *image;
	int raw_size;

	raw_size = 0;
	image = VideoGrab(&raw_size, &width, &height, 0);
	if (image) {			// can fail, suspended, ...
	    uint8_t *jpg_image;

	    jpg_image = CreateJpeg(image, size, quality, width, height);

	    free(image);
	    return jpg_image;
	}
	return NULL;
    }
    return VideoGrab(size, &width, &height, 1);
}

//////////////////////////////////////////////////////////////////////////////

/**
**	Set trick play speed.
**
**	Every single frame shall then be displayed the given number of
**	times.
**
**	@param speed	trick speed
*/
void TrickSpeed(int speed)
{
	fprintf(stderr, "TrickSpeed: speed %d\n", speed);

	MyVideoStream->TrickSpeed = speed;
	if (MyVideoStream->Render) {
		VideoSetTrickSpeed(MyVideoStream->Render, speed);
	} else {
		// can happen, during startup
		Debug(3, "softhddev: %s called without hw decoder\n", __FUNCTION__);
	}
	StreamFreezed = 0;
	MyVideoStream->Freezed = 0;
}

/**
**	Clears all video and audio data from the device.
*/
void Clear(void)
{
	int i;

//	fprintf(stderr, "Clear(void)\n");

	VideoResetPacket(MyVideoStream);	// terminate work
	MyVideoStream->ClearBuffers = 1;
	if (!SkipAudio) {
		AudioFlushBuffers();
		//NewAudioStream = 1;
	}
	// FIXME: audio avcodec_flush_buffers, video is done by VideoClearBuffers

	// wait for empty buffers
	// FIXME: without softstart sync VideoDecode isn't called.
	for (i = 0; MyVideoStream->ClearBuffers && i < 20; ++i) {
		usleep(1 * 1000);
	}
	Debug(3, "[softhddev]%s: %dms buffers %d\n", __FUNCTION__, i,
	VideoGetPackets(MyVideoStream));
}

/**
**	Sets the device into play mode.
*/
void Play(void)
{
	fprintf(stderr, "Play(void)\n");

	TrickSpeed(0);			// normal play
	SkipAudio = 0;
	AudioPlay();
}

/**
**	Sets the device into "freeze frame" mode.
*/
void Freeze(void)
{
	fprintf(stderr, "Freeze(void)\n");

	StreamFreezed = 1;
	MyVideoStream->Freezed = 1;
	AudioPause();
}

/**
**	Turns off audio while replaying.
*/
void Mute(void)
{
	fprintf(stderr, "Mute(void)\n");

	SkipAudio = 1;
	AudioFlushBuffers();
    //AudioSetVolume(0);
}

/**
**	Poll if device is ready.  Called by replay.
**
**	This function is useless, the return value is ignored and
**	all buffers are overrun by vdr.
**
**	The dvd plugin is using this correct.
**
**	@param timeout	timeout to become ready in ms
**
**	@retval true	if ready
**	@retval false	if busy
*/
int Poll(int timeout)
{
    // poll is only called during replay, flush buffers after replay
    for (;;) {
	int full;
	int t;
	int used;
	int filled;

//	fprintf(stderr, "Poll: timeout %d\n", timeout);

	used = AudioUsedBytes();
	// FIXME: no video!
	filled = atomic_read(&MyVideoStream->PacketsFilled);
	// soft limit + hard limit
	full = (used > AUDIO_MIN_BUFFER_FREE && filled > 3)
	    || AudioFreeBytes() < AUDIO_MIN_BUFFER_FREE
	    || filled >= VIDEO_PACKET_MAX - 10;

	if (!full || !timeout) {
	    return !full;
	}

	t = 15;
	if (timeout < t) {
	    t = timeout;
	}
	usleep(t * 1000);		// let display thread work
	timeout -= t;
    }
}

/**
**	Flush the device output buffers.
**
**	@param timeout	timeout to flush in ms
*/
int Flush(int timeout)
{
	fprintf(stderr, "Flush: timeout %d\n", timeout);

	if (atomic_read(&MyVideoStream->PacketsFilled)) {
		if (timeout) {			// let display thread work
			usleep(timeout * 1000);
		}
		return !atomic_read(&MyVideoStream->PacketsFilled);
	}
	return 1;
}

#ifndef MMAL
void SetSWDeinterlacer(int deint)
{
	VideoSetSWDeinterlacer(MyVideoStream->Render ,deint);
}
#endif

void GetScreenSize(int *width, int *height, double *pixel_aspect)
{
	VideoGetScreenSize(MyVideoStream->Render, width, height, pixel_aspect);
}

/**
**	Set play mode, called on channel switch.
**
**	@param play_mode	play mode (none, video+audio, audio-only, ...)
*/
int SetPlayMode(int play_mode)
{
//	fprintf(stderr, "SetPlayMode: %d\n", play_mode);

	switch (play_mode) {
	case 0:			// audio/video from decoder
	    // tell video parser we get new stream
	    if (MyVideoStream->Decoder && !MyVideoStream->SkipStream) {
			// clear buffers on close configured always or replay only
		    Clear();		// flush all buffers
		}
		if (MyVideoStream->CodecID != AV_CODEC_ID_NONE) {
			MyVideoStream->NewStream = 1;
			MyVideoStream->InvalidPesCounter = 0;
			// tell render we are closing stream
			VideoSetClosing(MyVideoStream->Render, 1);
			VideoResetStart(MyVideoStream->Render);
#ifdef DEBUG
			VideoSwitch = GetMsTicks();
			Debug(3, "video: new stream start\n");
#endif
	    }

	    if (MyAudioDecoder) {	// tell audio parser we have new stream
		if (AudioCodecID != AV_CODEC_ID_NONE) {
		    NewAudioStream = 1;
		}
	    }
	    break;
	case 1:			// audio/video from player
	    VideoThreadWakeup(MyVideoStream->Render);
		//Play(); Play is a vdr command!!!
	    break;
	case 2:			// audio only from player, video from decoder
		VideoThreadExit();
		break;
	case 3:			// audio only from player, no video (black screen)
	    Debug(3, "softhddev: FIXME: audio only, silence video errors\n");
	    VideoThreadWakeup(MyVideoStream->Render);
	    //Play();
	    break;
	case 4:			// video only from player, audio from decoder
	    VideoThreadWakeup(MyVideoStream->Render);
	    //Play();
	    break;
    }
    return 1;
}

//////////////////////////////////////////////////////////////////////////////
//	Init/Exit
//////////////////////////////////////////////////////////////////////////////

/**
**	Exit + cleanup.
*/
void SoftHdDeviceExit(void)
{
//	fprintf(stderr, "SoftHdDeviceExit(void):\n");
    // lets hope that vdr does a good thread cleanup

    AudioExit();
    if (MyAudioDecoder) {
	CodecAudioClose(MyAudioDecoder);
	CodecAudioDelDecoder(MyAudioDecoder);
	MyAudioDecoder = NULL;
    }
    NewAudioStream = 0;
    av_packet_unref(AudioAvPkt);

    VideoExit(MyVideoStream->Render);
    VideoStreamClose(MyVideoStream);

    CodecExit();

    pthread_mutex_destroy(&SuspendLockMutex);
    pthread_mutex_destroy(&MyVideoStream->DecoderLockMutex);
}


/**
**	Prepare plugin.
**
**	@retval 0	normal start
**	@retval 1	suspended start
**	@retval -1	detached start
*/
int Start(void)
{
//	fprintf(stderr, "Start(void):\n");

	pthread_mutex_init(&MyVideoStream->DecoderLockMutex, NULL);
	pthread_mutex_init(&SuspendLockMutex, NULL);

	AudioInit();
	av_new_packet(AudioAvPkt, AUDIO_BUFFER_SIZE);
	MyAudioDecoder = CodecAudioNewDecoder();
	AudioCodecID = AV_CODEC_ID_NONE;
	AudioChannelID = -1;

	CodecInit();
	if (!MyVideoStream->Decoder) {
		MyVideoStream->SkipStream = 1;
		MyVideoStream->CodecID = AV_CODEC_ID_NONE;
		MyVideoStream->LastCodecID = AV_CODEC_ID_NONE;
	}

	if ((MyVideoStream->Render = VideoNewRender(MyVideoStream))) {
		VideoInit(MyVideoStream->Render);
		MyVideoStream->Decoder = CodecVideoNewDecoder(MyVideoStream->Render);
		VideoPacketInit(MyVideoStream);
		MyVideoStream->SkipStream = 0;
	}

	return 0;
}


/**
**	Stop plugin.
**
**	@note stop everything, but don't cleanup, module is still called.
*/
void Stop(void)
{
//	fprintf(stderr, "Stop(void): nothing to do.\n");
}


/**
**	Get decoder statistics.
**
**	@param[out] missed	missed frames
**	@param[out] duped	duped frames
**	@param[out] dropped	dropped frames
**	@param[out] count	number of decoded frames
*/
void GetStats(int *missed, int *duped, int *dropped, int *counter)
{
	*missed = 0;
	*duped = 0;
	*dropped = 0;
	*counter = 0;
	if (MyVideoStream->Render) {
		VideoGetStats(MyVideoStream->Render, missed, duped, dropped,
			counter);
	}
}


//////////////////////////////////////////////////////////////////////////////
//	OSD
//////////////////////////////////////////////////////////////////////////////

/**
**	Close OSD.
*/
void OsdClose(void)
{
    VideoOsdClear(MyVideoStream->Render);
}

/**
**	Draw an OSD pixmap.
**
**	@param xi	x-coordinate in argb image
**	@param yi	y-coordinate in argb image
**	@paran height	height in pixel in argb image
**	@paran width	width in pixel in argb image
**	@param pitch	pitch of argb image
**	@param argb	32bit ARGB image data
**	@param x	x-coordinate on screen of argb image
**	@param y	y-coordinate on screen of argb image
*/
void OsdDrawARGB(int xi, int yi, int height, int width, int pitch,
	const uint8_t * argb, int x, int y)
{
	VideoOsdDrawARGB(MyVideoStream->Render, xi, yi, height, width,
			pitch, argb, x, y);
}

//////////////////////////////////////////////////////////////////////////////

/**
**	Return command line help string.
*/
const char *CommandLineHelp(void)
{
    return "  -a device\taudio device (fe. alsa: hw:0,0)\n"
	"  -p device\taudio device for pass-through (hw:0,1)\n"
	"  -c channel\taudio mixer channel name (fe. PCM)\n"
	"  -s screen size\tset screen size  (hdr for 1280 x 720)\n"
	"  -w workaround\tenable/disable workarounds\n"
	"\talsa-driver-broken\tdisable broken alsa driver message\n"
	"\talsa-no-close-open\tdisable close open to fix alsa no sound bug\n"
	"\talsa-close-open-delay\tenable close open delay to fix no sound bug\n"
	"\tignore-repeat-pict\tdisable repeat pict message\n";
}

/**
**	Process the command line arguments.
**
**	@param argc	number of arguments
**	@param argv	arguments vector
*/
int ProcessArgs(int argc, char *const argv[])
{
    //
    //	Parse arguments.
    //

    for (;;) {
	switch (getopt(argc, argv, "-a:c:p:s:w:")) {
	    case 'a':			// audio device for pcm
		AudioSetDevice(optarg);
		continue;
	    case 'c':			// channel of audio mixer
		AudioSetChannel(optarg);
		continue;
	    case 'p':			// pass-through audio device
		AudioSetPassthroughDevice(optarg);
		continue;
	    case 's':			// screen size
		VideoSetScreenSize(optarg);
		continue;
	    case 'w':			// workarounds
		if (!strcasecmp("no-hw-decoder", optarg)) {
		    VideoHardwareDecoder = 0;
		} else if (!strcasecmp("no-mpeg-hw-decoder", optarg)) {
		    VideoHardwareDecoder = 1;
		    if (ConfigStillDecoder) {
			ConfigStillDecoder = 1;
		    }
		} else if (!strcasecmp("still-hw-decoder", optarg)) {
		    ConfigStillDecoder = -1;
		} else if (!strcasecmp("still-h264-hw-decoder", optarg)) {
		    ConfigStillDecoder = 1;
		} else if (!strcasecmp("alsa-driver-broken", optarg)) {
		    AudioAlsaDriverBroken = 1;
		} else if (!strcasecmp("alsa-no-close-open", optarg)) {
		    AudioAlsaNoCloseOpen = 1;
		} else if (!strcasecmp("alsa-close-open-delay", optarg)) {
		    AudioAlsaCloseOpenDelay = 1;
//		} else if (!strcasecmp("use-possible-defect-frames", optarg)) {
//		    CodecUsePossibleDefectFrames = 1;
		} else {
		    fprintf(stderr, _("Workaround '%s' unsupported\n"),
			optarg);
		    return 0;
		}
		continue;
	    case EOF:
		break;
	    case '-':
		fprintf(stderr, _("We need no long options\n"));
		return 0;
	    case ':':
		fprintf(stderr, _("Missing argument for option '%c'\n"),
		    optopt);
		return 0;
	    default:
		fprintf(stderr, _("Unknown option '%c'\n"), optopt);
		return 0;
	}
	break;
    }
    while (optind < argc) {
	fprintf(stderr, _("Unhandled argument '%s'\n"), argv[optind++]);
    }

    return 1;
}

