/**
 * @file h264parser.h
 * H264 parser header file
 *
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

#ifndef __H264PARSER_H
#define __H264PARSER_H

#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
}

typedef enum {
	NALU_TYPE_NON_IDR = (1 << 0),
	NALU_TYPE_PART_A  = (1 << 1),
	NALU_TYPE_PART_B  = (1 << 2),
	NALU_TYPE_PART_C  = (1 << 3),
	NALU_TYPE_IDR     = (1 << 4),
	NALU_TYPE_SEI     = (1 << 5),
	NALU_TYPE_SPS     = (1 << 6),
	NALU_TYPE_PPS     = (1 << 7),
	NALU_TYPE_AUD     = (1 << 8)
} NalUnitTypes;

/**
 * cH264Parser - H264 Parser class
 */
class cH264Parser
{
private:
	struct RefPicMod {
		int idc;
		int abs_diff_pic_num_minus1;
	};

public:
	cH264Parser(AVPacket *, int);
	int GetWidth(void) { return m_width; };
	int GetHeight(void) { return m_height; };
	bool IsIFrame(void);
	bool IsMbaff(void) { return m_mbaff; };
	bool HasSPS(void) { return m_hasSPS; };
	void PrintNalUnits(void);
	std::string GetNalUnitString(void) { return m_naluString; };
	void PrintStreamData(void);

	bool IsPSlice() const { return m_sliceType == 0; }
	bool IsIDR() const { return m_isIDR; }
	bool IsReference() const { return m_isReference; }
	int GetFrameNum() const { return m_frameNum; }
	bool HasRefListModification() const { return m_refListModFlagL0; }
	int GetNumRefIdxL0() const { return m_numRefIdxL0Active; }
	const std::vector<RefPicMod>& GetRefMods() const { return m_refMods; }
	int GetLog2MaxFrameNumMinus4() const { return m_log2MaxFrameNumMinus4; }
	void MarkInvalidReference(void);
	void AddFrameNumber(int);
	void AddInvalidReference(int);

private:
	AVPacket *m_pAvpkt;
	const unsigned char *m_pStart;
	unsigned short m_nLength;
	int m_nCurrentBit;

	int m_nalutype = 0;
	bool m_hasSPS = false;

	int m_width = 0;
	int m_height = 0;
	bool m_mbaff = false;
	std::string m_naluString;
	std::string m_invalidReferences;

	int  m_sliceType = -1;      // normalized: 0=P, 2=I
	int  m_frameNum = -1;
	int  m_nalRefIdc = 0;
	bool m_isIDR = false;
	bool m_isReference = false;
	bool m_refListModFlagL0 = false;
	int  m_numRefIdxL0Active = 1;
	int  m_log2MaxFrameNumMinus4 = 0; // from SPS
	std::vector<RefPicMod> m_refMods;
	bool m_hasInvalidReferences = false;

	unsigned int ReadBit(void);
	unsigned int ReadBits(int);
	unsigned int ReadExponentialGolombCode(void);
	unsigned int ReadSE(void);
	int GetSPSOffset(void);
	int GetSliceOffset(void);
};

#endif
