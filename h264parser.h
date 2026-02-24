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

#include <set>
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
		int list;
		int idc;
		int abs_diff_pic_num_minus1;
		int long_term_pic_num;
	};

public:
	cH264Parser(AVPacket *, int, int, int);
	int GetWidth(void) { return m_width; };
	int GetHeight(void) { return m_height; };
	bool IsIFrame(void);
	bool IsMbaff(void) { return m_mbaff; };
	bool HasSPS(void) { return m_hasSPS; };
	bool HasPPS(void) { return m_hasPPS; };
	void PrintNalUnits(void);
	std::string GetNalUnitString(void) { return m_naluString; };
	void PrintStreamData(void);

	bool IsPSlice() const { return m_sliceType == 0; }
	bool IsBSlice() const { return m_sliceType == 1; }
	bool IsIDR() const { return m_isIDR; }
	bool IsReference() const { return m_isReference; }
	int GetFrameNum() const { return m_frameNum; }
	const std::vector<RefPicMod>& GetRefMods() const { return m_refMods; }
	int GetLog2MaxFrameNumMinus4() const { return m_log2MaxFrameNumMinus4; }
	int GetPpsNumRefIdxL0DefaultActiveMinus1(void) { return m_ppsNumRefIdxL0DefaultActiveMinus1; };
	int GetPpsNumRefIdxL1DefaultActiveMinus1(void) { return m_ppsNumRefIdxL1DefaultActiveMinus1; };
	void PrintInvalidReference(void);
	void PrintValidReference(void);
	void AddFrameNumber(int);
	void AddInvalidReference(int);
	void AddValidReference(int);

private:
	AVPacket *m_pAvpkt;
	const unsigned char *m_pStart;
	unsigned short m_nLength;
	int m_nCurrentBit;

	int m_nalutype = 0;
	bool m_hasSPS = false;
	bool m_hasPPS = false;

	int m_width = 0;
	int m_height = 0;
	bool m_mbaff = false;
	std::string m_naluString;
	std::set<int> m_invalidReferences;
	std::set<int> m_validReferences;

	int  m_sliceType = -1;      // normalized: 0=P, 2=I
	int  m_frameNum = -1;
	int  m_nalRefIdc = 0;
	bool m_isIDR = false;
	bool m_isReference = false;
	int  m_log2MaxFrameNumMinus4 = -4; // from SPS
	std::vector<RefPicMod> m_refMods;
	bool m_hasInvalidReferences = false;
	bool m_hasValidReferences = false;

	int m_ppsNumRefIdxL0DefaultActiveMinus1 = -1; // from PPS
	int m_ppsNumRefIdxL1DefaultActiveMinus1 = -1; // from PPS

	unsigned int ReadBit(void);
	unsigned int ReadBits(int);
	unsigned int ReadExponentialGolombCode(void);
	unsigned int ReadSE(void);
	int GetSPSOffset(void);
	int GetPPSOffset(void);
	int GetSliceOffset(void);
};

#endif
