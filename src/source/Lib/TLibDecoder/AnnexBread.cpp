/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 \file     AnnexBread.cpp
 \brief    reading functions for Annex B byte streams
 */


#include <stdint.h>
#include <cassert>
#include <vector>
#include "AnnexBread.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "TLibCommon/TComCodingStatistics.h"
#endif

using namespace std;

//! \ingroup TLibDecoder
//! \{

/**
 * Parse an AVC AnnexB Bytestream bs to extract a single nalUnit
 * while accumulating bytestream statistics into stats.
 *
 * If EOF occurs while trying to extract a NALunit, an exception
 * of std::ios_base::failure is thrown.  The contsnts of stats will
 * be correct at this point.
 */
static Void
_byteStreamNALUnit(
  InputByteStream& bs,
  vector<uint8_t>& nalUnit,
  AnnexBStats& stats)
{
  /* At the beginning of the decoding process, the decoder initialises its
   * current position in the byte stream to the beginning of the byte stream.
   * It then extracts and discards each leading_zero_8bits syntax element (if
   * present), moving the current position in the byte stream forward one
   * byte at a time, until the current position in the byte stream is such
   * that the next four bytes in the bitstream form the four-byte sequence
   * 0x00000001.
   */
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  TComCodingStatistics::SStat &statBits=TComCodingStatistics::GetStatisticEP(STATS__NAL_UNIT_PACKING);
#endif
  while ((bs.eofBeforeNBytes(24/8) || bs.peekBytes(24/8) != 0x000001)
  &&     (bs.eofBeforeNBytes(32/8) || bs.peekBytes(32/8) != 0x00000001))
  {
    uint8_t leading_zero_8bits = bs.readByte();
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    statBits.bits+=8; statBits.count++;
#endif
    assert(leading_zero_8bits == 0);
    stats.m_numLeadingZero8BitsBytes++;
  }

  /* 1. When the next four bytes in the bitstream form the four-byte sequence
   * 0x00000001, the next byte in the byte stream (which is a zero_byte
   * syntax element) is extracted and discarded and the current position in
   * the byte stream is set equal to the position of the byte following this
   * discarded byte.
   */
  /* NB, the previous step guarantees this will succeed -- if EOF was
   * encountered, an exception will stop execution getting this far */
  if (bs.peekBytes(24/8) != 0x000001)
  {
    uint8_t zero_byte = bs.readByte();
#if RExt__DECODER_DEBUG_BIT_STATISTICS
