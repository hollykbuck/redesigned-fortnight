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

#include <stdint.h>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <cassert>

#define PRINT_NALUS 0

enum NalUnitType
{
  TRAIL_N = 0, // 0
  TRAIL_R,     // 1

  TSA_N,       // 2
  TSA_R,       // 3

  STSA_N,      // 4
  STSA_R,      // 5

  RADL_N,      // 6
  RADL_R,      // 7

  RASL_N,      // 8
  RASL_R,      // 9

  RESERVED_VCL_N10,
  RESERVED_VCL_R11,
  RESERVED_VCL_N12,
  RESERVED_VCL_R13,
  RESERVED_VCL_N14,
  RESERVED_VCL_R15,

  BLA_W_LP,    // 16
  BLA_W_RADL,  // 17
  BLA_N_LP,    // 18
  IDR_W_RADL,  // 19
  IDR_N_LP,    // 20
  CRA,         // 21
  RESERVED_IRAP_VCL22,
  RESERVED_IRAP_VCL23,

  RESERVED_VCL24,
  RESERVED_VCL25,
  RESERVED_VCL26,
  RESERVED_VCL27,
  RESERVED_VCL28,
  RESERVED_VCL29,
  RESERVED_VCL30,
  RESERVED_VCL31,

  VPS,                     // 32
  SPS,                     // 33
  PPS,                     // 34
  ACCESS_UNIT_DELIMITER,   // 35
  EOS,                     // 36
  EOB,                     // 37
  FILLER_DATA,             // 38
  PREFIX_SEI,              // 39
  SUFFIX_SEI,              // 40

  RESERVED_NVCL41,
  RESERVED_NVCL42,
  RESERVED_NVCL43,
  RESERVED_NVCL44,
  RESERVED_NVCL45,
  RESERVED_NVCL46,
  RESERVED_NVCL47,
  UNSPECIFIED_48,
  UNSPECIFIED_49,
  UNSPECIFIED_50,
  UNSPECIFIED_51,
  UNSPECIFIED_52,
  UNSPECIFIED_53,
  UNSPECIFIED_54,
  UNSPECIFIED_55,
  UNSPECIFIED_56,
  UNSPECIFIED_57,
  UNSPECIFIED_58,
  UNSPECIFIED_59,
  UNSPECIFIED_60,
  UNSPECIFIED_61,
  UNSPECIFIED_62,
  UNSPECIFIED_63,
  INVALID,
};

/**
 Find the beginning and end of a NAL (Network Abstraction Layer) unit in a byte buffer containing H264 bitstream data.
 @param[in]   buf        the buffer
 @param[in]   size       the size of the buffer
 @param[out]  nal_start  the beginning offset of the nal
 @param[out]  nal_end    the end offset of the nal
 @return                 the length of the nal, or 0 if did not find start of nal, or -1 if did not find end of nal
 */
// DEPRECATED - this will be replaced by a similar function with a slightly different API
int find_nal_unit(const uint8_t* buf, int size, int* nal_start, int* nal_end)
{
  int i;
  // find start
  *nal_start = 0;
  *nal_end = 0;

  i = 0;
  while (   //( next_bits( 24 ) != 0x000001 && next_bits( 32 ) != 0x00000001 )
    (buf[i] != 0 || buf[i+1] != 0 || buf[i+2] != 0x01) &&
    (buf[i] != 0 || buf[i+1] != 0 || buf[i+2] != 0 || buf[i+3] != 0x01)
    )
  {
    i++; // skip leading zero
    if (i+4 >= size) { return 0; } // did not find nal start
  }

  if  (buf[i] != 0 || buf[i+1] != 0 || buf[i+2] != 0x01) // ( next_bits( 24 ) != 0x000001 )
  {
    i++;
  }

  if  (buf[i] != 0 || buf[i+1] != 0 || buf[i+2] != 0x01) { /* error, should never happen */ return 0; }
  i+= 3;
  *nal_start = i;

  while (//( next_bits( 24 ) != 0x000000 && next_bits( 24 ) != 0x000001 )
    i+3 < size &&
    (buf[i] != 0 || buf[i+1] != 0 || buf[i+2] != 0) &&
    (buf[i] != 0 || buf[i+1] != 0 || buf[i+2] != 0x01)
    )
  {
    i++;
    // FIXME the next line fails when reading a nal that ends exactly at the end of the data
  }

  if (i+3 == size)
  {
    *nal_end = size;
  }
  else
  {
    *nal_end = i;
  }

  return (*nal_end - *nal_start);
}

const bool verbose = false;

const char * NALU_TYPE[] =
{
    "TRAIL_N",
    "TRAIL_R",
    "TSA_N",
    "TSA_R",
    "STSA_N",
    "STSA_R",
    "RADL_N",
    "RADL_R",
    "RASL_N",
    "RASL_R",
    "RSV_VCL_N10",
    "RSV_VCL_N12",
    "RSV_VCL_N14",
    "RSV_VCL_R11",
    "RSV_VCL_R13",
    "RSV_VCL_R15",
    "BLA_W_LP",
    "BLA_W_RADL",
    "BLA_N_LP",
    "IDR_W_RADL",
