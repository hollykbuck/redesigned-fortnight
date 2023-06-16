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
