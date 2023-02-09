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

/** \file     SEIRemovalApp.cpp
    \brief    Decoder application class
*/

#include <list>
#include <vector>
#include <stdio.h>
#include <fcntl.h>

#include "SEIRemovalApp.h"
#include "TLibDecoder/AnnexBread.h"
#include "TLibDecoder/NALread.h"

//! \ingroup DecoderApp
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

SEIRemovalApp::SEIRemovalApp()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - create internal class
 - initialize internal class
 - until the end of the bitstream, call decoding function in SEIRemovalApp class
 - delete allocated buffers
 - destroy internal class
 - returns the number of mismatching pictures
 */

Void read2(InputNALUnit& nalu)
{
  TComInputBitstream& bs = nalu.getBitstream();

  Bool forbidden_zero_bit = bs.read(1);           // forbidden_zero_bit
  if (forbidden_zero_bit != 0)
  {
    std::cerr << "Forbidden zero-bit not '0'" << std::endl;
    exit(1);
  }
  nalu.m_nalUnitType = (NalUnitType) bs.read(6);  // nal_unit_type
  nalu.m_nuhLayerId = bs.read(6);                 // nuh_layer_id
  nalu.m_temporalId = bs.read(3) - 1;             // nuh_temporal_id_plus1
}

UInt SEIRemovalApp::decode()
{
//  Int                 poc;
//  PicList* pcListPic = NULL;

  ifstream bitstreamFileIn(m_bitstreamFileNameIn.c_str(), ifstream::in | ifstream::binary);
  if (!bitstreamFileIn)
  {
    std::cerr << "failed to open bitstream file " << m_bitstreamFileNameIn.c_str() << " for reading";
    exit(1);
  }

  ofstream bitstreamFileOut(m_bitstreamFileNameOut.c_str(), ifstream::out | ifstream::binary);

  InputByteStream bytestream(bitstreamFileIn);
