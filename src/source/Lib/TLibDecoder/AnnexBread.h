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
 \file     AnnexBread.h
 \brief    reading functions for Annex B byte streams
 */

#pragma once

#ifndef __ANNEXBREAD__
#define __ANNEXBREAD__

#include <stdint.h>
#include <istream>
#include <vector>

#include "TLibCommon/CommonDef.h"

//! \ingroup TLibDecoder
//! \{

class InputByteStream
{
public:
  /**
   * Create a bytestream reader that will extract bytes from
   * istream.
   *
   * NB, it isn't safe to access istream while in use by a
   * InputByteStream.
   *
   * Side-effects: the exception mask of istream is set to eofbit
   */
  InputByteStream(std::istream& istream)
  : m_NumFutureBytes(0)
  , m_FutureBytes(0)
  , m_Input(istream)
  {
    istream.exceptions(std::istream::eofbit | std::istream::badbit);
  }

  /**
   * Reset the internal state.  Must be called if input stream is
   * modified externally to this class
   */
  Void reset()
  {
    m_NumFutureBytes = 0;
    m_FutureBytes = 0;
  }

  /**
   * returns true if an EOF will be encountered within the next
   * n bytes.
   */
  Bool eofBeforeNBytes(UInt n)
  {
    assert(n <= 4);
    if (m_NumFutureBytes >= n)
    {
      return false;
    }

    n -= m_NumFutureBytes;
    try
    {
      for (UInt i = 0; i < n; i++)
      {
        m_FutureBytes = (m_FutureBytes << 8) | m_Input.get();
