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

/** \file     TEncTemporalFilter.h
    \brief    TEncTemporalFilter class (header)
 */

#ifndef __TEMPORAL_FILTER__
#define __TEMPORAL_FILTER__
#include "TLibCommon/TComPicYuv.h"
#include "Utilities/TVideoIOYuv.h"
#include <sstream>
#include <map>
#include <deque>

 //! \ingroup EncoderLib
 //! \{

struct MotionVector
{
  Int x, y;
  Int error;
#if JVET_V0056_MCTF || JVET_Y0077_BIM
  Int noise;
  MotionVector() : x(0), y(0), error(INT_LEAST32_MAX), noise(0) {}
#else
  MotionVector() : x(0), y(0), error(INT_LEAST32_MAX) {}
#endif
  void set(Int nx, Int ny, Int ne) { x = nx; y = ny; error = ne; }
};

template <class T>
struct Array2D
{
private:
  UInt m_width, m_height;
  std::vector< T > v;
public:
  Array2D() : m_width(0), m_height(0), v() { }
  Array2D(UInt width, UInt height, const T& value=T()) : m_width(0), m_height(0), v() { allocate(width, height, value); }

#if JVET_Y0077_BIM
  UInt w() const { return m_width;  }
  UInt h() const { return m_height; }
#endif

  Void allocate(UInt width, UInt height, const T& value=T())
  {
    m_width=width;
    m_height=height;
    v.resize(std::size_t(m_width*m_height), value);
  }

  T& get(UInt x, UInt y)
  {
    assert(x<m_width && y<m_height);
    return v[y*m_width+x];
  }

  const T& get(UInt x, UInt y) const
  {
    assert(x<m_width && y<m_height);
    return v[y*m_width+x];
  }
};

struct TemporalFilterSourcePicInfo
{
  TemporalFilterSourcePicInfo() : picBuffer(), mvs(), origOffset(0) { }
  TComPicYuv            picBuffer;
