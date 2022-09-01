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

/** \file     TComSlice.h
    \brief    slice header and SPS class (header)
*/

#ifndef __TCOMSLICE__
#define __TCOMSLICE__

#include <cstring>
#include <map>
#include <vector>
#include "CommonDef.h"
#include "TComRom.h"
#include "TComList.h"
#include "TComChromaFormat.h"

//! \ingroup TLibCommon
//! \{

class TComPic;
class TComTrQuant;
// ====================================================================================================================
// Constants
// ====================================================================================================================

static const UInt REF_PIC_LIST_NUM_IDX=32;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// Reference Picture Set class
class TComReferencePictureSet
{
private:
  Int  m_numberOfPictures;
  Int  m_numberOfNegativePictures;
  Int  m_numberOfPositivePictures;
  Int  m_numberOfLongtermPictures;
  Int  m_deltaPOC[MAX_NUM_REF_PICS];
  Int  m_POC[MAX_NUM_REF_PICS];
  Bool m_used[MAX_NUM_REF_PICS];
  Bool m_interRPSPrediction;
  Int  m_deltaRIdxMinus1;
  Int  m_deltaRPS;
  Int  m_numRefIdc;
  Int  m_refIdc[MAX_NUM_REF_PICS+1];
  Bool m_bCheckLTMSB[MAX_NUM_REF_PICS];
  Int  m_pocLSBLT[MAX_NUM_REF_PICS];
  Int  m_deltaPOCMSBCycleLT[MAX_NUM_REF_PICS];
  Bool m_deltaPocMSBPresentFlag[MAX_NUM_REF_PICS];

public:
          TComReferencePictureSet();
  virtual ~TComReferencePictureSet();
  Int     getPocLSBLT(Int i) const                     { return m_pocLSBLT[i];               }
  Void    setPocLSBLT(Int i, Int x)                    { m_pocLSBLT[i] = x;                  }
  Int     getDeltaPocMSBCycleLT(Int i) const           { return m_deltaPOCMSBCycleLT[i];     }
  Void    setDeltaPocMSBCycleLT(Int i, Int x)          { m_deltaPOCMSBCycleLT[i] = x;        }
  Bool    getDeltaPocMSBPresentFlag(Int i) const       { return m_deltaPocMSBPresentFlag[i]; }
  Void    setDeltaPocMSBPresentFlag(Int i, Bool x)     { m_deltaPocMSBPresentFlag[i] = x;    }
  Void    setUsed(Int bufferNum, Bool used);
  Void    setDeltaPOC(Int bufferNum, Int deltaPOC);
  Void    setPOC(Int bufferNum, Int deltaPOC);
  Void    setNumberOfPictures(Int numberOfPictures);
  Void    setCheckLTMSBPresent(Int bufferNum, Bool b );
  Bool    getCheckLTMSBPresent(Int bufferNum) const;

