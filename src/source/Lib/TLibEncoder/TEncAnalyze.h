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

/** \file     TEncAnalyze.h
    \brief    encoder analyzer class (header)
*/

#ifndef __TENCANALYZE__
#define __TENCANALYZE__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <stdio.h>
#include <memory.h>
#include <assert.h>
#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComChromaFormat.h"
#include "math.h"
#if EXTENSION_360_VIDEO
#include "TAppEncHelper360/TExt360EncAnalyze.h"
#endif

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder analyzer class
class TEncAnalyze
{
public:
  struct OutputLogControl
  {
    Bool printMSEBasedSNR;
    Bool printSequenceMSE;
    Bool printFrameMSE;
    Bool printMSSSIM;
    Bool printXPSNR;
    Bool printHexPerPOCPSNRs;
  };

  struct ResultData
  {
    ResultData () : bits(0)
      , xpsnr(0)
    {
      for(Int i=0; i<MAX_NUM_COMPONENT; i++)
      {
        psnr[i]=0;
        MSEyuvframe[i]=0;
        MSSSIM[i]=0;
      }
    }
    Double psnr[MAX_NUM_COMPONENT];
    Double bits;
    Double MSEyuvframe[MAX_NUM_COMPONENT];
    Double MSSSIM[MAX_NUM_COMPONENT];
    Double xpsnr;
  };

private:
  ResultData m_runningTotal;
  UInt      m_uiNumPic;
  Double    m_dFrmRate; //--CFG_KDY

#if EXTENSION_360_VIDEO
