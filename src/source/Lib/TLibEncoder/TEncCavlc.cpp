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

/** \file     TEncCavlc.cpp
    \brief    CAVLC encoder class
*/

#include "../TLibCommon/CommonDef.h"
#include "TEncCavlc.h"
#include "SEIwrite.h"

//! \ingroup TLibEncoder
//! \{

#if ENC_DEC_TRACE
#if MCTS_EXTRACTION
Void  xTraceVPSHeaderEnc()
#else
Void  xTraceVPSHeader()
#endif
{
  fprintf( g_hTrace, "=========== Video Parameter Set     ===========\n" );
}

#if MCTS_EXTRACTION
Void  xTraceSPSHeaderEnc()
#else
Void  xTraceSPSHeader()
#endif
{
  fprintf( g_hTrace, "=========== Sequence Parameter Set  ===========\n" );
}

#if MCTS_EXTRACTION
Void  xTracePPSHeaderEnc()
#else
Void  xTracePPSHeader()
#endif
{
  fprintf( g_hTrace, "=========== Picture Parameter Set  ===========\n");
}

#if MCTS_EXTRACTION
Void  xTraceSliceHeaderEnc()
#else
Void  xTraceSliceHeader()
#endif
{
  fprintf( g_hTrace, "=========== Slice ===========\n");
}

#if MCTS_EXTRACTION
Void  xTraceAccessUnitDelimiterEnc()
#else
Void  xTraceAccessUnitDelimiter()
#endif
{
  fprintf( g_hTrace, "=========== Access Unit Delimiter ===========\n");
}

#endif

Void AUDWriter::codeAUD(TComBitIf& bs, const Int pictureType)
{
#if ENC_DEC_TRACE
#if MCTS_EXTRACTION
  xTraceAccessUnitDelimiterEnc();
#else
  xTraceAccessUnitDelimiter();
#endif
