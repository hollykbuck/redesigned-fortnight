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

/** \file     SyntaxElementParser.cpp
    \brief    Parsing functionality high level syntax
*/

//! \ingroup TLibDecoder
//! \{

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComRom.h"
#include "TLibCommon/TComBitStream.h"
#include "SyntaxElementParser.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "TLibCommon/TComCodingStatistics.h"
#endif

#if ENC_DEC_TRACE

Void  xTraceAccessUnitDelimiter ()
{
  fprintf( g_hTrace, "=========== Access Unit Delimiter ===========\n");
}

Void xTraceFillerData ()
{
  fprintf( g_hTrace, "=========== Filler Data ===========\n");
}

#endif



#if DECODER_PARTIAL_CONFORMANCE_CHECK!=0

Void SyntaxElementParser::xReadSCodeChk ( UInt   length, Int& val, const TChar *pSymbolName, const Int minValIncl, const Int maxValIncl )
{
  READ_SCODE(length, val, pSymbolName);
  TDecConformanceCheck::checkRange(val, pSymbolName, minValIncl, maxValIncl);
}

Void SyntaxElementParser::xReadCodeChk ( UInt   length, UInt& val, const TChar *pSymbolName, const UInt minValIncl, const UInt maxValIncl )
{
  READ_CODE(length, val, pSymbolName);
  TDecConformanceCheck::checkRange(val, pSymbolName, minValIncl, maxValIncl);
}

Void SyntaxElementParser::xReadUvlcChk ( UInt&  val, const TChar *pSymbolName, const UInt minValIncl, const UInt maxValIncl )
{
  READ_UVLC(val, pSymbolName);
  TDecConformanceCheck::checkRange(val, pSymbolName, minValIncl, maxValIncl);
}

Void SyntaxElementParser::xReadSvlcChk ( Int&   val, const TChar *pSymbolName, const Int  minValIncl, const Int  maxValIncl )
{
  READ_SVLC(val, pSymbolName);
  TDecConformanceCheck::checkRange(val, pSymbolName, minValIncl, maxValIncl);
}

Void SyntaxElementParser::xReadFlagChk ( UInt&  val, const TChar *pSymbolName, const UInt minValIncl, const UInt maxValIncl )
{
  READ_FLAG(val, pSymbolName);
  TDecConformanceCheck::checkRange(val, pSymbolName, minValIncl, maxValIncl);
}
#endif

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
