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

/** \file     TComLoopFilter.cpp
    \brief    deblocking filter
*/

#include "TComLoopFilter.h"
#include "TComSlice.h"
#include "TComMv.h"
#include "TComTU.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

//#define   EDGE_VER    0
//#define   EDGE_HOR    1

#define DEFAULT_INTRA_TC_OFFSET 2 ///< Default intra TC offset

// ====================================================================================================================
// Tables
// ====================================================================================================================

const UChar TComLoopFilter::sm_tcTable[MAX_QP + 1 + DEFAULT_INTRA_TC_OFFSET] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,5,5,6,6,7,8,9,10,11,13,14,16,18,20,22,24
};

const UChar TComLoopFilter::sm_betaTable[MAX_QP + 1] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,7,8,9,10,11,12,13,14,15,16,17,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64
};

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComLoopFilter::TComLoopFilter()
: m_uiNumPartitions(0)
, m_bLFCrossTileBoundary(true)
{
  for( Int edgeDir = 0; edgeDir < NUM_EDGE_DIR; edgeDir++ )
  {
    m_aapucBS       [edgeDir] = NULL;
    m_aapbEdgeFilter[edgeDir] = NULL;
  }
}

TComLoopFilter::~TComLoopFilter()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================
Void TComLoopFilter::setCfg( Bool bLFCrossTileBoundary )
{
  m_bLFCrossTileBoundary = bLFCrossTileBoundary;
}

Void TComLoopFilter::create( UInt uiMaxCUDepth )
{
  destroy();
  m_uiNumPartitions = 1 << ( uiMaxCUDepth<<1 );
  for( Int edgeDir = 0; edgeDir < NUM_EDGE_DIR; edgeDir++ )
