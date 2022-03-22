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

/** \file     TComRdCostWeightPrediction.cpp
    \brief    RD cost computation class with Weighted-Prediction
*/

#include <math.h>
#include <assert.h>
#include "TComRdCost.h"
#include "TComRdCostWeightPrediction.h"

static Distortion xCalcHADs2x2w( const WPScalingParam &wpCur, const Pel *piOrg, const Pel *piCurr, Int iStrideOrg, Int iStrideCur, Int iStep );
static Distortion xCalcHADs4x4w( const WPScalingParam &wpCur, const Pel *piOrg, const Pel *piCurr, Int iStrideOrg, Int iStrideCur, Int iStep );
static Distortion xCalcHADs8x8w( const WPScalingParam &wpCur, const Pel *piOrg, const Pel *piCurr, Int iStrideOrg, Int iStrideCur, Int iStep );


// --------------------------------------------------------------------------------------------------------------------
// SAD
// --------------------------------------------------------------------------------------------------------------------
/** get weighted SAD cost
 * \param pcDtParam
 * \returns Distortion
 */
Distortion TComRdCostWeightPrediction::xGetSADw( DistParam* pcDtParam )
{
  const Pel            *piOrg      = pcDtParam->pOrg;
  const Pel            *piCur      = pcDtParam->pCur;
  const Int             iCols      = pcDtParam->iCols;
  const Int             iStrideCur = pcDtParam->iStrideCur;
  const Int             iStrideOrg = pcDtParam->iStrideOrg;
  const ComponentID     compID     = pcDtParam->compIdx;

  assert(compID<MAX_NUM_COMPONENT);

  const WPScalingParam &wpCur      = pcDtParam->wpCur[compID];

  const Int             w0         = wpCur.w;
  const Int             offset     = wpCur.offset;
  const Int             shift      = wpCur.shift;
  const Int             round      = wpCur.round;
  const Int        distortionShift = DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8);

  Distortion uiSum = 0;

  // Default weight
  if (w0 == 1 << shift)
  {
    // no offset
    if (offset == 0)
    {
      for(Int iRows = pcDtParam->iRows; iRows != 0; iRows-- )
      {
        for (Int n = 0; n < iCols; n++ )
        {
          uiSum += abs( piOrg[n] - piCur[n] );
        }
        if (pcDtParam->m_maximumDistortionForEarlyExit <  ( uiSum >> distortionShift))
        {
          return uiSum >> distortionShift;
        }
        piOrg += iStrideOrg;
        piCur += iStrideCur;
      }
    }
    else
    {
      // Lets not clip for the bipredictive case since clipping should be done after
      // combining both elements. Unfortunately the code uses the suboptimal "subtraction"
      // method, which is faster but introduces the clipping issue (therefore Bipred is suboptimal).
