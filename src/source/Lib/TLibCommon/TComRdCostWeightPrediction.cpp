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
      if (pcDtParam->bIsBiPred)
      {
        for(Int iRows = pcDtParam->iRows; iRows != 0; iRows-- )
        {
          for (Int n = 0; n < iCols; n++ )
          {
            uiSum += abs( piOrg[n] - (piCur[n] + offset) );
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
        const Pel iMaxValue = (Pel) ((1 << pcDtParam->bitDepth) - 1);
        for(Int iRows = pcDtParam->iRows; iRows != 0; iRows-- )
        {
          for (Int n = 0; n < iCols; n++ )
          {
            const Pel pred = Clip3((Pel) 0, iMaxValue, (Pel) (piCur[n] + offset)) ;

            uiSum += abs( piOrg[n] - pred );
          }
          if (pcDtParam->m_maximumDistortionForEarlyExit <  ( uiSum >> distortionShift))
          {
            return uiSum >> distortionShift;
          }
          piOrg += iStrideOrg;
          piCur += iStrideCur;
        }
      }
    }
  }
  else
  {
    // Lets not clip for the bipredictive case since clipping should be done after
    // combining both elements. Unfortunately the code uses the suboptimal "subtraction"
    // method, which is faster but introduces the clipping issue (therefore Bipred is suboptimal).
    if (pcDtParam->bIsBiPred)
    {
      for(Int iRows = pcDtParam->iRows; iRows != 0; iRows-- )
      {
        for (Int n = 0; n < iCols; n++ )
        {
          const Pel pred = ( (w0*piCur[n] + round) >> shift ) + offset ;
          uiSum += abs( piOrg[n] - pred );
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
      const Pel iMaxValue = (Pel) ((1 << pcDtParam->bitDepth) - 1);

      if (offset == 0)
      {
        for(Int iRows = pcDtParam->iRows; iRows != 0; iRows-- )
        {
          for (Int n = 0; n < iCols; n++ )
          {
            const Pel pred = Clip3((Pel) 0, iMaxValue, (Pel) (( (w0*piCur[n] + round) >> shift ))) ;

            uiSum += abs( piOrg[n] - pred );
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
        for(Int iRows = pcDtParam->iRows; iRows != 0; iRows-- )
        {
          for (Int n = 0; n < iCols; n++ )
          {
            const Pel pred = Clip3((Pel) 0, iMaxValue, (Pel) (( (w0*piCur[n] + round) >> shift ) + offset)) ;

            uiSum += abs( piOrg[n] - pred );
          }
          if (pcDtParam->m_maximumDistortionForEarlyExit <  ( uiSum >> distortionShift))
          {
            return uiSum >> distortionShift;
          }
          piOrg += iStrideOrg;
          piCur += iStrideCur;
        }
      }
    }
  }
  //pcDtParam->compIdx = MAX_NUM_COMPONENT;  // reset for DEBUG (assert test)

  return uiSum >> distortionShift;
}


// --------------------------------------------------------------------------------------------------------------------
// SSE
// --------------------------------------------------------------------------------------------------------------------
/** get weighted SSD cost
 * \param pcDtParam
 * \returns Distortion
 */
Distortion TComRdCostWeightPrediction::xGetSSEw( DistParam* pcDtParam )
{
  const Pel            *piOrg           = pcDtParam->pOrg;
  const Pel            *piCur           = pcDtParam->pCur;
  const Int             iCols           = pcDtParam->iCols;
  const Int             iStrideOrg      = pcDtParam->iStrideOrg;
  const Int             iStrideCur      = pcDtParam->iStrideCur;
  const ComponentID     compIdx         = pcDtParam->compIdx;

  assert( pcDtParam->iSubShift == 0 ); // NOTE: what is this protecting?

  assert(compIdx<MAX_NUM_COMPONENT);
  const WPScalingParam &wpCur           = pcDtParam->wpCur[compIdx];
  const Int             w0              = wpCur.w;
  const Int             offset          = wpCur.offset;
  const Int             shift           = wpCur.shift;
  const Int             round           = wpCur.round;
  const UInt            distortionShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);

  Distortion sum = 0;

  if (pcDtParam->bIsBiPred)
  {
    for(Int iRows = pcDtParam->iRows ; iRows != 0; iRows-- )
    {
      for (Int n = 0; n < iCols; n++ )
      {
        const Pel pred     = ( (w0*piCur[n] + round) >> shift ) + offset ;
        const Pel residual = piOrg[n] - pred;
        sum += ( Distortion(residual) * Distortion(residual) ) >> distortionShift;
      }
      piOrg += iStrideOrg;
      piCur += iStrideCur;
    }
  }
  else
  {
    const Pel iMaxValue = (Pel) ((1 << pcDtParam->bitDepth) - 1);

    for(Int iRows = pcDtParam->iRows ; iRows != 0; iRows-- )
    {
      for (Int n = 0; n < iCols; n++ )
      {
        const Pel pred     = Clip3((Pel) 0, iMaxValue, (Pel) (( (w0*piCur[n] + round) >> shift ) + offset)) ;
        const Pel residual = piOrg[n] - pred;
        sum += ( Distortion(residual) * Distortion(residual) ) >> distortionShift;
      }
      piOrg += iStrideOrg;
      piCur += iStrideCur;
    }
  }

  //pcDtParam->compIdx = MAX_NUM_COMPONENT; // reset for DEBUG (assert test)

  return sum;
}


// --------------------------------------------------------------------------------------------------------------------
// HADAMARD with step (used in fractional search)
// --------------------------------------------------------------------------------------------------------------------
//! get weighted Hadamard cost for 2x2 block
Distortion xCalcHADs2x2w( const WPScalingParam &wpCur, const Pel *piOrg, const Pel *piCur, Int iStrideOrg, Int iStrideCur, Int iStep )
{
  const Int round  = wpCur.round;
  const Int shift  = wpCur.shift;
  const Int offset = wpCur.offset;
  const Int w0     = wpCur.w;

  Distortion satd  = 0;
  TCoeff     diff[4];
  TCoeff     m[4];

  Pel   pred;

  pred    = ( (w0*piCur[0*iStep             ] + round) >> shift ) + offset ;
  diff[0] = piOrg[0             ] - pred;
  pred    = ( (w0*piCur[1*iStep             ] + round) >> shift ) + offset ;
  diff[1] = piOrg[1             ] - pred;
  pred    = ( (w0*piCur[0*iStep + iStrideCur] + round) >> shift ) + offset ;
  diff[2] = piOrg[iStrideOrg    ] - pred;
  pred    = ( (w0*piCur[1*iStep + iStrideCur] + round) >> shift ) + offset ;
  diff[3] = piOrg[iStrideOrg + 1] - pred;

