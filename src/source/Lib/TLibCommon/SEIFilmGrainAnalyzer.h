/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ITU/ISO/IEC
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

/** \file     SEIFilmGrainAnalyzer.h
    \brief    SMPTE RDD5 based film grain analysis functionality from SEI messages
*/

#ifndef __SEIFILMGRAINANALYZER__
#define __SEIFILMGRAINANALYZER__

#pragma once

#include "TLibCommon/TComPic.h"
#include "TLibCommon/SEI.h"
#include "Utilities/TVideoIOYuv.h"
#include "TLibCommon/CommonDef.h"

#include <numeric>
#include <cmath>
#include <algorithm>

#if JVET_X0048_X0103_FILM_GRAIN
static const double PI                                      = 3.14159265358979323846;

static const int MAXPAIRS                                   = 256;
static const int MAXORDER                                   = 8;     // maximum order of polinomial fitting
static const int MAX_REAL_SCALE                             = 16;
static const int ORDER                                      = 4;     // order of polinomial function
static const int QUANT_LEVELS                               = 4;     // number of quantization levels in lloyd max quantization
static const int INTERVAL_SIZE                              = 16;
static const int MIN_ELEMENT_NUMBER_PER_INTENSITY_INTERVAL  = 8;
static const int MIN_POINTS_FOR_INTENSITY_ESTIMATION        = 40;    // 5*8 = 40; 5 intervals with at least 8 points
static const int MIN_BLOCKS_FOR_CUTOFF_ESTIMATION           = 2;     // 2 blocks of 64 x 64 size
static const int POINT_STEP                                 = 16;    // step size in point extension
static const int MAX_NUM_POINT_TO_EXTEND                    = 4;     // max point in extension
static const double POINT_SCALE                             = 1.25;  // scaling in point extension
static const double VAR_SCALE_DOWN                          = 1.2;   // filter out large points
static const double VAR_SCALE_UP                            = 0.6;   // filter out large points
static const int NUM_PASSES                                 = 2;     // number of passes when fitting the function
static const int NBRS                                       = 1;     // minimum number of surrounding points in order to keep it for further analysis (within the widnow range)
static const int WINDOW                                     = 1;     // window to check surrounding points
static const int MIN_INTENSITY                              = 40;
static const int MAX_INTENSITY                              = 950;

//! \ingroup SEIFilmGrainAnalyzer
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

struct Picture;

typedef std::vector<std::vector<Intermediate_Int>> PelMatrix;
typedef std::vector<std::vector<double>>           PelMatrixDouble;

typedef std::vector<std::vector<long double>>      PelMatrixLongDouble;
typedef std::vector<long double>                   PelVectorLongDouble;

class Canny
{
public:
  Canny();
  ~Canny();

  unsigned int      m_convWidthG = 5, m_convHeightG = 5;		  // Pixel's row and col positions for Gauss filtering

  void detect_edges(const TComPicYuv* orig, TComPicYuv* dest, unsigned int uiBitDepth, ComponentID compID);

private:
