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
 \file     SEIFilmGrainSynthesizer.h
 \brief    SMPTE RDD5 based film grain synthesis functionality from SEI messages
 */

#ifndef __SEIFILMGRAINSYNTHESIZER__
#define __SEIFILMGRAINSYNTHESIZER__

#include "SEI.h"
#include "TComPicYuv.h"

#if JVET_X0048_X0103_FILM_GRAIN

//! \ingroup SEIFilmGrainSynthesizer
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

static const int MIN_LOG2SCALE_VALUE             = 2;
static const int MAX_LOG2SCALE_VALUE             = 7;
static const int FILM_GRAIN_MODEL_ID_VALUE       = 0;
static const int BLENDING_MODE_VALUE             = 0;
static const int MIN_CUT_OFF_FREQUENCY           = 2;
static const int MAX_CUT_OFF_FREQUENCY           = 14;
static const int DEFAULT_HORZ_CUT_OFF_FREQUENCY  = 8;
static const int NUM_CUT_OFF_FREQ                = 13;

static const int SCALE_DOWN_422                  = 181; /* in Q-format of 8 : 1/sqrt(2) */
static const int Q_FORMAT_SCALING                = 8;
static const int GRAIN_SCALE                     = 6;
static const int MIN_CHROMA_FORMAT_IDC           = 0;
static const int MAX_CHROMA_FORMAT_IDC           = 3;
static const int MIN_BIT_DEPTH                   = 8;
static const int MAX_BIT_DEPTH                   = 16;
static const int FG_BLK_8_shift                  = 6;
static const int FG_BLK_16_shift                 = 8;
static const int FG_BLK_32_shift                 = 10;
static const int NUM_8x8_BLKS_16x16              = 4;
static const int NUM_16x16_BLKS_32x32            = 4;
static const int FG_BLK_AREA_8x8                 = 64;
static const int FG_BLK_AREA_16x16               = 256;
static const int INTENSITY_INTERVAL_MATCH_FAIL   = -1;
static const int COLOUR_OFFSET_LUMA              = 0;
static const int COLOUR_OFFSET_CR                = 85;
static const int COLOUR_OFFSET_CB                = 170;

static const int FG_MIN_WIDTH                    = 128;
static const int FG_MAX_WIDTH                    = 7680;
static const int FG_MIN_HEIGHT                   = 128;
static const int FG_MAX_HEIGHT                   = 4320;

#define CLIP3(min, max, x)              (((x) > (max)) ? (max) :(((x) < (min))? (min):(x)))
#define MIN(x,y)                        (((x) > (y)) ? (y) : (x))
#define MAX(x,y)                        (((x) > (y)) ? (x) : (y))
#define MSB16(x)                        ((x&0xFFFF0000)>>16)
#define LSB16(x)                        (x&0x0000FFFF)
#define BIT0(x)                         (x&0x1)
#define POS_30                          (1<<30)
#define POS_2                           (1<<2)

/* Error start codes for various classes of errors */
#define FGS_FILE_IO_ERROR               0x0010
#define FGS_PARAM_ERROR                 0x0020

/* Error codes for various errors in SMPTE-RDD5 standalone grain synthesizer */
