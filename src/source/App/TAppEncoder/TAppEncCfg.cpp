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

/** \file     TAppEncCfg.cpp
    \brief    Handle encoder configuration parameters
*/

#include <stdio.h>

#include <stdlib.h>
#include <cassert>
#include <cstring>
#include <string>
#include <limits>
#include <map>

#include "TLibCommon/TComRom.h"
#if DPB_ENCODER_USAGE_CHECK
#include "TLibCommon/ProfileLevelTierFeatures.h"
#endif

template <class T1, class T2>
static inline std::istream& operator >> (std::istream &in, std::map<T1, T2> &map);

#include "TAppEncCfg.h"
#include "Utilities/program_options_lite.h"
#include "TLibEncoder/TEncRateCtrl.h"
#ifdef WIN32
#define strdup _strdup
#endif

#define MACRO_TO_STRING_HELPER(val) #val
#define MACRO_TO_STRING(val) MACRO_TO_STRING_HELPER(val)

using namespace std;

enum UIProfileName // this is used for determining profile strings, where multiple profiles map to a single profile idc with various constraint flag combinations
{
  UI_NONE = 0,
  UI_MAIN = 1,
  UI_MAIN10 = 2,
  UI_MAIN10_STILL_PICTURE=10002,
  UI_MAINSTILLPICTURE = 3,
  UI_MAINREXT = 4,
  UI_HIGHTHROUGHPUTREXT = 5,
  // The following are RExt profiles, which would map to the MAINREXT profile idc.
  // The enumeration indicates the bit-depth constraint in the bottom 2 digits
  //                           the chroma format in the next digit
  //                           the intra constraint in the next digit (1 for no intra constraint, 2 for intra constraint)
  //                           If it is a RExt still picture, there is a '1' for the top digit.
  UI_MONOCHROME_8      = 1008,
  UI_MONOCHROME_12     = 1012,
  UI_MONOCHROME_16     = 1016,
  UI_MAIN_12           = 1112,
  UI_MAIN_422_10       = 1210,
  UI_MAIN_422_12       = 1212,
  UI_MAIN_444          = 1308,
  UI_MAIN_444_10       = 1310,
  UI_MAIN_444_12       = 1312,
  UI_MAIN_444_16       = 1316, // non-standard profile definition, used for development purposes
  UI_MAIN_INTRA        = 2108,
  UI_MAIN_10_INTRA     = 2110,
  UI_MAIN_12_INTRA     = 2112,
  UI_MAIN_422_10_INTRA = 2210,
  UI_MAIN_422_12_INTRA = 2212,
  UI_MAIN_444_INTRA    = 2308,
  UI_MAIN_444_10_INTRA = 2310,
  UI_MAIN_444_12_INTRA = 2312,
  UI_MAIN_444_16_INTRA = 2316,
  UI_MAIN_444_STILL_PICTURE = 11308,
  UI_MAIN_444_16_STILL_PICTURE = 12316,
  // The following are high throughput profiles, which would map to the HIGHTHROUGHPUTREXT profile idc.
  // The enumeration indicates the bit-depth constraint in the bottom 2 digits
  //                           the chroma format in the next digit
  //                           the intra constraint in the next digit
  //                           There is a '2' for the top digit to indicate it is high throughput profile
  
  UI_HIGHTHROUGHPUT_444     = 21308,
  UI_HIGHTHROUGHPUT_444_10  = 21310,
  UI_HIGHTHROUGHPUT_444_14  = 21314,
  UI_HIGHTHROUGHPUT_444_16_INTRA  = 22316
};

constexpr int TF_DEFAULT_REFS = 4;

//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

TAppEncCfg::TAppEncCfg()
: m_inputColourSpaceConvert(IPCOLOURSPACE_UNCHANGED)
, m_snrInternalColourSpace(false)
, m_outputInternalColourSpace(false)
#if EXTENSION_360_VIDEO
, m_ext360(*this)
#endif
{
  m_aidQP = NULL;
  m_startOfCodedInterval = NULL;
  m_codedPivotValue = NULL;
  m_targetPivotValue = NULL;
}

TAppEncCfg::~TAppEncCfg()
{
  if ( m_aidQP )
  {
    delete[] m_aidQP;
  }
  if ( m_startOfCodedInterval )
  {
    delete[] m_startOfCodedInterval;
    m_startOfCodedInterval = NULL;
  }
   if ( m_codedPivotValue )
  {
    delete[] m_codedPivotValue;
    m_codedPivotValue = NULL;
  }
  if ( m_targetPivotValue )
  {
    delete[] m_targetPivotValue;
    m_targetPivotValue = NULL;
  }
}

Void TAppEncCfg::create()
{
}

Void TAppEncCfg::destroy()
{
}

std::istringstream &operator>>(std::istringstream &in, GOPEntry &entry)     //input
{
  in>>entry.m_sliceType;
  in>>entry.m_POC;
  in>>entry.m_QPOffset;
  in>>entry.m_QPOffsetModelOffset;
  in>>entry.m_QPOffsetModelScale;
  in>>entry.m_CbQPoffset;
  in>>entry.m_CrQPoffset;
  in>>entry.m_QPFactor;
  in>>entry.m_tcOffsetDiv2;
  in>>entry.m_betaOffsetDiv2;
  in>>entry.m_temporalId;
  in>>entry.m_numRefPicsActive;
  in>>entry.m_numRefPics;
  for ( Int i = 0; i < entry.m_numRefPics; i++ )
  {
    in>>entry.m_referencePics[i];
  }
  in>>entry.m_interRPSPrediction;
  if (entry.m_interRPSPrediction==1)
  {
    in>>entry.m_deltaRPS;
    in>>entry.m_numRefIdc;
    for ( Int i = 0; i < entry.m_numRefIdc; i++ )
    {
      in>>entry.m_refIdc[i];
    }
  }
  else if (entry.m_interRPSPrediction==2)
  {
    in>>entry.m_deltaRPS;
  }
  return in;
}

Bool confirmPara(Bool bflag, const TChar* message);

static inline ChromaFormat numberToChromaFormat(const Int val)
{
  switch (val)
  {
    case 400: return CHROMA_400; break;
    case 420: return CHROMA_420; break;
    case 422: return CHROMA_422; break;
    case 444: return CHROMA_444; break;
    default:  return NUM_CHROMA_FORMAT;
  }
}

static const struct MapStrToProfile
{
  const TChar* str;
  Profile::Name value;
}
strToProfile[] =
{
  {"none",                 Profile::NONE               },
  {"main",                 Profile::MAIN               },
  {"main10",               Profile::MAIN10             },
  {"main-still-picture",   Profile::MAINSTILLPICTURE   },
  {"main10-still-picture", Profile::MAIN10             },
  {"main-RExt",            Profile::MAINREXT           },
  {"high-throughput-RExt", Profile::HIGHTHROUGHPUTREXT }
};

static const struct MapStrToUIProfileName
{
  const TChar* str;
  UIProfileName value;
}
strToUIProfileName[] =
{
    {"none",                      UI_NONE             },
    {"main",                      UI_MAIN             },
    {"main10",                    UI_MAIN10           },
    {"main10_still_picture",      UI_MAIN10_STILL_PICTURE },
    {"main10-still-picture",      UI_MAIN10_STILL_PICTURE },
    {"main_still_picture",        UI_MAINSTILLPICTURE },
    {"main-still-picture",        UI_MAINSTILLPICTURE },
    {"main_RExt",                 UI_MAINREXT         },
    {"main-RExt",                 UI_MAINREXT         },
    {"main_rext",                 UI_MAINREXT         },
    {"main-rext",                 UI_MAINREXT         },
    {"high_throughput_RExt",      UI_HIGHTHROUGHPUTREXT },
    {"high-throughput-RExt",      UI_HIGHTHROUGHPUTREXT },
    {"high_throughput_rext",      UI_HIGHTHROUGHPUTREXT },
    {"high-throughput-rext",      UI_HIGHTHROUGHPUTREXT },
    {"monochrome",                UI_MONOCHROME_8     },
    {"monochrome12",              UI_MONOCHROME_12    },
    {"monochrome16",              UI_MONOCHROME_16    },
    {"main12",                    UI_MAIN_12          },
    {"main_422_10",               UI_MAIN_422_10      },
    {"main_422_12",               UI_MAIN_422_12      },
    {"main_444",                  UI_MAIN_444         },
    {"main_444_10",               UI_MAIN_444_10      },
    {"main_444_12",               UI_MAIN_444_12      },
    {"main_444_16",               UI_MAIN_444_16      },
    {"main_intra",                UI_MAIN_INTRA       },
    {"main_10_intra",             UI_MAIN_10_INTRA    },
    {"main_12_intra",             UI_MAIN_12_INTRA    },
    {"main_422_10_intra",         UI_MAIN_422_10_INTRA},
    {"main_422_12_intra",         UI_MAIN_422_12_INTRA},
    {"main_444_intra",            UI_MAIN_444_INTRA   },
    {"main_444_still_picture",    UI_MAIN_444_STILL_PICTURE },
    {"main_444_10_intra",         UI_MAIN_444_10_INTRA},
    {"main_444_12_intra",         UI_MAIN_444_12_INTRA},
    {"main_444_16_intra",         UI_MAIN_444_16_INTRA},
    {"main_444_16_still_picture", UI_MAIN_444_16_STILL_PICTURE },
    {"high_throughput_444",       UI_HIGHTHROUGHPUT_444    },
    {"high_throughput_444_10",    UI_HIGHTHROUGHPUT_444_10 },
    {"high_throughput_444_14",    UI_HIGHTHROUGHPUT_444_14 },
    {"high_throughput_444_16_intra", UI_HIGHTHROUGHPUT_444_16_INTRA }
};

static const UIProfileName validRExtHighThroughPutProfileNames[2/* intraConstraintFlag*/][4/* bit depth constraint 8=0, 10=1, 12=2, 16=3*/]=
{
    { UI_HIGHTHROUGHPUT_444,          UI_HIGHTHROUGHPUT_444_10,          UI_HIGHTHROUGHPUT_444_14,         UI_NONE                         }, // intraConstraintFlag 0 - 8-bit,10-bit,14-bit and 16-bit
    { UI_NONE,                        UI_NONE,                           UI_NONE,                          UI_HIGHTHROUGHPUT_444_16_INTRA  }  // intraConstraintFlag 1 - 8-bit,10-bit,14-bit and 16-bit
};

static const UIProfileName validRExtProfileNames[2/* intraConstraintFlag*/][4/* bit depth constraint 8=0, 10=1, 12=2, 16=3*/][4/*chroma format*/]=
{
    {
        { UI_MONOCHROME_8,  UI_NONE,          UI_NONE,              UI_MAIN_444          }, // 8-bit  inter for 400, 420, 422 and 444
        { UI_NONE,          UI_NONE,          UI_MAIN_422_10,       UI_MAIN_444_10       }, // 10-bit inter for 400, 420, 422 and 444
        { UI_MONOCHROME_12, UI_MAIN_12,       UI_MAIN_422_12,       UI_MAIN_444_12       }, // 12-bit inter for 400, 420, 422 and 444
        { UI_MONOCHROME_16, UI_NONE,          UI_NONE,              UI_MAIN_444_16       }  // 16-bit inter for 400, 420, 422 and 444 (the latter is non standard used for development)
    },
    {
        { UI_NONE,          UI_MAIN_INTRA,    UI_NONE,              UI_MAIN_444_INTRA    }, // 8-bit  intra for 400, 420, 422 and 444
        { UI_NONE,          UI_MAIN_10_INTRA, UI_MAIN_422_10_INTRA, UI_MAIN_444_10_INTRA }, // 10-bit intra for 400, 420, 422 and 444
        { UI_NONE,          UI_MAIN_12_INTRA, UI_MAIN_422_12_INTRA, UI_MAIN_444_12_INTRA }, // 12-bit intra for 400, 420, 422 and 444
