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
        { UI_NONE,          UI_NONE,          UI_NONE,              UI_MAIN_444_16_INTRA }  // 16-bit intra for 400, 420, 422 and 444
    }
};

static const struct MapStrToTier
{
  const TChar* str;
  Level::Tier value;
}
strToTier[] =
{
  {"main", Level::MAIN},
  {"high", Level::HIGH},
};

static const struct MapStrToLevel
{
  const TChar* str;
  Level::Name value;
}
strToLevel[] =
{
  {"none",Level::NONE},
  {"1",   Level::LEVEL1},
  {"2",   Level::LEVEL2},
  {"2.1", Level::LEVEL2_1},
  {"3",   Level::LEVEL3},
  {"3.1", Level::LEVEL3_1},
  {"4",   Level::LEVEL4},
  {"4.1", Level::LEVEL4_1},
  {"5",   Level::LEVEL5},
  {"5.1", Level::LEVEL5_1},
  {"5.2", Level::LEVEL5_2},
  {"6",   Level::LEVEL6},
  {"6.1", Level::LEVEL6_1},
  {"6.2", Level::LEVEL6_2},
#if JVET_X0079_MODIFIED_BITRATES
  {"6.3", Level::LEVEL6_3},
#endif
  {"8.5", Level::LEVEL8_5},
};

#if !DPB_ENCODER_USAGE_CHECK
#if JVET_X0079_MODIFIED_BITRATES
UInt g_uiMaxCpbSize[2][28] =
{
  //            LEVEL1,          LEVEL2,  LEVEL2_1,      LEVEL3,  LEVEL3_1,       LEVEL4,   LEVEL4_1,       LEVEL5,    LEVEL5_1,  LEVEL5_2,     LEVEL6,    LEVEL6_1,  LEVEL6_2,  LEVEL6_3
  { 0, 0, 0, 0, 350000, 0, 0, 0, 1500000, 3000000, 0, 0, 6000000, 10000000, 0, 0, 12000000, 20000000, 0, 0,  25000000,  40000000,  60000000, 0,  60000000, 120000000, 240000000,  240000000 },
  { 0, 0, 0, 0,      0, 0, 0, 0,       0,       0, 0, 0,       0,        0, 0, 0, 30000000, 50000000, 0, 0, 100000000, 160000000, 240000000, 0, 240000000, 480000000, 800000000, 1600000000 }
};
#else
UInt g_uiMaxCpbSize[2][21] =
{
  //         LEVEL1,        LEVEL2,LEVEL2_1,     LEVEL3, LEVEL3_1,      LEVEL4, LEVEL4_1,       LEVEL5,  LEVEL5_1,  LEVEL5_2,    LEVEL6,  LEVEL6_1,  LEVEL6_2 
  { 0, 0, 0, 350000, 0, 0, 1500000, 3000000, 0, 6000000, 10000000, 0, 12000000, 20000000, 0,  25000000,  40000000,  60000000,  60000000, 120000000, 240000000 },
  { 0, 0, 0,      0, 0, 0,       0,       0, 0,       0,        0, 0, 30000000, 50000000, 0, 100000000, 160000000, 240000000, 240000000, 480000000, 800000000 }
};
#endif
#endif

static const struct MapStrToCostMode
{
  const TChar* str;
  CostMode    value;
}
strToCostMode[] =
{
  {"lossy",                     COST_STANDARD_LOSSY},
  {"sequence_level_lossless",   COST_SEQUENCE_LEVEL_LOSSLESS},
  {"lossless",                  COST_LOSSLESS_CODING},
  {"mixed_lossless_lossy",      COST_MIXED_LOSSLESS_LOSSY_CODING}
};

static const struct MapStrToScalingListMode
{
  const TChar* str;
  ScalingListMode value;
}
strToScalingListMode[] =
{
  {"0",       SCALING_LIST_OFF},
  {"1",       SCALING_LIST_DEFAULT},
  {"2",       SCALING_LIST_FILE_READ},
  {"off",     SCALING_LIST_OFF},
  {"default", SCALING_LIST_DEFAULT},
  {"file",    SCALING_LIST_FILE_READ}
};

template<typename T, typename P>
static std::string enumToString(P map[], UInt mapLen, const T val)
{
  for (UInt i = 0; i < mapLen; i++)
  {
    if (val == map[i].value)
    {
      return map[i].str;
    }
  }
  return std::string();
}

template<typename T, typename P>
static istream& readStrToEnum(P map[], UInt mapLen, istream &in, T &val)
{
  string str;
  in >> str;

  UInt i=0;
  for (; i < mapLen && str!=map[i].str; i++);

  if (i < mapLen)
  {
    val = map[i].value;
  }
  else
  {
    in.setstate(ios::failbit);
  }
  return in;
}

//inline to prevent compiler warnings for "unused static function"

static inline istream& operator >> (istream &in, UIProfileName &profile)
{
  return readStrToEnum(strToUIProfileName, sizeof(strToUIProfileName)/sizeof(*strToUIProfileName), in, profile);
}

namespace Level
{
  static inline istream& operator >> (istream &in, Tier &tier)
  {
    return readStrToEnum(strToTier, sizeof(strToTier)/sizeof(*strToTier), in, tier);
  }

  static inline istream& operator >> (istream &in, Name &level)
  {
    return readStrToEnum(strToLevel, sizeof(strToLevel)/sizeof(*strToLevel), in, level);
  }
}

static inline istream& operator >> (istream &in, CostMode &mode)
{
  return readStrToEnum(strToCostMode, sizeof(strToCostMode)/sizeof(*strToCostMode), in, mode);
}

static inline istream& operator >> (istream &in, ScalingListMode &mode)
{
  return readStrToEnum(strToScalingListMode, sizeof(strToScalingListMode)/sizeof(*strToScalingListMode), in, mode);
}

#if !JVET_X0048_X0103_FILM_GRAIN
template <class T>
struct SMultiValueInput
{
  const T              minValIncl;
  const T              maxValIncl;
  const std::size_t    minNumValuesIncl;
  const std::size_t    maxNumValuesIncl; // Use 0 for unlimited
        std::vector<T> values;
  SMultiValueInput() : minValIncl(0), maxValIncl(0), minNumValuesIncl(0), maxNumValuesIncl(0), values() { }
  SMultiValueInput(std::vector<T> &defaults) : minValIncl(0), maxValIncl(0), minNumValuesIncl(0), maxNumValuesIncl(0), values(defaults) { }
  SMultiValueInput(const T &minValue, const T &maxValue, std::size_t minNumberValues=0, std::size_t maxNumberValues=0)
    : minValIncl(minValue), maxValIncl(maxValue), minNumValuesIncl(minNumberValues), maxNumValuesIncl(maxNumberValues), values()  { }
  SMultiValueInput(const T &minValue, const T &maxValue, std::size_t minNumberValues, std::size_t maxNumberValues, const T* defValues, const UInt numDefValues)
    : minValIncl(minValue), maxValIncl(maxValue), minNumValuesIncl(minNumberValues), maxNumValuesIncl(maxNumberValues), values(defValues, defValues+numDefValues)  { }
  SMultiValueInput<T> &operator=(const std::vector<T> &userValues) { values=userValues; return *this; }
  SMultiValueInput<T> &operator=(const SMultiValueInput<T> &userValues) { values=userValues.values; return *this; }

  T readValue(const TChar *&pStr, Bool &bSuccess);

  istream& readValues(std::istream &in);
};
#endif

template <class T>
static inline istream& operator >> (std::istream &in, SMultiValueInput<T> &values)
{
  return values.readValues(in);
}

template <class T>
T SMultiValueInput<T>::readValue(const char *&pStr, bool &bSuccess)
{
  T val=T();
  std::string s(pStr);
  std::replace(s.begin(), s.end(), ',', ' '); // make comma separated into space separated
  std::istringstream iss(s);
  iss>>val;
  bSuccess=!iss.fail() // check nothing has gone wrong
           && !(val<minValIncl || val>maxValIncl) // check value is within range
           && (int)iss.tellg() !=  0 // check we've actually read something
           && (iss.eof() || iss.peek()==' '); // check next character is a space, or eof
  pStr+= (iss.eof() ? s.size() : (std::size_t)iss.tellg());
  return val;
}

template <class T>
istream& SMultiValueInput<T>::readValues(std::istream &in)
{
  values.clear();
  string str;
  while (!in.eof())
  {
    string tmp; in >> tmp; str+=" " + tmp;
  }
  if (!str.empty())
  {
    const TChar *pStr=str.c_str();
    // soak up any whitespace
    for(;isspace(*pStr);pStr++);

    while (*pStr != 0)
    {
      Bool bSuccess=true;
      T val=readValue(pStr, bSuccess);
      if (!bSuccess)
      {
        in.setstate(ios::failbit);
        break;
      }

      if (maxNumValuesIncl != 0 && values.size() >= maxNumValuesIncl)
      {
        in.setstate(ios::failbit);
        break;
      }
      values.push_back(val);
      // soak up any whitespace and up to 1 comma.
      for(;isspace(*pStr);pStr++);
      if (*pStr == ',')
      {
        pStr++;
      }
      for(;isspace(*pStr);pStr++);
    }
  }
  if (values.size() < minNumValuesIncl)
  {
    in.setstate(ios::failbit);
  }
  return in;
}

template <class T>
static inline istream& operator >> (std::istream &in, TAppEncCfg::OptionalValue<T> &value)
{
  in >> std::ws;
  if (in.eof())
  {
    value.bPresent=false;
  }
  else
  {
    in >> value.value;
    value.bPresent=true;
  }
  return in;
}

template <class T1, class T2>
static inline istream& operator >> (std::istream &in, std::map<T1, T2> &map)
{
  T1 key;
  T2 value;
  try
  {
    in >> key;
    in >> value;
  }
  catch (...)
  {
    in.setstate(ios::failbit);
  }

  map[key] = value;
  return in;
}

static Void
automaticallySelectRExtProfile(const Bool bUsingGeneralRExtTools,
                               const Bool bUsingChromaQPAdjustment,
                               const Bool bUsingExtendedPrecision,
                               const Bool bIntraConstraintFlag,
                               UInt &bitDepthConstraint,
                               ChromaFormat &chromaFormatConstraint,
                               const Int  maxBitDepth,
                               const ChromaFormat chromaFormat)
{
  // Try to choose profile, according to table in Q1013.
  UInt trialBitDepthConstraint=maxBitDepth;
  if (trialBitDepthConstraint<8)
  {
    trialBitDepthConstraint=8;
  }
  else if (trialBitDepthConstraint==9 || trialBitDepthConstraint==11)
  {
    trialBitDepthConstraint++;
  }
  else if (trialBitDepthConstraint>12)
  {
    trialBitDepthConstraint=16;
  }

  // both format and bit depth constraints are unspecified
  if (bUsingExtendedPrecision || trialBitDepthConstraint==16)
  {
    bitDepthConstraint = 16;
    chromaFormatConstraint = (!bIntraConstraintFlag && chromaFormat==CHROMA_400) ? CHROMA_400 : CHROMA_444;
  }
  else if (bUsingGeneralRExtTools)
  {
    if (chromaFormat == CHROMA_400 && !bIntraConstraintFlag)
    {
      bitDepthConstraint = 16;
      chromaFormatConstraint = CHROMA_400;
    }
    else
    {
      bitDepthConstraint = trialBitDepthConstraint;
      chromaFormatConstraint = CHROMA_444;
    }
  }
  else if (chromaFormat == CHROMA_400)
  {
    if (bIntraConstraintFlag)
    {
      chromaFormatConstraint = CHROMA_420; // there is no intra 4:0:0 profile.
      bitDepthConstraint     = trialBitDepthConstraint;
    }
    else
    {
      chromaFormatConstraint = CHROMA_400;
      bitDepthConstraint     = trialBitDepthConstraint == 8 ? 8 : 12;
    }
  }
  else
  {
    bitDepthConstraint = trialBitDepthConstraint;
    chromaFormatConstraint = chromaFormat;
    if (bUsingChromaQPAdjustment && chromaFormat == CHROMA_420)
    {
      chromaFormatConstraint = CHROMA_422; // 4:2:0 cannot use the chroma qp tool.
    }
    if (chromaFormatConstraint == CHROMA_422 && bitDepthConstraint == 8)
    {
      bitDepthConstraint = 10; // there is no 8-bit 4:2:2 profile.
    }
    if (chromaFormatConstraint == CHROMA_420 && !bIntraConstraintFlag)
    {
      bitDepthConstraint = 12; // there is no 8 or 10-bit 4:2:0 inter RExt profile.
    }
  }
}
// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param  argc        number of arguments
    \param  argv        array of arguments
    \retval             true when success
 */
Bool TAppEncCfg::parseCfg( Int argc, TChar* argv[] )
{
  Bool do_help = false;

  Int tmpChromaFormat;
  Int tmpInputChromaFormat;
  Int tmpConstraintChromaFormat;
  Int tmpWeightedPredictionMethod;
  Int tmpFastInterSearchMode;
  Int tmpMotionEstimationSearchMethod;
  Int tmpSliceMode;
  Int tmpSliceSegmentMode;
  Int tmpDecodedPictureHashSEIMappedType;
  string inputColourSpaceConvert;
  string inputPathPrefix;
  UIProfileName UIProfile;
  Int saoOffsetBitShift[MAX_NUM_CHANNEL_TYPE];

  // Multi-value input fields:                                // minval, maxval (incl), min_entries, max_entries (incl) [, default values, number of default values]
  SMultiValueInput<UInt> cfg_ColumnWidth                     (0, std::numeric_limits<UInt>::max(), 0, std::numeric_limits<UInt>::max());
  SMultiValueInput<UInt> cfg_RowHeight                       (0, std::numeric_limits<UInt>::max(), 0, std::numeric_limits<UInt>::max());
  SMultiValueInput<Int>  cfg_startOfCodedInterval            (std::numeric_limits<Int>::min(), std::numeric_limits<Int>::max(), 0, 1<<16);
  SMultiValueInput<Int>  cfg_codedPivotValue                 (std::numeric_limits<Int>::min(), std::numeric_limits<Int>::max(), 0, 1<<16);
  SMultiValueInput<Int>  cfg_targetPivotValue                (std::numeric_limits<Int>::min(), std::numeric_limits<Int>::max(), 0, 1<<16);

  SMultiValueInput<Double> cfg_adIntraLambdaModifier         (0, std::numeric_limits<Double>::max(), 0, MAX_TLAYER); ///< Lambda modifier for Intra pictures, one for each temporal layer. If size>temporalLayer, then use [temporalLayer], else if size>0, use [size()-1], else use m_adLambdaModifier.

  const Int defaultLumaLevelTodQp_QpChangePoints[]   =  {-3,  -2,  -1,   0,   1,   2,   3,   4,   5,   6};
  const Int defaultLumaLevelTodQp_LumaChangePoints[] =  { 0, 301, 367, 434, 501, 567, 634, 701, 767, 834};
  SMultiValueInput<Int>  cfg_lumaLeveltoDQPMappingQP         (-MAX_QP, MAX_QP,                    0, LUMA_LEVEL_TO_DQP_LUT_MAXSIZE, defaultLumaLevelTodQp_QpChangePoints,   sizeof(defaultLumaLevelTodQp_QpChangePoints  )/sizeof(Int));
  SMultiValueInput<Int>  cfg_lumaLeveltoDQPMappingLuma       (0, std::numeric_limits<Int>::max(), 0, LUMA_LEVEL_TO_DQP_LUT_MAXSIZE, defaultLumaLevelTodQp_LumaChangePoints, sizeof(defaultLumaLevelTodQp_LumaChangePoints)/sizeof(Int));
  UInt lumaLevelToDeltaQPMode;

  const UInt defaultInputKneeCodes[3]  = { 600, 800, 900 };
  const UInt defaultOutputKneeCodes[3] = { 100, 250, 450 };
  Int cfg_kneeSEINumKneePointsMinus1=0;
  SMultiValueInput<UInt> cfg_kneeSEIInputKneePointValue      (1,  999, 0, 999, defaultInputKneeCodes,  sizeof(defaultInputKneeCodes )/sizeof(UInt));
  SMultiValueInput<UInt> cfg_kneeSEIOutputKneePointValue     (0, 1000, 0, 999, defaultOutputKneeCodes, sizeof(defaultOutputKneeCodes)/sizeof(UInt));
