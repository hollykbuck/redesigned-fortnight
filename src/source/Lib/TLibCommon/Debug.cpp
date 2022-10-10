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

/** \file     Debug.cpp
    \brief    Defines types and objects for environment-variable-based debugging and feature control
*/

#include "Debug.h"
#include <algorithm>
#include <math.h>
#include "TComDataCU.h"
#include "TComPic.h"
#include "TComYuv.h"

static const UInt settingNameWidth  = 66;
static const UInt settingHelpWidth  = 84;
static const UInt settingValueWidth = 3;

#if DEBUG_STRING
// these strings are used to reorder the debug output so that the encoder and decoder match.
const TChar *debug_reorder_data_inter_token[MAX_NUM_COMPONENT+1]
 = {"Start of channel 0 inter debug\n", "Start of channel 1 inter debug\n", "Start of channel 2 inter debug\n", "End of inter residual debug\n"} ;
const TChar *partSizeToString[NUMBER_OF_PART_SIZES]={"2Nx2N(0)", "2NxN(1)", "Nx2N(2)", "NxN(3)", "2Nx(N/2+3N/2)(4)", "2Nx(3N/2+N/2)(5)", "(N/2+3N/2)x2N(6)", "(3N/2+N/2)x2N(7)"};
#endif

// --------------------------------------------------------------------------------------------------------------------- //

//EnvVar definition

std::list<std::pair<std::string, std::string> > &EnvVar::getEnvVarList()
{
  static std::list<std::pair<std::string, std::string> > varInfoList;
  return varInfoList;
}

std::list<EnvVar*> &EnvVar::getEnvVarInUse()
{
  static std::list<EnvVar*> varInUseList;
  return varInUseList;
}

static inline Void printPair(const std::pair<std::string, std::string> &p)
{
  if (p.second=="")
  {
    std::cout << "\n" << std::setw(settingNameWidth) << p.first << "\n" << std::endl;
  }
  else
  {
    std::cout << std::setw(settingNameWidth) << p.first << ":   " << p.second << "\n" << std::endl;
  }
}

static inline Void printVal(const EnvVar* env)
{
  std::cout << std::setw(settingNameWidth) << env->getName() << " = " << std::setw(settingValueWidth) << env->getInt() << " (string = " << std::setw(15) << env->getString() << ")" << std::endl;
}

//static inline Bool sameEnvName( const std::pair<std::string, std::string> &a,
//                                const std::pair<std::string, std::string> &b )
//{
//  // only check env name
//  return (a.first==b.first);
//}

Void EnvVar::printEnvVar()
{
//  getEnvVarList().unique(sameEnvName);
  if (getEnvVarList().size()!=0)
  {
    std::cout << "--- Environment variables:\n" << std::endl;
    for_each(getEnvVarList().begin(), getEnvVarList().end(), printPair);
  }
  std::cout << std::endl;
}

Void EnvVar::printEnvVarInUse()
{
  if (getEnvVarInUse().size()!=0)
  {
    std::cout << "RExt Environment variables set as follows: \n" << std::endl;
    for_each(getEnvVarInUse().begin(), getEnvVarInUse().end(), printVal);
  }
  std::cout << std::endl;
}

EnvVar::EnvVar(const std::string &sName, const std::string &sDefault, const std::string &sHelp) :
                                                                m_sName(sName),
                                                                m_sHelp(sHelp),
                                                                m_sVal(),
                                                                m_dVal(0),
                                                                m_iVal(0),
                                                                m_bSet(false)
{
  if (getenv(m_sName.c_str()))
  {
    m_sVal = getenv(m_sName.c_str());
    m_bSet = true;
    getEnvVarInUse().push_back(this);
  }
  else
  {
    m_sVal = sDefault;
  }

  m_dVal = strtod(m_sVal.c_str(), 0);
  m_iVal = Int(m_dVal);

  getEnvVarList().push_back( std::pair<std::string, std::string>(m_sName, indentNewLines(lineWrap(splitOnSettings(m_sHelp), settingHelpWidth), (settingNameWidth + 4))) );
}


// --------------------------------------------------------------------------------------------------------------------- //

// Debug environment variables:

EnvVar Debug("-- Debugging","","");

EnvVar DebugOptionList::DebugSBAC             ("DEBUG_SBAC",        "0", "Output debug data from SBAC entropy coder (coefficient data etc.)"                              );
EnvVar DebugOptionList::DebugRQT              ("DEBUG_RQT",         "0", "Output RQT debug data from entropy coder"                                                       );
EnvVar DebugOptionList::DebugPred             ("DEBUG_PRED",        "0", "Output prediction debug"                                                                        );
EnvVar DebugOptionList::ForceLumaMode         ("FORCE_LUMA_MODE",   "0", "Force a particular intra direction for Luma (0-34)"                                             );
EnvVar DebugOptionList::ForceChromaMode       ("FORCE_CHROMA_MODE", "0", "Force a particular intra direction for chroma (0-5)"                                            );

#if DEBUG_STRING
EnvVar DebugOptionList::DebugString_Structure ("DEBUG_STRUCTURE",   "0", "Produce output on chosen structure                        bit0=intra, bit1=inter");
EnvVar DebugOptionList::DebugString_Pred      ("DEBUG_PRED",        "0", "Produce output on prediction data.                        bit0=intra, bit1=inter");
EnvVar DebugOptionList::DebugString_Resi      ("DEBUG_RESI",        "0", "Produce output on residual data.                          bit0=intra, bit1=inter");
EnvVar DebugOptionList::DebugString_Reco      ("DEBUG_RECO",        "0", "Produce output on reconstructed data.                     bit0=intra, bit1=inter");
EnvVar DebugOptionList::DebugString_InvTran   ("DEBUG_INV_QT",      "0", "Produce output on inverse-quantiser and transform stages. bit0=intra, bit1=inter");
#endif

// --------------------------------------------------------------------------------------------------------------------- //

//macro value printing function

Void printMacroSettings()
{
  std::cout << "Non-environment-variable-controlled macros set as follows: \n" << std::endl;

  //------------------------------------------------

  //setting macros

  PRINT_CONSTANT(RExt__DECODER_DEBUG_BIT_STATISTICS,                                settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(RExt__HIGH_BIT_DEPTH_SUPPORT,                                      settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(RExt__HIGH_PRECISION_FORWARD_TRANSFORM,                            settingNameWidth, settingValueWidth);

  PRINT_CONSTANT(O0043_BEST_EFFORT_DECODING,                                        settingNameWidth, settingValueWidth);

  PRINT_CONSTANT(ME_ENABLE_ROUNDING_OF_MVS,                                         settingNameWidth, settingValueWidth);

  //------------------------------------------------

  std::cout << std::endl;
}


// --------------------------------------------------------------------------------------------------------------------- //

//Debugging

UInt  g_debugCounter  = 0;

#if DEBUG_ENCODER_SEARCH_BINS
const UInt debugEncoderSearchBinTargetLine = 0;
const UInt debugEncoderSearchBinWindow     = 1000000;
#endif

#if DEBUG_CABAC_BINS
