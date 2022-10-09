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
