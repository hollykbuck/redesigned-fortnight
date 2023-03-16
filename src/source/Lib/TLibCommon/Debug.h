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

/** \file     Debug.h
    \brief    Defines types and objects for environment-variable-based debugging and feature control
*/

#ifndef __DEBUG__
#define __DEBUG__

#include <iostream>
#include <iomanip>
#include <string>
#include <list>
#include <stdlib.h>
#include <sstream>
#include <TLibCommon/CommonDef.h>

#if DEBUG_STRING
extern const TChar *debug_reorder_data_inter_token[MAX_NUM_COMPONENT+1];
extern const TChar *partSizeToString[NUMBER_OF_PART_SIZES];
#endif

// ---------------------------------------------------------------------------------------------- //

//constant print-out macro

#define PRINT_CONSTANT(NAME, NAME_WIDTH, VALUE_WIDTH) std::cout << std::setw(NAME_WIDTH) << #NAME << " = " << std::setw(VALUE_WIDTH) << NAME << std::endl;

// ---------------------------------------------------------------------------------------------- //

// ---- Environment variables for test/debug ---- //

class EnvVar
{
private:
  std::string m_sName;
  std::string m_sHelp;
  std::string m_sVal;
  Double      m_dVal;
  Int         m_iVal;
  Bool        m_bSet;

public:

  static std::list< std::pair<std::string, std::string> > &getEnvVarList();
  static std::list<EnvVar*>                               &getEnvVarInUse();
  static Void printEnvVar();
  static Void printEnvVarInUse();

  EnvVar(const std::string &sName, const std::string &sDefault, const std::string &sHelp);

  Double              getDouble()   const       { return m_dVal;    }
  Int                 getInt()      const       { return m_iVal;    }
  const std::string  &getString()   const       { return m_sVal;    }
  Bool                isSet()       const       { return m_bSet;    }
  Bool                isTrue()      const       { return m_iVal!=0; }
  const std::string  &getName()     const       { return m_sName;   }

};


// ---------------------------------------------------------------------------------------------- //

// ---- Control switches for debugging and feature control ---- //

namespace DebugOptionList
{
  extern EnvVar DebugSBAC;
  extern EnvVar DebugRQT;
