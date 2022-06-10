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
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <list>
#include <map>
#include <algorithm>
#include "program_options_lite.h"

using namespace std;

//! \ingroup TAppCommon
//! \{

namespace df
{
  namespace program_options_lite
  {
    ErrorReporter default_error_reporter;

    ostream& ErrorReporter::error(const string& where)
    {
      is_errored = 1;
      cerr << where << " error: ";
      return cerr;
    }

    ostream& ErrorReporter::warn(const string& where)
    {
      cerr << where << " warning: ";
      return cerr;
    }

    Options::~Options()
    {
      for(Options::NamesPtrList::iterator it = opt_list.begin(); it != opt_list.end(); it++)
      {
        delete *it;
      }
    }

    void Options::addOption(OptionBase *opt)
    {
      Names* names = new Names();
      names->opt = opt;
      string& opt_string = opt->opt_string;

      size_t opt_start = 0;
      for (size_t opt_end = 0; opt_end != string::npos;)
      {
        opt_end = opt_string.find_first_of(',', opt_start);
        bool force_short = 0;
        if (opt_string[opt_start] == '-')
        {
          opt_start++;
          force_short = 1;
        }
        string opt_name = opt_string.substr(opt_start, opt_end - opt_start);
        if (force_short || opt_name.size() == 1)
        {
          names->opt_short.push_back(opt_name);
          opt_short_map[opt_name].push_back(names);
        }
        else
        {
          if (opt_name.size() > 0 && opt_name.back() == '*')
          {
