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

#include <stdint.h>
#include <cassert>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string.h>

#include "TLibDecoder/AnnexBread.h"

using namespace std;

static const struct {
  AnnexBStats expected;
  unsigned data_len;
  const char data[10];
} tests[] = {
  /* trivial cases: startcode, no payload */
  {{0, 0, 3, 0, 0}, 3, {0,0,1}},
  {{0, 1, 3, 0, 0}, 4, {0,0,0,1}},
  {{2, 1, 3, 0, 0}, 6, {0,0,0,0,0,1}},
  /* trivial cases: startcode, payload */
  {{0, 0, 3, 1, 0}, 4, {0,0,1,2}},
  {{0, 0, 3, 2, 0}, 5, {0,0,1,2,0}},
  {{0, 0, 3, 3, 0}, 6, {0,0,1,2,0,0}},
  {{0, 0, 3, 1, 3}, 7, {0,0,1,2,0,0,0}},
  /* trivial cases: two nal units: extract the first */
  {{0, 0, 3, 1, 0}, 8, {0,0,1,2,0,0,1,3}},
  {{0, 0, 3, 1, 0}, 9, {0,0,1,2,0,0,0,1,3}},
  {{0, 0, 3, 1, 1}, 10, {0,0,1,2,0,0,0,0,1,3}},
  /* edge cases with EOF near start*/
  {{0, 0, 0, 0, 0}, 0, {}},
  {{1, 0, 0, 0, 0}, 1, {0}},
  {{2, 0, 0, 0, 0}, 2, {0,0}},
  {{3, 0, 0, 0, 0}, 3, {0,0,0}},
};

void selftest()
{
  /* test */
  for (unsigned i = 0; i < sizeof(tests)/sizeof(*tests); i++)
  {
    istringstream in(string(tests[i].data, tests[i].data_len));
    InputByteStream bs(in);

    AnnexBStats actual = AnnexBStats();
    vector<uint8_t> nalUnit;

    byteStreamNALUnit(bs, nalUnit, actual);

    cout << "Self-Test: " << i << ", {";
    for (unsigned j = 0; j < tests[i].data_len; j++)
    {
      cout << hex << (unsigned int)tests[i].data[j] << dec;
      if (j < tests[i].data_len-1)
      {
        cout << ",";
      }
    }
    cout << "} ";

    bool ok = true;
#define VERIFY(a,b,m) \
  if (a.m != b.m) { \
    ok = false; \
    cout << endl << "  MISSMATCH " #m << ", E(" << b.m << ") != " << a.m; \
  }
    VERIFY(actual, tests[i].expected, m_numLeadingZero8BitsBytes);
