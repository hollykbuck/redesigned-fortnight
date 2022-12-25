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

/** \file     SEIFilmGrainSynthesizer.cpp
    \brief    SMPTE RDD5 based film grain synthesis functionality from SEI messages
*/

#include "SEIFilmGrainSynthesizer.h"

#include <stdio.h>
#include <cmath>

#if JVET_X0048_X0103_FILM_GRAIN

/* static look up table definitions */
static const int8_t gaussianLUT[2048] =
{
-11, 12, 103, -11, 42, -35, 12, 59, 77, 98, -87, 3, 65, -78, 45, 56,
-51, 21, 13, -11, -20, -19, 33, -127, 17, -6, -105, 18, 19, 71, 48, -10,
-38, 42, -2, 75, -67, 52, -90, 33, -47, 21, -3, -56, 49, 1, -57, -42,
-1, 120, -127, -108, -49, 9, 14, 127, 122, 109, 52, 127, 2, 7, 114, 19,
30, 12, 77, 112, 82, -61, -127, 111, -52, -29, 2, -49, -24, 58, -29, -73,
12, 112, 67, 79, -3, -114, -87, -6, -5, 40, 58, -81, 49, -27, -31, -34,
-105, 50, 16, -24, -35, -14, -15, -127, -55, -22, -55, -127, -112, 5, -26, -72,
127, 127, -2, 41, 87, -65, -16, 55, 19, 91, -81, -65, -64, 35, -7, -54,
99, -7, 88, 125, -26, 91, 0, 63, 60, -14, -23, 113, -33, 116, 14, 26,
51, -16, 107, -8, 53, 38, -34, 17, -7, 4, -91, 6, 63, 63, -15, 39,
-36, 19, 55, 17, -51, 40, 33, -37, 126, -39, -118, 17, -30, 0, 19, 98,
60, 101, -12, -73, -17, -52, 98, 3, 3, 60, 33, -3, -2, 10, -42, -106,
-38, 14, 127, 16, -127, -31, -86, -39, -56, 46, -41, 75, 23, -19, -22, -70,
74, -54, -2, 32, -45, 17, -92, 59, -64, -67, 56, -102, -29, -87, -34, -92,
68, 5, -74, -61, 93, -43, 14, -26, -38, -126, -17, 16, -127, 64, 34, 31,
93, 17, -51, -59, 71, 77, 81, 127, 127, 61, 33, -106, -93, 0, 0, 75,
-69, 71, 127, -19, -111, 30, 23, 15, 2, 39, 92, 5, 42, 2, -6, 38,
15, 114, -30, -37, 50, 44, 106, 27, 119, 7, -80, 25, -68, -21, 92, -11,
-1, 18, 41, -50, 79, -127, -43, 127, 18, 11, -21, 32, -52, 27, -88, -90,
-39, -19, -10, 24, -118, 72, -24, -44, 2, 12, 86, -107, 39, -33, -127, 47,
51, -24, -22, 46, 0, 15, -35, -69, -2, -74, 24, -6, 0, 29, -3, 45,
32, -32, 117, -45, 79, -24, -17, -109, -10, -70, 88, -48, 24, -91, 120, -37,
50, -127, 58, 32, -82, -10, -17, -7, 46, -127, -15, 89, 127, 17, 98, -39,
-33, 37, 42, -40, -32, -21, 105, -19, 19, 19, -59, -9, 30, 0, -127, 34,
127, -84, 75, 24, -40, -49, -127, -107, -14, 45, -75, 1, 30, -20, 41, -68,
-40, 12, 127, -3, 5, 20, -73, -59, -127, -3, -3, -53, -6, -119, 93, 120,
-80, -50, 0, 20, -46, 67, 78, -12, -22, -127, 36, -41, 56, 119, -5, -116,
-22, 68, -14, -90, 24, -82, -44, -127, 107, -25, -37, 40, -7, -7, -82, 5,
-87, 44, -34, 9, -127, 39, 70, 49, -63, 74, -49, 109, -27, -89, -47, -39,
44, 49, -4, 60, -42, 80, 9, -127, -9, -56, -49, 125, -66, 47, 36, 117,
15, -11, -96, 109, 94, -17, -56, 70, 8, -14, -5, 50, 37, -45, 120, -30,
-76, 40, -46, 6, 3, 69, 17, -78, 1, -79, 6, 127, 43, 26, 127, -127,
28, -55, -26, 55, 112, 48, 107, -1, -77, -1, 53, -9, -22, -43, 123, 108,
127, 102, 68, 46, 5, 1, 123, -13, -55, -34, -49, 89, 65, -105, -5, 94,
-53, 62, 45, 30, 46, 18, -35, 15, 41, 47, -98, -24, 94, -75, 127, -114,
127, -68, 1, -17, 51, -95, 47, 12, 34, -45, -75, 89, -107, -9, -58, -29,
-109, -24, 127, -61, -13, 77, -45, 17, 19, 83, -24, 9, 127, -66, 54, 4,
26, 13, 111, 43, -113, -22, 10, -24, 83, 67, -14, 75, -123, 59, 127, -12,
99, -19, 64, -38, 54, 9, 7, 61, -56, 3, -57, 113, -104, -59, 3, -9,
-47, 74, 85, -55, -34, 12, 118, 28, 93, -72, 13, -99, -72, -20, 30, 72,
-94, 19, -54, 64, -12, -63, -25, 65, 72, -10, 127, 0, -127, 103, -20, -73,
-112, -103, -6, 28, -42, -21, -59, -29, -26, 19, -4, -51, 94, -58, -95, -37,
35, 20, -69, 127, -19, -127, -22, -120, -53, 37, 74, -127, -1, -12, -119, -53,
-28, 38, 69, 17, 16, -114, 89, 62, 24, 37, -23, 49, -101, -32, -9, -95,
-53, 5, 93, -23, -49, -8, 51, 3, -75, -90, -10, -39, 127, -86, -22, 20,
20, 113, 75, 52, -31, 92, -63, 7, -12, 46, 36, 101, -43, -17, -53, -7,
-38, -76, -31, -21, 62, 31, 62, 20, -127, 31, 64, 36, 102, -85, -10, 77,
80, 58, -79, -8, 35, 8, 80, -24, -9, 3, -17, 72, 127, 83, -87, 55,
18, -119, -123, 36, 10, 127, 56, -55, 113, 13, 26, 32, -13, -48, 22, -13,
5, 58, 27, 24, 26, -11, -36, 37, -92, 78, 81, 9, 51, 14, 67, -13,
0, 32, 45, -76, 32, -39, -22, -49, -127, -27, 31, -9, 36, 14, 71, 13,
57, 12, -53, -86, 53, -44, -35, 2, 127, 12, -66, -44, 46, -115, 3, 10,
56, -35, 119, -19, -61, 52, -59, -127, -49, -23, 4, -5, 17, -82, -6, 127,
