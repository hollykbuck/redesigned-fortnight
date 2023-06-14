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

/** \file     TypeDef.h
    \brief    Define macros, basic types, new types and enumerations
*/

#ifndef __TYPEDEF__
#define __TYPEDEF__

#ifndef __COMMONDEF__
#error Include CommonDef.h not TypeDef.h
#endif

#include <vector>
#include <utility>
#include <iostream>

//! \ingroup TLibCommon
//! \{

#define JCTVC_AD0021_SEI_MANIFEST                         1  // JCTVC_AD0021(JVET-T0056): SEI manifest SEI message
#define JCTVC_AD0021_SEI_PREFIX_INDICATION                1  // JCTVC_AD0021(JVET-T0056): SEI prefix indication SEI message
#define JVET_X0079_MODIFIED_BITRATES                      1 // JVET-X0079: Addition of level 6.3

#define JVET_V0078                                        1 // JVET-V0078: QP control for very smooth blocks
#define JVET_Y0077_BIM                                    1 // JVET-Y0077: Block Importance Mapping

#define JVET_Y0105_SW_AND_QDF                             1 // An adaptive smooth window (SW) size and extension of quality dependency factor (QDF) to low frame rate in rate control 


// ====================================================================================================================
// Debugging
// ====================================================================================================================

#define DEBUG_STRING                                      0 ///< When enabled, prints out final decision debug info at encoder and decoder
#define DEBUG_ENCODER_SEARCH_BINS                         0 ///< When enabled, prints out each bin as it is coded during encoder search
#define DEBUG_CABAC_BINS                                  0 ///< When enabled, prints out each bin as it is coded during final encode and decode
#define DEBUG_INTRA_SEARCH_COSTS                          0 ///< When enabled, prints out the cost for each mode during encoder search
#define DEBUG_TRANSFORM_AND_QUANTISE                      0 ///< When enabled, prints out each TU as it passes through the transform-quantise-dequantise-inverseTransform process

#define ENVIRONMENT_VARIABLE_DEBUG_AND_TEST               0 ///< When enabled, allows control of debug modifications via environment variables
#define PRINT_MACRO_VALUES                                1 ///< When enabled, the encoder prints out a list of the non-environment-variable controlled macros and their values on startup

// TODO: rename this macro to DECODER_DEBUG_BIT_STATISTICS (may currently cause merge issues with other branches)
// This can be enabled by the makefile
#ifndef RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS                0 ///< 0 (default) = decoder reports as normal, 1 = decoder produces bit usage statistics (will impact decoder run time by up to ~10%)
#endif

// This can be enabled by the makefile
#ifndef ENC_DEC_TRACE
#define ENC_DEC_TRACE                                     0
#endif
#define DEC_NUH_TRACE                                     0 ///< When trace enabled, enable tracing of NAL unit headers at the decoder (currently not possible at the encoder)

#define PRINT_RPS_INFO                                    0 ///< Enable/disable the printing of bits used to send the RPS.

#define MCTS_EXTRACTION                                   1 ///< Additional project for MCTS Extraction as in JCTVC-AC1005

// ====================================================================================================================
// Tool Switches - transitory (these macros are likely to be removed in future revisions)
// ====================================================================================================================

#define JVET_V0056_MCTF                                   1 // JVET-V0056: Changes to MCTF

#define JVET_K0390_RATE_CTRL                              1
#if JVET_K0390_RATE_CTRL
#define JVET_M0600_RATE_CTRL                              1
#endif
#define DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES  1 ///< TODO: integrate this macro into a broader conformance checking system.
#define MCTS_ENC_CHECK                                    1  ///< Temporal MCTS encoder constraint and decoder checks. Also requires SEITMCTSTileConstraint to be enabled to enforce constraint
#define SHUTTER_INTERVAL_SEI_MESSAGE                      1  ///< support for shutter interval SEI message 
#define SEI_ENCODER_CONTROL                               1  ///< add encoder control for the following SEI: film grain characteristics, content light level, ambient viewing environment
#define DPB_ENCODER_USAGE_CHECK                           1 ///< Adds DPB encoder usage check.
#define JVET_X0048_X0103_FILM_GRAIN                       1 ///< JVET-X0048-X0103: SMPTE RDD-5 based film grain analysis and synthesis model for film grain characterstics (FGC) SEI

#define JVET_T0050_ANNOTATED_REGIONS_SEI                  1 ///< Detect static objects and use it in Annotated Regions SEI message

#if SHUTTER_INTERVAL_SEI_MESSAGE
#define SHUTTER_INTERVAL_SEI_PROCESSING                   1 ///< JCTVC-AM0024: pre-/post-processing to use shutter interval SEI
#endif
// ====================================================================================================================
// Tool Switches
// ====================================================================================================================

// Please also refer to "TDecConformance.h" for DECODER_PARTIAL_CONFORMANCE_CHECK

#ifndef EXTENSION_360_VIDEO
#define EXTENSION_360_VIDEO                               0   ///< extension for 360/spherical video coding support; this macro should be controlled by makefile, as it would be used to control whether the library is built and linked
#endif

#define REDUCED_ENCODER_MEMORY                            1 ///< When 1, the encoder will allocate TComPic memory when required and release it when no longer required.

#define ADAPTIVE_QP_SELECTION                             1 ///< G382: Adaptive reconstruction levels, non-normative part for adaptive QP selection

#define AMP_ENC_SPEEDUP                                   1 ///< encoder only speed-up by AMP mode skipping
#if AMP_ENC_SPEEDUP
#define AMP_MRG                                           1 ///< encoder only force merge for AMP partition (no motion search for AMP)
#endif

#define FAST_BIT_EST                                      1 ///< G763: Table-based bit estimation for CABAC

#define HHI_RQT_INTRA_SPEEDUP                             1 ///< tests one best mode with full rqt
#define HHI_RQT_INTRA_SPEEDUP_MOD                         0 ///< tests two best modes with full rqt

#if HHI_RQT_INTRA_SPEEDUP_MOD && !HHI_RQT_INTRA_SPEEDUP
#error
#endif

#define MATRIX_MULT                                       0 ///< Brute force matrix multiplication instead of partial butterfly

#define O0043_BEST_EFFORT_DECODING                        0 ///< 0 (default) = disable code related to best effort decoding, 1 = enable code relating to best effort decoding [ decode-side only ].

#define ME_ENABLE_ROUNDING_OF_MVS                         1 ///< 0 (default) = disables rounding of motion vectors when right shifted,  1 = enables rounding

#define RDOQ_CHROMA_LAMBDA                                1 ///< F386: weighting of chroma for RDOQ

// This can be enabled by the makefile
#ifndef RExt__HIGH_BIT_DEPTH_SUPPORT
#define RExt__HIGH_BIT_DEPTH_SUPPORT                      0 ///< 0 (default) use data type definitions for 8-10 bit video, 1 = use larger data types to allow for up to 16-bit video (originally developed as part of N0188)
#endif

#if defined __SSE2__ || defined __AVX2__ || defined __AVX__ || defined _M_AMD64 || defined _M_X64
#define VECTOR_CODING__INTERPOLATION_FILTER               1 ///< enable vector coding for the interpolation filter. 1 (default if SSE possible) disable SSE vector coding. Should not affect RD costs/decisions. Code back-ported from JEM2.0.
#define VECTOR_CODING__DISTORTION_CALCULATIONS            1 ///< enable vector coding for distortion calculations   1 (default if SSE possible) disable SSE vector coding. Should not affect RD costs/decisions. Code back-ported from JEM2.0.
#else
#define VECTOR_CODING__INTERPOLATION_FILTER               0 ///< enable vector coding for the interpolation filter. 0 (default if SSE not possible) disable SSE vector coding. Should not affect RD costs/decisions. Code back-ported from JEM2.0.
#define VECTOR_CODING__DISTORTION_CALCULATIONS            0 ///< enable vector coding for distortion calculations   0 (default if SSE not possible) disable SSE vector coding. Should not affect RD costs/decisions. Code back-ported from JEM2.0.
#endif

// ====================================================================================================================
// Derived macros
// ====================================================================================================================

#if RExt__HIGH_BIT_DEPTH_SUPPORT
#define FULL_NBIT                                         1 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#define RExt__HIGH_PRECISION_FORWARD_TRANSFORM            1 ///< 0 use original 6-bit transform matrices for both forward and inverse transform, 1 (default) = use original matrices for inverse transform and high precision matrices for forward transform
#else
#define FULL_NBIT                                         1 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#define RExt__HIGH_PRECISION_FORWARD_TRANSFORM            0 ///< 0 (default) use original 6-bit transform matrices for both forward and inverse transform, 1 = use original matrices for inverse transform and high precision matrices for forward transform
#endif

#if FULL_NBIT
# define DISTORTION_PRECISION_ADJUSTMENT(x)  0
#else
# define DISTORTION_PRECISION_ADJUSTMENT(x) (x)
#endif

#if DEBUG_STRING
  #define DEBUG_STRING_PASS_INTO(name) , name
  #define DEBUG_STRING_PASS_INTO_OPTIONAL(name, exp) , (exp==0)?0:name
  #define DEBUG_STRING_FN_DECLARE(name) , std::string &name
  #define DEBUG_STRING_FN_DECLAREP(name) , std::string *name
  #define DEBUG_STRING_NEW(name) std::string name;
  #define DEBUG_STRING_OUTPUT(os, name) os << name;
  #define DEBUG_STRING_APPEND(str1, str2) str1+=str2;
  #define DEBUG_STRING_SWAP(str1, str2) str1.swap(str2);
  #define DEBUG_STRING_CHANNEL_CONDITION(compID) (true)
  #include <sstream>
  #include <iomanip>
#else
  #define DEBUG_STRING_PASS_INTO(name)
  #define DEBUG_STRING_PASS_INTO_OPTIONAL(name, exp)
  #define DEBUG_STRING_FN_DECLARE(name)
  #define DEBUG_STRING_FN_DECLAREP(name)
  #define DEBUG_STRING_NEW(name)
  #define DEBUG_STRING_OUTPUT(os, name)
  #define DEBUG_STRING_APPEND(str1, str2)
  #define DEBUG_STRING_SWAP(srt1, str2)
