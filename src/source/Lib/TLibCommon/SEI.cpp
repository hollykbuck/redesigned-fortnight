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

/** \file     SEI.cpp
    \brief    helper functions for SEI handling
*/

#include "CommonDef.h"
#include "SEI.h"
#include <iostream>

const std::vector<SEI::PayloadType> SEI::prefix_sei_messages({
  SEI::BUFFERING_PERIOD,
  SEI::PICTURE_TIMING,
  SEI::PAN_SCAN_RECT,
  SEI::FILLER_PAYLOAD,
  SEI::USER_DATA_REGISTERED_ITU_T_T35,
  SEI::USER_DATA_UNREGISTERED,
  SEI::RECOVERY_POINT,
  SEI::SCENE_INFO,
  SEI::PICTURE_SNAPSHOT,
  SEI::PROGRESSIVE_REFINEMENT_SEGMENT_START,
  SEI::PROGRESSIVE_REFINEMENT_SEGMENT_END,
  SEI::FILM_GRAIN_CHARACTERISTICS,
  SEI::POST_FILTER_HINT,
  SEI::TONE_MAPPING_INFO,
  SEI::FRAME_PACKING,
  SEI::DISPLAY_ORIENTATION,
  SEI::GREEN_METADATA,
  SEI::SOP_DESCRIPTION,
  SEI::ACTIVE_PARAMETER_SETS,
  SEI::DECODING_UNIT_INFO,
  SEI::TEMPORAL_LEVEL0_INDEX,
  SEI::SCALABLE_NESTING,
  SEI::REGION_REFRESH_INFO,
  SEI::NO_DISPLAY,
  SEI::TIME_CODE,
  SEI::MASTERING_DISPLAY_COLOUR_VOLUME,
  SEI::SEGM_RECT_FRAME_PACKING,
  SEI::TEMP_MOTION_CONSTRAINED_TILE_SETS,
  SEI::CHROMA_RESAMPLING_FILTER_HINT,
  SEI::KNEE_FUNCTION_INFO,
  SEI::COLOUR_REMAPPING_INFO,
  SEI::DEINTERLACE_FIELD_IDENTIFICATION,
  SEI::CONTENT_LIGHT_LEVEL_INFO,
  SEI::DEPENDENT_RAP_INDICATION,
  SEI::CODED_REGION_COMPLETION,
  SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS,
  SEI::AMBIENT_VIEWING_ENVIRONMENT
  , SEI::CONTENT_COLOUR_VOLUME
  , SEI::EQUIRECTANGULAR_PROJECTION
  , SEI::SPHERE_ROTATION
  , SEI::OMNI_VIEWPORT
  , SEI::CUBEMAP_PROJECTION
  , SEI::REGION_WISE_PACKING
  , SEI::FISHEYE_VIDEO_INFO
  , SEI::REGIONAL_NESTING
#if SHUTTER_INTERVAL_SEI_MESSAGE
  , SEI::SHUTTER_INTERVAL_INFO
#endif
});

const std::vector<SEI::PayloadType> SEI::suffix_sei_messages({
  SEI::FILLER_PAYLOAD,
  SEI::USER_DATA_REGISTERED_ITU_T_T35,
  SEI::USER_DATA_UNREGISTERED,
  SEI::PROGRESSIVE_REFINEMENT_SEGMENT_END,
  SEI::POST_FILTER_HINT,
  SEI::DECODED_PICTURE_HASH,
  SEI::CODED_REGION_COMPLETION,
