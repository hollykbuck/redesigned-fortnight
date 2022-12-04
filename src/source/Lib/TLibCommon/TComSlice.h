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

/** \file     TComSlice.h
    \brief    slice header and SPS class (header)
*/

#ifndef __TCOMSLICE__
#define __TCOMSLICE__

#include <cstring>
#include <map>
#include <vector>
#include "CommonDef.h"
#include "TComRom.h"
#include "TComList.h"
#include "TComChromaFormat.h"

//! \ingroup TLibCommon
//! \{

class TComPic;
class TComTrQuant;
// ====================================================================================================================
// Constants
// ====================================================================================================================

static const UInt REF_PIC_LIST_NUM_IDX=32;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// Reference Picture Set class
class TComReferencePictureSet
{
private:
  Int  m_numberOfPictures;
  Int  m_numberOfNegativePictures;
  Int  m_numberOfPositivePictures;
  Int  m_numberOfLongtermPictures;
  Int  m_deltaPOC[MAX_NUM_REF_PICS];
  Int  m_POC[MAX_NUM_REF_PICS];
  Bool m_used[MAX_NUM_REF_PICS];
  Bool m_interRPSPrediction;
  Int  m_deltaRIdxMinus1;
  Int  m_deltaRPS;
  Int  m_numRefIdc;
  Int  m_refIdc[MAX_NUM_REF_PICS+1];
  Bool m_bCheckLTMSB[MAX_NUM_REF_PICS];
  Int  m_pocLSBLT[MAX_NUM_REF_PICS];
  Int  m_deltaPOCMSBCycleLT[MAX_NUM_REF_PICS];
  Bool m_deltaPocMSBPresentFlag[MAX_NUM_REF_PICS];

public:
          TComReferencePictureSet();
  virtual ~TComReferencePictureSet();
  Int     getPocLSBLT(Int i) const                     { return m_pocLSBLT[i];               }
  Void    setPocLSBLT(Int i, Int x)                    { m_pocLSBLT[i] = x;                  }
  Int     getDeltaPocMSBCycleLT(Int i) const           { return m_deltaPOCMSBCycleLT[i];     }
  Void    setDeltaPocMSBCycleLT(Int i, Int x)          { m_deltaPOCMSBCycleLT[i] = x;        }
  Bool    getDeltaPocMSBPresentFlag(Int i) const       { return m_deltaPocMSBPresentFlag[i]; }
  Void    setDeltaPocMSBPresentFlag(Int i, Bool x)     { m_deltaPocMSBPresentFlag[i] = x;    }
  Void    setUsed(Int bufferNum, Bool used);
  Void    setDeltaPOC(Int bufferNum, Int deltaPOC);
  Void    setPOC(Int bufferNum, Int deltaPOC);
  Void    setNumberOfPictures(Int numberOfPictures);
  Void    setCheckLTMSBPresent(Int bufferNum, Bool b );
  Bool    getCheckLTMSBPresent(Int bufferNum) const;

  Int     getUsed(Int bufferNum) const;
  Int     getDeltaPOC(Int bufferNum) const;
  Int     getPOC(Int bufferNum) const;
  Int     getNumberOfPictures() const;

  Void    setNumberOfNegativePictures(Int number)      { m_numberOfNegativePictures = number; }
  Int     getNumberOfNegativePictures() const          { return m_numberOfNegativePictures;   }
  Void    setNumberOfPositivePictures(Int number)      { m_numberOfPositivePictures = number; }
  Int     getNumberOfPositivePictures() const          { return m_numberOfPositivePictures;   }
  Void    setNumberOfLongtermPictures(Int number)      { m_numberOfLongtermPictures = number; }
  Int     getNumberOfLongtermPictures() const          { return m_numberOfLongtermPictures;   }

  Void    setInterRPSPrediction(Bool flag)             { m_interRPSPrediction = flag;         }
  Bool    getInterRPSPrediction() const                { return m_interRPSPrediction;         }
  Void    setDeltaRIdxMinus1(Int x)                    { m_deltaRIdxMinus1 = x;               }
  Int     getDeltaRIdxMinus1() const                   { return m_deltaRIdxMinus1;            }
  Void    setDeltaRPS(Int x)                           { m_deltaRPS = x;                      }
  Int     getDeltaRPS() const                          { return m_deltaRPS;                   }
  Void    setNumRefIdc(Int x)                          { m_numRefIdc = x;                     }
  Int     getNumRefIdc() const                         { return m_numRefIdc;                  }

  Void    setRefIdc(Int bufferNum, Int refIdc);
  Int     getRefIdc(Int bufferNum) const ;

  Void    sortDeltaPOC();
  Void    printDeltaPOC() const;
};

/// Reference Picture Set set class
class TComRPSList
{
private:
  std::vector<TComReferencePictureSet> m_referencePictureSets;

public:
                                 TComRPSList()                                            { }
  virtual                        ~TComRPSList()                                           { }

  Void                           create  (Int numberOfEntries)                            { m_referencePictureSets.resize(numberOfEntries);         }
  Void                           destroy ()                                               { }


  TComReferencePictureSet*       getReferencePictureSet(Int referencePictureSetNum)       { return &m_referencePictureSets[referencePictureSetNum]; }
  const TComReferencePictureSet* getReferencePictureSet(Int referencePictureSetNum) const { return &m_referencePictureSets[referencePictureSetNum]; }

  Int                            getNumberOfReferencePictureSets() const                  { return Int(m_referencePictureSets.size());              }
};

/// SCALING_LIST class
class TComScalingList
{
public:
             TComScalingList();
  virtual    ~TComScalingList()                                                 { }
  Int*       getScalingListAddress(UInt sizeId, UInt listId)                    { return &(m_scalingListCoef[sizeId][listId][0]);            } //!< get matrix coefficient
  const Int* getScalingListAddress(UInt sizeId, UInt listId) const              { return &(m_scalingListCoef[sizeId][listId][0]);            } //!< get matrix coefficient
  Void       checkPredMode(UInt sizeId, UInt listId);

  Void       setRefMatrixId(UInt sizeId, UInt listId, UInt u)                   { m_refMatrixId[sizeId][listId] = u;                         } //!< set reference matrix ID
  UInt       getRefMatrixId(UInt sizeId, UInt listId) const                     { return m_refMatrixId[sizeId][listId];                      } //!< get reference matrix ID

  const Int* getScalingListDefaultAddress(UInt sizeId, UInt listId);                                                                           //!< get default matrix coefficient
  Void       processDefaultMatrix(UInt sizeId, UInt listId);

  Void       setScalingListDC(UInt sizeId, UInt listId, UInt u)                 { m_scalingListDC[sizeId][listId] = u;                       } //!< set DC value
  Int        getScalingListDC(UInt sizeId, UInt listId) const                   { return m_scalingListDC[sizeId][listId];                    } //!< get DC value

  Void       setScalingListPredModeFlag(UInt sizeId, UInt listId, Bool bIsDPCM) { m_scalingListPredModeFlagIsDPCM[sizeId][listId] = bIsDPCM; }
  Bool       getScalingListPredModeFlag(UInt sizeId, UInt listId) const         { return m_scalingListPredModeFlagIsDPCM[sizeId][listId];    }

  Void       checkDcOfMatrix();
  Void       processRefMatrix(UInt sizeId, UInt listId , UInt refListId );
  Bool       xParseScalingList(const std::string &fileName);
  Void       setDefaultScalingList();
  Bool       checkDefaultScalingList();

private:
  Void       outputScalingLists(std::ostream &os) const;
  Bool             m_scalingListPredModeFlagIsDPCM [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< reference list index
  Int              m_scalingListDC                 [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< the DC value of the matrix coefficient for 16x16
  UInt             m_refMatrixId                   [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< RefMatrixID
  std::vector<Int> m_scalingListCoef               [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< quantization matrix
};

class ProfileTierLevel
{
  Int               m_profileSpace;
  Level::Tier       m_tierFlag;
  Profile::Name     m_profileIdc;
  Bool              m_profileCompatibilityFlag[32];
  Level::Name       m_levelIdc;

  Bool              m_progressiveSourceFlag;
  Bool              m_interlacedSourceFlag;
  Bool              m_nonPackedConstraintFlag;
  Bool              m_frameOnlyConstraintFlag;
  UInt              m_bitDepthConstraintValue;
  ChromaFormat      m_chromaFormatConstraintValue;
  Bool              m_intraConstraintFlag;
  Bool              m_onePictureOnlyConstraintFlag;
  Bool              m_lowerBitRateConstraintFlag;

public:
                ProfileTierLevel();

  Int           getProfileSpace() const                     { return m_profileSpace;                }
  Void          setProfileSpace(Int x)                      { m_profileSpace = x;                   }

  Level::Tier   getTierFlag() const                         { return m_tierFlag;                    }
  Void          setTierFlag(Level::Tier x)                  { m_tierFlag = x;                       }

  Profile::Name getProfileIdc() const                       { return m_profileIdc;                  }
  Void          setProfileIdc(Profile::Name x)              { m_profileIdc = x;                     }

  Bool          getProfileCompatibilityFlag(Int i) const    { return m_profileCompatibilityFlag[i]; }
  Void          setProfileCompatibilityFlag(Int i, Bool x)  { m_profileCompatibilityFlag[i] = x;    }

  Level::Name   getLevelIdc() const                         { return m_levelIdc;                    }
  Void          setLevelIdc(Level::Name x)                  { m_levelIdc = x;                       }

  Bool          getProgressiveSourceFlag() const            { return m_progressiveSourceFlag;       }
  Void          setProgressiveSourceFlag(Bool b)            { m_progressiveSourceFlag = b;          }

  Bool          getInterlacedSourceFlag() const             { return m_interlacedSourceFlag;        }
  Void          setInterlacedSourceFlag(Bool b)             { m_interlacedSourceFlag = b;           }

  Bool          getNonPackedConstraintFlag() const          { return m_nonPackedConstraintFlag;     }
  Void          setNonPackedConstraintFlag(Bool b)          { m_nonPackedConstraintFlag = b;        }

  Bool          getFrameOnlyConstraintFlag() const          { return m_frameOnlyConstraintFlag;     }
  Void          setFrameOnlyConstraintFlag(Bool b)          { m_frameOnlyConstraintFlag = b;        }

  UInt          getBitDepthConstraint() const               { return m_bitDepthConstraintValue;     }
  Void          setBitDepthConstraint(UInt bitDepth)        { m_bitDepthConstraintValue=bitDepth;   }

  ChromaFormat  getChromaFormatConstraint() const           { return m_chromaFormatConstraintValue; }
  Void          setChromaFormatConstraint(ChromaFormat fmt) { m_chromaFormatConstraintValue=fmt;    }

  Bool          getIntraConstraintFlag() const              { return m_intraConstraintFlag;         }
  Void          setIntraConstraintFlag(Bool b)              { m_intraConstraintFlag = b;            }

  Bool          getOnePictureOnlyConstraintFlag() const     { return m_onePictureOnlyConstraintFlag;}
  Void          setOnePictureOnlyConstraintFlag(Bool b)     { m_onePictureOnlyConstraintFlag = b;   }

  Bool          getLowerBitRateConstraintFlag() const       { return m_lowerBitRateConstraintFlag;  }
  Void          setLowerBitRateConstraintFlag(Bool b)       { m_lowerBitRateConstraintFlag = b;     }
};


class TComPTL
{
  ProfileTierLevel m_generalPTL;
  ProfileTierLevel m_subLayerPTL    [MAX_TLAYER-1];      // max. value of max_sub_layers_minus1 is MAX_TLAYER-1 (= 6)
  Bool m_subLayerProfilePresentFlag [MAX_TLAYER-1];
  Bool m_subLayerLevelPresentFlag   [MAX_TLAYER-1];

public:
                          TComPTL();
  Bool                    getSubLayerProfilePresentFlag(Int i) const   { return m_subLayerProfilePresentFlag[i]; }
  Void                    setSubLayerProfilePresentFlag(Int i, Bool x) { m_subLayerProfilePresentFlag[i] = x;    }

  Bool                    getSubLayerLevelPresentFlag(Int i) const     { return m_subLayerLevelPresentFlag[i];   }
  Void                    setSubLayerLevelPresentFlag(Int i, Bool x)   { m_subLayerLevelPresentFlag[i] = x;      }

  ProfileTierLevel*       getGeneralPTL()                              { return &m_generalPTL;                   }
  const ProfileTierLevel* getGeneralPTL() const                        { return &m_generalPTL;                   }
  ProfileTierLevel*       getSubLayerPTL(Int i)                        { return &m_subLayerPTL[i];               }
  const ProfileTierLevel* getSubLayerPTL(Int i) const                  { return &m_subLayerPTL[i];               }
};

/// VPS class

struct HrdSubLayerInfo
{
  Bool fixedPicRateFlag;
  Bool fixedPicRateWithinCvsFlag;
  UInt picDurationInTcMinus1;
  Bool lowDelayHrdFlag;
  UInt cpbCntMinus1;
  UInt bitRateValueMinus1[MAX_CPB_CNT][2];
  UInt cpbSizeValue      [MAX_CPB_CNT][2];
  UInt ducpbSizeValue    [MAX_CPB_CNT][2];
  Bool cbrFlag           [MAX_CPB_CNT][2];
  UInt duBitRateValue    [MAX_CPB_CNT][2];
};

class TComHRD
{
private:
  Bool m_nalHrdParametersPresentFlag;
  Bool m_vclHrdParametersPresentFlag;
  Bool m_subPicCpbParamsPresentFlag;
  UInt m_tickDivisorMinus2;
  UInt m_duCpbRemovalDelayLengthMinus1;
  Bool m_subPicCpbParamsInPicTimingSEIFlag;
  UInt m_dpbOutputDelayDuLengthMinus1;
  UInt m_bitRateScale;
  UInt m_cpbSizeScale;
  UInt m_ducpbSizeScale;
  UInt m_initialCpbRemovalDelayLengthMinus1;
  UInt m_cpbRemovalDelayLengthMinus1;
  UInt m_dpbOutputDelayLengthMinus1;
  HrdSubLayerInfo m_HRD[MAX_TLAYER];

public:
  TComHRD()
  :m_nalHrdParametersPresentFlag       (0)
  ,m_vclHrdParametersPresentFlag       (0)
  ,m_subPicCpbParamsPresentFlag        (false)
  ,m_tickDivisorMinus2                 (0)
  ,m_duCpbRemovalDelayLengthMinus1     (0)
  ,m_subPicCpbParamsInPicTimingSEIFlag (false)
  ,m_dpbOutputDelayDuLengthMinus1      (0)
  ,m_bitRateScale                      (0)
  ,m_cpbSizeScale                      (0)
  ,m_initialCpbRemovalDelayLengthMinus1(23)
  ,m_cpbRemovalDelayLengthMinus1       (23)
  ,m_dpbOutputDelayLengthMinus1        (23)
  {}

  virtual ~TComHRD() {}

  Void    setNalHrdParametersPresentFlag( Bool flag )                                { m_nalHrdParametersPresentFlag = flag;                      }
  Bool    getNalHrdParametersPresentFlag( ) const                                    { return m_nalHrdParametersPresentFlag;                      }

  Void    setVclHrdParametersPresentFlag( Bool flag )                                { m_vclHrdParametersPresentFlag = flag;                      }
  Bool    getVclHrdParametersPresentFlag( ) const                                    { return m_vclHrdParametersPresentFlag;                      }

  Void    setSubPicCpbParamsPresentFlag( Bool flag )                                 { m_subPicCpbParamsPresentFlag = flag;                       }
  Bool    getSubPicCpbParamsPresentFlag( ) const                                     { return m_subPicCpbParamsPresentFlag;                       }

  Void    setTickDivisorMinus2( UInt value )                                         { m_tickDivisorMinus2 = value;                               }
  UInt    getTickDivisorMinus2( ) const                                              { return m_tickDivisorMinus2;                                }

  Void    setDuCpbRemovalDelayLengthMinus1( UInt value )                             { m_duCpbRemovalDelayLengthMinus1 = value;                   }
  UInt    getDuCpbRemovalDelayLengthMinus1( ) const                                  { return m_duCpbRemovalDelayLengthMinus1;                    }

  Void    setSubPicCpbParamsInPicTimingSEIFlag( Bool flag)                           { m_subPicCpbParamsInPicTimingSEIFlag = flag;                }
  Bool    getSubPicCpbParamsInPicTimingSEIFlag( ) const                              { return m_subPicCpbParamsInPicTimingSEIFlag;                }

  Void    setDpbOutputDelayDuLengthMinus1(UInt value )                               { m_dpbOutputDelayDuLengthMinus1 = value;                    }
  UInt    getDpbOutputDelayDuLengthMinus1( ) const                                   { return m_dpbOutputDelayDuLengthMinus1;                     }

  Void    setBitRateScale( UInt value )                                              { m_bitRateScale = value;                                    }
  UInt    getBitRateScale( ) const                                                   { return m_bitRateScale;                                     }

  Void    setCpbSizeScale( UInt value )                                              { m_cpbSizeScale = value;                                    }
  UInt    getCpbSizeScale( ) const                                                   { return m_cpbSizeScale;                                     }
  Void    setDuCpbSizeScale( UInt value )                                            { m_ducpbSizeScale = value;                                  }
  UInt    getDuCpbSizeScale( ) const                                                 { return m_ducpbSizeScale;                                   }

  Void    setInitialCpbRemovalDelayLengthMinus1( UInt value )                        { m_initialCpbRemovalDelayLengthMinus1 = value;              }
  UInt    getInitialCpbRemovalDelayLengthMinus1( ) const                             { return m_initialCpbRemovalDelayLengthMinus1;               }

  Void    setCpbRemovalDelayLengthMinus1( UInt value )                               { m_cpbRemovalDelayLengthMinus1 = value;                     }
  UInt    getCpbRemovalDelayLengthMinus1( ) const                                    { return m_cpbRemovalDelayLengthMinus1;                      }

  Void    setDpbOutputDelayLengthMinus1( UInt value )                                { m_dpbOutputDelayLengthMinus1 = value;                      }
  UInt    getDpbOutputDelayLengthMinus1( ) const                                     { return m_dpbOutputDelayLengthMinus1;                       }

  Void    setFixedPicRateFlag( Int layer, Bool flag )                                { m_HRD[layer].fixedPicRateFlag = flag;                      }
  Bool    getFixedPicRateFlag( Int layer ) const                                     { return m_HRD[layer].fixedPicRateFlag;                      }

  Void    setFixedPicRateWithinCvsFlag( Int layer, Bool flag )                       { m_HRD[layer].fixedPicRateWithinCvsFlag = flag;             }
  Bool    getFixedPicRateWithinCvsFlag( Int layer ) const                            { return m_HRD[layer].fixedPicRateWithinCvsFlag;             }

  Void    setPicDurationInTcMinus1( Int layer, UInt value )                          { m_HRD[layer].picDurationInTcMinus1 = value;                }
  UInt    getPicDurationInTcMinus1( Int layer ) const                                { return m_HRD[layer].picDurationInTcMinus1;                 }

  Void    setLowDelayHrdFlag( Int layer, Bool flag )                                 { m_HRD[layer].lowDelayHrdFlag = flag;                       }
  Bool    getLowDelayHrdFlag( Int layer ) const                                      { return m_HRD[layer].lowDelayHrdFlag;                       }

  Void    setCpbCntMinus1( Int layer, UInt value )                                   { m_HRD[layer].cpbCntMinus1 = value;                         }
  UInt    getCpbCntMinus1( Int layer ) const                                         { return m_HRD[layer].cpbCntMinus1;                          }

  Void    setBitRateValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl, UInt value )   { m_HRD[layer].bitRateValueMinus1[cpbcnt][nalOrVcl] = value; }
  UInt    getBitRateValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl ) const         { return m_HRD[layer].bitRateValueMinus1[cpbcnt][nalOrVcl];  }

  Void    setCpbSizeValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl, UInt value )   { m_HRD[layer].cpbSizeValue[cpbcnt][nalOrVcl] = value;       }
  UInt    getCpbSizeValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl ) const         { return m_HRD[layer].cpbSizeValue[cpbcnt][nalOrVcl];        }
  Void    setDuCpbSizeValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl, UInt value ) { m_HRD[layer].ducpbSizeValue[cpbcnt][nalOrVcl] = value;     }
  UInt    getDuCpbSizeValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl ) const       { return m_HRD[layer].ducpbSizeValue[cpbcnt][nalOrVcl];      }
  Void    setDuBitRateValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl, UInt value ) { m_HRD[layer].duBitRateValue[cpbcnt][nalOrVcl] = value;     }
  UInt    getDuBitRateValueMinus1(Int layer, Int cpbcnt, Int nalOrVcl ) const        { return m_HRD[layer].duBitRateValue[cpbcnt][nalOrVcl];      }
  Void    setCbrFlag( Int layer, Int cpbcnt, Int nalOrVcl, Bool value )              { m_HRD[layer].cbrFlag[cpbcnt][nalOrVcl] = value;            }
  Bool    getCbrFlag( Int layer, Int cpbcnt, Int nalOrVcl ) const                    { return m_HRD[layer].cbrFlag[cpbcnt][nalOrVcl];             }

  Bool    getCpbDpbDelaysPresentFlag( ) const                      { return getNalHrdParametersPresentFlag() || getVclHrdParametersPresentFlag(); }
};

class TimingInfo
{
  Bool m_timingInfoPresentFlag;
  UInt m_numUnitsInTick;
  UInt m_timeScale;
  Bool m_pocProportionalToTimingFlag;
  Int  m_numTicksPocDiffOneMinus1;
public:
  TimingInfo()
  : m_timingInfoPresentFlag      (false)
  , m_numUnitsInTick             (1001)
  , m_timeScale                  (60000)
  , m_pocProportionalToTimingFlag(false)
  , m_numTicksPocDiffOneMinus1   (0)
  {}

  Void setTimingInfoPresentFlag( Bool flag )   { m_timingInfoPresentFlag = flag;       }
  Bool getTimingInfoPresentFlag( ) const       { return m_timingInfoPresentFlag;       }

  Void setNumUnitsInTick( UInt value )         { m_numUnitsInTick = value;             }
  UInt getNumUnitsInTick( ) const              { return m_numUnitsInTick;              }

  Void setTimeScale( UInt value )              { m_timeScale = value;                  }
  UInt getTimeScale( ) const                   { return m_timeScale;                   }

  Void setPocProportionalToTimingFlag(Bool x)  { m_pocProportionalToTimingFlag = x;    }
  Bool getPocProportionalToTimingFlag( ) const { return m_pocProportionalToTimingFlag; }

  Void setNumTicksPocDiffOneMinus1(Int x)      { m_numTicksPocDiffOneMinus1 = x;       }
  Int  getNumTicksPocDiffOneMinus1( ) const    { return m_numTicksPocDiffOneMinus1;    }
};

struct ChromaQpAdj
{
  union
  {
    struct {
      Int CbOffset;
      Int CrOffset;
    } comp;
    Int offset[2]; /* two chroma components */
  } u;
};

class TComVPS
{
private:
  Int                   m_VPSId;
  UInt                  m_uiMaxTLayers;
  UInt                  m_uiMaxLayers;
  Bool                  m_bTemporalIdNestingFlag;

  UInt                  m_numReorderPics[MAX_TLAYER];
  UInt                  m_uiMaxDecPicBuffering[MAX_TLAYER];
  UInt                  m_uiMaxLatencyIncrease[MAX_TLAYER]; // Really max latency increase plus 1 (value 0 expresses no limit)

  UInt                  m_numHrdParameters;
  UInt                  m_maxNuhReservedZeroLayerId;
  std::vector<TComHRD>  m_hrdParameters;
  std::vector<UInt>     m_hrdOpSetIdx;
  std::vector<Bool>     m_cprmsPresentFlag;
  UInt                  m_numOpSets;
  Bool                  m_layerIdIncludedFlag[MAX_VPS_OP_SETS_PLUS1][MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1];

  TComPTL               m_pcPTL;
  TimingInfo            m_timingInfo;

public:
                    TComVPS();

  virtual           ~TComVPS();

  Void              createHrdParamBuffer()
  {
    m_hrdParameters.resize(getNumHrdParameters());
    m_hrdOpSetIdx.resize(getNumHrdParameters());
    m_cprmsPresentFlag.resize(getNumHrdParameters());
  }

  TComHRD*          getHrdParameters( UInt i )                           { return &m_hrdParameters[ i ];                                    }
  const TComHRD*    getHrdParameters( UInt i ) const                     { return &m_hrdParameters[ i ];                                    }
  UInt              getHrdOpSetIdx( UInt i ) const                       { return m_hrdOpSetIdx[ i ];                                       }
  Void              setHrdOpSetIdx( UInt val, UInt i )                   { m_hrdOpSetIdx[ i ] = val;                                        }
  Bool              getCprmsPresentFlag( UInt i ) const                  { return m_cprmsPresentFlag[ i ];                                  }
  Void              setCprmsPresentFlag( Bool val, UInt i )              { m_cprmsPresentFlag[ i ] = val;                                   }

  Int               getVPSId() const                                     { return m_VPSId;                                                  }
  Void              setVPSId(Int i)                                      { m_VPSId = i;                                                     }

  UInt              getMaxTLayers() const                                { return m_uiMaxTLayers;                                           }
  Void              setMaxTLayers(UInt t)                                { m_uiMaxTLayers = t;                                              }

  UInt              getMaxLayers() const                                 { return m_uiMaxLayers;                                            }
  Void              setMaxLayers(UInt l)                                 { m_uiMaxLayers = l;                                               }

  Bool              getTemporalNestingFlag() const                       { return m_bTemporalIdNestingFlag;                                 }
  Void              setTemporalNestingFlag(Bool t)                       { m_bTemporalIdNestingFlag = t;                                    }

  Void              setNumReorderPics(UInt v, UInt tLayer)               { m_numReorderPics[tLayer] = v;                                    }
  UInt              getNumReorderPics(UInt tLayer) const                 { return m_numReorderPics[tLayer];                                 }

  Void              setMaxDecPicBuffering(UInt v, UInt tLayer)           { assert(tLayer < MAX_TLAYER); m_uiMaxDecPicBuffering[tLayer] = v; }
  UInt              getMaxDecPicBuffering(UInt tLayer) const             { return m_uiMaxDecPicBuffering[tLayer];                           }

  Void              setMaxLatencyIncrease(UInt v, UInt tLayer)           { m_uiMaxLatencyIncrease[tLayer] = v;                              }
  UInt              getMaxLatencyIncrease(UInt tLayer) const             { return m_uiMaxLatencyIncrease[tLayer];                           }

  UInt              getNumHrdParameters() const                          { return m_numHrdParameters;                                       }
  Void              setNumHrdParameters(UInt v)                          { m_numHrdParameters = v;                                          }

  UInt              getMaxNuhReservedZeroLayerId() const                 { return m_maxNuhReservedZeroLayerId;                              }
  Void              setMaxNuhReservedZeroLayerId(UInt v)                 { m_maxNuhReservedZeroLayerId = v;                                 }

  UInt              getMaxOpSets() const                                 { return m_numOpSets;                                              }
  Void              setMaxOpSets(UInt v)                                 { m_numOpSets = v;                                                 }
  Bool              getLayerIdIncludedFlag(UInt opsIdx, UInt id) const   { return m_layerIdIncludedFlag[opsIdx][id];                        }
  Void              setLayerIdIncludedFlag(Bool v, UInt opsIdx, UInt id) { m_layerIdIncludedFlag[opsIdx][id] = v;                           }

  TComPTL*          getPTL()                                             { return &m_pcPTL;                                                 }
  const TComPTL*    getPTL() const                                       { return &m_pcPTL;                                                 }
  TimingInfo*       getTimingInfo()                                      { return &m_timingInfo;                                            }
  const TimingInfo* getTimingInfo() const                                { return &m_timingInfo;                                            }
};

class TComVUI
{
private:
  Bool       m_aspectRatioInfoPresentFlag;
  Int        m_aspectRatioIdc;
  Int        m_sarWidth;
  Int        m_sarHeight;
  Bool       m_overscanInfoPresentFlag;
  Bool       m_overscanAppropriateFlag;
  Bool       m_videoSignalTypePresentFlag;
  Int        m_videoFormat;
  Bool       m_videoFullRangeFlag;
  Bool       m_colourDescriptionPresentFlag;
  Int        m_colourPrimaries;
  Int        m_transferCharacteristics;
  Int        m_matrixCoefficients;
  Bool       m_chromaLocInfoPresentFlag;
  Int        m_chromaSampleLocTypeTopField;
  Int        m_chromaSampleLocTypeBottomField;
  Bool       m_neutralChromaIndicationFlag;
  Bool       m_fieldSeqFlag;
  Window     m_defaultDisplayWindow;
  Bool       m_frameFieldInfoPresentFlag;
  Bool       m_hrdParametersPresentFlag;
  Bool       m_bitstreamRestrictionFlag;
  Bool       m_tilesFixedStructureFlag;
  Bool       m_motionVectorsOverPicBoundariesFlag;
  Bool       m_restrictedRefPicListsFlag;
  Int        m_minSpatialSegmentationIdc;
  Int        m_maxBytesPerPicDenom;
  Int        m_maxBitsPerMinCuDenom;
  Int        m_log2MaxMvLengthHorizontal;
  Int        m_log2MaxMvLengthVertical;
  TComHRD    m_hrdParameters;
  TimingInfo m_timingInfo;

public:
  TComVUI()
    : m_aspectRatioInfoPresentFlag        (false) //TODO: This initialiser list contains magic numbers
    , m_aspectRatioIdc                    (0)
    , m_sarWidth                          (0)
    , m_sarHeight                         (0)
    , m_overscanInfoPresentFlag           (false)
    , m_overscanAppropriateFlag           (false)
    , m_videoSignalTypePresentFlag        (false)
    , m_videoFormat                       (5)
    , m_videoFullRangeFlag                (false)
    , m_colourDescriptionPresentFlag      (false)
    , m_colourPrimaries                   (2)
    , m_transferCharacteristics           (2)
    , m_matrixCoefficients                (2)
    , m_chromaLocInfoPresentFlag          (false)
    , m_chromaSampleLocTypeTopField       (0)
    , m_chromaSampleLocTypeBottomField    (0)
    , m_neutralChromaIndicationFlag       (false)
    , m_fieldSeqFlag                      (false)
    , m_frameFieldInfoPresentFlag         (false)
    , m_hrdParametersPresentFlag          (false)
    , m_bitstreamRestrictionFlag          (false)
    , m_tilesFixedStructureFlag           (false)
    , m_motionVectorsOverPicBoundariesFlag(true)
    , m_restrictedRefPicListsFlag         (1)
    , m_minSpatialSegmentationIdc         (0)
    , m_maxBytesPerPicDenom               (2)
    , m_maxBitsPerMinCuDenom              (1)
    , m_log2MaxMvLengthHorizontal         (15)
    , m_log2MaxMvLengthVertical           (15)
  {}

  virtual           ~TComVUI() {}

  Bool              getAspectRatioInfoPresentFlag() const                  { return m_aspectRatioInfoPresentFlag;           }
  Void              setAspectRatioInfoPresentFlag(Bool i)                  { m_aspectRatioInfoPresentFlag = i;              }

  Int               getAspectRatioIdc() const                              { return m_aspectRatioIdc;                       }
  Void              setAspectRatioIdc(Int i)                               { m_aspectRatioIdc = i;                          }

  Int               getSarWidth() const                                    { return m_sarWidth;                             }
  Void              setSarWidth(Int i)                                     { m_sarWidth = i;                                }

  Int               getSarHeight() const                                   { return m_sarHeight;                            }
  Void              setSarHeight(Int i)                                    { m_sarHeight = i;                               }

  Bool              getOverscanInfoPresentFlag() const                     { return m_overscanInfoPresentFlag;              }
  Void              setOverscanInfoPresentFlag(Bool i)                     { m_overscanInfoPresentFlag = i;                 }

  Bool              getOverscanAppropriateFlag() const                     { return m_overscanAppropriateFlag;              }
  Void              setOverscanAppropriateFlag(Bool i)                     { m_overscanAppropriateFlag = i;                 }

  Bool              getVideoSignalTypePresentFlag() const                  { return m_videoSignalTypePresentFlag;           }
  Void              setVideoSignalTypePresentFlag(Bool i)                  { m_videoSignalTypePresentFlag = i;              }

  Int               getVideoFormat() const                                 { return m_videoFormat;                          }
  Void              setVideoFormat(Int i)                                  { m_videoFormat = i;                             }

  Bool              getVideoFullRangeFlag() const                          { return m_videoFullRangeFlag;                   }
  Void              setVideoFullRangeFlag(Bool i)                          { m_videoFullRangeFlag = i;                      }

  Bool              getColourDescriptionPresentFlag() const                { return m_colourDescriptionPresentFlag;         }
  Void              setColourDescriptionPresentFlag(Bool i)                { m_colourDescriptionPresentFlag = i;            }

  Int               getColourPrimaries() const                             { return m_colourPrimaries;                      }
  Void              setColourPrimaries(Int i)                              { m_colourPrimaries = i;                         }

  Int               getTransferCharacteristics() const                     { return m_transferCharacteristics;              }
  Void              setTransferCharacteristics(Int i)                      { m_transferCharacteristics = i;                 }

  Int               getMatrixCoefficients() const                          { return m_matrixCoefficients;                   }
  Void              setMatrixCoefficients(Int i)                           { m_matrixCoefficients = i;                      }

  Bool              getChromaLocInfoPresentFlag() const                    { return m_chromaLocInfoPresentFlag;             }
  Void              setChromaLocInfoPresentFlag(Bool i)                    { m_chromaLocInfoPresentFlag = i;                }

  Int               getChromaSampleLocTypeTopField() const                 { return m_chromaSampleLocTypeTopField;          }
  Void              setChromaSampleLocTypeTopField(Int i)                  { m_chromaSampleLocTypeTopField = i;             }

  Int               getChromaSampleLocTypeBottomField() const              { return m_chromaSampleLocTypeBottomField;       }
  Void              setChromaSampleLocTypeBottomField(Int i)               { m_chromaSampleLocTypeBottomField = i;          }

  Bool              getNeutralChromaIndicationFlag() const                 { return m_neutralChromaIndicationFlag;          }
  Void              setNeutralChromaIndicationFlag(Bool i)                 { m_neutralChromaIndicationFlag = i;             }

  Bool              getFieldSeqFlag() const                                { return m_fieldSeqFlag;                         }
  Void              setFieldSeqFlag(Bool i)                                { m_fieldSeqFlag = i;                            }

  Bool              getFrameFieldInfoPresentFlag() const                   { return m_frameFieldInfoPresentFlag;            }
  Void              setFrameFieldInfoPresentFlag(Bool i)                   { m_frameFieldInfoPresentFlag = i;               }

  Window&           getDefaultDisplayWindow()                              { return m_defaultDisplayWindow;                 }
  const Window&     getDefaultDisplayWindow() const                        { return m_defaultDisplayWindow;                 }
  Void              setDefaultDisplayWindow(Window& defaultDisplayWindow ) { m_defaultDisplayWindow = defaultDisplayWindow; }

  Bool              getHrdParametersPresentFlag() const                    { return m_hrdParametersPresentFlag;             }
  Void              setHrdParametersPresentFlag(Bool i)                    { m_hrdParametersPresentFlag = i;                }

  Bool              getBitstreamRestrictionFlag() const                    { return m_bitstreamRestrictionFlag;             }
  Void              setBitstreamRestrictionFlag(Bool i)                    { m_bitstreamRestrictionFlag = i;                }

  Bool              getTilesFixedStructureFlag() const                     { return m_tilesFixedStructureFlag;              }
  Void              setTilesFixedStructureFlag(Bool i)                     { m_tilesFixedStructureFlag = i;                 }

  Bool              getMotionVectorsOverPicBoundariesFlag() const          { return m_motionVectorsOverPicBoundariesFlag;   }
  Void              setMotionVectorsOverPicBoundariesFlag(Bool i)          { m_motionVectorsOverPicBoundariesFlag = i;      }

  Bool              getRestrictedRefPicListsFlag() const                   { return m_restrictedRefPicListsFlag;            }
  Void              setRestrictedRefPicListsFlag(Bool b)                   { m_restrictedRefPicListsFlag = b;               }

  Int               getMinSpatialSegmentationIdc() const                   { return m_minSpatialSegmentationIdc;            }
  Void              setMinSpatialSegmentationIdc(Int i)                    { m_minSpatialSegmentationIdc = i;               }

  Int               getMaxBytesPerPicDenom() const                         { return m_maxBytesPerPicDenom;                  }
  Void              setMaxBytesPerPicDenom(Int i)                          { m_maxBytesPerPicDenom = i;                     }

  Int               getMaxBitsPerMinCuDenom() const                        { return m_maxBitsPerMinCuDenom;                 }
  Void              setMaxBitsPerMinCuDenom(Int i)                         { m_maxBitsPerMinCuDenom = i;                    }

  Int               getLog2MaxMvLengthHorizontal() const                   { return m_log2MaxMvLengthHorizontal;            }
  Void              setLog2MaxMvLengthHorizontal(Int i)                    { m_log2MaxMvLengthHorizontal = i;               }

  Int               getLog2MaxMvLengthVertical() const                     { return m_log2MaxMvLengthVertical;              }
  Void              setLog2MaxMvLengthVertical(Int i)                      { m_log2MaxMvLengthVertical = i;                 }

  TComHRD*          getHrdParameters()                                     { return &m_hrdParameters;                       }
  const TComHRD*    getHrdParameters()  const                              { return &m_hrdParameters;                       }

  TimingInfo*       getTimingInfo()                                        { return &m_timingInfo;                          }
  const TimingInfo* getTimingInfo() const                                  { return &m_timingInfo;                          }
};

/// SPS RExt class
class TComSPSRExt // Names aligned to text specification
{
private:
  Bool             m_transformSkipRotationEnabledFlag;
  Bool             m_transformSkipContextEnabledFlag;
  Bool             m_rdpcmEnabledFlag[NUMBER_OF_RDPCM_SIGNALLING_MODES];
  Bool             m_extendedPrecisionProcessingFlag;
  Bool             m_intraSmoothingDisabledFlag;
  Bool             m_highPrecisionOffsetsEnabledFlag;
  Bool             m_persistentRiceAdaptationEnabledFlag;
  Bool             m_cabacBypassAlignmentEnabledFlag;

public:
  TComSPSRExt();

  Bool settingsDifferFromDefaults() const
  {
    return getTransformSkipRotationEnabledFlag()
        || getTransformSkipContextEnabledFlag()
        || getRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT)
        || getRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT)
        || getExtendedPrecisionProcessingFlag()
        || getIntraSmoothingDisabledFlag()
        || getHighPrecisionOffsetsEnabledFlag()
        || getPersistentRiceAdaptationEnabledFlag()
        || getCabacBypassAlignmentEnabledFlag();
  }


  Bool getTransformSkipRotationEnabledFlag() const                                     { return m_transformSkipRotationEnabledFlag;     }
  Void setTransformSkipRotationEnabledFlag(const Bool value)                           { m_transformSkipRotationEnabledFlag = value;    }

  Bool getTransformSkipContextEnabledFlag() const                                      { return m_transformSkipContextEnabledFlag;      }
  Void setTransformSkipContextEnabledFlag(const Bool value)                            { m_transformSkipContextEnabledFlag = value;     }

  Bool getRdpcmEnabledFlag(const RDPCMSignallingMode signallingMode) const             { return m_rdpcmEnabledFlag[signallingMode];     }
  Void setRdpcmEnabledFlag(const RDPCMSignallingMode signallingMode, const Bool value) { m_rdpcmEnabledFlag[signallingMode] = value;    }

  Bool getExtendedPrecisionProcessingFlag() const                                      { return m_extendedPrecisionProcessingFlag;      }
  Void setExtendedPrecisionProcessingFlag(Bool value)                                  { m_extendedPrecisionProcessingFlag = value;     }

  Bool getIntraSmoothingDisabledFlag() const                                           { return m_intraSmoothingDisabledFlag;           }
  Void setIntraSmoothingDisabledFlag(Bool bValue)                                      { m_intraSmoothingDisabledFlag=bValue;           }

  Bool getHighPrecisionOffsetsEnabledFlag() const                                      { return m_highPrecisionOffsetsEnabledFlag;      }
  Void setHighPrecisionOffsetsEnabledFlag(Bool value)                                  { m_highPrecisionOffsetsEnabledFlag = value;     }

  Bool getPersistentRiceAdaptationEnabledFlag() const                                  { return m_persistentRiceAdaptationEnabledFlag;  }
  Void setPersistentRiceAdaptationEnabledFlag(const Bool value)                        { m_persistentRiceAdaptationEnabledFlag = value; }

  Bool getCabacBypassAlignmentEnabledFlag() const                                      { return m_cabacBypassAlignmentEnabledFlag;      }
  Void setCabacBypassAlignmentEnabledFlag(const Bool value)                            { m_cabacBypassAlignmentEnabledFlag = value;     }
};

/// SPS class
class TComSPS
{
private:
  Int              m_SPSId;
  Int              m_VPSId;
  ChromaFormat     m_chromaFormatIdc;

  UInt             m_uiMaxTLayers;           // maximum number of temporal layers

  // Structure
  UInt             m_picWidthInLumaSamples;
  UInt             m_picHeightInLumaSamples;

  Int              m_log2MinCodingBlockSize;
  Int              m_log2DiffMaxMinCodingBlockSize;
  UInt             m_uiMaxCUWidth;
  UInt             m_uiMaxCUHeight;
  UInt             m_uiMaxTotalCUDepth; ///< Total CU depth, relative to the smallest possible transform block size.

  Window           m_conformanceWindow;

  TComRPSList      m_RPSList;
  Bool             m_bLongTermRefsPresent;
  Bool             m_SPSTemporalMVPEnabledFlag;
  Int              m_numReorderPics[MAX_TLAYER];

  // Tool list
  UInt             m_uiQuadtreeTULog2MaxSize;
  UInt             m_uiQuadtreeTULog2MinSize;
  UInt             m_uiQuadtreeTUMaxDepthInter;
  UInt             m_uiQuadtreeTUMaxDepthIntra;
  Bool             m_usePCM;
  UInt             m_pcmLog2MaxSize;
  UInt             m_uiPCMLog2MinSize;
  Bool             m_useAMP;

  // Parameter
  BitDepths        m_bitDepths;
  Int              m_qpBDOffset[MAX_NUM_CHANNEL_TYPE];
  Int              m_pcmBitDepths[MAX_NUM_CHANNEL_TYPE];
  Bool             m_bPCMFilterDisableFlag;

  UInt             m_uiBitsForPOC;
  UInt             m_numLongTermRefPicSPS;
  UInt             m_ltRefPicPocLsbSps[MAX_NUM_LONG_TERM_REF_PICS];
  Bool             m_usedByCurrPicLtSPSFlag[MAX_NUM_LONG_TERM_REF_PICS];
  // Max physical transform size
  UInt             m_uiMaxTrSize;

  Bool             m_bUseSAO;

  Bool             m_bTemporalIdNestingFlag; // temporal_id_nesting_flag

  Bool             m_scalingListEnabledFlag;
  Bool             m_scalingListPresentFlag;
  TComScalingList  m_scalingList;
  UInt             m_uiMaxDecPicBuffering[MAX_TLAYER];
  UInt             m_uiMaxLatencyIncreasePlus1[MAX_TLAYER];

  Bool             m_useStrongIntraSmoothing;

  Bool             m_vuiParametersPresentFlag;
  TComVUI          m_vuiParameters;

  TComSPSRExt      m_spsRangeExtension;

  static const Int m_winUnitX[NUM_CHROMA_FORMAT];
  static const Int m_winUnitY[NUM_CHROMA_FORMAT];
  TComPTL          m_pcPTL;

#if O0043_BEST_EFFORT_DECODING
  UInt             m_forceDecodeBitDepth; // 0 = do not force the decoder's bit depth, other = force the decoder's bit depth to this value (best effort decoding)
#endif

public:
                         TComSPS();
  virtual                ~TComSPS();
#if O0043_BEST_EFFORT_DECODING
  Void                   setForceDecodeBitDepth(UInt bitDepth)                                           { m_forceDecodeBitDepth = bitDepth;                                    }
  UInt                   getForceDecodeBitDepth()        const                                           { return m_forceDecodeBitDepth;                                        }
#endif

  Int                    getVPSId() const                                                                { return m_VPSId;                                                      }
  Void                   setVPSId(Int i)                                                                 { m_VPSId = i;                                                         }
  Int                    getSPSId() const                                                                { return m_SPSId;                                                      }
  Void                   setSPSId(Int i)                                                                 { m_SPSId = i;                                                         }
  ChromaFormat           getChromaFormatIdc () const                                                     { return m_chromaFormatIdc;                                            }
  Void                   setChromaFormatIdc (ChromaFormat i)                                             { m_chromaFormatIdc = i;                                               }

  static Int             getWinUnitX (Int chromaFormatIdc)                                               { assert (chromaFormatIdc >= 0 && chromaFormatIdc < NUM_CHROMA_FORMAT); return m_winUnitX[chromaFormatIdc]; }
  static Int             getWinUnitY (Int chromaFormatIdc)                                               { assert (chromaFormatIdc >= 0 && chromaFormatIdc < NUM_CHROMA_FORMAT); return m_winUnitY[chromaFormatIdc]; }

  // structure
  Void                   setPicWidthInLumaSamples( UInt u )                                              { m_picWidthInLumaSamples = u;                                         }
  UInt                   getPicWidthInLumaSamples() const                                                { return  m_picWidthInLumaSamples;                                     }
  Void                   setPicHeightInLumaSamples( UInt u )                                             { m_picHeightInLumaSamples = u;                                        }
  UInt                   getPicHeightInLumaSamples() const                                               { return  m_picHeightInLumaSamples;                                    }

  Window&                getConformanceWindow()                                                          { return  m_conformanceWindow;                                         }
  const Window&          getConformanceWindow() const                                                    { return  m_conformanceWindow;                                         }
  Void                   setConformanceWindow(Window& conformanceWindow )                                { m_conformanceWindow = conformanceWindow;                             }

  UInt                   getNumLongTermRefPicSPS() const                                                 { return m_numLongTermRefPicSPS;                                       }
  Void                   setNumLongTermRefPicSPS(UInt val)                                               { m_numLongTermRefPicSPS = val;                                        }

  UInt                   getLtRefPicPocLsbSps(UInt index) const                                          { assert( index < MAX_NUM_LONG_TERM_REF_PICS ); return m_ltRefPicPocLsbSps[index]; }
  Void                   setLtRefPicPocLsbSps(UInt index, UInt val)                                      { assert( index < MAX_NUM_LONG_TERM_REF_PICS ); m_ltRefPicPocLsbSps[index] = val;  }

  Bool                   getUsedByCurrPicLtSPSFlag(Int i) const                                          { assert( i < MAX_NUM_LONG_TERM_REF_PICS ); return m_usedByCurrPicLtSPSFlag[i];    }
  Void                   setUsedByCurrPicLtSPSFlag(Int i, Bool x)                                        { assert( i < MAX_NUM_LONG_TERM_REF_PICS ); m_usedByCurrPicLtSPSFlag[i] = x;       }

  Int                    getLog2MinCodingBlockSize() const                                               { return m_log2MinCodingBlockSize;                                     }
  Void                   setLog2MinCodingBlockSize(Int val)                                              { m_log2MinCodingBlockSize = val;                                      }
  Int                    getLog2DiffMaxMinCodingBlockSize() const                                        { return m_log2DiffMaxMinCodingBlockSize;                              }
  Void                   setLog2DiffMaxMinCodingBlockSize(Int val)                                       { m_log2DiffMaxMinCodingBlockSize = val;                               }

  Void                   setMaxCUWidth( UInt u )                                                         { m_uiMaxCUWidth = u;                                                  }
  UInt                   getMaxCUWidth() const                                                           { return  m_uiMaxCUWidth;                                              }
  Void                   setMaxCUHeight( UInt u )                                                        { m_uiMaxCUHeight = u;                                                 }
  UInt                   getMaxCUHeight() const                                                          { return  m_uiMaxCUHeight;                                             }
  Void                   setMaxTotalCUDepth( UInt u )                                                    { m_uiMaxTotalCUDepth = u;                                             }
  UInt                   getMaxTotalCUDepth() const                                                      { return  m_uiMaxTotalCUDepth;                                         }
  Void                   setUsePCM( Bool b )                                                             { m_usePCM = b;                                                        }
  Bool                   getUsePCM() const                                                               { return m_usePCM;                                                     }
  Void                   setPCMLog2MaxSize( UInt u )                                                     { m_pcmLog2MaxSize = u;                                                }
  UInt                   getPCMLog2MaxSize() const                                                       { return  m_pcmLog2MaxSize;                                            }
  Void                   setPCMLog2MinSize( UInt u )                                                     { m_uiPCMLog2MinSize = u;                                              }
  UInt                   getPCMLog2MinSize() const                                                       { return  m_uiPCMLog2MinSize;                                          }
  Void                   setBitsForPOC( UInt u )                                                         { m_uiBitsForPOC = u;                                                  }
  UInt                   getBitsForPOC() const                                                           { return m_uiBitsForPOC;                                               }
  Bool                   getUseAMP() const                                                               { return m_useAMP;                                                     }
  Void                   setUseAMP( Bool b )                                                             { m_useAMP = b;                                                        }
  Void                   setQuadtreeTULog2MaxSize( UInt u )                                              { m_uiQuadtreeTULog2MaxSize = u;                                       }
  UInt                   getQuadtreeTULog2MaxSize() const                                                { return m_uiQuadtreeTULog2MaxSize;                                    }
  Void                   setQuadtreeTULog2MinSize( UInt u )                                              { m_uiQuadtreeTULog2MinSize = u;                                       }
  UInt                   getQuadtreeTULog2MinSize() const                                                { return m_uiQuadtreeTULog2MinSize;                                    }
  Void                   setQuadtreeTUMaxDepthInter( UInt u )                                            { m_uiQuadtreeTUMaxDepthInter = u;                                     }
  Void                   setQuadtreeTUMaxDepthIntra( UInt u )                                            { m_uiQuadtreeTUMaxDepthIntra = u;                                     }
  UInt                   getQuadtreeTUMaxDepthInter() const                                              { return m_uiQuadtreeTUMaxDepthInter;                                  }
  UInt                   getQuadtreeTUMaxDepthIntra() const                                              { return m_uiQuadtreeTUMaxDepthIntra;                                  }
  Void                   setNumReorderPics(Int i, UInt tlayer)                                           { m_numReorderPics[tlayer] = i;                                        }
  Int                    getNumReorderPics(UInt tlayer) const                                            { return m_numReorderPics[tlayer];                                     }
  Void                   createRPSList( Int numRPS );
  const TComRPSList*     getRPSList() const                                                              { return &m_RPSList;                                                   }
  TComRPSList*           getRPSList()                                                                    { return &m_RPSList;                                                   }
  Bool                   getLongTermRefsPresent() const                                                  { return m_bLongTermRefsPresent;                                       }
  Void                   setLongTermRefsPresent(Bool b)                                                  { m_bLongTermRefsPresent=b;                                            }
  Bool                   getSPSTemporalMVPEnabledFlag() const                                            { return m_SPSTemporalMVPEnabledFlag;                                  }
  Void                   setSPSTemporalMVPEnabledFlag(Bool b)                                            { m_SPSTemporalMVPEnabledFlag=b;                                       }
  // physical transform
  Void                   setMaxTrSize( UInt u )                                                          { m_uiMaxTrSize = u;                                                   }
  UInt                   getMaxTrSize() const                                                            { return  m_uiMaxTrSize;                                               }

  // Bit-depth
  Int                    getBitDepth(ChannelType type) const                                             { return m_bitDepths.recon[type];                                      }
  Void                   setBitDepth(ChannelType type, Int u )                                           { m_bitDepths.recon[type] = u;                                         }
#if O0043_BEST_EFFORT_DECODING
  Int                    getStreamBitDepth(ChannelType type) const                                       { return m_bitDepths.stream[type];                                     }
  Void                   setStreamBitDepth(ChannelType type, Int u )                                     { m_bitDepths.stream[type] = u;                                        }
#endif
  const BitDepths&       getBitDepths() const                                                            { return m_bitDepths;                                                  }
  Int                    getMaxLog2TrDynamicRange(ChannelType channelType) const                         { return getSpsRangeExtension().getExtendedPrecisionProcessingFlag() ? std::max<Int>(15, Int(m_bitDepths.recon[channelType] + 6)) : 15; }

  Int                    getDifferentialLumaChromaBitDepth() const                                       { return Int(m_bitDepths.recon[CHANNEL_TYPE_LUMA]) - Int(m_bitDepths.recon[CHANNEL_TYPE_CHROMA]); }
  Int                    getQpBDOffset(ChannelType type) const                                           { return m_qpBDOffset[type];                                           }
  Void                   setQpBDOffset(ChannelType type, Int i)                                          { m_qpBDOffset[type] = i;                                              }

  Void                   setUseSAO(Bool bVal)                                                            { m_bUseSAO = bVal;                                                    }
  Bool                   getUseSAO() const                                                               { return m_bUseSAO;                                                    }

  UInt                   getMaxTLayers() const                                                           { return m_uiMaxTLayers; }
  Void                   setMaxTLayers( UInt uiMaxTLayers )                                              { assert( uiMaxTLayers <= MAX_TLAYER ); m_uiMaxTLayers = uiMaxTLayers; }

  Bool                   getTemporalIdNestingFlag() const                                                { return m_bTemporalIdNestingFlag;                                     }
  Void                   setTemporalIdNestingFlag( Bool bValue )                                         { m_bTemporalIdNestingFlag = bValue;                                   }
  UInt                   getPCMBitDepth(ChannelType type) const                                          { return m_pcmBitDepths[type];                                         }
  Void                   setPCMBitDepth(ChannelType type, UInt u)                                        { m_pcmBitDepths[type] = u;                                            }
  Void                   setPCMFilterDisableFlag( Bool bValue )                                          { m_bPCMFilterDisableFlag = bValue;                                    }
  Bool                   getPCMFilterDisableFlag() const                                                 { return m_bPCMFilterDisableFlag;                                      }

  Bool                   getScalingListFlag() const                                                      { return m_scalingListEnabledFlag;                                     }
  Void                   setScalingListFlag( Bool b )                                                    { m_scalingListEnabledFlag  = b;                                       }
  Bool                   getScalingListPresentFlag() const                                               { return m_scalingListPresentFlag;                                     }
  Void                   setScalingListPresentFlag( Bool b )                                             { m_scalingListPresentFlag  = b;                                       }
  TComScalingList&       getScalingList()                                                                { return m_scalingList;                                                }
  const TComScalingList& getScalingList() const                                                          { return m_scalingList;                                                }
  UInt                   getMaxDecPicBuffering(UInt tlayer) const                                        { return m_uiMaxDecPicBuffering[tlayer];                               }
  Void                   setMaxDecPicBuffering( UInt ui, UInt tlayer )                                   { assert(tlayer < MAX_TLAYER); m_uiMaxDecPicBuffering[tlayer] = ui;    }
  UInt                   getMaxLatencyIncreasePlus1(UInt tlayer) const                                   { return m_uiMaxLatencyIncreasePlus1[tlayer];                          }
  Void                   setMaxLatencyIncreasePlus1( UInt ui , UInt tlayer)                              { m_uiMaxLatencyIncreasePlus1[tlayer] = ui;                            }

  Void                   setUseStrongIntraSmoothing(Bool bVal)                                           { m_useStrongIntraSmoothing = bVal;                                    }
  Bool                   getUseStrongIntraSmoothing() const                                              { return m_useStrongIntraSmoothing;                                    }

  Bool                   getVuiParametersPresentFlag() const                                             { return m_vuiParametersPresentFlag;                                   }
  Void                   setVuiParametersPresentFlag(Bool b)                                             { m_vuiParametersPresentFlag = b;                                      }
  TComVUI*               getVuiParameters()                                                              { return &m_vuiParameters;                                             }
  const TComVUI*         getVuiParameters() const                                                        { return &m_vuiParameters;                                             }
  const TComPTL*         getPTL() const                                                                  { return &m_pcPTL;                                                     }
  TComPTL*               getPTL()                                                                        { return &m_pcPTL;                                                     }

  const TComSPSRExt&     getSpsRangeExtension() const                                                    { return m_spsRangeExtension;                                          }
  TComSPSRExt&           getSpsRangeExtension()                                                          { return m_spsRangeExtension;                                          }
};


/// Reference Picture Lists class

class TComRefPicListModification
{
private:
  Bool m_refPicListModificationFlagL0;
  Bool m_refPicListModificationFlagL1;
  UInt m_RefPicSetIdxL0[REF_PIC_LIST_NUM_IDX];
  UInt m_RefPicSetIdxL1[REF_PIC_LIST_NUM_IDX];

public:
          TComRefPicListModification();
  virtual ~TComRefPicListModification();

  Bool    getRefPicListModificationFlagL0() const        { return m_refPicListModificationFlagL0;                                  }
  Void    setRefPicListModificationFlagL0(Bool flag)     { m_refPicListModificationFlagL0 = flag;                                  }
  Bool    getRefPicListModificationFlagL1() const        { return m_refPicListModificationFlagL1;                                  }
  Void    setRefPicListModificationFlagL1(Bool flag)     { m_refPicListModificationFlagL1 = flag;                                  }
  UInt    getRefPicSetIdxL0(UInt idx) const              { assert(idx<REF_PIC_LIST_NUM_IDX); return m_RefPicSetIdxL0[idx];         }
  Void    setRefPicSetIdxL0(UInt idx, UInt refPicSetIdx) { assert(idx<REF_PIC_LIST_NUM_IDX); m_RefPicSetIdxL0[idx] = refPicSetIdx; }
  UInt    getRefPicSetIdxL1(UInt idx) const              { assert(idx<REF_PIC_LIST_NUM_IDX); return m_RefPicSetIdxL1[idx];         }
  Void    setRefPicSetIdxL1(UInt idx, UInt refPicSetIdx) { assert(idx<REF_PIC_LIST_NUM_IDX); m_RefPicSetIdxL1[idx] = refPicSetIdx; }
};



/// PPS RExt class
class TComPPSRExt // Names aligned to text specification
{
private:
  Int              m_log2MaxTransformSkipBlockSize;
  Bool             m_crossComponentPredictionEnabledFlag;

  // Chroma QP Adjustments
  Int              m_diffCuChromaQpOffsetDepth;
  Int              m_chromaQpOffsetListLen; // size (excludes the null entry used in the following array).
  ChromaQpAdj      m_ChromaQpAdjTableIncludingNullEntry[1+MAX_QP_OFFSET_LIST_SIZE]; //!< Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise

  UInt             m_log2SaoOffsetScale[MAX_NUM_CHANNEL_TYPE];

public:
  TComPPSRExt();

  Bool settingsDifferFromDefaults(const bool bTransformSkipEnabledFlag) const
  {
    return (bTransformSkipEnabledFlag && (getLog2MaxTransformSkipBlockSize() !=2))
        || (getCrossComponentPredictionEnabledFlag() )
        || (getChromaQpOffsetListEnabledFlag() )
        || (getLog2SaoOffsetScale(CHANNEL_TYPE_LUMA) !=0 )
        || (getLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA) !=0 );
  }

  UInt                   getLog2MaxTransformSkipBlockSize() const                         { return m_log2MaxTransformSkipBlockSize;         }
  Void                   setLog2MaxTransformSkipBlockSize( UInt u )                       { m_log2MaxTransformSkipBlockSize  = u;           }

  Bool                   getCrossComponentPredictionEnabledFlag() const                   { return m_crossComponentPredictionEnabledFlag;   }
  Void                   setCrossComponentPredictionEnabledFlag(Bool value)               { m_crossComponentPredictionEnabledFlag = value;  }

  Void                   clearChromaQpOffsetList()                                        { m_chromaQpOffsetListLen = 0;                    }

  UInt                   getDiffCuChromaQpOffsetDepth () const                            { return m_diffCuChromaQpOffsetDepth;             }
  Void                   setDiffCuChromaQpOffsetDepth ( UInt u )                          { m_diffCuChromaQpOffsetDepth = u;                }

  Bool                   getChromaQpOffsetListEnabledFlag() const                         { return getChromaQpOffsetListLen()>0;            }
  Int                    getChromaQpOffsetListLen() const                                 { return m_chromaQpOffsetListLen;                 }

  const ChromaQpAdj&     getChromaQpOffsetListEntry( Int cuChromaQpOffsetIdxPlus1 ) const
  {
    assert(cuChromaQpOffsetIdxPlus1 < m_chromaQpOffsetListLen+1);
    return m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1]; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
  }

  Void                   setChromaQpOffsetListEntry( Int cuChromaQpOffsetIdxPlus1, Int cbOffset, Int crOffset )
  {
    assert (cuChromaQpOffsetIdxPlus1 != 0 && cuChromaQpOffsetIdxPlus1 <= MAX_QP_OFFSET_LIST_SIZE);
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CbOffset = cbOffset; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CrOffset = crOffset;
    m_chromaQpOffsetListLen = max(m_chromaQpOffsetListLen, cuChromaQpOffsetIdxPlus1);
  }

  // Now: getPpsRangeExtension().getLog2SaoOffsetScale and getPpsRangeExtension().setLog2SaoOffsetScale
  UInt                   getLog2SaoOffsetScale(ChannelType type) const                    { return m_log2SaoOffsetScale[type];             }
  Void                   setLog2SaoOffsetScale(ChannelType type, UInt uiBitShift)         { m_log2SaoOffsetScale[type] = uiBitShift;       }

};


/// PPS class
class TComPPS
{
private:
  Int              m_PPSId;                    // pic_parameter_set_id
  Int              m_SPSId;                    // seq_parameter_set_id
  Int              m_picInitQPMinus26;
  Bool             m_useDQP;
  Bool             m_bConstrainedIntraPred;    // constrained_intra_pred_flag
  Bool             m_bSliceChromaQpFlag;       // slicelevel_chroma_qp_flag

  // access channel
  UInt             m_uiMaxCuDQPDepth;

  Int              m_chromaCbQpOffset;
  Int              m_chromaCrQpOffset;

  UInt             m_numRefIdxL0DefaultActive;
  UInt             m_numRefIdxL1DefaultActive;

  Bool             m_bUseWeightPred;                    //!< Use of Weighting Prediction (P_SLICE)
  Bool             m_useWeightedBiPred;                 //!< Use of Weighting Bi-Prediction (B_SLICE)
  Bool             m_OutputFlagPresentFlag;             //!< Indicates the presence of output_flag in slice header
  Bool             m_TransquantBypassEnabledFlag;       //!< Indicates presence of cu_transquant_bypass_flag in CUs.
  Bool             m_useTransformSkip;
  Bool             m_dependentSliceSegmentsEnabledFlag; //!< Indicates the presence of dependent slices
  Bool             m_tilesEnabledFlag;                  //!< Indicates the presence of tiles
  Bool             m_entropyCodingSyncEnabledFlag;      //!< Indicates the presence of wavefronts

  Bool             m_loopFilterAcrossTilesEnabledFlag;
  Bool             m_uniformSpacingFlag;
  Int              m_numTileColumnsMinus1;
  Int              m_numTileRowsMinus1;
  std::vector<Int> m_tileColumnWidth;
  std::vector<Int> m_tileRowHeight;

  Bool             m_signDataHidingEnabledFlag;

  Bool             m_cabacInitPresentFlag;

  Bool             m_sliceHeaderExtensionPresentFlag;
  Bool             m_loopFilterAcrossSlicesEnabledFlag;
  Bool             m_deblockingFilterControlPresentFlag;
  Bool             m_deblockingFilterOverrideEnabledFlag;
  Bool             m_ppsDeblockingFilterDisabledFlag;
  Int              m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  Int              m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
  Bool             m_scalingListPresentFlag;
  TComScalingList  m_scalingList;                       //!< ScalingList class
  Bool             m_listsModificationPresentFlag;
  UInt             m_log2ParallelMergeLevelMinus2;
  Int              m_numExtraSliceHeaderBits;

  TComPPSRExt      m_ppsRangeExtension;

public:
                         TComPPS();
  virtual                ~TComPPS();

  Int                    getPPSId() const                                                 { return m_PPSId;                               }
  Void                   setPPSId(Int i)                                                  { m_PPSId = i;                                  }
  Int                    getSPSId() const                                                 { return m_SPSId;                               }
  Void                   setSPSId(Int i)                                                  { m_SPSId = i;                                  }

  Int                    getPicInitQPMinus26() const                                      { return  m_picInitQPMinus26;                   }
  Void                   setPicInitQPMinus26( Int i )                                     { m_picInitQPMinus26 = i;                       }
  Bool                   getUseDQP() const                                                { return m_useDQP;                              }
  Void                   setUseDQP( Bool b )                                              { m_useDQP   = b;                               }
  Bool                   getConstrainedIntraPred() const                                  { return  m_bConstrainedIntraPred;              }
  Void                   setConstrainedIntraPred( Bool b )                                { m_bConstrainedIntraPred = b;                  }
  Bool                   getSliceChromaQpFlag() const                                     { return  m_bSliceChromaQpFlag;                 }
  Void                   setSliceChromaQpFlag( Bool b )                                   { m_bSliceChromaQpFlag = b;                     }

  Void                   setMaxCuDQPDepth( UInt u )                                       { m_uiMaxCuDQPDepth = u;                        }
  UInt                   getMaxCuDQPDepth() const                                         { return m_uiMaxCuDQPDepth;                     }

  Void                   setQpOffset(ComponentID compID, Int i )
  {
    if      (compID==COMPONENT_Cb)
    {
      m_chromaCbQpOffset = i;
    }
    else if (compID==COMPONENT_Cr)
    {
      m_chromaCrQpOffset = i;
    }
    else
    {
      assert(0);
    }
  }
  Int                    getQpOffset(ComponentID compID) const
  {
    return (compID==COMPONENT_Y) ? 0 : (compID==COMPONENT_Cb ? m_chromaCbQpOffset : m_chromaCrQpOffset );
  }

  Void                   setNumRefIdxL0DefaultActive(UInt ui)                             { m_numRefIdxL0DefaultActive=ui;                }
  UInt                   getNumRefIdxL0DefaultActive() const                              { return m_numRefIdxL0DefaultActive;            }
  Void                   setNumRefIdxL1DefaultActive(UInt ui)                             { m_numRefIdxL1DefaultActive=ui;                }
  UInt                   getNumRefIdxL1DefaultActive() const                              { return m_numRefIdxL1DefaultActive;            }

  Bool                   getUseWP() const                                                 { return m_bUseWeightPred;                      }
  Bool                   getWPBiPred() const                                              { return m_useWeightedBiPred;                   }
  Void                   setUseWP( Bool b )                                               { m_bUseWeightPred = b;                         }
  Void                   setWPBiPred( Bool b )                                            { m_useWeightedBiPred = b;                      }

  Void                   setOutputFlagPresentFlag( Bool b )                               { m_OutputFlagPresentFlag = b;                  }
  Bool                   getOutputFlagPresentFlag() const                                 { return m_OutputFlagPresentFlag;               }
  Void                   setTransquantBypassEnabledFlag( Bool b )                         { m_TransquantBypassEnabledFlag = b;            }
  Bool                   getTransquantBypassEnabledFlag() const                           { return m_TransquantBypassEnabledFlag;         }

  Bool                   getUseTransformSkip() const                                      { return m_useTransformSkip;                    }
  Void                   setUseTransformSkip( Bool b )                                    { m_useTransformSkip  = b;                      }

  Void                   setLoopFilterAcrossTilesEnabledFlag(Bool b)                      { m_loopFilterAcrossTilesEnabledFlag = b;       }
  Bool                   getLoopFilterAcrossTilesEnabledFlag() const                      { return m_loopFilterAcrossTilesEnabledFlag;    }
  Bool                   getDependentSliceSegmentsEnabledFlag() const                     { return m_dependentSliceSegmentsEnabledFlag;   }
  Void                   setDependentSliceSegmentsEnabledFlag(Bool val)                   { m_dependentSliceSegmentsEnabledFlag = val;    }
  Bool                   getEntropyCodingSyncEnabledFlag() const                          { return m_entropyCodingSyncEnabledFlag;        }
  Void                   setEntropyCodingSyncEnabledFlag(Bool val)                        { m_entropyCodingSyncEnabledFlag = val;         }

  Void                   setTilesEnabledFlag(Bool val)                                    { m_tilesEnabledFlag = val;                     }
  Bool                   getTilesEnabledFlag() const                                      { return m_tilesEnabledFlag;                    }
  Void                   setTileUniformSpacingFlag(Bool b)                                { m_uniformSpacingFlag = b;                     }
  Bool                   getTileUniformSpacingFlag() const                                { return m_uniformSpacingFlag;                  }
  Void                   setNumTileColumnsMinus1(Int i)                                   { m_numTileColumnsMinus1 = i;                   }
  Int                    getNumTileColumnsMinus1() const                                  { return m_numTileColumnsMinus1;                }
  Void                   setTileColumnWidth(const std::vector<Int>& columnWidth )         { m_tileColumnWidth = columnWidth;              }
  UInt                   getTileColumnWidth(UInt columnIdx) const                         { return  m_tileColumnWidth[columnIdx];         }
  Void                   setNumTileRowsMinus1(Int i)                                      { m_numTileRowsMinus1 = i;                      }
  Int                    getNumTileRowsMinus1() const                                     { return m_numTileRowsMinus1;                   }
  Void                   setTileRowHeight(const std::vector<Int>& rowHeight)              { m_tileRowHeight = rowHeight;                  }
  UInt                   getTileRowHeight(UInt rowIdx) const                              { return m_tileRowHeight[rowIdx];               }

  Void                   setSignDataHidingEnabledFlag( Bool b )                           { m_signDataHidingEnabledFlag = b;              }
  Bool                   getSignDataHidingEnabledFlag() const                             { return m_signDataHidingEnabledFlag;           }

  Void                   setCabacInitPresentFlag( Bool flag )                             { m_cabacInitPresentFlag = flag;                }
  Bool                   getCabacInitPresentFlag() const                                  { return m_cabacInitPresentFlag;                }
  Void                   setDeblockingFilterControlPresentFlag( Bool val )                { m_deblockingFilterControlPresentFlag = val;   }
  Bool                   getDeblockingFilterControlPresentFlag() const                    { return m_deblockingFilterControlPresentFlag;  }
  Void                   setDeblockingFilterOverrideEnabledFlag( Bool val )               { m_deblockingFilterOverrideEnabledFlag = val;  }
  Bool                   getDeblockingFilterOverrideEnabledFlag() const                   { return m_deblockingFilterOverrideEnabledFlag; }
  Void                   setPPSDeblockingFilterDisabledFlag(Bool val)                     { m_ppsDeblockingFilterDisabledFlag = val;      } //!< set offset for deblocking filter disabled
  Bool                   getPPSDeblockingFilterDisabledFlag() const                       { return m_ppsDeblockingFilterDisabledFlag;     } //!< get offset for deblocking filter disabled
  Void                   setDeblockingFilterBetaOffsetDiv2(Int val)                       { m_deblockingFilterBetaOffsetDiv2 = val;       } //!< set beta offset for deblocking filter
  Int                    getDeblockingFilterBetaOffsetDiv2() const                        { return m_deblockingFilterBetaOffsetDiv2;      } //!< get beta offset for deblocking filter
  Void                   setDeblockingFilterTcOffsetDiv2(Int val)                         { m_deblockingFilterTcOffsetDiv2 = val;         } //!< set tc offset for deblocking filter
  Int                    getDeblockingFilterTcOffsetDiv2() const                          { return m_deblockingFilterTcOffsetDiv2;        } //!< get tc offset for deblocking filter
  Bool                   getScalingListPresentFlag() const                                { return m_scalingListPresentFlag;              }
  Void                   setScalingListPresentFlag( Bool b )                              { m_scalingListPresentFlag  = b;                }
  TComScalingList&       getScalingList()                                                 { return m_scalingList;                         }
  const TComScalingList& getScalingList() const                                           { return m_scalingList;                         }
  Bool                   getListsModificationPresentFlag() const                          { return m_listsModificationPresentFlag;        }
  Void                   setListsModificationPresentFlag( Bool b )                        { m_listsModificationPresentFlag = b;           }
  UInt                   getLog2ParallelMergeLevelMinus2() const                          { return m_log2ParallelMergeLevelMinus2;        }
  Void                   setLog2ParallelMergeLevelMinus2(UInt mrgLevel)                   { m_log2ParallelMergeLevelMinus2 = mrgLevel;    }
  Int                    getNumExtraSliceHeaderBits() const                               { return m_numExtraSliceHeaderBits;             }
  Void                   setNumExtraSliceHeaderBits(Int i)                                { m_numExtraSliceHeaderBits = i;                }
  Void                   setLoopFilterAcrossSlicesEnabledFlag( Bool bValue )              { m_loopFilterAcrossSlicesEnabledFlag = bValue; }
  Bool                   getLoopFilterAcrossSlicesEnabledFlag() const                     { return m_loopFilterAcrossSlicesEnabledFlag;   }
  Bool                   getSliceHeaderExtensionPresentFlag() const                       { return m_sliceHeaderExtensionPresentFlag;     }
  Void                   setSliceHeaderExtensionPresentFlag(Bool val)                     { m_sliceHeaderExtensionPresentFlag = val;      }

  const TComPPSRExt&     getPpsRangeExtension() const                                     { return m_ppsRangeExtension;                   }
  TComPPSRExt&           getPpsRangeExtension()                                           { return m_ppsRangeExtension;                   }
};

