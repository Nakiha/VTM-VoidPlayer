/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2026, ITU/ISO/IEC
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

/** \file     dtrace_blockstatistics.cpp
 *  \brief    DTrace block statistcis support for next software
 */

#include "dtrace_blockstatistics.h"
#include "dtrace.h"
#include "dtrace_next.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/Slice.h"
//#include "CommonLib/CodingStructure.h"
#include <queue>
#include <cstdlib>
#include <sstream>

// ---------------------------------------------------------------------------
// Stats output mode selection via environment variables:
//   VTM_BINARY_STATS=<filepath>  → binary VBS1 format (preferred)
//   VTM_COMPACT_STATS=1          → text compact (one line per CU)
//   (neither)                    → original verbose VTM format
// ---------------------------------------------------------------------------

static bool isCompactStatsMode()
{
  static int s_mode = -1;
  if (s_mode == -1)
  {
    const char* env = std::getenv("VTM_COMPACT_STATS");
    s_mode = (env && std::string(env) == "1") ? 1 : 0;
  }
  return s_mode == 1;
}

static const char* binaryStatsPath()
{
  static const char* s_path = (const char*)-1;
  if (s_path == (const char*)-1)
  {
    s_path = std::getenv("VTM_BINARY_STATS");
    if (s_path && s_path[0] == '\0') s_path = nullptr;
  }
  return s_path;
}

static bool isBinaryStatsMode() { return binaryStatsPath() != nullptr; }

// ===========================================================================
// VBS2 Binary Stats Format
// ===========================================================================
//
// File layout:
//   [Vbs2Header  16 bytes]
//   [Frame 0: Vbs2FrameHeader(134B) + CU records ...]
//   [Frame 1: ...]
//   ...
//   [Frame Index: Vbs2IndexEntry(8B) × num_frames]
//
// CU record (variable length):
//   Common (9B):  x(2) y(2) w(1) h(1) depth(1) qp(1) pred_mode(1)
//   If intra (+3B): intra_mode(1) mip(1) isp(1)
//   If inter (+13B): skip(1) merge(1) inter_dir(1) mvL0(4) mvL1(4) refL0(1) refL1(1)

#pragma pack(push, 1)
struct Vbs2Header {
  char     magic[4];       // "VBS2"
  uint16_t width;
  uint16_t height;
  uint32_t num_frames;     // filled at finalize
  uint32_t index_offset;   // filled at finalize
};
struct Vbs2FrameHeader {
  int32_t  poc;            // -1 = sentinel
  int32_t  num_cus;        // patched at endFrame
  uint8_t  temporal_id;    // TId from slice
  uint8_t  slice_type;     // 0=B, 1=P, 2=I (SliceType enum)
  uint8_t  nal_unit_type;  // NalUnitType enum value
  uint8_t  avg_qp;         // patched at endFrame
  uint8_t  num_ref_l0;     // active ref count (0-15)
  uint8_t  num_ref_l1;
  int32_t  ref_pocs_l0[15]; // -1 = unused slot
  int32_t  ref_pocs_l1[15];
};
static_assert(sizeof(Vbs2FrameHeader) == 134, "Vbs2FrameHeader must be 134 bytes");
struct Vbs2CuCommon {
  uint16_t x;
  uint16_t y;
  uint8_t  w;
  uint8_t  h;
  uint8_t  depth;
  uint8_t  qp;
  uint8_t  pred_mode;      // 0=inter 1=intra 2=ibc 3=plt
};
struct Vbs2CuIntra {
  uint8_t  intra_mode;
  uint8_t  mip_flag;
  uint8_t  isp_mode;
};
struct Vbs2CuInter {
  uint8_t  skip;
  uint8_t  merge_flag;
  uint8_t  inter_dir;
  int16_t  mv_l0_x;
  int16_t  mv_l0_y;
  int16_t  mv_l1_x;
  int16_t  mv_l1_y;
  int8_t   ref_l0;
  int8_t   ref_l1;
};
struct Vbs2IndexEntry {
  uint32_t offset;         // file offset of Vbs2FrameHeader
  uint32_t num_cus;
};
#pragma pack(pop)

struct BinaryStatsState {
  FILE*   file = nullptr;
  int     currentPoc = -1;
  long    frameHeaderPos = 0;
  uint32_t frameCuCount = 0;
  uint32_t qpSum = 0;
  uint32_t numFrames = 0;
  uint16_t seqWidth = 0;
  uint16_t seqHeight = 0;
  std::vector<Vbs2IndexEntry> index;

  bool open() {
    if (file) return true;
    const char* path = binaryStatsPath();
    if (!path) return false;
    file = fopen(path, "w+b");
    if (!file) { fprintf(stderr, "VTM_BINARY_STATS: cannot open %s\n", path); return false; }
    // write placeholder header (filled at finalize)
    Vbs2Header hdr = {};
    hdr.magic[0]='V'; hdr.magic[1]='B'; hdr.magic[2]='S'; hdr.magic[3]='2';
    hdr.width = 0; hdr.height = 0;
    hdr.num_frames = 0; hdr.index_offset = 0;
    fwrite(&hdr, sizeof(hdr), 1, file);
    return true;
  }

  void setDimensions(uint16_t w, uint16_t h) {
    seqWidth = w; seqHeight = h;
  }

  void beginFrame(int poc, const Slice* slice) {
    if (!file) return;
    if (currentPoc == poc) return;  // same frame, different CTU
    if (currentPoc >= 0) endFrame();
    currentPoc = poc;
    frameCuCount = 0;
    qpSum = 0;

    // Build extended frame header
    Vbs2FrameHeader fh = {};
    fh.poc = poc;
    fh.num_cus = 0;
    fh.temporal_id = slice ? slice->getTLayer() : 0;
    fh.slice_type = slice ? slice->getSliceType() : 0;
    fh.nal_unit_type = slice ? slice->getNalUnitType() : 0;
    fh.avg_qp = 0;

    if (slice) {
      int nL0 = slice->getNumRefIdx(REF_PIC_LIST_0);
      int nL1 = slice->getNumRefIdx(REF_PIC_LIST_1);
      fh.num_ref_l0 = (uint8_t)std::min(nL0, 15);
      fh.num_ref_l1 = (uint8_t)std::min(nL1, 15);
      for (int i = 0; i < 15; i++) {
        fh.ref_pocs_l0[i] = (i < nL0) ? slice->getRefPOC(REF_PIC_LIST_0, i) : -1;
        fh.ref_pocs_l1[i] = (i < nL1) ? slice->getRefPOC(REF_PIC_LIST_1, i) : -1;
      }
    } else {
      for (int i = 0; i < 15; i++) {
        fh.ref_pocs_l0[i] = -1;
        fh.ref_pocs_l1[i] = -1;
      }
    }

    frameHeaderPos = ftell(file);
    fwrite(&fh, sizeof(fh), 1, file);
  }

  void endFrame() {
    if (!file || currentPoc < 0) return;
    // read back frame header, patch num_cus and avg_qp
    long saved = ftell(file);
    fseek(file, frameHeaderPos, SEEK_SET);
    Vbs2FrameHeader fh;
    fread(&fh, sizeof(fh), 1, file);
    fh.num_cus = (int32_t)frameCuCount;
    fh.avg_qp = frameCuCount > 0 ? (uint8_t)(qpSum / frameCuCount) : 0;
    fseek(file, frameHeaderPos, SEEK_SET);
    fwrite(&fh, sizeof(fh), 1, file);
    fseek(file, saved, SEEK_SET);
    index.push_back({ (uint32_t)frameHeaderPos, frameCuCount });
    numFrames++;
    currentPoc = -1;
  }

  void finalize() {
    if (!file) return;
    endFrame();
    // write frame index
    uint32_t idxOff = (uint32_t)ftell(file);
    for (const auto& e : index) fwrite(&e, sizeof(e), 1, file);
    // patch file header
    fseek(file, 0, SEEK_SET);
    Vbs2Header hdr = {};
    hdr.magic[0]='V'; hdr.magic[1]='B'; hdr.magic[2]='S'; hdr.magic[3]='2';
    hdr.width = seqWidth; hdr.height = seqHeight;
    hdr.num_frames = numFrames; hdr.index_offset = idxOff;
    fwrite(&hdr, sizeof(hdr), 1, file);
    fclose(file); file = nullptr;
  }

  ~BinaryStatsState() { finalize(); }
};

static BinaryStatsState g_binStats;

static void writeAllCodedDataBinary(const CodingStructure& cs, const UnitArea& ctuArea)
{
  const int nShift = MV_FRACTIONAL_BITS_DIFF;
  const int nOffset = 1 << (nShift - 1);
  const int maxNumChannelType = isChromaEnabled(cs.pcv->chrFormat) && CS::isDualITree(cs) ? 2 : 1;

  for (int ch = 0; ch < maxNumChannelType; ch++)
  {
    const ChannelType chType = ChannelType(ch);
    for (const CodingUnit &cu : cs.traverseCUs(CS::getArea(cs, ctuArea, chType), chType))
    {
      if (!isLuma(chType)) continue;

      const int poc = cs.picture->poc;
      g_binStats.beginFrame(poc, cs.slice);
      if (!g_binStats.file) return;

      Vbs2CuCommon common;
      common.x = (uint16_t)cu.lx();
      common.y = (uint16_t)cu.ly();
      common.w = (uint8_t)cu.lwidth();
      common.h = (uint8_t)cu.lheight();
      common.depth = cu.depth;
      common.qp = (uint8_t)cu.qp;
      common.pred_mode = (uint8_t)cu.predMode;
      fwrite(&common, sizeof(common), 1, g_binStats.file);

      switch (cu.predMode)
      {
      case MODE_INTRA:
      {
        Vbs2CuIntra ext = {};
        for (const PredictionUnit &pu : CU::traversePUs(cu))
        {
          if (pu.Y().valid())
          {
            ext.intra_mode = (uint8_t)PU::getFinalIntraMode(pu, ChannelType::LUMA);
            ext.mip_flag = cu.mipFlag ? 1 : 0;
            ext.isp_mode = (uint8_t)to_uint(cu.ispMode);
            break;
          }
        }
        fwrite(&ext, sizeof(ext), 1, g_binStats.file);
        break;
      }
      case MODE_INTER:
      {
        Vbs2CuInter ext = {};
        ext.skip = cu.skip ? 1 : 0;
        for (const PredictionUnit &pu : CU::traversePUs(cu))
        {
          ext.merge_flag = pu.mergeFlag ? 1 : 0;
          ext.inter_dir = (uint8_t)pu.interDir;
          if (pu.interDir != 2)
          {
            Mv mv = pu.mv[REF_PIC_LIST_0];
            mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
            mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
            ext.mv_l0_x = (int16_t)mv.hor;
            ext.mv_l0_y = (int16_t)mv.ver;
          }
          if (pu.interDir != 1)
          {
            Mv mv = pu.mv[REF_PIC_LIST_1];
            mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
            mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
            ext.mv_l1_x = (int16_t)mv.hor;
            ext.mv_l1_y = (int16_t)mv.ver;
          }
          ext.ref_l0 = (int8_t)pu.refIdx[REF_PIC_LIST_0];
          ext.ref_l1 = (int8_t)pu.refIdx[REF_PIC_LIST_1];
          break;
        }
        fwrite(&ext, sizeof(ext), 1, g_binStats.file);
        break;
      }
      default:
        break;
      }
      g_binStats.frameCuCount++;
      g_binStats.qpSum += cu.qp;
    }
  }
}

#define BLOCK_STATS_POLYGON_MIN_POINTS                    3
#define BLOCK_STATS_POLYGON_MAX_POINTS                    5

#if K0149_BLOCK_STATISTICS
std::string GetBlockStatisticName(BlockStatistic statistic)
{
  auto statisticIterator = blockstatistic2description.find(statistic);
  // enforces that all delcared statistic enum items are also part of the map
  assert(statisticIterator != blockstatistic2description.end() && "A block statistics declared in the enum is missing in the map for statistic description.");

  return std::get<0>(statisticIterator->second);
}

std::string GetBlockStatisticTypeString(BlockStatistic statistic)
{
  auto statisticIterator = blockstatistic2description.find(statistic);
  // enforces that all delcared statistic enum items are also part of the map
  assert(statisticIterator != blockstatistic2description.end() && "A block statistics declared in the enum is missing in the map for statistic description.");

  BlockStatisticType statisticType = std::get<1>(statisticIterator->second);
  switch (statisticType) {
  case BlockStatisticType::Flag:
    return std::string("Flag");
    break;
  case BlockStatisticType::Vector:
    return std::string("Vector");
    break;
  case BlockStatisticType::Integer:
    return std::string("Integer");
    break;
  case BlockStatisticType::AffineTFVectors:
    return std::string("AffineTFVectors");
    break;
  case BlockStatisticType::Line:
    return std::string("Line");
    break;
  case BlockStatisticType::FlagPolygon:
    return std::string("FlagPolygon");
    break;
  case BlockStatisticType::VectorPolygon:
    return std::string("VectorPolygon");
    break;
  case BlockStatisticType::IntegerPolygon:
    return std::string("IntegerPolygon");
    break;
  default:
    assert(0);
    break;
  }
  return std::string();
}

std::string GetBlockStatisticTypeSpecificInfo(BlockStatistic statistic)
{
  auto statisticIterator = blockstatistic2description.find(statistic);
  // enforces that all delcared statistic enum items are also part of the map
  assert(statisticIterator != blockstatistic2description.end() && "A block statistics declared in the enum is missing in the map for statistic description.");

  return std::get<2>(statisticIterator->second);
}

void CDTrace::dtrace_block_scalar( int k, const CodingStructure &cs, std::string stat_type, signed value )
{
#if BLOCK_STATS_AS_CSV
  dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, cs.area.lx(), cs.area.ly(), cs.area.lwidth(), cs.area.lheight(), stat_type.c_str(), value );
#else
  dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, cs.area.lx(), cs.area.ly(), cs.area.lwidth(), cs.area.lheight(), stat_type.c_str(), value );
#endif
}

void CDTrace::dtrace_block_scalar( int k, const CodingUnit &cu, std::string stat_type, signed value,  bool isChroma /*= false*/  )
{
  const CodingStructure& cs = *cu.cs;
#if BLOCK_STATS_AS_CSV
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, cu.Cb().x*2, cu.Cb().y*2, cu.Cb().width*2, cu.Cb().height*2, stat_type.c_str(), value );
  }
  else
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, cu.lx(), cu.ly(), cu.lwidth(), cu.lheight(), stat_type.c_str(), value );
  }
#else
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, cu.Cb().x*2, cu.Cb().y*2, cu.Cb().width*2, cu.Cb().height*2, stat_type.c_str(), value );
  }
  else
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, cu.lx(), cu.ly(), cu.lwidth(), cu.lheight(), stat_type.c_str(), value );
  }
#endif
}

void CDTrace::dtrace_block_vector( int k, const CodingUnit &cu, std::string stat_type, signed val_x, signed val_y )
{
  const CodingStructure& cs = *cu.cs;
#if BLOCK_STATS_AS_CSV
  dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n", cs.picture->poc, cu.lx(), cu.ly(), cu.lwidth(), cu.lheight(), stat_type.c_str(), val_x, val_y );
#else
  dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", cs.picture->poc, cu.lx(), cu.ly(), cu.lwidth(), cu.lheight(), stat_type.c_str(), val_x, val_y );
#endif
}

void CDTrace::dtrace_block_scalar( int k, const PredictionUnit &pu, std::string stat_type, signed value, bool isChroma /*= false*/  )
{
  const CodingStructure& cs = *pu.cs;
#if BLOCK_STATS_AS_CSV
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, pu.Cb().x*2, pu.Cb().y*2, pu.Cb().width*2, pu.Cb().height*2, stat_type.c_str(), value );
  }
  else
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), stat_type.c_str(), value );
  }
#else
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, pu.Cb().x*2, pu.Cb().y*2, pu.Cb().width*2, pu.Cb().height*2, stat_type.c_str(), value );
  }
  else
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), stat_type.c_str(), value );
  }
#endif
}

void CDTrace::dtrace_block_vector( int k, const PredictionUnit &pu, std::string stat_type, signed val_x, signed val_y, bool isChroma /*= false*/  )
{
  const CodingStructure& cs = *pu.cs;
#if BLOCK_STATS_AS_CSV
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n", cs.picture->poc,  pu.Cb().x*2, pu.Cb().y*2, pu.Cb().width*2, pu.Cb().height*2, stat_type.c_str(), val_x*2, val_y*2 );
  }
  else
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n", cs.picture->poc, pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), stat_type.c_str(), val_x, val_y );
  }
#else
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", cs.picture->poc, pu.Cb().x*2, pu.Cb().y*2, pu.Cb().width*2, pu.Cb().height*2, stat_type.c_str(), val_x*2, val_y*2 );
  }
  else
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", cs.picture->poc, pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), stat_type.c_str(), val_x, val_y );
  }
#endif
}

void CDTrace::dtrace_block_scalar(int k, const TransformUnit &tu, std::string stat_type, signed value, bool isChroma /*= false*/  )
{
  const CodingStructure& cs = *tu.cs;
#if BLOCK_STATS_AS_CSV
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, tu.Cb().x*2, tu.Cb().y*2, tu.Cb().width*2, tu.Cb().height*2, stat_type.c_str(), value );
  }
  else
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(), stat_type.c_str(), value );
  }
#else
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, tu.Cb().x*2, tu.Cb().y*2, tu.Cb().width*2, tu.Cb().height*2, stat_type.c_str(), value );
  }
  else
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(), stat_type.c_str(), value );
  }
#endif
}

void CDTrace::dtrace_block_vector(int k, const TransformUnit &tu, std::string stat_type, signed val_x, signed val_y)
{
  const CodingStructure& cs = *tu.cs;
#if BLOCK_STATS_AS_CSV
  dtrace<false>(k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n", cs.picture->poc, tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(), stat_type.c_str(), val_x, val_y);
#else
  dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", cs.picture->poc, tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(), stat_type.c_str(), val_x, val_y);
#endif
}

void CDTrace::dtrace_block_affinetf( int k, const PredictionUnit &pu, std::string stat_type, signed val_x0, signed val_y0, signed val_x1, signed val_y1, signed val_x2, signed val_y2 )
{
  const CodingStructure& cs = *pu.cs;
#if BLOCK_STATS_AS_CSV
  dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d;%4d;%4d;%4d;%4d\n",
                 cs.picture->poc, pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), stat_type.c_str(),
                 val_x0, val_y0, val_x1, val_y1 , val_x2, val_y2  );
#else
  dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d,%4d,%4d,%4d,%4d}\n",
                 cs.picture->poc, pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), stat_type.c_str(),
                 val_x0, val_y0, val_x1, val_y1 , val_x2, val_y2  );
#endif
}

void CDTrace::dtrace_block_line(int k, const CodingUnit &cu, std::string stat_type, int x0, int y0, int x1, int y1)
{
#if BLOCK_STATS_AS_CSV
  dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d;%4d;%4d;\n", cu.slice->getPOC(), cu.lx(), cu.ly(), cu.lwidth(), cu.lheight(), stat_type.c_str(), x0, y0, x1, y1);
#else
  dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d,%4d,%4d}\n", cu.slice->getPOC(), cu.lx(), cu.ly(), cu.lwidth(), cu.lheight(), stat_type.c_str(), x0, y0, x1, y1);
#endif
}

void CDTrace::dtrace_polygon_scalar(int k, int poc, const std::vector<Position> &polygon, std::string stat_type, signed value)
{
  assert(polygon.size() >= BLOCK_STATS_POLYGON_MIN_POINTS && "Not enough points to from polygon!");
  assert(polygon.size() <= BLOCK_STATS_POLYGON_MAX_POINTS && "Too many points. Unsupported polygon!");
  std::string polygonDescription;
#if BLOCK_STATS_AS_CSV
  for (auto position : polygon)
  {
    polygonDescription += std::to_string(position.x) + ";" + std::to_string(position.y) + ";";
  }

  dtrace<false>( k, "BlockStat;%d;%s%s;%d\n",poc, polygonDescription.c_str(), stat_type.c_str(), value);
#else
  for (auto position : polygon)
  {
    polygonDescription += "(" + std::to_string(position.x) + ", " + std::to_string(position.y) + ")--";
  }

  dtrace<false>(k, "BlockStat: POC %d @[%s] %s=%d\n", poc, polygonDescription.c_str(), stat_type.c_str(), value);
#endif
}

void CDTrace::dtrace_polygon_vector(int k, int poc, const std::vector<Position> &polygon, std::string stat_type, signed val_x, signed val_y)
{
  assert(polygon.size() >= BLOCK_STATS_POLYGON_MIN_POINTS && "Not enough points to from polygon!");
  assert(polygon.size() <= BLOCK_STATS_POLYGON_MAX_POINTS && "Too many points. Unsupported polygon!");
  std::string polygonDescription;
#if BLOCK_STATS_AS_CSV
  for (auto position : polygon)
  {
    polygonDescription += std::to_string(position.x) + ";" + std::to_string(position.y) + ";";
  }

  dtrace<false>( k, "BlockStat;%d;%s%s;%d;%d\n",poc, polygonDescription.c_str(), stat_type.c_str(), val_x, val_y);
#else
  for (auto position : polygon)
  {
    polygonDescription += "(" + std::to_string(position.x) + ", " + std::to_string(position.y) + ")--";
  }

  dtrace<false>(k, "BlockStat: POC %d @[%s] %s={%4d,%4d}\n", poc, polygonDescription.c_str(), stat_type.c_str(), val_x, val_y);
#endif
}

void retrieveGeoPolygons(const CodingUnit& cu, std::vector<Position> (&geoPartitions)[2], Position (&linePositions)[2])
{
  // adapted code from interpolation filter to find geo partition polygons like this:
  // use SAD mask, which should clearly partition the two polygons.
  // loop over boundary pixels and find positions where there is a change, these should be the polygon corners
  static bool isInitialized = false;
  static std::vector<Position> allGeoPartitionings[GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][GEO_NUM_PARTITION_MODE][2];
  static Position allGeoPartitioningLines[GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][GEO_NUM_PARTITION_MODE][2];

  if(!isInitialized)
  {
    for( int hIdx = 0; hIdx < GEO_NUM_CU_SIZE; hIdx++ )
    {
      int16_t height = 1 << ( hIdx + GEO_MIN_CU_LOG2);
      for( int wIdx = 0; wIdx < GEO_NUM_CU_SIZE; wIdx++ )
      {
        int16_t width = 1 << (wIdx + GEO_MIN_CU_LOG2);
        for( int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++ )
        {
          const int angle = g_geoParams[splitDir].angleIdx;

          int maskStride = 0;
          int stepX = 1;
          Pel *sadMask;

          const int16_t *wOffset = g_weightOffset[splitDir][hIdx][wIdx];

          if (g_angle2mirror[angle] == 2)
          {
            maskStride = -GEO_WEIGHT_MASK_SIZE;

            sadMask =
              &g_globalGeoEncSADmask[g_angle2mask[angle]]
                                    [(GEO_WEIGHT_MASK_SIZE - 1 - wOffset[1]) * GEO_WEIGHT_MASK_SIZE + wOffset[0]];
          }
          else if (g_angle2mirror[angle] == 1)
          {
            stepX = -1;
            maskStride = GEO_WEIGHT_MASK_SIZE;

            sadMask = &g_globalGeoEncSADmask[g_angle2mask[angle]][wOffset[1] * GEO_WEIGHT_MASK_SIZE
                                                                  + (GEO_WEIGHT_MASK_SIZE - 1 - wOffset[0])];
          }
          else
          {
            maskStride = GEO_WEIGHT_MASK_SIZE;

            sadMask = &g_globalGeoEncSADmask[g_angle2mask[angle]][wOffset[1] * GEO_WEIGHT_MASK_SIZE + wOffset[0]];
          }

          int currentPartition = 0;
          std::vector<Pel> boundaryOfMask; // for debugging

          Area partitionArea = Area(0, 0, width, height);
          Position TL = partitionArea.topLeft();
          Position TR = partitionArea.topRight();    TR = TR.offset(1, 0);
          Position BL = partitionArea.bottomLeft();  BL = BL.offset(0, 1);
          Position BR = partitionArea.bottomRight(); BR = BR.offset(1, 1);

          std::vector<Position> oneGeoPartitioning[2];
          Position oneGeoPartitioningLine[2];
          // corner of block is a corner of the first partition
          oneGeoPartitioning[currentPartition].push_back(TL);

          // process top boundary
          for( int x = 0; x < width-1; x++ )
          {
            boundaryOfMask.push_back(*sadMask);
            if (*sadMask != *(sadMask + stepX))
            {
              // found a change of partitions, it is a corner of both partition polygons
              oneGeoPartitioning[currentPartition].push_back(Position(TL.x + x, TL.y));
              oneGeoPartitioningLine[currentPartition] = Position(TL.x + x, TL.y);
              currentPartition ^= 0x01;
              oneGeoPartitioning[currentPartition].push_back(Position(TL.x + x, TL.y));
            }
            sadMask += stepX;
          }

          // corner of block is a corner of the current partition
          oneGeoPartitioning[currentPartition].push_back(TR);

          // process right boundary
          for( int y = 0; y < height-1; y++ )
          {
            boundaryOfMask.push_back(*sadMask);
            if (*sadMask != *(sadMask + maskStride))
            {
              // found a change of partitions, it is a corner of both partition polygons
              oneGeoPartitioning[currentPartition].push_back(Position(TR.x, TR.y + y));
              oneGeoPartitioningLine[currentPartition] = Position(TR.x, TR.y + y);
              currentPartition ^= 0x01;
              oneGeoPartitioning[currentPartition].push_back(Position(TR.x, TR.y + y));
            }
            sadMask += maskStride;
          }

          // corner of block is a corner of the current partition
          oneGeoPartitioning[currentPartition].push_back(BR);

          // process bottom boundary
          for( int x = width-1; x > 0; x-- )
          {
            boundaryOfMask.push_back(*sadMask);
            if (*sadMask != *(sadMask - stepX))
            {
              // found a change of partitions, it is a corner of both partition polygons
              oneGeoPartitioning[currentPartition].push_back(Position(BL.x + x, BL.y));
              oneGeoPartitioningLine[currentPartition] = Position(BL.x + x, BL.y);
              currentPartition ^= 0x01;
              oneGeoPartitioning[currentPartition].push_back(Position(BL.x + x, BL.y));
            }
            sadMask -= stepX;
          }

          // corner of block is a corner of the current partition
          oneGeoPartitioning[currentPartition].push_back(BL);

          // process left boundary
          for( int y = height-1; y > 0; y-- )
          {
            boundaryOfMask.push_back(*sadMask);
            if (*sadMask != *(sadMask - maskStride))
            {
              // found a change of partitions, it is a corner of both partition polygons
              oneGeoPartitioning[currentPartition].push_back(Position(TL.x, TL.y + y));
              oneGeoPartitioningLine[currentPartition] = Position(TL.x, TL.y + y);
              currentPartition ^= 0x01;
              oneGeoPartitioning[currentPartition].push_back(Position(TL.x, TL.y + y));
            }
            sadMask -= maskStride;
          }

          // corner of block is a corner of the current partition
          oneGeoPartitioning[currentPartition].push_back(TL);

          // remove duplicate points
          for( auto geoPartIdx = 0; geoPartIdx < 2; geoPartIdx++)
          {
            // this will only remove consecutive duplicates
            auto last = std::unique(oneGeoPartitioning[geoPartIdx].begin(), oneGeoPartitioning[geoPartIdx].end());
            oneGeoPartitioning[geoPartIdx].erase(last, oneGeoPartitioning[geoPartIdx].end());
            // also check if first and last are the same
            if(oneGeoPartitioning[geoPartIdx].front() == oneGeoPartitioning[geoPartIdx].back())
            {
              oneGeoPartitioning[geoPartIdx].pop_back();
            }

            CHECK(!(oneGeoPartitioning[geoPartIdx].size() > 2 && oneGeoPartitioning[geoPartIdx].size() < 6), "Invalid geo partition shape. Polygon should have between 3 and 5 corners.");
          }

          allGeoPartitionings[hIdx][wIdx][splitDir][0] = oneGeoPartitioning[0];
          allGeoPartitionings[hIdx][wIdx][splitDir][1] = oneGeoPartitioning[1];
          allGeoPartitioningLines[hIdx][wIdx][splitDir][0] = oneGeoPartitioningLine[0];
          allGeoPartitioningLines[hIdx][wIdx][splitDir][1] = oneGeoPartitioningLine[1];
        }
      }
    }
    isInitialized = true;
  }

  const uint8_t splitDir = cu.firstPU->geoSplitDir;
  int16_t wIdx = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t hIdx = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2;

  Position TL = cu.Y().topLeft();

  geoPartitions[0] = allGeoPartitionings[hIdx][wIdx][splitDir][0];
  geoPartitions[1] = allGeoPartitionings[hIdx][wIdx][splitDir][1];
  linePositions[0] = allGeoPartitioningLines[hIdx][wIdx][splitDir][0];
  linePositions[1] = allGeoPartitioningLines[hIdx][wIdx][splitDir][1];

  // offset the partitioning to the current cu
  for( auto geoPartIdx = 0; geoPartIdx < 2; geoPartIdx++)
  {
    for( Position &polygonCorner : geoPartitions[geoPartIdx])
    {
      polygonCorner.repositionTo(polygonCorner.offset(TL));
    }
  }
}

std::queue<MergeCtx> geoMergeCtxtsOfCurrentCtu;
void storeGeoMergeCtx(MergeCtx geoMergeCtx)
{
  geoMergeCtxtsOfCurrentCtu.push(geoMergeCtx);
}

void writeBlockStatisticsHeader(const SPS *sps)
{
  static bool has_header_been_written = false;
  if (has_header_been_written)
  {
    return;
  }

  // only write header when block statistics are used
  bool write_blockstatistics =   g_trace_ctx->isChannelActive( D_BLOCK_STATISTICS_ALL) || g_trace_ctx->isChannelActive( D_BLOCK_STATISTICS_CODED);
  if(!write_blockstatistics)
  {
    return;
  }

  if (isBinaryStatsMode())
  {
    g_binStats.open();
    g_binStats.setDimensions(
      (uint16_t)sps->getMaxPicWidthInLumaSamples(),
      (uint16_t)sps->getMaxPicHeightInLumaSamples());
    // Suppress text header for binary mode by writing a brief note to dtrace
    DTRACE_HEADER( g_trace_ctx, "# VoidPlayer Binary Stats (.vbs2) — see %s\n", binaryStatsPath());
  }
  else if (isCompactStatsMode())
  {
    DTRACE_HEADER( g_trace_ctx, "# VoidPlayer Compact Block Statistics\n");
    DTRACE_HEADER( g_trace_ctx, "# Sequence size: %dx%d\n", sps->getMaxPicWidthInLumaSamples(), sps->getMaxPicHeightInLumaSamples() );
    DTRACE_HEADER( g_trace_ctx, "# Columns: poc x y w h depth qp pred\n");
    DTRACE_HEADER( g_trace_ctx, "# Intra extends: intra_mode mip isp\n");
    DTRACE_HEADER( g_trace_ctx, "# Inter extends: skip merge interDir mvL0x mvL0y mvL1x mvL1y refL0 refL1\n");
  }
  else
  {
    DTRACE_HEADER( g_trace_ctx, "# VTMBMS Block Statistics\n");
    // sequence info
    DTRACE_HEADER( g_trace_ctx, "# Sequence size: [%dx %d]\n", sps->getMaxPicWidthInLumaSamples(), sps->getMaxPicHeightInLumaSamples() );
    // list statistics
    for( auto i = static_cast<int>(BlockStatistic::PredMode); i < static_cast<int>(BlockStatistic::NumBlockStatistics); i++)
    {
      BlockStatistic statistic = BlockStatistic(i);
      std::string statitic_name = GetBlockStatisticName(statistic);
      std::string statitic_type = GetBlockStatisticTypeString(statistic);
      std::string statitic_type_specific_info = GetBlockStatisticTypeSpecificInfo(statistic);
      DTRACE_HEADER( g_trace_ctx, "# Block Statistic Type: %s; %s; %s\n", statitic_name.c_str(), statitic_type.c_str(), statitic_type_specific_info.c_str());
    }
  }

  has_header_been_written = true;
}

// Forward declaration
static void writeAllCodedDataCompact(const CodingStructure& cs, const UnitArea& ctuArea);

void getAndStoreBlockStatistics(const CodingStructure& cs, const UnitArea& ctuArea)
{
  // two differemt behaviors, depending on which information is needed
  bool writeAll =   g_trace_ctx->isChannelActive( D_BLOCK_STATISTICS_ALL);
  bool writeCoded =   g_trace_ctx->isChannelActive( D_BLOCK_STATISTICS_CODED);

  CHECK(writeAll && writeCoded, "Either used D_BLOCK_STATISTICS_ALL or D_BLOCK_STATISTICS_CODED. Not both at once!")

  if (writeCoded)
  {
    if (isBinaryStatsMode())
      writeAllCodedDataBinary(cs, ctuArea);
    else if (isCompactStatsMode())
      writeAllCodedDataCompact(cs, ctuArea);
    else
      writeAllCodedData(cs, ctuArea);
  }
  else if (writeAll)
    writeAllData(cs, ctuArea);         // this will write out all inter- or intra-prediction related data
}

void writeAllData(const CodingStructure& cs, const UnitArea& ctuArea)
{
  const int maxNumChannelType = isChromaEnabled(cs.pcv->chrFormat) && CS::isDualITree(cs) ? 2 : 1;
  const int nShift = MV_FRACTIONAL_BITS_DIFF;
  const int nOffset = 1 << (nShift - 1);
  for( int ch = 0; ch < maxNumChannelType; ch++ )
  {
    const ChannelType chType = ChannelType( ch );

    for( const CodingUnit &cu : cs.traverseCUs( CS::getArea( cs, ctuArea, chType ), chType ) )
    {
      if (isLuma(chType))
      {
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::PredMode), cu.predMode);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::Depth), cu.depth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::QT_Depth), cu.qtDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::BT_Depth), cu.btDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::MT_Depth), cu.mtDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::ChromaQPAdj), cu.chromaQpAdj);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::QP), cu.qp);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SplitSeries), (int)cu.splitSeries);

        // skip flag
        if (!cs.slice->isIntra())
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SkipFlag), cu.skip);
        }

        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::BDPCM),
                            to_underlying(cu.bdpcmMode));
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::BDPCMChroma),
                            to_underlying(cu.bdpcmModeChroma));
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::TileIdx), cu.tileIdx);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::IndependentSliceIdx), cu.slice->getIndependentSliceIdx());
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::LFNSTIdx), cu.lfnstIdx);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::MMVDSkipFlag), cu.mmvdSkip);
      }
      else if (chType == ChannelType::CHROMA)
      {
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::Depth_Chroma), cu.depth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::QT_Depth_Chroma), cu.qtDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::BT_Depth_Chroma), cu.btDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::MT_Depth_Chroma), cu.mtDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::ChromaQPAdj_Chroma), cu.chromaQpAdj);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::QP_Chroma), cu.qp);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SplitSeries_Chroma), (int)cu.splitSeries);

        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::BDPCMChroma),
                            to_underlying(cu.bdpcmModeChroma));
      }


      switch( cu.predMode )
      {
      case MODE_INTER:
        {
          for( const PredictionUnit &pu : CU::traversePUs( cu ) )
          {
            if (!pu.cu->skip)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MergeFlag), pu.mergeFlag);
            }
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::RegularMergeFlag), pu.regularMergeFlag);
            if( pu.mergeFlag )
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MergeIdx),  pu.mergeIdx);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu,
                                  GetBlockStatisticName(BlockStatistic::MergeType), to_underlying(pu.mergeType));
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MMVDMergeFlag),  pu.mmvdMergeFlag);
              if (cu.mmvdSkip || pu.mmvdMergeFlag)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu,
                                    GetBlockStatisticName(BlockStatistic::MMVDMergeIdx), pu.mmvdMergeIdx.val);
              }
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::CiipFlag),  pu.ciipFlag);
              if (pu.ciipFlag)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu,
                                    GetBlockStatisticName(BlockStatistic::Luma_IntraMode),
                                    pu.intraDir[ChannelType::LUMA]);
              }
            }
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::AffineFlag), pu.cu->affine);
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu,
                                GetBlockStatisticName(BlockStatistic::AffineType), to_underlying(pu.cu->affineType));
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::InterDir), pu.interDir);

            if (pu.interDir != 2 /* PRED_L1 */)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MVPIdxL0), pu.mvpIdx[REF_PIC_LIST_0]);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::RefIdxL0), pu.refIdx[REF_PIC_LIST_0]);
            }
            if (pu.interDir != 1 /* PRED_L1 */)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MVPIdxL1), pu.mvpIdx[REF_PIC_LIST_1]);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::RefIdxL1), pu.refIdx[REF_PIC_LIST_1]);
            }
            if (!pu.cu->affine && !pu.cu->geoFlag)
            {
              if (pu.interDir != 2 /* PRED_L1 */)
              {
                Mv mv = pu.mv[REF_PIC_LIST_0];
                Mv mvd = pu.mvd[REF_PIC_LIST_0];
                mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
                mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
                mvd.hor = mvd.hor >= 0 ? (mvd.hor + nOffset) >> nShift : -((-mvd.hor + nOffset) >> nShift);
                mvd.ver = mvd.ver >= 0 ? (mvd.ver + nOffset) >> nShift : -((-mvd.ver + nOffset) >> nShift);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MVDL0), mvd.hor, mvd.ver);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MVL0), mv.hor, mv.ver);
              }
              if (pu.interDir != 1 /* PRED_L1 */)
              {
                Mv mv = pu.mv[REF_PIC_LIST_1];
                Mv mvd = pu.mvd[REF_PIC_LIST_1];
                mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
                mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
                mvd.hor = mvd.hor >= 0 ? (mvd.hor + nOffset) >> nShift : -((-mvd.hor + nOffset) >> nShift);
                mvd.ver = mvd.ver >= 0 ? (mvd.ver + nOffset) >> nShift : -((-mvd.ver + nOffset) >> nShift);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MVDL1), mvd.hor, mvd.ver);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MVL1), mv.hor, mv.ver);
              }
            }
            else if (pu.cu->affine)
            {
              if (pu.interDir != 2 /* PRED_L1 */)
              {
                Mv mv[3];
                const CMotionBuf &mb = pu.getMotionBuf();
                mv[0] = mb.at(0, 0).mv[REF_PIC_LIST_0];
                mv[1] = mb.at(mb.width - 1, 0).mv[REF_PIC_LIST_0];
                mv[2] = mb.at(0, mb.height - 1).mv[REF_PIC_LIST_0];
                // motion vectors should use low precision or they will appear to large
                mv[0].hor = mv[0].hor >= 0 ? (mv[0].hor + nOffset) >> nShift : -((-mv[0].hor + nOffset) >> nShift);
                mv[0].ver = mv[0].ver >= 0 ? (mv[0].ver + nOffset) >> nShift : -((-mv[0].ver + nOffset) >> nShift);
                mv[1].hor = mv[1].hor >= 0 ? (mv[1].hor + nOffset) >> nShift : -((-mv[1].hor + nOffset) >> nShift);
                mv[1].ver = mv[1].ver >= 0 ? (mv[1].ver + nOffset) >> nShift : -((-mv[1].ver + nOffset) >> nShift);
                mv[2].hor = mv[2].hor >= 0 ? (mv[2].hor + nOffset) >> nShift : -((-mv[2].hor + nOffset) >> nShift);
                mv[2].ver = mv[2].ver >= 0 ? (mv[2].ver + nOffset) >> nShift : -((-mv[2].ver + nOffset) >> nShift);
                DTRACE_BLOCK_AFFINETF(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::AffineMVL0), mv[0].hor, mv[0].ver, mv[1].hor, mv[1].ver, mv[2].hor, mv[2].ver);
              }
              if (pu.interDir != 1 /* PRED_L1 */)
              {
                Mv mv[3];
                const CMotionBuf &mb = pu.getMotionBuf();
                mv[0] = mb.at(0, 0).mv[REF_PIC_LIST_1];
                mv[1] = mb.at(mb.width - 1, 0).mv[REF_PIC_LIST_1];
                mv[2] = mb.at(0, mb.height - 1).mv[REF_PIC_LIST_1];
                // motion vectors should use low precision or they will appear to large
                mv[0].hor = mv[0].hor >= 0 ? (mv[0].hor + nOffset) >> nShift : -((-mv[0].hor + nOffset) >> nShift);
                mv[0].ver = mv[0].ver >= 0 ? (mv[0].ver + nOffset) >> nShift : -((-mv[0].ver + nOffset) >> nShift);
                mv[1].hor = mv[1].hor >= 0 ? (mv[1].hor + nOffset) >> nShift : -((-mv[1].hor + nOffset) >> nShift);
                mv[1].ver = mv[1].ver >= 0 ? (mv[1].ver + nOffset) >> nShift : -((-mv[1].ver + nOffset) >> nShift);
                mv[2].hor = mv[2].hor >= 0 ? (mv[2].hor + nOffset) >> nShift : -((-mv[2].hor + nOffset) >> nShift);
                mv[2].ver = mv[2].ver >= 0 ? (mv[2].ver + nOffset) >> nShift : -((-mv[2].ver + nOffset) >> nShift);
                DTRACE_BLOCK_AFFINETF(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::AffineMVL1), mv[0].hor, mv[0].ver, mv[1].hor, mv[1].ver, mv[2].hor, mv[2].ver);
              }
            }

            // tracing Motion buffers
            CMotionBuf mb = pu.getMotionBuf();
            // todo: assuming granulatiry == 4. can it be derived?
            for( int y = 0; y < mb.height; y++ )
            {
              for( int x = 0; x < mb.width; x++ )
              {
                const MotionInfo &pixMi = mb.at( x, y );

                if( pixMi.interDir == 1)
                {
                  const Mv mv = pixMi.mv[REF_PIC_LIST_0];
#if BLOCK_STATS_AS_CSV
                  g_trace_ctx->dtrace<false>(
                    D_BLOCK_STATISTICS_ALL,
                    "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n",
                     cs.picture->poc,
                     pu.lx() + 4*x,
                     pu.ly() + 4*y,
                     4,
                     4,
                     GetBlockStatisticName(BlockStatistic::MotionBufL0).c_str(),
                     mv.hor,
                     mv.ver);
#else
                  g_trace_ctx->dtrace<false>(
                    D_BLOCK_STATISTICS_ALL,
                    "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n",
                     cs.picture->poc,
                     pu.lx() + 4*x,
                     pu.ly() + 4*y,
                     4,
                     4,
                     GetBlockStatisticName(BlockStatistic::MotionBufL0).c_str(),
                     mv.hor,
                     mv.ver);
#endif
                }
                else if( pixMi.interDir == 2)
                {
                  const Mv mv = pixMi.mv[REF_PIC_LIST_1];
#if BLOCK_STATS_AS_CSV
                  g_trace_ctx->dtrace<false>(
                    D_BLOCK_STATISTICS_ALL,
                    "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n",
                     cs.picture->poc,
                     pu.lx() + 4*x,
                     pu.ly() + 4*y,
                     4,
                     4,
                     GetBlockStatisticName(BlockStatistic::MotionBufL1).c_str(),
                     mv.hor,
                     mv.ver);
#else
                  g_trace_ctx->dtrace<false>(
                    D_BLOCK_STATISTICS_ALL,
                    "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n",
                     cs.picture->poc,
                     pu.lx() + 4*x,
                     pu.ly() + 4*y,
                     4,
                     4,
                     GetBlockStatisticName(BlockStatistic::MotionBufL1).c_str(),
                     mv.hor,
                     mv.ver);
#endif
                }
                else if( pixMi.interDir == 3)
                {
                  {
                    const Mv mv = pixMi.mv[REF_PIC_LIST_0];
#if BLOCK_STATS_AS_CSV
                  g_trace_ctx->dtrace<false>(
                    D_BLOCK_STATISTICS_ALL,
                    "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n",
                     cs.picture->poc,
                     pu.lx() + 4*x,
                     pu.ly() + 4*y,
                     4,
                     4,
                     GetBlockStatisticName(BlockStatistic::MotionBufL0).c_str(),
                     mv.hor,
                     mv.ver);
#else
                  g_trace_ctx->dtrace<false>(
                    D_BLOCK_STATISTICS_ALL,
                    "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n",
                     cs.picture->poc,
                     pu.lx() + 4*x,
                     pu.ly() + 4*y,
                     4,
                     4,
                     GetBlockStatisticName(BlockStatistic::MotionBufL0).c_str(),
                     mv.hor,
                     mv.ver);
#endif
                  }
                  {
                    const Mv mv = pixMi.mv[REF_PIC_LIST_1];
#if BLOCK_STATS_AS_CSV
                  g_trace_ctx->dtrace<false>(
                    D_BLOCK_STATISTICS_ALL,
                    "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n",
                     cs.picture->poc,
                     pu.lx() + 4*x,
                     pu.ly() + 4*y,
                     4,
                     4,
                     GetBlockStatisticName(BlockStatistic::MotionBufL1).c_str(),
                     mv.hor,
                     mv.ver);
#else
                  g_trace_ctx->dtrace<false>(
                    D_BLOCK_STATISTICS_ALL,
                    "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n",
                     cs.picture->poc,
                     pu.lx() + 4*x,
                     pu.ly() + 4*y,
                     4,
                     4,
                     GetBlockStatisticName(BlockStatistic::MotionBufL1).c_str(),
                     mv.hor,
                     mv.ver);
#endif
                  }
                }
              }
            }
          }

          if (cu.geoFlag)
          {
            const uint8_t         candIdx0 = cu.firstPU->geoMergeIdx[0];
            const uint8_t         candIdx1 = cu.firstPU->geoMergeIdx[1];
            std::vector<Position> geoPartitions[2];
            Position linePositions[2];
            retrieveGeoPolygons(cu, geoPartitions, linePositions);
            DTRACE_LINE(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::GeoPartitioning), linePositions[0].x, linePositions[0].y, linePositions[1].x, linePositions[1].y);

            if(geoMergeCtxtsOfCurrentCtu.size() > 0)
            // Geo partition MVs can only be stored when using the statistics with the decoder. Encoder is not supported
            {
              MergeCtx geoMrgCtx = geoMergeCtxtsOfCurrentCtu.front();
              geoMergeCtxtsOfCurrentCtu.pop();

              // first partition
              {
                PredictionUnit tmpPu = *cu.firstPU;
                geoMrgCtx.setMergeInfo( tmpPu, candIdx0 );
                const int geoPartIdx = 0;
                for (int refIdx = 0; refIdx < 2; refIdx++)
                {
                  if (tmpPu.refIdx[refIdx] != -1)
                  {
                    Mv tmpMv = tmpPu.mv[refIdx];
                    tmpMv.hor = tmpMv.hor >= 0 ? (tmpMv.hor + nOffset) >> nShift : -((-tmpMv.hor + nOffset) >> nShift);
                    tmpMv.ver = tmpMv.ver >= 0 ? (tmpMv.ver + nOffset) >> nShift : -((-tmpMv.ver + nOffset) >> nShift);
                    DTRACE_POLYGON_VECTOR(g_trace_ctx,
                                          D_BLOCK_STATISTICS_ALL,
                                          cu.slice->getPOC(),
                                          geoPartitions[geoPartIdx],
                                          GetBlockStatisticName(refIdx==0?BlockStatistic::GeoMVL0:BlockStatistic::GeoMVL1),
                                          tmpMv.hor,
                                          tmpMv.ver
                                          );
                  }
                }
              }

              // second partition
              {
                PredictionUnit tmpPu = *cu.firstPU;
                geoMrgCtx.setMergeInfo( tmpPu, candIdx1 );
                const int geoPartIdx = 1;
                {
                  for (int refIdx = 0; refIdx < 2; refIdx++)
                  {
                    if (tmpPu.refIdx[refIdx] != -1)
                    {
                      Mv tmpMv = tmpPu.mv[refIdx];
                      tmpMv.hor = tmpMv.hor >= 0 ? (tmpMv.hor + nOffset) >> nShift : -((-tmpMv.hor + nOffset) >> nShift);
                      tmpMv.ver = tmpMv.ver >= 0 ? (tmpMv.ver + nOffset) >> nShift : -((-tmpMv.ver + nOffset) >> nShift);
                      DTRACE_POLYGON_VECTOR(g_trace_ctx,
                                            D_BLOCK_STATISTICS_ALL,
                                            cu.slice->getPOC(),
                                            geoPartitions[geoPartIdx],
                                            GetBlockStatisticName(refIdx==0?BlockStatistic::GeoMVL0:BlockStatistic::GeoMVL1),
                                            tmpMv.hor,
                                            tmpMv.ver
                                            );
                    }
                  }
                }
              }
            }
          }

          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SMVDFlag), cu.smvdMode);
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::IMVMode), cu.imv);
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::RootCbf), cu.rootCbf);
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::BCWIndex),
                              cu.bcwIdx);
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SbtIdx), cu.getSbtIdx());
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SbtPos), cu.getSbtPos());
        }
        break;
      case MODE_INTRA:
        {
          if (isLuma(chType))
          {
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::MIPFlag), cu.mipFlag);
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::ISPMode), to_uint(cu.ispMode));
          }

          for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(cu.chromaFormat); chType++)
          {
            if (cu.block(chType).valid())
            {
              for( const PredictionUnit &pu : CU::traversePUs( cu ) )
              {
                if (isLuma(chType))
                {
                  const uint32_t chFinalMode = PU::getFinalIntraMode(pu, ChannelType(chType));
                  DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu,
                                      GetBlockStatisticName(BlockStatistic::Luma_IntraMode), chFinalMode);
                  DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MultiRefIdx), pu.multiRefIdx);
                }
                else
                {
                  const uint32_t chFinalMode = PU::getFinalIntraMode(pu, ChannelType(chType));
                  DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu,
                                             GetBlockStatisticName(BlockStatistic::Chroma_IntraMode), chFinalMode);
                  assert(0);
                }
              }
            }
          }
        }
        break;
      default:
        THROW( "Invalid prediction mode" );
        break;
      }

      for (const TransformUnit &tu : CU::traverseTUs(cu))
      {
        if (tu.Y().valid())
        {
          int cbf = tu.cbf[COMPONENT_Y] & (1 << tu.depth);
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::Cbf_Y), cbf);
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::MTSIdx_Y),
                              to_underlying(tu.mtsIdx[COMPONENT_Y]));
        }
        if ( tu.Cb().valid() )
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::JointCbCr), tu.jointCbCr);
        }

        if (!(!isChromaEnabled(cu.chromaFormat) || (cu.isSepTree() && isLuma(cu.chType))))
        {
          int cbf = tu.cbf[COMPONENT_Cb] & (1 << tu.depth);
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::Cbf_Cb), cbf);
          cbf = tu.cbf[COMPONENT_Cr] & (1 << tu.depth);
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::Cbf_Cr), cbf);
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu,
                                     GetBlockStatisticName(BlockStatistic::MTSIdx_Cb),
                                     to_underlying(tu.mtsIdx[COMPONENT_Cb]));
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu,
                                     GetBlockStatisticName(BlockStatistic::MTSIdx_Cr),
                                     to_underlying(tu.mtsIdx[COMPONENT_Cr]));
        }
      }
    }
  }

  CHECK(geoMergeCtxtsOfCurrentCtu.size() != 0, "Did not use all pushed back geo merge contexts. Should not be possible!");
}

void writeAllCodedData(const CodingStructure & cs, const UnitArea & ctuArea)
{
  const int nShift = MV_FRACTIONAL_BITS_DIFF;
  const int nOffset = 1 << (nShift - 1);
  const int maxNumChannelType = isChromaEnabled(cs.pcv->chrFormat) && CS::isDualITree(cs) ? 2 : 1;

  for (int ch = 0; ch < maxNumChannelType; ch++)
  {
    const ChannelType chType = ChannelType(ch);

    for (const CodingUnit &cu : cs.traverseCUs(CS::getArea(cs, ctuArea, chType), chType))
    {
      if (isLuma(chType))
      {
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::Depth), cu.depth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::QT_Depth), cu.qtDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::BT_Depth), cu.btDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::MT_Depth), cu.mtDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::ChromaQPAdj), cu.chromaQpAdj);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::QP), cu.qp);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::SplitSeries), (int)cu.splitSeries);
        // skip flag
        if (!cs.slice->isIntra() && cu.Y().valid())
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::SkipFlag), cu.skip);
          if (cu.skip)
          {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::MMVDSkipFlag), cu.mmvdSkip);
          }
        }

        // prediction mode and partitioning data
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::PredMode), cu.predMode);

      }
      else if (chType == ChannelType::CHROMA)
      {
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::Depth_Chroma), cu.depth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::QT_Depth_Chroma), cu.qtDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::BT_Depth_Chroma), cu.btDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::MT_Depth_Chroma), cu.mtDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::ChromaQPAdj_Chroma), cu.chromaQpAdj);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::QP_Chroma), cu.qp);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::SplitSeries_Chroma), (int)cu.splitSeries);

      }

      for (const PredictionUnit &pu : CU::traversePUs(cu))
      {
        switch (pu.cu->predMode)
        {
          case MODE_INTRA:
          {
            if (pu.Y().valid())
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::Luma_IntraMode), PU::getFinalIntraMode(pu, ChannelType(chType)));
            }
            if (!(!isChromaEnabled(pu.chromaFormat) || (pu.cu->isSepTree() && isLuma(pu.chType))))
            {
              DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu,
                                         GetBlockStatisticName(BlockStatistic::Chroma_IntraMode),
                                         PU::getFinalIntraMode(pu, ChannelType::CHROMA));
            }
            if (cu.Y().valid() && isLuma(cu.chType))
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MultiRefIdx), pu.multiRefIdx);
            }
            break;
          }
          case MODE_INTER:
          {
            if (!pu.cu->skip)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MergeFlag), pu.mergeFlag);
            }
            if (pu.mergeFlag)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MergeIdx),  pu.mergeIdx);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu,
                                  GetBlockStatisticName(BlockStatistic::MergeType), to_underlying(pu.mergeType));
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MMVDMergeFlag), pu.mmvdMergeFlag);
              if (pu.mmvdMergeFlag)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu,
                                    GetBlockStatisticName(BlockStatistic::MMVDMergeIdx), pu.mmvdMergeIdx.val);
              }
              if (!cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8
                && !pu.mmvdMergeFlag && !cu.mmvdSkip
                )
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::AffineFlag), pu.cu->affine);
              }
              if (pu.cs->sps->getUseCiip() && !pu.cu->skip && !pu.cu->affine && !(pu.cu->lwidth() * pu.cu->lheight() < 64 || pu.cu->lwidth() >= MAX_CU_SIZE || pu.cu->lheight() >= MAX_CU_SIZE)
                && !pu.mmvdMergeFlag
                )
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::CiipFlag), pu.ciipFlag);
                if (pu.ciipFlag)
                {
                  if (cu.Y().valid())
                  {
                    DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu,
                                        GetBlockStatisticName(BlockStatistic::Luma_IntraMode),
                                        pu.intraDir[ChannelType::LUMA]);
                    DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu,
                                        GetBlockStatisticName(BlockStatistic::Chroma_IntraMode),
                                        pu.intraDir[ChannelType::CHROMA]);
                  }
                }
              }
            }
            else
            {
              if (!pu.cs->slice->isInterP())
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::InterDir), pu.interDir);
              }
              if (!cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width > 8 && cu.lumaSize().height > 8)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::AffineFlag), pu.cu->affine);
                if (cu.affine && !cu.firstPU->mergeFlag && cu.cs->sps->getUseAffineType())
                {
                  DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu,
                                      GetBlockStatisticName(BlockStatistic::AffineType),
                                      to_underlying(pu.cu->affineType));
                }
              }
            }
            if (pu.interDir != 2 /* PRED_L1 */)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MVPIdxL0), pu.mvpIdx[REF_PIC_LIST_0]);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::RefIdxL0), pu.refIdx[REF_PIC_LIST_0]);
            }
            if (pu.interDir != 1 /* PRED_L1 */)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MVPIdxL1), pu.mvpIdx[REF_PIC_LIST_1]);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::RefIdxL1), pu.refIdx[REF_PIC_LIST_1]);
            }
            if (!pu.cu->affine && !pu.cu->geoFlag)
            {
              if (pu.interDir != 2 /* PRED_L1 */)
              {
                Mv mv = pu.mv[REF_PIC_LIST_0];
                Mv mvd = pu.mvd[REF_PIC_LIST_0];
                mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
                mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
                mvd.hor = mvd.hor >= 0 ? (mvd.hor + nOffset) >> nShift : -((-mvd.hor + nOffset) >> nShift);
                mvd.ver = mvd.ver >= 0 ? (mvd.ver + nOffset) >> nShift : -((-mvd.ver + nOffset) >> nShift);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MVDL0), mvd.hor, mvd.ver);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MVL0), mv.hor, mv.ver);
              }
              if (pu.interDir != 1 /* PRED_L1 */)
              {
                Mv mv = pu.mv[REF_PIC_LIST_1];
                Mv mvd = pu.mvd[REF_PIC_LIST_1];
                mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
                mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
                mvd.hor = mvd.hor >= 0 ? (mvd.hor + nOffset) >> nShift : -((-mvd.hor + nOffset) >> nShift);
                mvd.ver = mvd.ver >= 0 ? (mvd.ver + nOffset) >> nShift : -((-mvd.ver + nOffset) >> nShift);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MVDL1), mvd.hor, mvd.ver);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MVL1), mv.hor, mv.ver);
              }
            }
            else
            {
              if (pu.interDir != 2 /* PRED_L1 */)
              {
                Mv mv[3];
                const CMotionBuf &mb = pu.getMotionBuf();
                mv[0] = mb.at(0, 0).mv[REF_PIC_LIST_0];
                mv[1] = mb.at(mb.width - 1, 0).mv[REF_PIC_LIST_0];
                mv[2] = mb.at(0, mb.height - 1).mv[REF_PIC_LIST_0];
                // motion vectors should use low precision or they will appear to large
                mv[0].hor = mv[0].hor >= 0 ? (mv[0].hor + nOffset) >> nShift : -((-mv[0].hor + nOffset) >> nShift);
                mv[0].ver = mv[0].ver >= 0 ? (mv[0].ver + nOffset) >> nShift : -((-mv[0].ver + nOffset) >> nShift);
                mv[1].hor = mv[1].hor >= 0 ? (mv[1].hor + nOffset) >> nShift : -((-mv[1].hor + nOffset) >> nShift);
                mv[1].ver = mv[1].ver >= 0 ? (mv[1].ver + nOffset) >> nShift : -((-mv[1].ver + nOffset) >> nShift);
                mv[2].hor = mv[2].hor >= 0 ? (mv[2].hor + nOffset) >> nShift : -((-mv[2].hor + nOffset) >> nShift);
                mv[2].ver = mv[2].ver >= 0 ? (mv[2].ver + nOffset) >> nShift : -((-mv[2].ver + nOffset) >> nShift);
                DTRACE_BLOCK_AFFINETF(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::AffineMVL0), mv[0].hor, mv[0].ver, mv[1].hor, mv[1].ver, mv[2].hor, mv[2].ver);
              }
              if (pu.interDir != 1 /* PRED_L1 */)
              {
                Mv mv[3];
                const CMotionBuf &mb = pu.getMotionBuf();
                mv[0] = mb.at(0, 0).mv[REF_PIC_LIST_1];
                mv[1] = mb.at(mb.width - 1, 0).mv[REF_PIC_LIST_1];
                mv[2] = mb.at(0, mb.height - 1).mv[REF_PIC_LIST_1];
                // motion vectors should use low precision or they will appear to large
                mv[0].hor = mv[0].hor >= 0 ? (mv[0].hor + nOffset) >> nShift : -((-mv[0].hor + nOffset) >> nShift);
                mv[0].ver = mv[0].ver >= 0 ? (mv[0].ver + nOffset) >> nShift : -((-mv[0].ver + nOffset) >> nShift);
                mv[1].hor = mv[1].hor >= 0 ? (mv[1].hor + nOffset) >> nShift : -((-mv[1].hor + nOffset) >> nShift);
                mv[1].ver = mv[1].ver >= 0 ? (mv[1].ver + nOffset) >> nShift : -((-mv[1].ver + nOffset) >> nShift);
                mv[2].hor = mv[2].hor >= 0 ? (mv[2].hor + nOffset) >> nShift : -((-mv[2].hor + nOffset) >> nShift);
                mv[2].ver = mv[2].ver >= 0 ? (mv[2].ver + nOffset) >> nShift : -((-mv[2].ver + nOffset) >> nShift);
                DTRACE_BLOCK_AFFINETF(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::AffineMVL1), mv[0].hor, mv[0].ver, mv[1].hor, mv[1].ver, mv[2].hor, mv[2].ver);
              }
            }
            if (cu.cs->sps->getAMVREnabledFlag() && CU::hasSubCUNonZeroMVd(cu))
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::IMVMode), cu.imv);
            }
            if (CU::isBcwIdxCoded(cu))
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                  GetBlockStatisticName(BlockStatistic::BCWIndex), cu.bcwIdx);
            }
            break;
          }
          default:
          {
            THROW("Invalid prediction mode");
            break;
          }
        }
      } // end pu
      if (CU::isInter(cu))
      {
        const PredictionUnit &pu = *cu.firstPU;
        if ( !pu.mergeFlag )
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::RootCbf), cu.rootCbf);
        }
      }
      if (cu.rootCbf || CU::isIntra(cu))
      {
        for (const TransformUnit &tu : CU::traverseTUs(cu))
        {
          if (tu.Y().valid())
          {
            int cbf = tu.cbf[COMPONENT_Y] & (1 << tu.depth);
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu, GetBlockStatisticName(BlockStatistic::Cbf_Y), cbf);
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu,
                                GetBlockStatisticName(BlockStatistic::MTSIdx_Y), to_underlying(tu.mtsIdx[COMPONENT_Y]));
          }
          if (!(!isChromaEnabled(cu.chromaFormat) || (cu.isSepTree() && isLuma(cu.chType))))
          {
            int cbf = tu.cbf[COMPONENT_Cb] & (1 << tu.depth);
            DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu, GetBlockStatisticName(BlockStatistic::Cbf_Cb), cbf);
            cbf = tu.cbf[COMPONENT_Cr] & (1 << tu.depth);
            DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu, GetBlockStatisticName(BlockStatistic::Cbf_Cr), cbf);
            DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu,
                                       GetBlockStatisticName(BlockStatistic::MTSIdx_Cb),
                                       to_underlying(tu.mtsIdx[COMPONENT_Cb]));
            DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu,
                                       GetBlockStatisticName(BlockStatistic::MTSIdx_Cr),
                                       to_underlying(tu.mtsIdx[COMPONENT_Cr]));
          }
        }
      }
    }
  }
}

// ===========================================================================
// VoidPlayer Compact Stats: one line per CU, environment variable controlled
// ===========================================================================

static void writeAllCodedDataCompact(const CodingStructure& cs, const UnitArea& ctuArea)
{
  const int nShift = MV_FRACTIONAL_BITS_DIFF;
  const int nOffset = 1 << (nShift - 1);
  const int maxNumChannelType = isChromaEnabled(cs.pcv->chrFormat) && CS::isDualITree(cs) ? 2 : 1;

  for (int ch = 0; ch < maxNumChannelType; ch++)
  {
    const ChannelType chType = ChannelType(ch);

    for (const CodingUnit &cu : cs.traverseCUs(CS::getArea(cs, ctuArea, chType), chType))
    {
      if (!isLuma(chType))
        continue; // skip chroma CU in compact mode for now

      const int poc = cs.picture->poc;
      const int cx  = cu.lx();
      const int cy  = cu.ly();
      const int cw  = cu.lwidth();
      const int ch_ = cu.lheight();

      // Base CU info: poc x y w h depth qp pred
      // Cast uint8_t/bool/enum to int to avoid ostringstream treating them as chars
      std::ostringstream line;
      line << "CU " << poc << " " << cx << " " << cy << " " << cw << " " << ch_ << " "
           << int(cu.depth) << " " << int(cu.qp) << " " << int(cu.predMode);

      switch (cu.predMode)
      {
      case MODE_INTRA:
      {
        // intra: intra_mode mip isp
        for (const PredictionUnit &pu : CU::traversePUs(cu))
        {
          if (pu.Y().valid())
          {
            line << " " << PU::getFinalIntraMode(pu, ChannelType::LUMA);
            line << " " << int(cu.mipFlag);
            line << " " << to_uint(cu.ispMode);
            break; // one PU is enough for compact mode
          }
        }
        break;
      }
      case MODE_INTER:
      {
        // inter: skip merge interDir mvL0x mvL0y mvL1x mvL1y refL0 refL1
        line << " " << int(cu.skip);

        for (const PredictionUnit &pu : CU::traversePUs(cu))
        {
          line << " " << int(pu.mergeFlag);
          line << " " << int(pu.interDir);

          // L0 MV
          if (pu.interDir != 2 /* not PRED_L1 only */)
          {
            Mv mv = pu.mv[REF_PIC_LIST_0];
            mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
            mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
            line << " " << mv.hor << " " << mv.ver;
          }
          else
          {
            line << " 0 0";
          }

          // L1 MV
          if (pu.interDir != 1 /* not PRED_L0 only */)
          {
            Mv mv = pu.mv[REF_PIC_LIST_1];
            mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
            mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
            line << " " << mv.hor << " " << mv.ver;
          }
          else
          {
            line << " 0 0";
          }

          line << " " << int(pu.refIdx[REF_PIC_LIST_0]);
          line << " " << int(pu.refIdx[REF_PIC_LIST_1]);
          break; // one PU is enough for compact mode
        }
        break;
      }
      default:
        break;
      }

      line << "\n";
      const std::string str = line.str();
      g_trace_ctx->dtrace<false>(D_BLOCK_STATISTICS_CODED, "%s", str.c_str());
    }
  }
}

#endif
