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
#include <algorithm>
#include <queue>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#if defined(VOIDPLAYER_VBS4_ZSTD)
#include "zstd.h"
#endif

// ---------------------------------------------------------------------------
// Stats output mode selection via environment variables:
//   VTM_BINARY_STATS=<filepath>  -> binary VBS4 output
//   VTM_COMPACT_STATS=1          -> text compact (one line per CU)
//   (neither)                    -> original verbose VTM format
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

enum class BinaryStatsFormat
{
  Vbs2,
  Vbs3,
  Vbs4,
};

static BinaryStatsFormat binaryStatsFormat()
{
  static BinaryStatsFormat s_format = []() {
    const char* env = std::getenv("VTM_BINARY_STATS_FORMAT");
    if (!env || env[0] == '\0')
    {
      return BinaryStatsFormat::Vbs4;
    }
    const std::string value(env);
    if (value == "VBS4" || value == "vbs4" || value == "4")
    {
      return BinaryStatsFormat::Vbs4;
    }
    if (value == "VBS3" || value == "vbs3" || value == "3")
    {
      fprintf(stderr, "VTM_BINARY_STATS_FORMAT: VBS3 is retired, writing VBS4\n");
      return BinaryStatsFormat::Vbs4;
    }
    if (value == "VBS2" || value == "vbs2" || value == "2")
    {
      fprintf(stderr, "VTM_BINARY_STATS_FORMAT: VBS2 is retired, writing VBS4\n");
      return BinaryStatsFormat::Vbs4;
    }
    fprintf(stderr, "VTM_BINARY_STATS_FORMAT: unsupported value '%s', writing VBS4\n", env);
    return BinaryStatsFormat::Vbs4;
  }();
  return s_format;
}

static const char* binaryStatsFormatName()
{
  return "VBS4";
}

static int64_t binaryTell(FILE* file)
{
#if defined(_WIN32)
  return _ftelli64(file);
#else
  return static_cast<int64_t>(ftell(file));
#endif
}

static bool binarySeek(FILE* file, uint64_t offset)
{
#if defined(_WIN32)
  return _fseeki64(file, static_cast<int64_t>(offset), SEEK_SET) == 0;
#else
  return fseek(file, static_cast<long>(offset), SEEK_SET) == 0;
#endif
}

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

struct Vbs3Header {
  char     magic[4];       // "VBS3"
  uint16_t version_major;
  uint16_t version_minor;
  uint16_t header_size;
  uint16_t section_entry_size;
  uint32_t flags;
  uint32_t width;
  uint32_t height;
  uint32_t frame_count;
  uint32_t section_count;
  uint64_t section_table_offset;
  uint64_t file_size;
  uint64_t content_revision;
  uint64_t reserved;
};
static_assert(sizeof(Vbs3Header) == 64, "Vbs3Header must be 64 bytes");

struct Vbs3SectionEntry {
  char     type[4];
  uint32_t flags;
  uint64_t offset;
  uint64_t size;
  uint32_t entry_size;
  uint32_t entry_count;
  uint64_t checksum;
  uint64_t reserved;
};
static_assert(sizeof(Vbs3SectionEntry) == 48, "Vbs3SectionEntry must be 48 bytes");

struct Vbs3FrameSummary {
  int32_t  poc;
  uint32_t coded_order;
  uint32_t vcl_nalu_index;
  uint32_t flags;
  uint8_t  temporal_id;
  uint8_t  slice_type;
  uint8_t  nal_unit_type;
  uint8_t  avg_qp;
  uint8_t  num_ref_l0;
  uint8_t  num_ref_l1;
  uint8_t  qp_min;
  uint8_t  qp_max;
  int32_t  ref_pocs_l0[15];
  int32_t  ref_pocs_l1[15];
  uint32_t num_cus;
  uint32_t cu_index_entry;
  uint32_t reserved[2];
};
static_assert(sizeof(Vbs3FrameSummary) == 160, "Vbs3FrameSummary must be 160 bytes");

struct Vbs3CuIndexEntry {
  uint64_t offset;         // relative to CUBL payload
  uint64_t byte_size;
  uint32_t cu_count;
  uint32_t flags;
};
static_assert(sizeof(Vbs3CuIndexEntry) == 24, "Vbs3CuIndexEntry must be 24 bytes");

struct Vbs4Header {
  char     magic[4];       // "VBS4"
  uint16_t version_major;
  uint16_t version_minor;
  uint16_t header_size;
  uint16_t section_entry_size;
  uint16_t codec;
  uint16_t profile;
  uint32_t flags;
  uint32_t width;
  uint32_t height;
  uint32_t frame_count;
  uint32_t block_count;
  uint32_t section_count;
  uint32_t reserved0;
  uint64_t section_table_offset;
  uint64_t file_size;
  uint64_t content_revision;
  uint64_t reserved1;
  uint32_t reserved2;
};
static_assert(sizeof(Vbs4Header) == 80, "Vbs4Header must be 80 bytes");

struct Vbs4SectionEntry {
  char     type[4];
  uint32_t flags;
  uint64_t offset;
  uint64_t size;
  uint32_t entry_size;
  uint32_t entry_count;
  uint64_t checksum;
  uint64_t reserved0;
  uint64_t reserved1;
};
static_assert(sizeof(Vbs4SectionEntry) == 56, "Vbs4SectionEntry must be 56 bytes");

using Vbs4FrameSummary = Vbs3FrameSummary;
static_assert(sizeof(Vbs4FrameSummary) == 160, "Vbs4FrameSummary must be 160 bytes");

struct Vbs4FrameIndexEntry {
  uint32_t block_index;
  uint32_t local_frame;
  uint32_t first_record;
  uint32_t record_count;
  uint32_t flags;
  uint32_t reserved;
};
static_assert(sizeof(Vbs4FrameIndexEntry) == 24, "Vbs4FrameIndexEntry must be 24 bytes");

struct Vbs4BlockIndexEntry {
  uint32_t first_frame;
  uint32_t frame_count;
  uint32_t first_record;
  uint32_t record_count;
  uint64_t payload_offset;
  uint64_t payload_size;
  uint64_t decoded_size;
  uint16_t codec_profile;
  uint16_t compression;
  uint32_t flags;
  uint64_t checksum;
  uint64_t reserved;
};
static_assert(sizeof(Vbs4BlockIndexEntry) == 64, "Vbs4BlockIndexEntry must be 64 bytes");

struct Vbs4DecodedBlockHeader {
  char     magic[4];       // "BLK4"
  uint16_t header_size;
  uint16_t stream_entry_size;
  uint16_t codec_profile;
  uint16_t stream_count;
  uint32_t frame_count;
  uint32_t record_count;
  uint32_t flags;
  uint64_t reserved;
};
static_assert(sizeof(Vbs4DecodedBlockHeader) == 32, "Vbs4DecodedBlockHeader must be 32 bytes");

struct Vbs4StreamEntry {
  uint16_t stream_id;
  uint16_t encoding;
  uint32_t offset;
  uint32_t size;
  uint32_t value_count;
  uint32_t flags;
};
static_assert(sizeof(Vbs4StreamEntry) == 20, "Vbs4StreamEntry must be 20 bytes");
#pragma pack(pop)

static constexpr uint16_t VBS4_CODEC_VVC = 3;
static constexpr uint16_t VBS4_PROFILE_VVCCU1 = 1;
static constexpr uint16_t VBS4_COMPRESSION_NONE = 0;
static constexpr uint16_t VBS4_COMPRESSION_ZSTD = 1;
static constexpr uint16_t VBS4_ENC_RAW = 0;
static constexpr uint16_t VBS4_ENC_BITSET = 1;
static constexpr uint16_t VBS4_ENC_ULEB128 = 2;
static constexpr uint16_t VBS4_ENC_SLEB128_ZIGZAG = 3;
static constexpr uint16_t VBS4_ENC_FRAME_PREFIX_U32 = 7;

static constexpr uint16_t VBS4_STREAM_FRAME_PREFIX = 1;
static constexpr uint16_t VBS4_HEVC_X = 2;
static constexpr uint16_t VBS4_HEVC_Y = 3;
static constexpr uint16_t VBS4_HEVC_LOG2_W = 4;
static constexpr uint16_t VBS4_HEVC_LOG2_H = 5;
static constexpr uint16_t VBS4_HEVC_DEPTH = 6;
static constexpr uint16_t VBS4_HEVC_PRED_MODE = 7;
static constexpr uint16_t VBS4_HEVC_QP_DELTA = 8;
static constexpr uint16_t VBS4_HEVC_INTRA_MODE = 9;
static constexpr uint16_t VBS4_HEVC_MIP_FLAG = 10;
static constexpr uint16_t VBS4_HEVC_ISP_MODE = 11;
static constexpr uint16_t VBS4_HEVC_SKIP_FLAG = 12;
static constexpr uint16_t VBS4_HEVC_MERGE_FLAG = 13;
static constexpr uint16_t VBS4_HEVC_INTER_DIR = 14;
static constexpr uint16_t VBS4_HEVC_MV_L0_X = 15;
static constexpr uint16_t VBS4_HEVC_MV_L0_Y = 16;
static constexpr uint16_t VBS4_HEVC_MV_L1_X = 17;
static constexpr uint16_t VBS4_HEVC_MV_L1_Y = 18;
static constexpr uint16_t VBS4_HEVC_REF_L0 = 19;
static constexpr uint16_t VBS4_HEVC_REF_L1 = 20;

struct Vbs4CuRecord {
  uint16_t x = 0;
  uint16_t y = 0;
  uint8_t  w = 0;
  uint8_t  h = 0;
  uint8_t  depth = 0;
  uint8_t  qp = 0;
  uint8_t  pred_mode = 0;
  uint8_t  intra_mode = 0;
  uint8_t  mip_flag = 0;
  uint8_t  isp_mode = 0;
  uint8_t  skip = 0;
  uint8_t  merge_flag = 0;
  uint8_t  inter_dir = 0;
  int16_t  mv_l0_x = 0;
  int16_t  mv_l0_y = 0;
  int16_t  mv_l1_x = 0;
  int16_t  mv_l1_y = 0;
  int8_t   ref_l0 = -1;
  int8_t   ref_l1 = -1;
};

struct Vbs4FrameData {
  Vbs4FrameSummary summary = {};
  std::vector<Vbs4CuRecord> records;
};

struct Vbs4StreamData {
  uint16_t id = 0;
  uint16_t encoding = 0;
  uint32_t valueCount = 0;
  std::vector<uint8_t> bytes;
};

static void vbs4SetFourcc(char dst[4], const char src[4])
{
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
  dst[3] = src[3];
}

static void vbs4AppendBytes(std::vector<uint8_t>& out, const void* data, size_t size)
{
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  out.insert(out.end(), bytes, bytes + size);
}

static void vbs4AppendU8(std::vector<uint8_t>& out, uint8_t value)
{
  out.push_back(value);
}

static void vbs4AppendU32(std::vector<uint8_t>& out, uint32_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xff));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
  out.push_back(static_cast<uint8_t>((value >> 16) & 0xff));
  out.push_back(static_cast<uint8_t>((value >> 24) & 0xff));
}

static void vbs4AppendUleb(std::vector<uint8_t>& out, uint32_t value)
{
  do
  {
    uint8_t byte = static_cast<uint8_t>(value & 0x7f);
    value >>= 7;
    if (value)
    {
      byte |= 0x80;
    }
    out.push_back(byte);
  } while (value);
}

static void vbs4AppendSlebZigzag(std::vector<uint8_t>& out, int32_t value)
{
  const uint32_t zigzag = (static_cast<uint32_t>(value) << 1) ^ static_cast<uint32_t>(value >> 31);
  vbs4AppendUleb(out, zigzag);
}

static void vbs4AppendBit(std::vector<uint8_t>& out, uint32_t index, bool value)
{
  const size_t byteIndex = index >> 3;
  if (byteIndex >= out.size())
  {
    out.resize(byteIndex + 1, 0);
  }
  if (value)
  {
    out[byteIndex] |= static_cast<uint8_t>(1u << (index & 7));
  }
}

static uint8_t vbs4Log2Size(uint8_t value)
{
  uint8_t result = 0;
  while (value > 1)
  {
    value >>= 1;
    result++;
  }
  return result;
}

static Vbs4SectionEntry vbs4SectionEntry(const char type[4],
                                         uint64_t offset,
                                         uint64_t size,
                                         uint32_t entrySize,
                                         uint32_t entryCount)
{
  Vbs4SectionEntry entry = {};
  vbs4SetFourcc(entry.type, type);
  entry.offset = offset;
  entry.size = size;
  entry.entry_size = entrySize;
  entry.entry_count = entryCount;
  return entry;
}

static bool vbs4NoCompressionRequested()
{
  const char* env = std::getenv("VTM_BINARY_STATS_NO_COMPRESSION");
  return env && env[0] != '\0' && std::string(env) != "0";
}

struct BinaryStatsState {
  FILE*   file = nullptr;
  BinaryStatsFormat format = BinaryStatsFormat::Vbs4;
  int     currentPoc = -1;
  uint32_t frameCuCount = 0;
  uint32_t qpSum = 0;
  uint8_t  qpMin = 0;
  uint8_t  qpMax = 0;
  uint32_t numFrames = 0;
  uint32_t seqWidth = 0;
  uint32_t seqHeight = 0;
  Vbs4FrameSummary currentSummary = {};
  std::vector<Vbs4CuRecord> currentRecords;
  std::vector<Vbs4FrameData> vbs4Frames;

  bool open() {
    if (file) return true;
    const char* path = binaryStatsPath();
    if (!path) return false;
    format = binaryStatsFormat();
    file = fopen(path, "w+b");
    if (!file) { fprintf(stderr, "VTM_BINARY_STATS: cannot open %s\n", path); return false; }

    Vbs4Header hdr = {};
    vbs4SetFourcc(hdr.magic, "VBS4");
    hdr.version_major = 4;
    hdr.header_size = sizeof(Vbs4Header);
    hdr.section_entry_size = sizeof(Vbs4SectionEntry);
    hdr.codec = VBS4_CODEC_VVC;
    hdr.profile = VBS4_PROFILE_VVCCU1;
    fwrite(&hdr, sizeof(hdr), 1, file);
    return true;
  }

  void setDimensions(uint32_t w, uint32_t h) {
    seqWidth = w; seqHeight = h;
  }

  void beginFrame(int poc, const Slice* slice) {
    if (!file) return;
    if (currentPoc == poc) return;  // same frame, different CTU
    if (currentPoc >= 0) endFrame();
    currentPoc = poc;
    frameCuCount = 0;
    qpSum = 0;
    qpMin = 255;
    qpMax = 0;
    currentRecords.clear();

    currentSummary = {};
    currentSummary.poc = poc;
    currentSummary.coded_order = numFrames;
    currentSummary.vcl_nalu_index = 0xFFFFFFFFu;
    currentSummary.temporal_id = slice ? slice->getTLayer() : 0;
    currentSummary.slice_type = slice ? slice->getSliceType() : 0;
    currentSummary.nal_unit_type = slice ? slice->getNalUnitType() : 0;
    currentSummary.cu_index_entry = numFrames;

    if (slice) {
      const int nL0 = slice->getNumRefIdx(REF_PIC_LIST_0);
      const int nL1 = slice->getNumRefIdx(REF_PIC_LIST_1);
      currentSummary.num_ref_l0 = static_cast<uint8_t>(std::min(nL0, 15));
      currentSummary.num_ref_l1 = static_cast<uint8_t>(std::min(nL1, 15));
      for (int i = 0; i < 15; i++) {
        currentSummary.ref_pocs_l0[i] = (i < nL0) ? slice->getRefPOC(REF_PIC_LIST_0, i) : -1;
        currentSummary.ref_pocs_l1[i] = (i < nL1) ? slice->getRefPOC(REF_PIC_LIST_1, i) : -1;
      }
    } else {
      for (int i = 0; i < 15; i++) {
        currentSummary.ref_pocs_l0[i] = -1;
        currentSummary.ref_pocs_l1[i] = -1;
      }
    }
  }

  void recordCu(uint8_t qp) {
    frameCuCount++;
    qpSum += qp;
    qpMin = std::min(qpMin, qp);
    qpMax = std::max(qpMax, qp);
  }

  void recordVbs4Cu(const Vbs4CuRecord& record) {
    currentRecords.push_back(record);
    recordCu(record.qp);
  }

  void endFrame() {
    if (!file || currentPoc < 0) return;
    currentSummary.avg_qp = frameCuCount > 0 ? static_cast<uint8_t>(qpSum / frameCuCount) : 0;
    currentSummary.qp_min = frameCuCount > 0 ? qpMin : 0;
    currentSummary.qp_max = frameCuCount > 0 ? qpMax : 0;
    currentSummary.num_cus = frameCuCount;

    Vbs4FrameData frame;
    frame.summary = currentSummary;
    frame.records.swap(currentRecords);
    vbs4Frames.push_back(std::move(frame));

    numFrames++;
    currentPoc = -1;
  }

  static void pushStream(std::vector<Vbs4StreamData>& streams,
                         uint16_t id,
                         uint16_t encoding,
                         uint32_t valueCount,
                         std::vector<uint8_t>& bytes)
  {
    Vbs4StreamData stream;
    stream.id = id;
    stream.encoding = encoding;
    stream.valueCount = valueCount;
    stream.bytes.swap(bytes);
    streams.push_back(std::move(stream));
  }

  std::vector<uint8_t> buildDecodedBlock(size_t firstFrame,
                                         size_t frameCount,
                                         uint32_t& outRecordCount) const
  {
    std::vector<Vbs4StreamData> streams;
    std::vector<uint8_t> framePrefix;
    std::vector<uint8_t> x;
    std::vector<uint8_t> y;
    std::vector<uint8_t> log2w;
    std::vector<uint8_t> log2h;
    std::vector<uint8_t> depth;
    std::vector<uint8_t> predMode;
    std::vector<uint8_t> qpDelta;
    std::vector<uint8_t> intraMode;
    std::vector<uint8_t> mipFlag;
    std::vector<uint8_t> ispMode;
    std::vector<uint8_t> skipFlag;
    std::vector<uint8_t> mergeFlag;
    std::vector<uint8_t> interDir;
    std::vector<uint8_t> mvL0x;
    std::vector<uint8_t> mvL0y;
    std::vector<uint8_t> mvL1x;
    std::vector<uint8_t> mvL1y;
    std::vector<uint8_t> refL0;
    std::vector<uint8_t> refL1;

    uint32_t prefix = 0;
    for (size_t i = 0; i < frameCount; ++i)
    {
      vbs4AppendU32(framePrefix, prefix);
      prefix += static_cast<uint32_t>(vbs4Frames[firstFrame + i].records.size());
    }
    vbs4AppendU32(framePrefix, prefix);
    outRecordCount = prefix;

    int32_t prevQp = 0;
    uint32_t recordIndex = 0;
    for (size_t frameIdx = firstFrame; frameIdx < firstFrame + frameCount; ++frameIdx)
    {
      for (const Vbs4CuRecord& record : vbs4Frames[frameIdx].records)
      {
        vbs4AppendUleb(x, record.x);
        vbs4AppendUleb(y, record.y);
        vbs4AppendU8(log2w, vbs4Log2Size(record.w));
        vbs4AppendU8(log2h, vbs4Log2Size(record.h));
        vbs4AppendU8(depth, record.depth);
        vbs4AppendU8(predMode, record.pred_mode);
        vbs4AppendSlebZigzag(qpDelta, static_cast<int32_t>(record.qp) - prevQp);
        prevQp = record.qp;
        vbs4AppendU8(intraMode, record.intra_mode);
        vbs4AppendBit(mipFlag, recordIndex, record.mip_flag != 0);
        vbs4AppendU8(ispMode, record.isp_mode);
        vbs4AppendBit(skipFlag, recordIndex, record.skip != 0);
        vbs4AppendBit(mergeFlag, recordIndex, record.merge_flag != 0);
        vbs4AppendU8(interDir, record.inter_dir);
        vbs4AppendSlebZigzag(mvL0x, record.mv_l0_x);
        vbs4AppendSlebZigzag(mvL0y, record.mv_l0_y);
        vbs4AppendSlebZigzag(mvL1x, record.mv_l1_x);
        vbs4AppendSlebZigzag(mvL1y, record.mv_l1_y);
        vbs4AppendU8(refL0, static_cast<uint8_t>(record.ref_l0));
        vbs4AppendU8(refL1, static_cast<uint8_t>(record.ref_l1));
        recordIndex++;
      }
    }

    pushStream(streams, VBS4_STREAM_FRAME_PREFIX, VBS4_ENC_FRAME_PREFIX_U32, static_cast<uint32_t>(frameCount + 1), framePrefix);
    pushStream(streams, VBS4_HEVC_X, VBS4_ENC_ULEB128, outRecordCount, x);
    pushStream(streams, VBS4_HEVC_Y, VBS4_ENC_ULEB128, outRecordCount, y);
    pushStream(streams, VBS4_HEVC_LOG2_W, VBS4_ENC_RAW, outRecordCount, log2w);
    pushStream(streams, VBS4_HEVC_LOG2_H, VBS4_ENC_RAW, outRecordCount, log2h);
    pushStream(streams, VBS4_HEVC_DEPTH, VBS4_ENC_RAW, outRecordCount, depth);
    pushStream(streams, VBS4_HEVC_PRED_MODE, VBS4_ENC_RAW, outRecordCount, predMode);
    pushStream(streams, VBS4_HEVC_QP_DELTA, VBS4_ENC_SLEB128_ZIGZAG, outRecordCount, qpDelta);
    pushStream(streams, VBS4_HEVC_INTRA_MODE, VBS4_ENC_RAW, outRecordCount, intraMode);
    pushStream(streams, VBS4_HEVC_MIP_FLAG, VBS4_ENC_BITSET, outRecordCount, mipFlag);
    pushStream(streams, VBS4_HEVC_ISP_MODE, VBS4_ENC_RAW, outRecordCount, ispMode);
    pushStream(streams, VBS4_HEVC_SKIP_FLAG, VBS4_ENC_BITSET, outRecordCount, skipFlag);
    pushStream(streams, VBS4_HEVC_MERGE_FLAG, VBS4_ENC_BITSET, outRecordCount, mergeFlag);
    pushStream(streams, VBS4_HEVC_INTER_DIR, VBS4_ENC_RAW, outRecordCount, interDir);
    pushStream(streams, VBS4_HEVC_MV_L0_X, VBS4_ENC_SLEB128_ZIGZAG, outRecordCount, mvL0x);
    pushStream(streams, VBS4_HEVC_MV_L0_Y, VBS4_ENC_SLEB128_ZIGZAG, outRecordCount, mvL0y);
    pushStream(streams, VBS4_HEVC_MV_L1_X, VBS4_ENC_SLEB128_ZIGZAG, outRecordCount, mvL1x);
    pushStream(streams, VBS4_HEVC_MV_L1_Y, VBS4_ENC_SLEB128_ZIGZAG, outRecordCount, mvL1y);
    pushStream(streams, VBS4_HEVC_REF_L0, VBS4_ENC_RAW, outRecordCount, refL0);
    pushStream(streams, VBS4_HEVC_REF_L1, VBS4_ENC_RAW, outRecordCount, refL1);

    Vbs4DecodedBlockHeader header = {};
    vbs4SetFourcc(header.magic, "BLK4");
    header.header_size = sizeof(Vbs4DecodedBlockHeader);
    header.stream_entry_size = sizeof(Vbs4StreamEntry);
    header.codec_profile = VBS4_PROFILE_VVCCU1;
    header.stream_count = static_cast<uint16_t>(streams.size());
    header.frame_count = static_cast<uint32_t>(frameCount);
    header.record_count = outRecordCount;

    std::vector<uint8_t> out;
    vbs4AppendBytes(out, &header, sizeof(header));
    const size_t streamTableOffset = out.size();
    out.resize(out.size() + streams.size() * sizeof(Vbs4StreamEntry), 0);

    uint32_t payloadOffset = static_cast<uint32_t>(out.size());
    for (size_t i = 0; i < streams.size(); ++i)
    {
      Vbs4StreamEntry entry = {};
      entry.stream_id = streams[i].id;
      entry.encoding = streams[i].encoding;
      entry.offset = payloadOffset;
      entry.size = static_cast<uint32_t>(streams[i].bytes.size());
      entry.value_count = streams[i].valueCount;
      std::memcpy(out.data() + streamTableOffset + i * sizeof(Vbs4StreamEntry), &entry, sizeof(entry));
      out.insert(out.end(), streams[i].bytes.begin(), streams[i].bytes.end());
      payloadOffset += entry.size;
    }
    return out;
  }

  void finalizeVbs4() {
    constexpr uint64_t targetDecodedBytes = 16ull * 1024ull * 1024ull;
    constexpr uint32_t maxFramesPerBlock = 4096;
    constexpr uint64_t estimatedBytesPerRecord = 24;

    binarySeek(file, sizeof(Vbs4Header));

    std::vector<Vbs4FrameIndexEntry> frameIndex;
    std::vector<Vbs4BlockIndexEntry> blockIndex;
    frameIndex.reserve(vbs4Frames.size());

    uint64_t cpayBytes = 0;
    uint32_t globalFirstRecord = 0;
    size_t firstFrame = 0;
    while (firstFrame < vbs4Frames.size())
    {
      size_t frameCount = 0;
      uint64_t estimate = sizeof(Vbs4DecodedBlockHeader) + 20ull * sizeof(Vbs4StreamEntry);
      do
      {
        estimate += vbs4Frames[firstFrame + frameCount].records.size() * estimatedBytesPerRecord;
        frameCount++;
      } while (firstFrame + frameCount < vbs4Frames.size() &&
               frameCount < maxFramesPerBlock &&
               estimate < targetDecodedBytes);

      uint32_t recordCount = 0;
      std::vector<uint8_t> decoded = buildDecodedBlock(firstFrame, frameCount, recordCount);
      const std::vector<uint8_t>* payload = &decoded;
      std::vector<uint8_t> compressed;
      bool compressedUsed = false;
#if defined(VOIDPLAYER_VBS4_ZSTD)
      if (!vbs4NoCompressionRequested() && !decoded.empty())
      {
        const size_t bound = ZSTD_compressBound(decoded.size());
        compressed.resize(bound);
        const size_t compressedSize = ZSTD_compress(compressed.data(), compressed.size(), decoded.data(), decoded.size(), 3);
        if (!ZSTD_isError(compressedSize) && compressedSize < decoded.size())
        {
          compressed.resize(compressedSize);
          payload = &compressed;
          compressedUsed = true;
        }
      }
#endif

      const uint32_t blockIdx = static_cast<uint32_t>(blockIndex.size());
      uint32_t localFirstRecord = 0;
      for (size_t i = 0; i < frameCount; ++i)
      {
        const uint32_t frameRecords = static_cast<uint32_t>(vbs4Frames[firstFrame + i].records.size());
        Vbs4FrameIndexEntry fidx = {};
        fidx.block_index = blockIdx;
        fidx.local_frame = static_cast<uint32_t>(i);
        fidx.first_record = globalFirstRecord + localFirstRecord;
        fidx.record_count = frameRecords;
        frameIndex.push_back(fidx);
        localFirstRecord += frameRecords;
      }

      Vbs4BlockIndexEntry bidx = {};
      bidx.first_frame = static_cast<uint32_t>(firstFrame);
      bidx.frame_count = static_cast<uint32_t>(frameCount);
      bidx.first_record = globalFirstRecord;
      bidx.record_count = recordCount;
      bidx.payload_offset = cpayBytes;
      bidx.payload_size = payload->size();
      bidx.decoded_size = decoded.size();
      bidx.codec_profile = VBS4_PROFILE_VVCCU1;
      bidx.compression = compressedUsed ? VBS4_COMPRESSION_ZSTD : VBS4_COMPRESSION_NONE;
      blockIndex.push_back(bidx);

      if (!payload->empty())
      {
        fwrite(payload->data(), 1, payload->size(), file);
      }
      cpayBytes += payload->size();
      globalFirstRecord += recordCount;
      firstFrame += frameCount;
    }

    const uint64_t fsumOffset = static_cast<uint64_t>(binaryTell(file));
    for (const auto& frame : vbs4Frames)
    {
      fwrite(&frame.summary, sizeof(frame.summary), 1, file);
    }

    const uint64_t fidxOffset = static_cast<uint64_t>(binaryTell(file));
    for (const auto& entry : frameIndex)
    {
      fwrite(&entry, sizeof(entry), 1, file);
    }

    const uint64_t bidxOffset = static_cast<uint64_t>(binaryTell(file));
    for (const auto& entry : blockIndex)
    {
      fwrite(&entry, sizeof(entry), 1, file);
    }

    const uint64_t sectionTableOffset = static_cast<uint64_t>(binaryTell(file));
    std::vector<Vbs4SectionEntry> sections;
    sections.push_back(vbs4SectionEntry("FSUM", fsumOffset, vbs4Frames.size() * sizeof(Vbs4FrameSummary), sizeof(Vbs4FrameSummary), static_cast<uint32_t>(vbs4Frames.size())));
    sections.push_back(vbs4SectionEntry("FIDX", fidxOffset, frameIndex.size() * sizeof(Vbs4FrameIndexEntry), sizeof(Vbs4FrameIndexEntry), static_cast<uint32_t>(frameIndex.size())));
    sections.push_back(vbs4SectionEntry("BIDX", bidxOffset, blockIndex.size() * sizeof(Vbs4BlockIndexEntry), sizeof(Vbs4BlockIndexEntry), static_cast<uint32_t>(blockIndex.size())));
    sections.push_back(vbs4SectionEntry("CPAY", sizeof(Vbs4Header), cpayBytes, 0, static_cast<uint32_t>(blockIndex.size())));
    for (const auto& section : sections)
    {
      fwrite(&section, sizeof(section), 1, file);
    }

    const uint64_t fileSize = static_cast<uint64_t>(binaryTell(file));
    binarySeek(file, 0);
    Vbs4Header hdr = {};
    vbs4SetFourcc(hdr.magic, "VBS4");
    hdr.version_major = 4;
    hdr.version_minor = 0;
    hdr.header_size = sizeof(Vbs4Header);
    hdr.section_entry_size = sizeof(Vbs4SectionEntry);
    hdr.codec = VBS4_CODEC_VVC;
    hdr.profile = VBS4_PROFILE_VVCCU1;
    hdr.width = seqWidth;
    hdr.height = seqHeight;
    hdr.frame_count = static_cast<uint32_t>(vbs4Frames.size());
    hdr.block_count = static_cast<uint32_t>(blockIndex.size());
    hdr.section_count = static_cast<uint32_t>(sections.size());
    hdr.section_table_offset = sectionTableOffset;
    hdr.file_size = fileSize;
    hdr.content_revision = 1;
    fwrite(&hdr, sizeof(hdr), 1, file);
  }

  void finalize() {
    if (!file) return;
    endFrame();
    finalizeVbs4();
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

      Vbs4CuRecord record;
      record.x = (uint16_t)cu.lx();
      record.y = (uint16_t)cu.ly();
      record.w = (uint8_t)cu.lwidth();
      record.h = (uint8_t)cu.lheight();
      record.depth = cu.depth;
      record.qp = (uint8_t)cu.qp;
      record.pred_mode = (uint8_t)cu.predMode;

      switch (cu.predMode)
      {
      case MODE_INTRA:
      {
        for (const PredictionUnit &pu : CU::traversePUs(cu))
        {
          if (pu.Y().valid())
          {
            record.intra_mode = (uint8_t)PU::getFinalIntraMode(pu, ChannelType::LUMA);
            record.mip_flag = cu.mipFlag ? 1 : 0;
            record.isp_mode = (uint8_t)to_uint(cu.ispMode);
            break;
          }
        }
        break;
      }
      case MODE_INTER:
      {
        record.skip = cu.skip ? 1 : 0;
        for (const PredictionUnit &pu : CU::traversePUs(cu))
        {
          record.merge_flag = pu.mergeFlag ? 1 : 0;
          record.inter_dir = (uint8_t)pu.interDir;
          if (pu.interDir != 2)
          {
            Mv mv = pu.mv[REF_PIC_LIST_0];
            mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
            mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
            record.mv_l0_x = (int16_t)mv.hor;
            record.mv_l0_y = (int16_t)mv.ver;
          }
          if (pu.interDir != 1)
          {
            Mv mv = pu.mv[REF_PIC_LIST_1];
            mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
            mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
            record.mv_l1_x = (int16_t)mv.hor;
            record.mv_l1_y = (int16_t)mv.ver;
          }
          record.ref_l0 = (int8_t)pu.refIdx[REF_PIC_LIST_0];
          record.ref_l1 = (int8_t)pu.refIdx[REF_PIC_LIST_1];
          break;
        }
        break;
      }
      default:
        break;
      }
      g_binStats.recordVbs4Cu(record);
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
      (uint32_t)sps->getMaxPicWidthInLumaSamples(),
      (uint32_t)sps->getMaxPicHeightInLumaSamples());
    // Suppress text header for binary mode by writing a brief note to dtrace
    DTRACE_HEADER( g_trace_ctx, "# VoidPlayer Binary Stats (%s) — see %s\n", binaryStatsFormatName(), binaryStatsPath());
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
