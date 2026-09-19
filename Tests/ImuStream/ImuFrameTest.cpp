// ImuFrame 版本化二进制帧编解码主机测试。
// 覆盖：CRC-32 已知向量、小端字节序、字段逐位往返、尺寸上限、
// 魔数/版本/长度/CRC/样本数越界等失败路径。

#include "Libraries/Algorithm/alg_crc/Crc32.h"
#include "Libraries/Protocol/imu/ImuFrame.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>

using Frame = protocol::ImuFrame;

namespace {

std::uint32_t ReadLe32(const std::uint8_t *input) {
  return static_cast<std::uint32_t>(input[0]) |
         (static_cast<std::uint32_t>(input[1]) << 8U) |
         (static_cast<std::uint32_t>(input[2]) << 16U) |
         (static_cast<std::uint32_t>(input[3]) << 24U);
}

void WriteLe16(std::uint8_t *output, std::uint16_t value) {
  output[0] = static_cast<std::uint8_t>(value & 0xFFU);
  output[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

Frame::Sample MakeSample(std::uint32_t index, bool gyroscope) {
  Frame::Sample sample{};
  sample.sequence = 0x10000000U + index;
  sample.drdySequence = 0x20000000U + index;
  sample.drdyTick = 0x30000000U + index;
  sample.startTick = 0x40000000U + index;
  sample.completedTick = 0x50000000U + index;
  sample.harvestedTick = 0x60000000U + index;
  sample.xyz[0] = static_cast<std::int16_t>(-1000 - static_cast<int>(index));
  sample.xyz[1] = 0;
  sample.xyz[2] = static_cast<std::int16_t>(32000 - static_cast<int>(index));
  sample.flags = static_cast<std::uint16_t>(index & 0xFFFFU);
  sample.sensor = gyroscope ? Frame::Sensor::Gyroscope
                            : Frame::Sensor::Accelerometer;
  return sample;
}

bool SamplesEqual(const Frame::Sample &lhs, const Frame::Sample &rhs) {
  return lhs.sequence == rhs.sequence && lhs.drdySequence == rhs.drdySequence &&
         lhs.drdyTick == rhs.drdyTick && lhs.startTick == rhs.startTick &&
         lhs.completedTick == rhs.completedTick &&
         lhs.harvestedTick == rhs.harvestedTick &&
         lhs.xyz[0] == rhs.xyz[0] && lhs.xyz[1] == rhs.xyz[1] &&
         lhs.xyz[2] == rhs.xyz[2] && lhs.flags == rhs.flags &&
         lhs.sensor == rhs.sensor;
}

void TestCrc32() {
  const char *text = "123456789";
  assert(alg_crc::Crc32Calc(reinterpret_cast<const std::uint8_t *>(text),
                            9U) == 0xCBF43926U);
  assert(alg_crc::Crc32Calc(nullptr, 0U) == 0U);
  assert(alg_crc::Crc32Calc(nullptr, 4U) == 0U);
}

void TestLayoutConstants() {
  assert(Frame::OverheadSize == 44U);
  assert(Frame::RecordSize == 36U);
  assert(Frame::EncodedSize(0U) == 44U);
  assert(Frame::EncodedSize(1U) == 80U);
  assert(Frame::EncodedSize(Frame::MaxRecordsPerFrame) <=
         Frame::MaxTransmitSize);
  assert(Frame::EncodedSize(
             static_cast<std::uint16_t>(Frame::MaxRecordsPerFrame + 1U)) == 0U);
  assert(Frame::MakeConfigId(0x00U, 0x00U, 0xACU, 0x00U) == 0x0000AC00U);
  assert(Frame::MakeConfigId(0x01U, 0x02U, 0x03U, 0x04U) == 0x01020304U);
}

void TestRoundTrip() {
  std::array<Frame::Sample, Frame::MaxRecordsPerFrame> samples{};
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    samples[index] = MakeSample(static_cast<std::uint32_t>(index),
                                (index % 2U) != 0U);
  }

  Frame::Header header{};
  header.configId = 0x11223344U;
  header.tickHz = 480000000U;
  header.frameSequence = 0xDEADBEEFU;
  header.sampleCount = static_cast<std::uint16_t>(samples.size());
  header.flags = static_cast<std::uint16_t>(Frame::QueueOverflow |
                                            Frame::StreamEnabled);
  header.droppedRecords = 0x01020304U;
  header.temperature.sequence = 77U;
  header.temperature.tick = 0x0A0B0C0DU;
  header.temperature.celsius = 42.5F;
  header.temperature.valid = true;

  std::array<std::uint8_t, Frame::MaxTransmitSize> buffer{};
  const std::size_t encoded =
      Frame::Encode(header, samples.data(), buffer.data(), buffer.size());
  assert(encoded == Frame::EncodedSize(header.sampleCount));
  assert(encoded <= Frame::MaxTransmitSize);

  // 魔数、版本、类型与显式小端字段。
  assert(buffer[0] == 0x52U); // 'R'
  assert(buffer[1] == 0x49U); // 'I'
  assert(buffer[2] == Frame::Version);
  assert(buffer[3] == static_cast<std::uint8_t>(Frame::FrameType::Samples));
  assert(ReadLe32(&buffer[4]) == header.configId);   // 小端：44 33 22 11
  assert(buffer[4] == 0x44U && buffer[7] == 0x11U);
  assert(ReadLe32(&buffer[8]) == header.tickHz);
  assert(ReadLe32(&buffer[12]) == header.frameSequence);

  Frame::Header decoded{};
  std::array<Frame::Sample, Frame::MaxRecordsPerFrame> decodedSamples{};
  assert(Frame::Decode(buffer.data(), encoded, decoded, decodedSamples.data(),
                       decodedSamples.size()));
  assert(decoded.configId == header.configId);
  assert(decoded.tickHz == header.tickHz);
  assert(decoded.frameSequence == header.frameSequence);
  assert(decoded.sampleCount == header.sampleCount);
  assert(decoded.flags == header.flags);
  assert(decoded.droppedRecords == header.droppedRecords);
  assert(decoded.temperature.sequence == header.temperature.sequence);
  assert(decoded.temperature.tick == header.temperature.tick);
  assert(decoded.temperature.celsius == header.temperature.celsius);
  assert(decoded.temperature.valid);
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    assert(SamplesEqual(decodedSamples[index], samples[index]));
  }

  // 重新编码必须与原始字节完全一致。
  std::array<std::uint8_t, Frame::MaxTransmitSize> again{};
  const std::size_t reencoded =
      Frame::Encode(decoded, decodedSamples.data(), again.data(), again.size());
  assert(reencoded == encoded);
  assert(std::memcmp(buffer.data(), again.data(), encoded) == 0);
}

void TestZeroSamples() {
  Frame::Header header{};
  header.sampleCount = 0U;
  std::array<std::uint8_t, Frame::MaxTransmitSize> buffer{};
  const std::size_t encoded =
      Frame::Encode(header, nullptr, buffer.data(), buffer.size());
  assert(encoded == Frame::OverheadSize);
  Frame::Header decoded{};
  assert(Frame::Decode(buffer.data(), encoded, decoded, nullptr, 0U));
  assert(decoded.sampleCount == 0U);
}

void TestFailures() {
  std::array<Frame::Sample, 2U> samples{MakeSample(0U, true),
                                        MakeSample(1U, false)};
  Frame::Header header{};
  header.sampleCount = 2U;
  header.temperature.celsius = -5.25F;
  std::array<std::uint8_t, Frame::MaxTransmitSize> buffer{};
  const std::size_t encoded =
      Frame::Encode(header, samples.data(), buffer.data(), buffer.size());
  assert(encoded != 0U);

  Frame::Header decoded{};
  std::array<Frame::Sample, 2U> decodedSamples{};

  // 数据位翻转必须被 CRC 拦下。
  {
    auto corrupted = buffer;
    corrupted[encoded / 2U] ^= 0x01U;
    assert(!Frame::Decode(corrupted.data(), encoded, decoded,
                          decodedSamples.data(), decodedSamples.size()));
  }
  // CRC 位翻转必须被拦下。
  {
    auto corrupted = buffer;
    corrupted[encoded - 1U] ^= 0x80U;
    assert(!Frame::Decode(corrupted.data(), encoded, decoded,
                          decodedSamples.data(), decodedSamples.size()));
  }
  // 截断长度必须被拦下。
  assert(!Frame::Decode(buffer.data(), encoded - 1U, decoded,
                        decodedSamples.data(), decodedSamples.size()));
  // 过短输入必须被拦下。
  assert(!Frame::Decode(buffer.data(), 8U, decoded, decodedSamples.data(),
                        decodedSamples.size()));
  // 空指针必须被拦下。
  assert(!Frame::Decode(nullptr, encoded, decoded, decodedSamples.data(),
                        decodedSamples.size()));
  // 版本不匹配必须被拦下。
  {
    auto corrupted = buffer;
    corrupted[2] = static_cast<std::uint8_t>(Frame::Version + 1U);
    assert(!Frame::Decode(corrupted.data(), encoded, decoded,
                          decodedSamples.data(), decodedSamples.size()));
  }
  // 样本数被改写为越界值必须被拦下。
  {
    auto corrupted = buffer;
    WriteLe16(&corrupted[16], static_cast<std::uint16_t>(
                                  Frame::MaxRecordsPerFrame + 1U));
    assert(!Frame::Decode(corrupted.data(), encoded, decoded,
                          decodedSamples.data(), decodedSamples.size()));
  }
  // 解码目标容量不足必须被拦下。
  assert(!Frame::Decode(buffer.data(), encoded, decoded, decodedSamples.data(),
                        1U));
  // 非零样本数配空目标必须被拦下。
  assert(!Frame::Decode(buffer.data(), encoded, decoded, nullptr, 0U));
  // 编码缓冲区不足必须返回 0。
  {
    std::array<std::uint8_t, 16U> small{};
    assert(Frame::Encode(header, samples.data(), small.data(), small.size()) ==
           0U);
  }
  // 编码样本数越界必须返回 0。
  {
    Frame::Header oversized{};
    oversized.sampleCount = static_cast<std::uint16_t>(
        Frame::MaxRecordsPerFrame + 1U);
    assert(Frame::Encode(oversized, samples.data(), buffer.data(),
                         buffer.size()) == 0U);
  }
  // 负温度必须逐位往返。
  {
    Frame::Header decodedTemperature{};
    assert(Frame::Decode(buffer.data(), encoded, decodedTemperature,
                         decodedSamples.data(), decodedSamples.size()));
    assert(decodedTemperature.temperature.celsius == -5.25F);
  }
}

} // namespace

int main() {
  TestCrc32();
  TestLayoutConstants();
  TestRoundTrip();
  TestZeroSamples();
  TestFailures();
  std::cout << "imu_frame: PASS\n";
  return 0;
}
