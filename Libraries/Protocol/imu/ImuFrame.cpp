#include "Libraries/Protocol/imu/ImuFrame.hpp"
#include "Libraries/Algorithm/alg_crc/Crc32.h"

#include <cstring>

namespace protocol {
namespace {

void WriteLe16(std::uint8_t *&cursor, std::uint16_t value) noexcept {
  cursor[0] = static_cast<std::uint8_t>(value & 0xFFU);
  cursor[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  cursor += 2;
}

void WriteLe32(std::uint8_t *&cursor, std::uint32_t value) noexcept {
  cursor[0] = static_cast<std::uint8_t>(value & 0xFFU);
  cursor[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  cursor[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  cursor[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
  cursor += 4;
}

void WriteFloatLe(std::uint8_t *&cursor, float value) noexcept {
  std::uint32_t bits{0U};
  std::memcpy(&bits, &value, sizeof(bits));
  WriteLe32(cursor, bits);
}

std::uint8_t ReadLe8(const std::uint8_t *&cursor) noexcept {
  const std::uint8_t value = *cursor;
  cursor += 1;
  return value;
}

std::uint16_t ReadLe16(const std::uint8_t *&cursor) noexcept {
  const std::uint16_t value =
      static_cast<std::uint16_t>(cursor[0]) |
      static_cast<std::uint16_t>(static_cast<std::uint16_t>(cursor[1]) << 8U);
  cursor += 2;
  return value;
}

std::uint32_t ReadLe32(const std::uint8_t *&cursor) noexcept {
  const std::uint32_t value =
      static_cast<std::uint32_t>(cursor[0]) |
      (static_cast<std::uint32_t>(cursor[1]) << 8U) |
      (static_cast<std::uint32_t>(cursor[2]) << 16U) |
      (static_cast<std::uint32_t>(cursor[3]) << 24U);
  cursor += 4;
  return value;
}

float ReadFloatLe(const std::uint8_t *&cursor) noexcept {
  const std::uint32_t bits = ReadLe32(cursor);
  float value{0.0F};
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

} // namespace

std::size_t ImuFrame::Encode(const Header &header, const Sample *records,
                                std::uint8_t *output,
                                std::size_t capacity) noexcept {
  const std::size_t size = EncodedSize(header.sampleCount);
  if (size == 0U || output == nullptr || capacity < size) {
    return 0U;
  }
  if (header.sampleCount != 0U && records == nullptr) {
    return 0U;
  }

  std::uint8_t *cursor = output;
  WriteLe16(cursor, Magic);
  *cursor++ = Version;
  *cursor++ = static_cast<std::uint8_t>(FrameType::Samples);
  WriteLe32(cursor, header.configId);
  WriteLe32(cursor, header.tickHz);
  WriteLe32(cursor, header.frameSequence);
  WriteLe16(cursor, header.sampleCount);
  WriteLe16(cursor, header.flags);
  WriteLe32(cursor, header.droppedRecords);
  WriteLe32(cursor, header.temperature.sequence);
  WriteLe32(cursor, header.temperature.tick);
  WriteFloatLe(cursor, header.temperature.celsius);
  *cursor++ = header.temperature.valid ? 1U : 0U;
  *cursor++ = 0U;
  *cursor++ = 0U;
  *cursor++ = 0U;

  for (std::size_t index = 0U; index < header.sampleCount; ++index) {
    const Sample &sample = records[index];
    WriteLe32(cursor, sample.sequence);
    WriteLe32(cursor, sample.drdySequence);
    WriteLe32(cursor, sample.drdyTick);
    WriteLe32(cursor, sample.startTick);
    WriteLe32(cursor, sample.completedTick);
    WriteLe32(cursor, sample.harvestedTick);
    for (const std::int16_t axis : sample.xyz) {
      WriteLe16(cursor, static_cast<std::uint16_t>(axis));
    }
    WriteLe16(cursor, sample.flags);
    *cursor++ = static_cast<std::uint8_t>(sample.sensor);
    *cursor++ = 0U;
    *cursor++ = 0U;
    *cursor++ = 0U;
  }

  std::uint32_t crc = alg_crc::Crc32Calc(output, size - CrcSize);
  WriteLe32(cursor, crc);
  return size;
}

bool ImuFrame::Decode(const std::uint8_t *input, std::size_t length,
                         Header &header, Sample *records,
                         std::size_t maxRecords) noexcept {
  if (input == nullptr || length < OverheadSize) {
    return false;
  }

  const std::uint8_t *cursor = input;
  const std::uint16_t magic = ReadLe16(cursor);
  const std::uint8_t version = ReadLe8(cursor);
  const std::uint8_t frameType = ReadLe8(cursor);
  if (magic != Magic || version != Version ||
      frameType != static_cast<std::uint8_t>(FrameType::Samples)) {
    return false;
  }

  const std::uint32_t configId = ReadLe32(cursor);
  const std::uint32_t tickHz = ReadLe32(cursor);
  const std::uint32_t frameSequence = ReadLe32(cursor);
  const std::uint16_t sampleCount = ReadLe16(cursor);
  const std::uint16_t flags = ReadLe16(cursor);
  const std::uint32_t droppedRecords = ReadLe32(cursor);
  Temperature temperature{};
  temperature.sequence = ReadLe32(cursor);
  temperature.tick = ReadLe32(cursor);
  temperature.celsius = ReadFloatLe(cursor);
  temperature.valid = ReadLe8(cursor) != 0U;
  (void)ReadLe8(cursor);
  (void)ReadLe8(cursor);
  (void)ReadLe8(cursor);

  const std::size_t size = EncodedSize(sampleCount);
  if (size == 0U || length < size) {
    return false;
  }
  if (sampleCount != 0U && (records == nullptr || maxRecords < sampleCount)) {
    return false;
  }

  const std::uint8_t *crcCursor = input + size - CrcSize;
  const std::uint32_t expectedCrc = ReadLe32(crcCursor);
  if (alg_crc::Crc32Calc(input, size - CrcSize) != expectedCrc) {
    return false;
  }

  for (std::size_t index = 0U; index < sampleCount; ++index) {
    Sample sample{};
    sample.sequence = ReadLe32(cursor);
    sample.drdySequence = ReadLe32(cursor);
    sample.drdyTick = ReadLe32(cursor);
    sample.startTick = ReadLe32(cursor);
    sample.completedTick = ReadLe32(cursor);
    sample.harvestedTick = ReadLe32(cursor);
    for (std::int16_t &axis : sample.xyz) {
      axis = static_cast<std::int16_t>(ReadLe16(cursor));
    }
    sample.flags = ReadLe16(cursor);
    sample.sensor = static_cast<Sensor>(ReadLe8(cursor));
    (void)ReadLe8(cursor);
    (void)ReadLe8(cursor);
    (void)ReadLe8(cursor);
    records[index] = sample;
  }

  header.configId = configId;
  header.tickHz = tickHz;
  header.frameSequence = frameSequence;
  header.sampleCount = sampleCount;
  header.flags = flags;
  header.droppedRecords = droppedRecords;
  header.temperature = temperature;
  return true;
}

} // namespace protocol
