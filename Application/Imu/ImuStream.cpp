#include "Application/Imu/ImuStream.hpp"
#include "Libraries/Device/bmi088/Bmi088Accel.hpp"
#include "Libraries/Device/bmi088/Bmi088Gyro.hpp"
#include "Platform/Interface/Time.hpp"
#include "Platform/Interface/UsbCdc.hpp"

#include <array>
#include <atomic>
#include <cstring>

namespace application {

namespace {

// 队列占用约 36 KB，放在 AXI SRAM（NOLOAD 段），避免挤压 DTCM 中的
// 任务栈与 FreeRTOS 堆；Init 会显式复位全部索引，不依赖静态零初始化。
__attribute__((section(".dma_buffer"), aligned(64)))
ImuStream::Queue rawImuQueue;

std::atomic<bool> streamReady{false};
std::atomic<bool> streamEnabled{false};
std::atomic<std::uint32_t> frameSequence{0U};
std::atomic<std::uint32_t> configId{0U};

// 温度采用 seqlock 发布：序号为奇数表示生产者正在写入。
std::atomic<std::uint32_t> temperatureSequence{0U};
std::atomic<std::uint32_t> temperatureReadCount{0U};
std::atomic<std::uint32_t> temperatureTick{0U};
std::atomic<std::uint32_t> temperatureCelsiusBits{0U};
std::atomic<std::uint32_t> temperatureValid{0U};

std::atomic<std::uint32_t> pushedRecords{0U};
std::atomic<std::uint32_t> sentFrames{0U};
std::atomic<std::uint32_t> sentBytes{0U};
std::atomic<std::uint32_t> transmitStartedCount{0U};
std::atomic<std::uint32_t> transmitBusyCount{0U};
std::atomic<std::uint32_t> transmitNotReadyCount{0U};
std::atomic<std::uint32_t> transmitErrorCount{0U};
std::atomic<std::uint32_t> receivedCommandBytes{0U};
std::atomic<std::uint32_t> unknownCommandBytes{0U};

// 仅消费者（通信任务）访问。
std::uint32_t lastReportedDrops{0U};
alignas(4) std::array<std::uint8_t, protocol::ImuFrame::MaxTransmitSize>
    frameBuffer{};

using Frame = protocol::ImuFrame;

protocol::ImuFrame::Temperature ReadTemperature() noexcept {
  protocol::ImuFrame::Temperature temperature{};
  for (;;) {
    const std::uint32_t sequenceBefore =
        temperatureSequence.load(std::memory_order_acquire);
    if ((sequenceBefore & 1U) != 0U) {
      continue;
    }

    temperature.sequence = temperatureReadCount.load(std::memory_order_relaxed);
    temperature.tick = temperatureTick.load(std::memory_order_relaxed);
    const std::uint32_t bits =
        temperatureCelsiusBits.load(std::memory_order_relaxed);
    temperature.valid = temperatureValid.load(std::memory_order_relaxed) != 0U;
    std::memcpy(&temperature.celsius, &bits, sizeof(temperature.celsius));

    const std::uint32_t sequenceAfter =
        temperatureSequence.load(std::memory_order_acquire);
    if (sequenceBefore == sequenceAfter) {
      return temperature;
    }
  }
}

} // namespace

bool ImuStream::Init() noexcept {
  streamReady.store(false, std::memory_order_relaxed);
  streamEnabled.store(false, std::memory_order_relaxed);
  frameSequence.store(0U, std::memory_order_relaxed);
  temperatureSequence.store(0U, std::memory_order_relaxed);
  temperatureReadCount.store(0U, std::memory_order_relaxed);
  temperatureTick.store(0U, std::memory_order_relaxed);
  temperatureCelsiusBits.store(0U, std::memory_order_relaxed);
  temperatureValid.store(0U, std::memory_order_relaxed);
  pushedRecords.store(0U, std::memory_order_relaxed);
  sentFrames.store(0U, std::memory_order_relaxed);
  sentBytes.store(0U, std::memory_order_relaxed);
  transmitStartedCount.store(0U, std::memory_order_relaxed);
  transmitBusyCount.store(0U, std::memory_order_relaxed);
  transmitNotReadyCount.store(0U, std::memory_order_relaxed);
  transmitErrorCount.store(0U, std::memory_order_relaxed);
  receivedCommandBytes.store(0U, std::memory_order_relaxed);
  unknownCommandBytes.store(0U, std::memory_order_relaxed);
  lastReportedDrops = 0U;
  rawImuQueue.Reset();

  // 配置 ID 由锁定寄存器常量在编译期拼出，主机据此确认数据来源配置。
  const std::uint32_t id = Frame::MakeConfigId(
      device::Bmi088Gyro::BandwidthRegisterValue,
      device::Bmi088Gyro::RangeRegisterValue,
      device::Bmi088Accel::ConfRegisterValue,
      device::Bmi088Accel::RangeRegisterValue);
  configId.store(id, std::memory_order_relaxed);

  streamReady.store(true, std::memory_order_release);
  return true;
}

bool ImuStream::IsReady() noexcept {
  return streamReady.load(std::memory_order_acquire);
}

bool ImuStream::IsEnabled() noexcept {
  return streamEnabled.load(std::memory_order_acquire);
}

void ImuStream::SetEnabled(bool enabled) noexcept {
  streamEnabled.store(enabled, std::memory_order_release);
}

void ImuStream::Push(const device::Bmi088SampleRecord &record,
                        bool gyroscope) noexcept {
  if (!streamEnabled.load(std::memory_order_acquire)) {
    return;
  }

  Frame::Sample sample{};
  sample.sequence = record.sequence;
  sample.drdySequence = record.drdySequence;
  sample.drdyTick = record.drdyTick;
  sample.startTick = record.startTick;
  sample.completedTick = record.completedTick;
  sample.harvestedTick = record.harvestedTick;
  sample.xyz[0] = record.xyz[0];
  sample.xyz[1] = record.xyz[1];
  sample.xyz[2] = record.xyz[2];
  sample.flags = record.flags;
  sample.sensor = gyroscope ? Frame::Sensor::Gyroscope
                            : Frame::Sensor::Accelerometer;

  if (rawImuQueue.Push(sample)) {
    pushedRecords.fetch_add(1U, std::memory_order_relaxed);
  }
}

void ImuStream::PublishTemperature(std::uint32_t sequence,
                                      std::uint32_t tick, float celsius,
                                      bool valid) noexcept {
  // 温度仅约 25 Hz，同一读数在多轮任务循环中会重复到达，按序号去重。
  if (sequence == temperatureReadCount.load(std::memory_order_relaxed)) {
    return;
  }

  std::uint32_t bits{0U};
  std::memcpy(&bits, &celsius, sizeof(bits));

  temperatureSequence.fetch_add(1U, std::memory_order_acq_rel);
  temperatureReadCount.store(sequence, std::memory_order_relaxed);
  temperatureTick.store(tick, std::memory_order_relaxed);
  temperatureCelsiusBits.store(bits, std::memory_order_relaxed);
  temperatureValid.store(valid ? 1U : 0U, std::memory_order_relaxed);
  temperatureSequence.fetch_add(1U, std::memory_order_acq_rel);
}

void ImuStream::ProcessCommands() noexcept {
  if (!streamReady.load(std::memory_order_acquire)) {
    return;
  }

  std::array<std::uint8_t, 32U> received{};
  for (;;) {
    std::size_t length = 0U;
    const platform::UsbCdc::ReceiveResult result =
        platform::UsbCdc::TryRead(received.data(), received.size(), length);
    if (result == platform::UsbCdc::ReceiveResult::Empty) {
      break;
    }
    if (result != platform::UsbCdc::ReceiveResult::Received) {
      break;
    }

    receivedCommandBytes.fetch_add(static_cast<std::uint32_t>(length),
                                   std::memory_order_relaxed);
    for (std::size_t index = 0U; index < length; ++index) {
      switch (received[index]) {
      case 'R':
      case 'r':
        SetEnabled(true);
        break;
      case 'P':
      case 'p':
        SetEnabled(false);
        break;
      default:
        unknownCommandBytes.fetch_add(1U, std::memory_order_relaxed);
        break;
      }
    }
  }
}

void ImuStream::Drain() noexcept {
  if (!streamReady.load(std::memory_order_acquire)) {
    return;
  }

  Frame::Sample records[Frame::MaxRecordsPerFrame]{};
  const std::size_t count =
      rawImuQueue.Peek(records, Frame::MaxRecordsPerFrame);
  if (count == 0U) {
    return;
  }

  Frame::Header header{};
  header.configId = configId.load(std::memory_order_relaxed);
  header.tickHz = platform::Time::TickFrequencyHz();
  header.frameSequence = frameSequence.fetch_add(1U, std::memory_order_relaxed);
  header.sampleCount = static_cast<std::uint16_t>(count);
  const std::uint32_t dropped = rawImuQueue.DroppedCount();
  header.flags = IsEnabled() ? Frame::StreamEnabled : Frame::NoFlags;
  if (dropped != lastReportedDrops) {
    header.flags = static_cast<std::uint16_t>(header.flags |
                                              Frame::QueueOverflow);
  }
  header.droppedRecords = dropped;
  header.temperature = ReadTemperature();

  const std::size_t encoded =
      Frame::Encode(header, records, frameBuffer.data(), frameBuffer.size());
  if (encoded == 0U) {
    transmitErrorCount.fetch_add(1U, std::memory_order_relaxed);
    return;
  }

  switch (platform::UsbCdc::TryWrite(frameBuffer.data(), encoded)) {
  case platform::UsbCdc::TransmitResult::Started:
    rawImuQueue.Commit(count);
    lastReportedDrops = dropped;
    sentFrames.fetch_add(1U, std::memory_order_relaxed);
    sentBytes.fetch_add(static_cast<std::uint32_t>(encoded),
                        std::memory_order_relaxed);
    transmitStartedCount.fetch_add(1U, std::memory_order_relaxed);
    break;
  case platform::UsbCdc::TransmitResult::Busy:
    transmitBusyCount.fetch_add(1U, std::memory_order_relaxed);
    break;
  case platform::UsbCdc::TransmitResult::NotReady:
    transmitNotReadyCount.fetch_add(1U, std::memory_order_relaxed);
    break;
  case platform::UsbCdc::TransmitResult::InvalidArgument:
  case platform::UsbCdc::TransmitResult::Error:
  default:
    transmitErrorCount.fetch_add(1U, std::memory_order_relaxed);
    break;
  }
}

ImuStream::Statistics ImuStream::GetStatistics() noexcept {
  Statistics statistics{};
  statistics.pushedRecords = pushedRecords.load(std::memory_order_relaxed);
  statistics.droppedRecords = rawImuQueue.DroppedCount();
  statistics.queuedRecords =
      static_cast<std::uint32_t>(rawImuQueue.Size());
  statistics.sentFrames = sentFrames.load(std::memory_order_relaxed);
  statistics.sentBytes = sentBytes.load(std::memory_order_relaxed);
  statistics.transmitStartedCount =
      transmitStartedCount.load(std::memory_order_relaxed);
  statistics.transmitBusyCount =
      transmitBusyCount.load(std::memory_order_relaxed);
  statistics.transmitNotReadyCount =
      transmitNotReadyCount.load(std::memory_order_relaxed);
  statistics.transmitErrorCount =
      transmitErrorCount.load(std::memory_order_relaxed);
  statistics.receivedCommandBytes =
      receivedCommandBytes.load(std::memory_order_relaxed);
  statistics.unknownCommandBytes =
      unknownCommandBytes.load(std::memory_order_relaxed);
  return statistics;
}

} // namespace application
