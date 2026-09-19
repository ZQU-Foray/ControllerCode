// ImuStream 有界队列与非阻塞发送主机测试。
// 覆盖：默认关闭、主机命令开关、组帧与批量上限、Busy/NotReady 时保留数据、
// 队列溢出计数、温度 seqlock 与去重、帧序号、Init 复位。
// 依赖用桩实现：UsbCdcPort_* 与 TimePort_TickFrequencyHz。

#include "Application/Imu/ImuStream.hpp"
#include "Platform/Interface/UsbCdc.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>

using application::ImuStream;
using Frame = protocol::ImuFrame;

namespace {

constexpr std::uint32_t TickFrequency = 480000000U;

bool usbReady = true;
bool usbConnected = true;
std::uint8_t nextTransmitResult = USB_CDC_PORT_TRANSMIT_STARTED;
std::deque<std::uint8_t> commandBytes;
std::array<std::uint8_t, 8192U> capturedBytes{};
std::size_t capturedLength = 0U;

device::Bmi088SampleRecord MakeRecord(std::uint32_t index) {
  device::Bmi088SampleRecord record{};
  record.sequence = index;
  record.drdySequence = 0x1000U + index;
  record.drdyTick = 0x2000U + index;
  record.startTick = 0x3000U + index;
  record.completedTick = 0x4000U + index;
  record.harvestedTick = 0x5000U + index;
  record.xyz[0] = static_cast<std::int16_t>(index);
  record.xyz[1] = static_cast<std::int16_t>(-static_cast<int>(index));
  record.xyz[2] = 1234;
  record.flags = static_cast<std::uint16_t>(index & 0x3U);
  return record;
}

void ResetStream(bool connected = true) {
  assert(ImuStream::Init());
  usbReady = true;
  usbConnected = connected;
  nextTransmitResult = USB_CDC_PORT_TRANSMIT_STARTED;
  commandBytes.clear();
  capturedLength = 0U;
}

void Enable() {
  commandBytes.push_back('R');
  ImuStream::ProcessCommands();
  assert(ImuStream::IsEnabled());
}

void DecodeCaptured(Frame::Header &header,
                    std::array<Frame::Sample, Frame::MaxRecordsPerFrame> &samples) {
  assert(capturedLength != 0U);
  assert(Frame::Decode(capturedBytes.data(), capturedLength, header,
                       samples.data(), samples.size()));
}

void TestDisabledByDefault() {
  ResetStream();
  assert(ImuStream::IsReady());
  assert(!ImuStream::IsEnabled());

  ImuStream::Push(MakeRecord(1U), true);
  ImuStream::Drain();
  assert(ImuStream::GetStatistics().pushedRecords == 0U);
  assert(ImuStream::GetStatistics().queuedRecords == 0U);
  assert(capturedLength == 0U);
}

void TestResumeFrameAndSequence() {
  ResetStream();
  Enable();

  for (std::uint32_t index = 0U; index < 5U; ++index) {
    ImuStream::Push(MakeRecord(index), (index % 2U) != 0U);
  }
  // 同一温度序号重复发布只生效一次。
  ImuStream::PublishTemperature(3U, 0x1234U, 41.25F, true);
  ImuStream::PublishTemperature(3U, 0x9999U, 99.0F, true);

  ImuStream::Drain();
  Frame::Header header{};
  std::array<Frame::Sample, Frame::MaxRecordsPerFrame> samples{};
  DecodeCaptured(header, samples);
  assert(header.sampleCount == 5U);
  assert(header.tickHz == TickFrequency);
  assert(header.frameSequence == 0U);
  assert(header.configId != 0U);
  assert((header.flags & Frame::StreamEnabled) != 0U);
  assert(header.droppedRecords == 0U);
  assert(header.temperature.valid);
  assert(header.temperature.sequence == 3U);
  assert(header.temperature.tick == 0x1234U);
  assert(header.temperature.celsius == 41.25F);
  for (std::uint32_t index = 0U; index < 5U; ++index) {
    assert(samples[index].sequence == index);
    assert(samples[index].drdySequence == 0x1000U + index);
    assert(samples[index].harvestedTick == 0x5000U + index);
    assert(samples[index].xyz[0] == static_cast<std::int16_t>(index));
    assert(samples[index].xyz[1] == static_cast<std::int16_t>(-static_cast<int>(index)));
    assert(samples[index].xyz[2] == 1234);
    assert(samples[index].flags == (index & 0x3U));
    assert(samples[index].sensor == (((index % 2U) != 0U)
                                         ? Frame::Sensor::Gyroscope
                                         : Frame::Sensor::Accelerometer));
  }
  assert(ImuStream::GetStatistics().queuedRecords == 0U);
  assert(ImuStream::GetStatistics().sentFrames == 1U);
  assert(ImuStream::GetStatistics().sentBytes == capturedLength);

  // 第二帧序号递增。
  ImuStream::Push(MakeRecord(9U), false);
  ImuStream::Drain();
  DecodeCaptured(header, samples);
  assert(header.frameSequence == 1U);
}

void TestBatchLimit() {
  ResetStream();
  Enable();

  const std::uint32_t total =
      static_cast<std::uint32_t>(Frame::MaxRecordsPerFrame) + 3U;
  for (std::uint32_t index = 0U; index < total; ++index) {
    ImuStream::Push(MakeRecord(index), true);
  }

  ImuStream::Drain();
  Frame::Header header{};
  std::array<Frame::Sample, Frame::MaxRecordsPerFrame> samples{};
  DecodeCaptured(header, samples);
  assert(header.sampleCount == Frame::MaxRecordsPerFrame);
  assert(ImuStream::GetStatistics().queuedRecords == 3U);

  ImuStream::Drain();
  DecodeCaptured(header, samples);
  assert(header.sampleCount == 3U);
  assert(ImuStream::GetStatistics().queuedRecords == 0U);
}

void TestBusyRetainsData() {
  ResetStream();
  Enable();
  ImuStream::Push(MakeRecord(0U), true);
  ImuStream::Push(MakeRecord(1U), false);

  nextTransmitResult = USB_CDC_PORT_TRANSMIT_BUSY;
  capturedLength = 0U;
  ImuStream::Drain();
  assert(capturedLength == 0U);
  assert(ImuStream::GetStatistics().transmitBusyCount == 1U);
  assert(ImuStream::GetStatistics().queuedRecords == 2U);

  nextTransmitResult = USB_CDC_PORT_TRANSMIT_STARTED;
  ImuStream::Drain();
  Frame::Header header{};
  std::array<Frame::Sample, Frame::MaxRecordsPerFrame> samples{};
  DecodeCaptured(header, samples);
  assert(header.sampleCount == 2U);
  assert(samples[0].sequence == 0U && samples[1].sequence == 1U);
  assert(ImuStream::GetStatistics().queuedRecords == 0U);
}

void TestNotReadyRetainsData() {
  ResetStream();
  Enable();
  ImuStream::Push(MakeRecord(0U), true);

  usbConnected = false;
  ImuStream::Drain();
  assert(ImuStream::GetStatistics().transmitNotReadyCount == 1U);
  assert(ImuStream::GetStatistics().queuedRecords == 1U);
  assert(capturedLength == 0U);

  usbConnected = true;
  ImuStream::Drain();
  assert(ImuStream::GetStatistics().queuedRecords == 0U);
  assert(capturedLength != 0U);
}

void TestQueueOverflowCountsDrops() {
  ResetStream();
  Enable();

  const std::uint32_t capacity =
      static_cast<std::uint32_t>(ImuStream::QueueCapacity);
  for (std::uint32_t index = 0U; index < capacity + 5U; ++index) {
    ImuStream::Push(MakeRecord(index), true);
  }

  const ImuStream::Statistics statistics = ImuStream::GetStatistics();
  assert(statistics.pushedRecords == capacity);
  assert(statistics.droppedRecords == 5U);
  assert(statistics.queuedRecords == capacity);

  // 溢出后第一帧必须置位 QueueOverflow。
  ImuStream::Drain();
  Frame::Header header{};
  std::array<Frame::Sample, Frame::MaxRecordsPerFrame> samples{};
  DecodeCaptured(header, samples);
  assert((header.flags & Frame::QueueOverflow) != 0U);
  assert(header.droppedRecords == 5U);
}

void TestPauseAndUnknownCommand() {
  ResetStream();
  Enable();
  ImuStream::Push(MakeRecord(0U), true);
  assert(ImuStream::GetStatistics().pushedRecords == 1U);

  commandBytes.push_back('x');
  commandBytes.push_back('P');
  ImuStream::ProcessCommands();
  assert(!ImuStream::IsEnabled());
  assert(ImuStream::GetStatistics().unknownCommandBytes == 1U);

  ImuStream::Push(MakeRecord(1U), true);
  assert(ImuStream::GetStatistics().pushedRecords == 1U);
}

void TestInitResets() {
  ResetStream();
  Enable();
  ImuStream::Push(MakeRecord(0U), true);
  ImuStream::Drain();
  assert(ImuStream::GetStatistics().sentFrames == 1U);

  ResetStream();
  const ImuStream::Statistics statistics = ImuStream::GetStatistics();
  assert(statistics.pushedRecords == 0U);
  assert(statistics.droppedRecords == 0U);
  assert(statistics.queuedRecords == 0U);
  assert(statistics.sentFrames == 0U);
  assert(!ImuStream::IsEnabled());
}

} // namespace

extern "C" {

bool UsbCdcPort_Init(void) { return true; }

bool UsbCdcPort_IsReady(void) { return usbReady; }

bool UsbCdcPort_IsConnected(void) { return usbConnected; }

bool UsbCdcPort_SetReceiveNotification(UsbCdcPort_ReceiveNotification function,
                                       void *context) {
  (void)function;
  (void)context;
  return false;
}

bool UsbCdcPort_SetTransmitNotification(UsbCdcPort_TransmitNotification function,
                                        void *context) {
  (void)function;
  (void)context;
  return false;
}

UsbCdcPort_ReceiveResult UsbCdcPort_TryRead(std::uint8_t *data,
                                            std::uint32_t data_capacity,
                                            std::uint32_t *length) {
  if (!usbReady) {
    *length = 0U;
    return USB_CDC_PORT_RECEIVE_NOT_READY;
  }

  std::uint32_t count = 0U;
  while (count < data_capacity && !commandBytes.empty()) {
    data[count++] = commandBytes.front();
    commandBytes.pop_front();
  }
  *length = count;
  return count == 0U ? USB_CDC_PORT_RECEIVE_EMPTY
                     : USB_CDC_PORT_RECEIVE_RECEIVED;
}

UsbCdcPort_TransmitResult UsbCdcPort_TryWrite(const std::uint8_t *data,
                                              std::uint32_t length) {
  if (!usbConnected) {
    return USB_CDC_PORT_TRANSMIT_NOT_READY;
  }
  if (nextTransmitResult == USB_CDC_PORT_TRANSMIT_STARTED) {
    assert(length <= capturedBytes.size());
    std::memcpy(capturedBytes.data(), data, length);
    capturedLength = length;
  }
  return nextTransmitResult;
}

bool UsbCdcPort_GetStatistics(UsbCdcPort_Statistics *statistics) {
  (void)statistics;
  return false;
}

std::uint32_t TimePort_FrequencyHz(void) { return TickFrequency; }

} // extern "C"

int main() {
  TestDisabledByDefault();
  TestResumeFrameAndSequence();
  TestBatchLimit();
  TestBusyRetainsData();
  TestNotReadyRetainsData();
  TestQueueOverflowCountsDrops();
  TestPauseAndUnknownCommand();
  TestInitResets();
  std::cout << "imu_stream: PASS\n";
  return 0;
}
