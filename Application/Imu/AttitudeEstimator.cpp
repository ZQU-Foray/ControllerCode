#include "Application/Imu/AttitudeEstimator.hpp"
#include "Libraries/Algorithm/alg_estimate/ImuBodyMapping.hpp"
#include "Libraries/Algorithm/alg_estimate/QuaternionEkf.hpp"
#include "Platform/Interface/Time.hpp"
#include <cstddef>

namespace application {

namespace {

alg_estimate::QuaternionEkf estimator;
alg_estimate::QuaternionEkf::Config config{};
GyroBiasCalibration calibration;
bool ready{false};

// 上电标定结果：x/y 注入滤波器初值，z 只在输入侧扣除（算法没有 z 轴零偏状态）。
bool calibrationInjected{false};
float gyroZTrimRadPerSec{0.0F};

// 时序状态：陀螺节拍与最近的加速度样本。
bool gyroAnchored{false};
std::uint32_t lastGyroTick{0U};
std::uint32_t lastGyroSequence{0U};
std::uint32_t lastAccelSequence{0U};
std::uint32_t lastAccelTick{0U};
bool accelAvailable{false};
bool accelFresh{false};
alg_math::Vector3 accelBody{};

// 调试器可见存档。写读都走字节拷贝，避免逐字段复制漏项。
volatile AttitudeEstimator::Output outputArchive{};
volatile AttitudeEstimator::Statistics statisticsArchive{};
static_assert(sizeof(AttitudeEstimator::Output) == sizeof(outputArchive),
              "存档布局必须一致");

void PublishArchive(const AttitudeEstimator::Output &source) noexcept {
  const std::uint8_t *bytes = reinterpret_cast<const std::uint8_t *>(&source);
  volatile std::uint8_t *target =
      reinterpret_cast<volatile std::uint8_t *>(&outputArchive);
  for (std::size_t index = 0U; index < sizeof(source); ++index) {
    target[index] = bytes[index];
  }
}

AttitudeEstimator::Output ReadArchive() noexcept {
  AttitudeEstimator::Output result{};
  std::uint8_t *bytes = reinterpret_cast<std::uint8_t *>(&result);
  const volatile std::uint8_t *source =
      reinterpret_cast<const volatile std::uint8_t *>(&outputArchive);
  for (std::size_t index = 0U; index < sizeof(result); ++index) {
    bytes[index] = source[index];
  }
  return result;
}

void ResetArchives() noexcept {
  AttitudeEstimator::Output output{};
  PublishArchive(output);
  AttitudeEstimator::Statistics statistics{};
  const std::uint8_t *bytes =
      reinterpret_cast<const std::uint8_t *>(&statistics);
  volatile std::uint8_t *target =
      reinterpret_cast<volatile std::uint8_t *>(&statisticsArchive);
  for (std::size_t index = 0U; index < sizeof(statistics); ++index) {
    target[index] = bytes[index];
  }
}

/** 时间戳标志不确定的样本（晚启动或完成前又来了新的 DRDY）视为时序不可信。 */
bool HasUncertainTiming(const device::Bmi088SampleRecord &record) {
  return (record.flags &
          (device::Bmi088SampleRecord::LateStart |
           device::Bmi088SampleRecord::NewerEventBeforeCompletion)) != 0U;
}

void PublishOutput(const device::Bmi088SampleRecord &record) {
  const alg_estimate::QuaternionEkf::State state = estimator.GetState();
  const GyroBiasCalibration::Snapshot calibrationSnapshot =
      calibration.GetSnapshot();

  AttitudeEstimator::Output output{};
  output.version = AttitudeEstimator::OutputVersion;
  output.updateTick = record.drdyTick;
  output.quaternion[0] = state.quaternion.w;
  output.quaternion[1] = state.quaternion.x;
  output.quaternion[2] = state.quaternion.y;
  output.quaternion[3] = state.quaternion.z;
  output.eulerDegrees[0] = state.eulerDegrees.x;
  output.eulerDegrees[1] = state.eulerDegrees.y;
  output.eulerDegrees[2] = state.eulerDegrees.z;
  output.gyroBiasRadPerSec[0] = state.gyroBiasRadPerSec.x;
  output.gyroBiasRadPerSec[1] = state.gyroBiasRadPerSec.y;
  output.gyroBiasRadPerSec[2] = state.gyroBiasRadPerSec.z;
  output.yawTotalDegrees = state.yawTotalDegrees;
  output.chiSquare = state.chiSquare;
  output.updateCount = state.updateCount;
  output.divergenceCount = state.divergenceCount;
  output.valid = state.valid ? 1U : 0U;
  output.converged = state.converged ? 1U : 0U;
  output.stable = state.stable ? 1U : 0U;
  output.calibrationBiasRadPerSec[0] = calibrationSnapshot.biasRadPerSec.x;
  output.calibrationBiasRadPerSec[1] = calibrationSnapshot.biasRadPerSec.y;
  output.calibrationBiasRadPerSec[2] = calibrationSnapshot.biasRadPerSec.z;
  output.calibrationSpreadRadPerSec[0] = calibrationSnapshot.spreadRadPerSec.x;
  output.calibrationSpreadRadPerSec[1] = calibrationSnapshot.spreadRadPerSec.y;
  output.calibrationSpreadRadPerSec[2] = calibrationSnapshot.spreadRadPerSec.z;
  output.calibrationState =
      static_cast<std::uint32_t>(calibrationSnapshot.state);
  output.calibrationSamples = calibrationSnapshot.samples;
  output.calibrationTemperatureCelsius = calibrationSnapshot.temperatureCelsius;
  output.calibrationElapsedMs = calibrationSnapshot.elapsedMs;
  PublishArchive(output);

  statisticsArchive.calibrationRestarts = calibrationSnapshot.restarts;
}

void ProcessGyroscope(const device::Bmi088SampleRecord &record,
                      float temperatureCelsius) noexcept {
  if (record.sequence <= lastGyroSequence) {
    ++statisticsArchive.rejectedSequence;
    return;
  }
  lastGyroSequence = record.sequence;

  if (!gyroAnchored) {
    // 首个陀螺样本只建立时间基准，不积分。
    lastGyroTick = record.drdyTick;
    gyroAnchored = true;
    return;
  }

  const std::uint32_t elapsedTicks = record.drdyTick - lastGyroTick;
  lastGyroTick = record.drdyTick;
  const float stepSeconds = platform::Time::TicksToSeconds(elapsedTicks);
  if (!(stepSeconds > 0.0F) ||
      stepSeconds > AttitudeEstimator::GyroGapLimitSeconds) {
    // 断流：不虚构缺失运动，仅重新建立基准。
    ++statisticsArchive.gyroGaps;
    return;
  }

  float gyroBody[3]{};
  alg_estimate::GyroRecordToBody(record, gyroBody);
  alg_math::Vector3 gyro{gyroBody[0], gyroBody[1], gyroBody[2]};

  // 上电静止标定使用未经修正的角速度；加速度样本仅作为静止门控。
  calibration.Process(gyro, accelAvailable ? accelBody : alg_math::Vector3{},
                      temperatureCelsius, record.drdyTick);
  if (!calibrationInjected && calibration.IsBiasValid()) {
    const alg_math::Vector3 bias = calibration.GetBiasRadPerSec();
    // x/y 作为滤波器初值，z 在输入侧扣除：两者不重复抵消。
    estimator.SeedGyroBias(bias);
    gyroZTrimRadPerSec = bias.z;
    calibrationInjected = true;
  }
  if (calibrationInjected) {
    gyro.z -= gyroZTrimRadPerSec;
  }

  const std::uint32_t accelAgeTicks = record.drdyTick - lastAccelTick;
  const float accelAgeSeconds = platform::Time::TicksToSeconds(accelAgeTicks);
  if (accelAvailable &&
      (accelFresh ||
       accelAgeSeconds <= AttitudeEstimator::AccelerometerHoldLimitSeconds)) {
    if (!accelFresh) {
      ++statisticsArchive.accelHoldReuse; // 沿用上一条加速度样本
    } else {
      ++statisticsArchive.accelConsumed;
      accelFresh = false;
    }
    ++statisticsArchive.gyroConsumed;
    estimator.Update(gyro, accelBody, stepSeconds);
  } else {
    // 没有可信加速度：只预测，并把过期样本标记出来。
    ++statisticsArchive.accelStale;
    ++statisticsArchive.predictOnly;
    estimator.Predict(gyro, stepSeconds);
  }
  PublishOutput(record);
}

void ProcessAccelerometer(const device::Bmi088SampleRecord &record) noexcept {
  if (record.sequence <= lastAccelSequence) {
    ++statisticsArchive.rejectedSequence;
    return;
  }
  if (accelAvailable && accelFresh) {
    // 上一条加速度样本还没被陀螺节拍消费就被覆盖。
    ++statisticsArchive.pendingOverwritten;
  }
  lastAccelSequence = record.sequence;
  lastAccelTick = record.drdyTick;
  float accel[3]{};
  alg_estimate::AccelRecordToBody(record, accel);
  accelBody = alg_math::Vector3{accel[0], accel[1], accel[2]};
  accelAvailable = true;
  accelFresh = true;
}

} // namespace

bool AttitudeEstimator::Init() noexcept {
  // 参数使用移植来源的默认整定：Q1=10、Q2=0.001、R=1e7、渐消关闭。
  config = alg_estimate::QuaternionEkf::Config{};
  ready = estimator.Init(config);

  GyroBiasCalibration::Config calibrationConfig{};
  calibrationConfig.tickFrequencyHz = platform::Time::TickFrequencyHz();
  calibration.Init(calibrationConfig);

  calibrationInjected = false;
  gyroZTrimRadPerSec = 0.0F;
  gyroAnchored = false;
  lastGyroTick = 0U;
  lastGyroSequence = 0U;
  lastAccelSequence = 0U;
  lastAccelTick = 0U;
  accelAvailable = false;
  accelFresh = false;
  accelBody = alg_math::Vector3{};
  ResetArchives();
  return ready;
}

bool AttitudeEstimator::IsReady() noexcept { return ready; }

void AttitudeEstimator::Process(const device::Bmi088SampleRecord &record,
                                bool gyroscope,
                                float temperatureCelsius) noexcept {
  if (gyroscope) {
    ++statisticsArchive.gyroReceived;
  } else {
    ++statisticsArchive.accelReceived;
  }

  if (!ready) {
    return;
  }
  if (HasUncertainTiming(record)) {
    ++statisticsArchive.rejectedFlags;
    return;
  }

  if (gyroscope) {
    ProcessGyroscope(record, temperatureCelsius);
  } else {
    ProcessAccelerometer(record);
  }
}

AttitudeEstimator::Output AttitudeEstimator::GetOutput() noexcept {
  return ReadArchive();
}

AttitudeEstimator::Statistics AttitudeEstimator::GetStatistics() noexcept {
  Statistics result{};
  std::uint8_t *bytes = reinterpret_cast<std::uint8_t *>(&result);
  const volatile std::uint8_t *source =
      reinterpret_cast<const volatile std::uint8_t *>(&statisticsArchive);
  for (std::size_t index = 0U; index < sizeof(result); ++index) {
    bytes[index] = source[index];
  }
  return result;
}

} // namespace application
