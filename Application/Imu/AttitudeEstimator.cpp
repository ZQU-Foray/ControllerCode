#include "Application/Imu/AttitudeEstimator.hpp"
#include "Application/Imu/ZeroRateBiasObserver.hpp"
#include "Libraries/Algorithm/alg_estimate/ImuBodyMapping.hpp"
#include "Libraries/Algorithm/alg_estimate/QuaternionEkf.hpp"
#include "Platform/Interface/Time.hpp"
#include <cstddef>

namespace application {

namespace {

alg_estimate::QuaternionEkf estimator;
alg_estimate::QuaternionEkf::Config config{};
GyroBiasCalibration calibration;
// 零速零偏观测器（ZARU）：静止时用"ω=0"持续修正 z 轴零偏。
ZeroRateBiasObserver zeroRate;
bool ready{false};

// 当前实际从 z 轴扣掉的零偏，仅供诊断存档。
float appliedZTrimRadPerSec{0.0F};
// 上一次发布零速观测器存档时的窗口计数，用于按窗口（约 1 Hz）刷新。
std::uint32_t lastZeroRateWindowCount{0U};

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

/**
 * @brief 零速观测器诊断存档，与 Output/Statistics 分开存放。
 *
 * 单独开一块是为了不动既有 Output/Statistics 的布局（调试器与工具
 * 依赖它们的字节偏移），同时把 ZARU 的判据统计与估计值暴露出来，
 * 便于先跑 monitor-only 校验判据是否会误触发。
 */
constexpr std::uint32_t ZeroRateArchiveVersion{1U};

struct ZeroRateArchive final {
  std::uint32_t version{ZeroRateArchiveVersion};
  std::uint32_t flags{0U}; // bit0 当前窗口判据合格；bit1 正在用观测值修正
  float biasRadPerSec{0.0F};
  float varianceRadPerSec2{0.0F};
  float appliedTrimRadPerSec{0.0F};
  float windowMeanRadPerSec[3]{};
  float windowStdRadPerSec[3]{};
  float accelNormMeanMps2{0.0F};
  float accelNormStdMps2{0.0F};
  std::uint32_t staticMs{0U};
  std::uint32_t qualifiedWindows{0U};
  std::uint32_t observationCount{0U};
  std::uint32_t rejectedWindows{0U};
};
static_assert(sizeof(ZeroRateArchive) == 68U, "零速观测器存档布局必须稳定");
volatile ZeroRateArchive zeroRateArchive{};

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
  ZeroRateArchive zeroRateState{};
  const std::uint8_t *zeroRateBytes =
      reinterpret_cast<const std::uint8_t *>(&zeroRateState);
  volatile std::uint8_t *zeroRateTarget =
      reinterpret_cast<volatile std::uint8_t *>(&zeroRateArchive);
  for (std::size_t index = 0U; index < sizeof(zeroRateState); ++index) {
    zeroRateTarget[index] = zeroRateBytes[index];
  }
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

/** 发布零速观测器诊断存档；布局与 ZeroRateArchive 一致。 */
void PublishZeroRate() noexcept {
  const ZeroRateBiasObserver::Snapshot snapshot = zeroRate.GetSnapshot();
  ZeroRateArchive archive{};
  archive.version = ZeroRateArchiveVersion;
  archive.flags = (snapshot.staticQualified ? 1U : 0U) |
                  (snapshot.correctionApplied ? 2U : 0U);
  archive.biasRadPerSec = snapshot.biasRadPerSec;
  archive.varianceRadPerSec2 = snapshot.varianceRadPerSec2;
  archive.appliedTrimRadPerSec = appliedZTrimRadPerSec;
  for (int axis = 0; axis < 3; ++axis) {
    archive.windowMeanRadPerSec[axis] = snapshot.windowMeanRadPerSec[axis];
    archive.windowStdRadPerSec[axis] = snapshot.windowStdRadPerSec[axis];
  }
  archive.accelNormMeanMps2 = snapshot.accelNormMeanMps2;
  archive.accelNormStdMps2 = snapshot.accelNormStdMps2;
  archive.staticMs = snapshot.staticMs;
  archive.qualifiedWindows = snapshot.qualifiedWindows;
  archive.observationCount = snapshot.observationCount;
  archive.rejectedWindows = snapshot.rejectedWindows;

  const std::uint8_t *bytes = reinterpret_cast<const std::uint8_t *>(&archive);
  volatile std::uint8_t *target =
      reinterpret_cast<volatile std::uint8_t *>(&zeroRateArchive);
  for (std::size_t index = 0U; index < sizeof(archive); ++index) {
    target[index] = bytes[index];
  }
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

  // 零速观测器与上电静止标定都使用未经修正的角速度；加速度样本仅作为静止门控。
  const alg_math::Vector3 gateAccel =
      accelAvailable ? accelBody : alg_math::Vector3{};
  calibration.Process(gyro, gateAccel, temperatureCelsius, record.drdyTick);
  zeroRate.Process(gyro, gateAccel, record.drdyTick);
  if (!calibrationInjected && calibration.IsBiasValid()) {
    const alg_math::Vector3 bias = calibration.GetBiasRadPerSec();
    // x/y 作为滤波器初值；z 只作为观测器尚未给出估计时的回退值。
    estimator.SeedGyroBias(bias);
    gyroZTrimRadPerSec = bias.z;
    calibrationInjected = true;
  }
  // z 轴零偏修正优先级：零速观测器（持续跟踪）→ 上电标定值 → 不修正。
  const float zTrim = zeroRate.IsCorrectionApplied()
                          ? zeroRate.GetBiasRadPerSec()
                          : (calibrationInjected ? gyroZTrimRadPerSec : 0.0F);
  appliedZTrimRadPerSec = zTrim;
  gyro.z -= zTrim;

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
  // 零速观测器的诊断量只在窗口结束时变化（约 1 Hz），不必每拍都拷贝存档。
  const std::uint32_t zeroRateWindows = zeroRate.GetWindowCount();
  if (zeroRateWindows != lastZeroRateWindowCount) {
    lastZeroRateWindowCount = zeroRateWindows;
    PublishZeroRate();
  }
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

  ZeroRateBiasObserver::Config zeroRateConfig{};
  zeroRateConfig.tickFrequencyHz = platform::Time::TickFrequencyHz();
  zeroRate.Init(zeroRateConfig);

  calibrationInjected = false;
  gyroZTrimRadPerSec = 0.0F;
  appliedZTrimRadPerSec = 0.0F;
  lastZeroRateWindowCount = zeroRate.GetWindowCount();
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
