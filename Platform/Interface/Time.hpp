#ifndef PLATFORM_INTERFACE_TIME_HPP
#define PLATFORM_INTERFACE_TIME_HPP

#include "Detail/Time.h"
#include <cstdint>
#include <limits>

namespace platform {

class Time final {
public:
  using Tick = std::uint32_t;

  Time() = delete;

  [[nodiscard]] static bool IsReady() noexcept { return TimePort_IsReady(); }

  [[nodiscard]] static Tick NowTicks() noexcept { return TimePort_NowTicks(); }

  [[nodiscard]] static std::uint32_t TickFrequencyHz() noexcept {
    return TimePort_FrequencyHz();
  }

  [[nodiscard]] static float TicksToSeconds(Tick elapsedTicks) noexcept {
    const std::uint32_t frequencyHz = TickFrequencyHz();
    if (frequencyHz == 0U) {
      return 0.0F;
    }

    return static_cast<float>(elapsedTicks) / static_cast<float>(frequencyHz);
  }

  [[nodiscard]] static float ElapsedSeconds(Tick startTick) noexcept {
    return TicksToSeconds(NowTicks() - startTick);
  }

  // 计算从 previousTick 到当前时刻的时间差，并将 previousTick
  // 更新为当前计数值
  [[nodiscard]] static float DeltaSeconds(Tick &previousTick) noexcept {
    const Tick currentTick = NowTicks();
    const float deltaSeconds = TicksToSeconds(currentTick - previousTick);
    previousTick = currentTick;
    return deltaSeconds;
  }

  [[nodiscard]] static bool HasElapsedMs(Tick startTick,
                                         std::uint32_t timeoutMs) noexcept {
    const std::uint32_t frequencyHz = TickFrequencyHz();
    if (frequencyHz == 0U) {
      return false;
    }

    if (timeoutMs == 0U) {
      return true;
    }

    constexpr std::uint64_t millisecondsPerSecond = 1000ULL;
    const std::uint64_t timeoutTicks =
        (static_cast<std::uint64_t>(timeoutMs) * frequencyHz +
         millisecondsPerSecond - 1ULL) /
        millisecondsPerSecond;

    if (timeoutTicks > std::numeric_limits<Tick>::max()) {
      return false;
    }

    return static_cast<Tick>(NowTicks() - startTick) >=
           static_cast<Tick>(timeoutTicks);
  }

  static void DelayUs(std::uint32_t delayUs) noexcept {
    TimePort_DelayUs(delayUs);
  }
};

} // namespace platform

#endif
