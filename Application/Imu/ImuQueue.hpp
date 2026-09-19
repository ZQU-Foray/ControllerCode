#ifndef APPLICATION_IMU_IMU_QUEUE_HPP
#define APPLICATION_IMU_IMU_QUEUE_HPP

#include "Libraries/Protocol/imu/ImuFrame.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace application {

/**
 * @brief 单生产者-单消费者的定长原始样本环形队列。
 * @note 生产者（IMU 任务）只写 head 并读 tail，消费者（通信任务）只写 tail
 *       并读 head；head/tail 为单调递增的 32 位计数，按容量取模定位。
 * @note 队列满时丢弃最新样本并累加计数，绝不覆盖尚未发送的旧样本，也不阻塞
 *       生产者。容量必须是 2 的幂。
 */
template <std::size_t Capacity> class ImuQueue final {
public:
  using Entry = protocol::ImuFrame::Sample;

  static_assert(Capacity > 0U, "queue capacity must be non-zero");
  static_assert((Capacity & (Capacity - 1U)) == 0U,
                "queue capacity must be a power of two");

  ImuQueue() noexcept = default;

  /**
   * @brief 清空队列并复位计数。只允许在生产者尚未运行时调用。
   */
  void Reset() noexcept {
    head_.store(0U, std::memory_order_relaxed);
    tail_.store(0U, std::memory_order_relaxed);
    dropped_.store(0U, std::memory_order_relaxed);
    pushed_.store(0U, std::memory_order_relaxed);
  }

  /**
   * @brief 生产者入队。常数时间；满时丢弃并计数。
   * @return 成功入队返回 true；队列满丢弃返回 false。
   */
  bool Push(const Entry &entry) noexcept {
    const std::uint32_t head = head_.load(std::memory_order_relaxed);
    const std::uint32_t tail = tail_.load(std::memory_order_acquire);
    if ((head - tail) >= static_cast<std::uint32_t>(Capacity)) {
      dropped_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }

    entries_[head & Mask] = entry;
    head_.store(head + 1U, std::memory_order_release);
    pushed_.fetch_add(1U, std::memory_order_relaxed);
    return true;
  }

  /**
   * @brief 消费者窥视队首若干条，不推进 tail。
   * @param entries 输出数组。
   * @param maxCount entries 可容纳的条数。
   * @return 实际写入的条数。
   */
  std::size_t Peek(Entry *entries, std::size_t maxCount) const noexcept {
    if (entries == nullptr || maxCount == 0U) {
      return 0U;
    }

    const std::uint32_t head = head_.load(std::memory_order_acquire);
    const std::uint32_t tail = tail_.load(std::memory_order_relaxed);
    std::uint32_t available = head - tail;
    if (available > maxCount) {
      available = static_cast<std::uint32_t>(maxCount);
    }

    for (std::uint32_t index = 0U; index < available; ++index) {
      entries[index] = entries_[(tail + index) & Mask];
    }
    return available;
  }

  /**
   * @brief 消费者确认已成功发送的条数并推进 tail。
   * @param count 必须不超过上一次 Peek 返回的条数。
   */
  void Commit(std::size_t count) noexcept {
    const std::uint32_t tail = tail_.load(std::memory_order_relaxed);
    tail_.store(tail + static_cast<std::uint32_t>(count),
                std::memory_order_release);
  }

  [[nodiscard]] std::size_t Size() const noexcept {
    const std::uint32_t head = head_.load(std::memory_order_acquire);
    const std::uint32_t tail = tail_.load(std::memory_order_acquire);
    return head - tail;
  }

  [[nodiscard]] std::uint32_t DroppedCount() const noexcept {
    return dropped_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::uint32_t PushedCount() const noexcept {
    return pushed_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] static constexpr std::size_t GetCapacity() noexcept {
    return Capacity;
  }

private:
  static constexpr std::uint32_t Mask{
      static_cast<std::uint32_t>(Capacity - 1U)};

  std::array<Entry, Capacity> entries_{};
  alignas(64) std::atomic<std::uint32_t> head_{0U};
  alignas(64) std::atomic<std::uint32_t> tail_{0U};
  alignas(64) std::atomic<std::uint32_t> dropped_{0U};
  alignas(64) std::atomic<std::uint32_t> pushed_{0U};
};

} // namespace application

#endif
