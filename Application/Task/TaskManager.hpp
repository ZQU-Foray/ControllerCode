#ifndef APPLICATION_TASK_TASK_MANAGER_HPP
#define APPLICATION_TASK_TASK_MANAGER_HPP

#include <cstdint>

namespace application::task {

/**
 * @brief 应用任务生命周期管理器。
 * @note Init 在内核启动前调用，Start 只能在 RTOS 进入 Running 后调用。
 */
class TaskManager final {
public:
  enum class State : std::uint8_t {
    Uninitialized,
    Initialized,
    Running,
    Error
  };

  TaskManager() = delete;

  [[nodiscard]] static bool Init() noexcept;
  [[nodiscard]] static bool Start() noexcept;
  [[nodiscard]] static State GetState() noexcept;
};

} //  application::task

#endif
