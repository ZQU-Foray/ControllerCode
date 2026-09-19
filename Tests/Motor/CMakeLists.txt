cmake_minimum_required(VERSION 3.22)
project(motor_tests LANGUAGES CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(ROOT "../.." ABSOLUTE)
add_compile_options(-Wall -Wextra -Werror -UNDEBUG)
include_directories("${ROOT}")
set(PROTOCOL "${ROOT}/Libraries/Protocol/dji/DjiEsc.cpp")
set(DJI "${ROOT}/Libraries/Device/motor/dji/DjiMotor.cpp")
set(ADAPTER "${ROOT}/Libraries/Device/motor/dji/DjiMotorAdapter.cpp")
set(CONTROL "${ROOT}/Application/Task/1_chassis/ChassisController.cpp"
    "${ROOT}/Libraries/Algorithm/alg_controller/Pid.cpp")
set(TASK "${ROOT}/Application/Task/1_chassis/ChassisTask.cpp")
function(motor_test name)
  add_executable(${name} ${ARGN})
  add_test(NAME ${name} COMMAND ${name})
endfunction()
function(mock_backend name)
  target_sources(${name} PRIVATE Support/MockMotor.cpp ${TASK})
  target_include_directories(${name} BEFORE PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/Support")
endfunction()
motor_test(dji_protocol "${ROOT}/Tests/Protocol/DjiEscTest.cpp" ${PROTOCOL})
motor_test(dji_motor "${ROOT}/Tests/Device/DjiMotorTest.cpp" ${DJI} ${PROTOCOL})
motor_test(chassis_controller "${ROOT}/Tests/Application/ChassisControllerTest.cpp"
    Support/TaskRuntimeStub.cpp ${CONTROL})
mock_backend(chassis_controller)
motor_test(dji_adapter DjiMotorAdapterTest.cpp ${ADAPTER} ${DJI} ${PROTOCOL})
motor_test(angle_wrap AngleWrapTest.cpp ${DJI} ${PROTOCOL})
motor_test(controller_equivalence ControllerEquivalenceTest.cpp Support/TaskRuntimeStub.cpp ${CONTROL})
mock_backend(controller_equivalence)
motor_test(motor_group MotorGroupTest.cpp ${ADAPTER} ${DJI} ${PROTOCOL})
motor_test(brand_extension BrandExtensionTest.cpp)
add_executable(chassis_mock ChassisMockTest.cpp ${CONTROL})
mock_backend(chassis_mock)
foreach(scenario normal read_failure command_failure init_failure)
  add_test(NAME mock_${scenario} COMMAND chassis_mock ${scenario})
endforeach()

# 任务允许声明通用型号、电机组与品牌映射包含（装配选择），但不能直接
# 引用具体协议、适配器或驱动实现。
file(GLOB boundary_files
    "${ROOT}/Application/Task/1_chassis/*.hpp"
    "${ROOT}/Application/Task/1_chassis/*.cpp"
    "${ROOT}/Libraries/Device/motor/Motor.hpp"
    "${ROOT}/Libraries/Device/motor/MotorGroup.hpp")
foreach(source IN LISTS boundary_files)
  file(READ "${source}" body)
  if(body MATCHES "DjiEsc|DjiMotorAdapter|DjiMotorBackend|DjiMotor\\.hpp|DjiMotor\\.cpp")
    message(FATAL_ERROR "Concrete motor dependency leaked into ${source}")
  endif()
endforeach()

# 电机数量与型号在编译期校验，替代旧的运行期 wrong_count 场景。
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_INCLUDES "${ROOT}")
check_cxx_source_compiles("
#include <Libraries/Device/motor/dji/DjiMotorModelMap.hpp>
#include <Libraries/Device/motor/MotorGroup.hpp>
static_assert(sizeof(device::MotorGroup<device::MotorModel::M2006, 4>) > 0);
int main() {}" MOTOR_GROUP_VALID)
if(NOT MOTOR_GROUP_VALID)
  message(FATAL_ERROR "Valid motor group must compile")
endif()
function(reject_group result model count)
  check_cxx_source_compiles("
#include <Libraries/Device/motor/dji/DjiMotorModelMap.hpp>
#include <Libraries/Device/motor/MotorGroup.hpp>
static_assert(sizeof(device::MotorGroup<${model}, ${count}>) > 0);
int main() {}" ${result})
  if(${result})
    message(FATAL_ERROR "Unsupported motor group unexpectedly compiled: ${result}")
  endif()
endfunction()
reject_group(MOTOR_ZERO_COMPILES device::MotorModel::M2006 0)
reject_group(MOTOR_NINE_COMPILES device::MotorModel::M2006 9)
reject_group(GIMBAL_EIGHT_COMPILES device::MotorModel::GM6020Current 8)
# 未映射型号：即使已包含 DJI 映射，255 仍无特化，必须在编译期拒绝。
reject_group(UNKNOWN_MODEL_COMPILES "static_cast<device::MotorModel>(255)" 1)
# 未包含任何品牌映射时，合法型号同样无法实例化（映射是唯一入口）。
check_cxx_source_compiles("
#include <Libraries/Device/motor/MotorGroup.hpp>
static_assert(sizeof(device::MotorGroup<device::MotorModel::M2006, 4>) > 0);
int main() {}" MAPPING_REQUIRED_COMPILES)
if(MAPPING_REQUIRED_COMPILES)
  message(FATAL_ERROR "MotorGroup compiled without a brand mapping")
endif()
