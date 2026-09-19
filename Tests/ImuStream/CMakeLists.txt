cmake_minimum_required(VERSION 3.22)
project(imu_stream_tests LANGUAGES CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(ROOT "../.." ABSOLUTE)
add_compile_options(-Wall -Wextra -Werror -UNDEBUG)
include_directories("${ROOT}")

set(CODEC
    "${ROOT}/Libraries/Protocol/imu/ImuFrame.cpp"
    "${ROOT}/Libraries/Algorithm/alg_crc/Crc32.cpp")
set(STREAM
    "${ROOT}/Application/Imu/ImuStream.cpp"
    ${CODEC})

add_executable(imu_frame ImuFrameTest.cpp ${CODEC})
add_test(NAME imu_frame COMMAND imu_frame)

add_executable(imu_stream ImuStreamTest.cpp ${STREAM})
add_test(NAME imu_stream COMMAND imu_stream)
