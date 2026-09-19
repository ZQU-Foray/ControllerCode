cmake_minimum_required(VERSION 3.22)
project(imu_acquisition_tests LANGUAGES CXX)
enable_testing()
find_package(Threads REQUIRED)
add_executable(bmi088_acquisition_test bmi088_acquisition_test.cpp
  ../../Libraries/Device/bmi088/Bmi088.cpp
  ../../Libraries/Device/bmi088/Bmi088Accel.cpp
  ../../Libraries/Device/bmi088/Bmi088Gyro.cpp
  ../../Libraries/Device/bmi088/Bmi088Temperature.cpp)
target_include_directories(bmi088_acquisition_test PRIVATE ../..)
target_compile_features(bmi088_acquisition_test PRIVATE cxx_std_17)
target_compile_options(bmi088_acquisition_test PRIVATE -O2 -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(bmi088_acquisition_test PRIVATE Threads::Threads)
add_test(NAME acquisition COMMAND bmi088_acquisition_test)
