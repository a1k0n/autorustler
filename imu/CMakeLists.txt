add_library(imu imu.cc)

add_executable(imu_log imu_main.cc imu.cc)
target_link_libraries(imu_log gpio)
