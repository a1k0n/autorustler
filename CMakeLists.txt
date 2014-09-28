cmake_minimum_required(VERSION 2.8)

add_subdirectory(userland)

project(Autorustler)

add_executable(ui ui.cc)
target_link_libraries(ui car lcd imu gpio)

add_executable(log log.cc)
target_link_libraries(log car imu gpio)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(userland)
include_directories(userland/interface/vcos/pthreads)
include_directories(userland/interface/vmcs_host/linux)
include_directories(userland/host_applications/linux/libs/bcm_host/include)

add_subdirectory(cam)
add_subdirectory(car)
add_subdirectory(gpio)
add_subdirectory(gps)
add_subdirectory(imu)
add_subdirectory(lcd)
