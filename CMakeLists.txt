cmake_minimum_required(VERSION 2.8)
find_package(PkgConfig)

add_subdirectory(userland)

project(Autorustler)

pkg_check_modules(EIGEN REQUIRED eigen3)

# add_executable(log log.cc)
# target_link_libraries(log car imu gpio)

# add_executable(cal cal.cc)
# target_link_libraries(cal imu gpio)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(userland)
include_directories(userland/interface/vcos/pthreads)
include_directories(userland/interface/vmcs_host/linux)
include_directories(userland/host_applications/linux/libs/bcm_host/include)

add_subdirectory(cam)
add_subdirectory(car)
add_subdirectory(gpio)
add_subdirectory(input)
# add_subdirectory(gps)
add_subdirectory(imu)
# add_subdirectory(lcd)
add_subdirectory(drive)

add_executable(rc rc.cc)
target_link_libraries(rc car input gpio)
