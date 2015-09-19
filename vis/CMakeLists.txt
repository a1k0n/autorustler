cmake_minimum_required(VERSION 2.8)

include(FindSDL)
find_package(PkgConfig)
find_package(OpenCV REQUIRED)
find_package(fast REQUIRED)
find_package(vikit_common REQUIRED)
pkg_check_modules(EIGEN REQUIRED eigen3)

include_directories(${EIGEN_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${fast_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${vikit_common_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(..)

add_executable(parse parse.cc)
target_link_libraries(parse ${OpenCV_LIBS} ${fast_LIBRARIES}
  ${vikit_common_LIBRARIES})

add_executable(vis vis.cc)
target_link_libraries(vis ${SDL_LIBRARY}
  ${OpenCV_LIBS} ${fast_LIBRARIES} ${vikit_common_LIBRARIES})

#add_executable(tracktest tracktest.cc)
#target_link_libraries(tracktest ${SDL_LIBRARY}
#  ${OpenCV_LIBS} ${fast_LIBRARIES} ${vikit_common_LIBRARIES})
