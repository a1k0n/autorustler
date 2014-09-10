cmake_minimum_required(VERSION 2.8)

include(FindSDL)
find_library(LIBOPENCVCORE NAMES opencv_core)
find_library(LIBOPENCV2D NAMES opencv_features2d)

add_executable(vis vis.cc)
target_link_libraries(vis ${SDL_LIBRARY} ${LIBOPENCVCORE} ${LIBOPENCV2D})
