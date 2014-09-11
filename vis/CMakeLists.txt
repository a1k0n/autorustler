cmake_minimum_required(VERSION 2.8)

include(FindSDL)
find_package(OpenCV REQUIRED)

add_executable(vis vis.cc)
target_link_libraries(vis ${SDL_LIBRARY} opencv_core opencv_features2d opencv_calib3d)
