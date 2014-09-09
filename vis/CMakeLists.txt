cmake_minimum_required(VERSION 3.0)
    
include(FindSDL)
include_directories(${SDL_INCLUDE_DIR})

add_executable(vis vis.cc)
target_link_libraries(vis ${SDL_LIBRARY})
