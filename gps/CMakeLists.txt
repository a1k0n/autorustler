# add_executable(ubx ubx_main.cc ubx.cc)
add_executable(sirf_log sirf_main.cc sirf.cc)
add_library(sirf sirf.cc)

add_executable(ubx_main ubx_main.cc ubx.cc ubx.h)
