add_executable(ui ui.cc gps.cc cam.cc imu.cc recording.cc)
target_link_libraries(ui car lcd imu sirf cam gpio pthread)
