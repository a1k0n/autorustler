add_executable(drive drive.cc)
target_link_libraries(drive car cam mmal input gpio imu)
