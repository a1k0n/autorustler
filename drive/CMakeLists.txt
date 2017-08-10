add_executable(drive drive.cc imgproc.cc controller.cc ekf.cc)
target_link_libraries(drive car cam mmal input gpio imu)

add_executable(controltest controller.cc controltest.cc ekf.cc)

# add_executable(pfilt pfilt.cc)
