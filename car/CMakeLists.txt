add_library(car pca9685.cc)

add_executable(servotest servotest.cc)
target_link_libraries(servotest car gpio)
