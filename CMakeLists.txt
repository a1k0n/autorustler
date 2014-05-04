add_executable(ui ui.cc lcd/gpio.cc lcd/spi.cc lcd/font.cc lcd/lcd.cc
    imu/imu.cc imu/i2c.cc radio.cc)

add_executable(log log.cc lcd/spi.cc imu/imu.cc imu/i2c.cc radio.cc)
