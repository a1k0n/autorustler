add_library(lcd lcd.cc font.cc)
target_link_libraries(lcd gpio)
# add_executable(lcdtest lcdtest.cc gpio.cc spi.cc lcd.cc font.cc)
