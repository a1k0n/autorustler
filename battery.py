import spidev
import time
import sys

spi = spidev.SpiDev()
spi.open(0, 0)

spi.bits_per_word = 8
#spi.max_speed_hz = 1500000
spi.max_speed_hz = 400000

data = spi.xfer2([0x05, 0, 0])
adc = (data[2]<<8) + data[1]
# adc = 1024*battery*r2/((r1+r2) * 1.1)
r1 = 78.85
r2 = 10.01
battery = (r1+r2)*1.1*adc/(1024.0*r2)

print data, adc, battery
