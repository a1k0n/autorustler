import spidev
import time
import sys

spi = spidev.SpiDev()
spi.open(0, 0)

spi.bits_per_word = 8
#spi.max_speed_hz = 1500000
spi.max_speed_hz = 400000

while True:
    #print ['%02x' % x for x in spi.xfer2([0x01, 0, 0]*4)]
    data = spi.xfer2([0x02, 0, 0, 0, 0])
    ch1 = data[1] + (data[2]<<8)
    ch2 = data[3] + (data[4]<<8)
    sys.stdout.write("\r%5d %5d" % (ch1, ch2))
    sys.stdout.flush()
    time.sleep(0.2)
