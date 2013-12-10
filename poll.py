import spidev
import time
import sys

spi = spidev.SpiDev()
spi.open(0, 0)

spi.bits_per_word = 8
#spi.max_speed_hz = 1500000
spi.max_speed_hz = 400000

lastch1, lastch2 = None, None
while True:
    #print ['%02x' % x for x in spi.xfer2([0x01, 0, 0]*4)]
    data = spi.xfer2([0x02, 0])
    readlen = data[1]
    if readlen > 0:
        data2 = spi.xfer2([0]*readlen*2)
        ch1 = data2[(readlen-1)*2]
        ch2 = data2[(readlen-1)*2+1]
        if ch1 != lastch1 or ch2 != lastch2:
            sys.stdout.write("\r%3d %3d" % (ch1, ch2))
            sys.stdout.flush()
            lastch1, lastch2 = ch1, ch2
    time.sleep(0.01)
