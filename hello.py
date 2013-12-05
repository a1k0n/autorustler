
import spidev
spi = spidev.SpiDev()
spi.open(0, 0)

spi.bits_per_word = 8
spi.max_speed_hz = 1500000

print ['%02x' % x for x in spi.xfer2([0x01, 0, 0]*4)]
