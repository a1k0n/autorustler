
import spidev
spi = spidev.SpiDev()
spi.open(0, 0)

spi.bits_per_word = 8
spi.max_speed_hz = 500000

print map(chr,spi.xfer2([0]*13))
