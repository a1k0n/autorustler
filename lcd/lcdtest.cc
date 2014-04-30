#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "./gpio.h"

class SPIDev {
 public:
  SPIDev() { fd = -1; }
  ~SPIDev() {
    if (fd != -1)
      close(fd);
  }

  bool open(const char *devname) {
    fd = ::open(devname, O_RDWR);
    if (fd == -1) {
      perror(devname);
      return false;
    }
    uint8_t spi_mode;
    uint8_t bits;
    uint32_t speed;
    // TODO: set SPI_CPHA | SPI_CPOL if necessary?
    int ret = ioctl(fd, SPI_IOC_RD_MODE, &spi_mode);
    if (ret == -1) {
      perror("can't read spi mode");
      return false;
    }

    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1) {
      perror("can't read spi wordlen");
      return false;
    }
    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1) {
      perror("can't get max speed hz");
      return false;
    }
    fprintf(stderr, "%s SPI mode %02x bits %d speed %d\n",
            devname, spi_mode, bits, speed);
    return true;
  }

  int xfer(const uint8_t txbuf[], uint8_t *rxbuf, int len) {
    int ret;
    struct spi_ioc_transfer tr;
    tr.tx_buf = (unsigned long)txbuf;
    tr.rx_buf = (unsigned long)rxbuf;
    tr.len = len;
    tr.delay_usecs = 10;
    tr.speed_hz = 500000;
    tr.bits_per_word = 8;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
      perror("can't send spi message");
      return -1;
    }

    return ret;
  }

 private:
  int fd;
};

const int LCD_RST_GPIO = 27;
const int LCD_DC_GPIO = 17;

// reverse bits
uint8_t brev(uint8_t x) {
  uint8_t y = 0;
  for (int i = 0; i < 8; i++) {
    y = (y<<1) + (x&1);
    x >>= 1;
  }
  return y;
}

void lcd_init(SPIDev *spi) {
  // nothing is physically connected to the receive pin, so this is just a
  // scratch buffer
  uint8_t rxbuf[32];
  uint8_t txbuf[32];
  INP_GPIO(LCD_RST_GPIO);
  OUT_GPIO(LCD_RST_GPIO);
  OUT_GPIO(LCD_RST_GPIO);
  INP_GPIO(LCD_DC_GPIO);
  OUT_GPIO(LCD_DC_GPIO);
  OUT_GPIO(LCD_DC_GPIO);
  GPIO_CLR = 1 << LCD_RST_GPIO;
  GPIO_CLR = 1 << LCD_RST_GPIO;
  usleep(10);
  GPIO_SET = 1 << LCD_RST_GPIO;
  GPIO_SET = 1 << LCD_RST_GPIO;
  usleep(10);

  GPIO_CLR = 1 << LCD_DC_GPIO; // command mode
  txbuf[0] = 0x21;  // turn on chip, horizontal addressing, extended insn set
  txbuf[1] = 0xB8;  // set LCD Vop
  txbuf[2] = 0x04;  // set temp coeff
  txbuf[3] = 0x14;  // LCD bias mode
  txbuf[4] = 0x20;  // turn on chip, horizontal addressing, basic insn set
  txbuf[5] = 0x0c;  // normal mode (not blank or all on)
  txbuf[6] = 0x40;  // y = 0
  txbuf[7] = 0x80;  // x = 0
  spi->xfer(txbuf, rxbuf, 8);

  GPIO_SET = 1 << LCD_DC_GPIO; // set data mode
  GPIO_SET = 1 << LCD_DC_GPIO; // extra write
  for (int i = 0; i < 32; i++)
    txbuf[i] = i;
  spi->xfer(txbuf, rxbuf, 32);  // write some junk on the screen
}

int main() {
  if (!gpio_init())
    return -1;
  SPIDev spi;
  if (!spi.open("/dev/spidev0.1"))
    return -1;

  lcd_init(&spi);
}
