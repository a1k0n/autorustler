autorustler
===========

My work-in-progress self-driving RC car.

Most host code runs on a Raspberry Pi; some runs on an Atmel AVR connected to
it via SPI.

To build on a host with a raspberry pi cross-compiler (mine's called
arm-none-linux-gnueabi-gcc, you may need to edit crosscompile.cmake):

    mkdir build
    cd build
    cmake -DCMAKE_TOOLCHAIN_FILE=../crosscompile.cmake ..
    make -j4

Building on a raspberry pi is more straightforward but slow.
