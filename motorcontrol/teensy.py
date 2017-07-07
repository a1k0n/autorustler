# interface w/ teensy code via i2c
import smbus2

bus = smbus2.SMBus(1)

TEENSY_ADDR = 118


def set_controls(esc, srv):
    ''' set ESC and servo controls via i2c; values are -127 to +127 '''
    bus.write_i2c_block_data(TEENSY_ADDR, 1, [esc, srv])


def get_data():
    ''' returns servo feedback value (uint8) and four wheel encoder values
    (uint16) '''

    data = bus.read_i2c_block_data(TEENSY_ADDR, 3, 9)
    srvpos = data[0]
    encoders = [0, 0, 0, 0]
    for i in range(4):
        encoders[i] = data[1+2*i] + (data[2+2*i] << 8)

    return srvpos, encoders
