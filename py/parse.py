import numpy as np
import struct


# yield video and IMU frames from log
def ParseLog(f):
    while not f.closed:
        header = f.read(16)
        if len(header) < 16:
            break
        sec, usec, length, recordtype = struct.unpack("IIII", header)
        ts = sec + usec * 1e-6
        if length > 320*360:
            raise Exception("bad length in header")
        buf = f.read(length)
        if recordtype == 1:  # VideoFrame
            yield ('img', ts, np.frombuffer(
                buf[:320*240][::-1], np.uint8).reshape((240, 320)))
        elif recordtype == 2:  # IMUFrame
            data = np.frombuffer(buf, np.int16)
            gyro = data[0:4]
            mag = data[4:7]
            accel = data[7:10]
            yield ('imu', ts, gyro, mag, accel)
