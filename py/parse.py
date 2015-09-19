import numpy as np
import struct


# yield video frames from log
def ParseLog(f):
    while not f.closed:
        header = f.read(16)
        sec, usec, length, recordtype = struct.unpack("IIII", header)
        ts = sec + usec * 1e-6
        if length > 320*360:
            raise Exception("bad length in header")
        buf = f.read(length)
        if recordtype == 1:
            yield np.frombuffer(buf[:320*240][::-1], np.uint8).reshape(
                (240, 320))
