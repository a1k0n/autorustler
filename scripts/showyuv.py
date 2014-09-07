import matplotlib.pyplot as plt
# import matplotlib.cm as cm
import numpy as np
import sys


def clamp(x):
    return min(255, max(0, x))

w = 320
data = map(ord, open(sys.argv[1]).read())
h = len(data) / 320
img = np.zeros((240, 320, 3), dtype='uint8')
print img[0][0]
for j in range(240):
    for i in range(320):
        c = data[j*320 + i] - 16
        d = data[320*240 + (j//2)*160 + i//2] - 128
        e = data[320*300 + (j//2)*160 + i//2] - 128
        img[239-j][i][0] = clamp((298*c + 409*e + 128) >> 8)
        img[239-j][i][1] = clamp((298*c - 100*d - 208*e + 128) >> 8)
        img[239-j][i][2] = clamp((298*c + 516*d + 128) >> 8)

plt.imshow(img)
plt.show()
