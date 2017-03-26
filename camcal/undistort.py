import os
import numpy as np
import cv2

camera_matrix = np.float32([
    [  1.13826615e+03,  0.00000000e+00,  1.31095665e+03],
    [  0.00000000e+00,  1.13236467e+03,  9.29698331e+02],
    [  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

dist_coeffs = np.float32([
    [ 0.01476325],
    [-0.17737777],
    [ 0.3610746 ],
    [-0.23059519]])

#dist_coeffs = np.array(
#    [-0.29633025,  0.08367157,  0.00047389,  0.00120355, -0.00992998])

camera_matrix[:2] /= 4.
new_camera_matrix = np.copy(camera_matrix)
new_camera_matrix[:2, :2] /= 2.

for jpg in os.listdir("."):
    if not jpg.endswith(".jpg"):
        continue
    jpg = cv2.imread(jpg)
    jpg_undist = cv2.fisheye.undistortImage(jpg[::4, ::4], camera_matrix, D=np.zeros(4), Knew=new_camera_matrix)
    # jpg_undist = cv2.fisheye.undistortImage(jpg, camera_matrix, D=dist_coeffs, Knew=camera_matrix)
    cv2.imshow('img', jpg_undist)
    cv2.waitKey()

cv2.destroyAllWindows()
cv2.waitKey(1)

