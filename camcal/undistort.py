import os
import numpy as np
import cv2

camera_matrix = np.float32([
    [  1.09120762e+03,  0.00000000e+00,  1.24518781e+03],
    [  0.00000000e+00,  1.09241267e+03,  9.68593730e+02],
    [  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

dist_coeffs = np.float32([-0.02486993, 0, 0, 0])

camera_matrix = np.float32([
    [  1.07384901e+03,  0.00000000e+00,  1.22267553e+03],
    [  0.00000000e+00,  1.06616220e+03,  9.42042213e+02],
    [  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

dist_coeffs = np.float32([-0.02081262, 0, 0, 0])

new_camera_matrix = np.copy(camera_matrix)
new_camera_matrix[:2, :2] /= 2.

for jpgname in os.listdir("."):
    if not jpgname.endswith(".jpg"):
        continue
    if "undistort" in jpgname:
        continue
    jpg = cv2.imread(jpgname)
    # jpg_undist = cv2.fisheye.undistortImage(jpg[::4, ::4], camera_matrix, D=dist_coeffs, Knew=new_camera_matrix)
    jpg_undist = cv2.fisheye.undistortImage(jpg, camera_matrix, D=dist_coeffs, Knew=new_camera_matrix)
    cv2.imwrite(jpgname+'_undistort.jpg', jpg_undist)
    jpg_undist = cv2.fisheye.undistortImage(jpg, camera_matrix, D=np.zeros(4), Knew=new_camera_matrix)
    cv2.imwrite(jpgname+'_undistort_nodist.jpg', jpg_undist)
    # cv2.imshow('img', jpg_undist)
    # cv2.waitKey()

cv2.destroyAllWindows()
cv2.waitKey(1)

