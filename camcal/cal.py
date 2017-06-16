import os
import cv2
import cv2.aruco
import numpy as np

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
board = cv2.aruco.CharucoBoard_create(5, 7, 0.25375/7, .018, dictionary)
# img = board.draw((200*3, 200*3))

# Dump the calibration board to a file
# cv2.imwrite('charuco.png', img)


object_points = []
image_points = []

for jpg in os.listdir("."):
    if not jpg.endswith(".jpg"):
        continue
    if "undistort" in jpg:
        continue
    frame = cv2.imread(jpg)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray, dictionary)

    if len(res[0]) > 0:
        _, corners, ids = cv2.aruco.interpolateCornersCharuco(res[0], res[1], gray, board)
        if corners is not None and ids is not None and len(corners) > 3:
            print board.chessboardCorners[ids[:, 0]]
            print corners[:, 0]
            object_points.append(board.chessboardCorners[ids[:, 0]].reshape((1,len(ids),3)))
            image_points.append(corners.reshape(1,len(ids),2))
            print jpg, len(object_points), 'calibration frames'


imsize = gray.shape

_K = np.eye(3)
_D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(image_points))]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(image_points))]
calibration_flags = (
    cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
    + cv2.fisheye.CALIB_FIX_SKEW
    + cv2.fisheye.CALIB_CHECK_COND
    + cv2.fisheye.CALIB_FIX_K2
    + cv2.fisheye.CALIB_FIX_K3
    + cv2.fisheye.CALIB_FIX_K4)

cal = cv2.fisheye.calibrate(object_points, image_points, imsize[::-1], _K, _D, rvecs, tvecs, calibration_flags)
# cal = cv2.calibrateCamera(object_points, image_points, imsize[::-1], cam, None)

retval, cameraMatrix, distCoeffs, rvecs, tvecs = cal
print retval
print cameraMatrix
print distCoeffs
