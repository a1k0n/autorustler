import cv2
import sys
import numpy as np


# pre-calibrated
camera_matrix = np.array(
    [[294.19581094, 0.,           324.12443518],
     [0.,           292.35612953, 247.55823055],
     [0.,           0.,           1.]])

dist_coeffs = np.array(
    [-0.29633025,  0.08367157,  0.00047389,  0.00120355, -0.00992998])

camera_matrix = np.array(
    [[294.19508988, 0.,           321.0972783],
     [0.,           292.87055626, 247.27340786 - 16],
     [0.,           0.,           1.]])

dist_coeffs = np.array(
    [-0.28282436,  0.07235428, -0.00096748,  0.00131054, -0.00757548])

def main(fname):
    newcam = np.array([[200, 0, 320], [0, 200, 256], [0, 0, 1]])
    ud1, ud2 = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, np.eye(3), newcam, (640, 512),
        cv2.CV_16SC2)

    cap = cv2.VideoCapture(fname)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        remapped = cv2.remap(frame, ud1, ud2, cv2.INTER_AREA)
        cv2.imshow('rectified', remapped)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv[1])

