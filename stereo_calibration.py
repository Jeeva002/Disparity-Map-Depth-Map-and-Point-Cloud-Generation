import cv2
import numpy as np
import os

def calibrate_stereo_camera(left_images_dir, right_images_dir, chessboard_size=(9, 6), square_size=1.0):
    # Prepare object points
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size

    objpoints = []  # 3D points in real world space
    imgpoints_left = []  # 2D points in left image plane
    imgpoints_right = []  # 2D points in right image plane

    left_images = sorted([os.path.join(left_images_dir, f) for f in os.listdir(left_images_dir) if f.endswith(('.jpg', '.png'))])
    right_images = sorted([os.path.join(right_images_dir, f) for f in os.listdir(right_images_dir) if f.endswith(('.jpg', '.png'))])

    for left_img_path, right_img_path in zip(left_images, right_images):
        left_img = cv2.imread(left_img_path)
        right_img = cv2.imread(right_img_path)
        gray_left = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret_left, corners_left = cv2.findChessboardCorners(gray_left, chessboard_size, None)
        ret_right, corners_right = cv2.findChessboardCorners(gray_right, chessboard_size, None)

        if ret_left and ret_right:
            objpoints.append(objp)
            imgpoints_left.append(corners_left)
            imgpoints_right.append(corners_right)

    # Calibrate individual cameras
    ret_left, mtx_left, dist_left, _, _ = cv2.calibrateCamera(objpoints, imgpoints_left, gray_left.shape[::-1], None, None)
    ret_right, mtx_right, dist_right, _, _ = cv2.calibrateCamera(objpoints, imgpoints_right, gray_right.shape[::-1], None, None)

    # Stereo calibration
    _, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right,
        mtx_left, dist_left, mtx_right, dist_right,
        gray_left.shape[::-1], flags=cv2.CALIB_FIX_INTRINSIC
    )

    return mtx_left, dist_left, mtx_right, dist_right, R, T

if __name__ == "__main__":
    left_dir = "input_images/left/"
    right_dir = "input_images/right/"
    os.makedirs("calibration_data", exist_ok=True)

    print("Calibrating stereo camera...")
    mtx_left, dist_left, mtx_right, dist_right, R, T = calibrate_stereo_camera(left_dir, right_dir)

    # Save calibration data
    np.save("calibration_data/camera_matrix_left.npy", mtx_left)
    np.save("calibration_data/dist_coeffs_left.npy", dist_left)
    np.save("calibration_data/camera_matrix_right.npy", mtx_right)
    np.save("calibration_data/dist_coeffs_right.npy", dist_right)
    np.save("calibration_data/stereo_params.npy", {"R": R, "T": T})

    print("Calibration complete. Data saved in 'calibration_data/'.")

