import cv2
import numpy as np
import open3d as o3d
import os

def compute_disparity_and_depth(left_img, right_img, camera_matrix_left, baseline, focal_length):
    # Stereo BM for disparity map
    stereo = cv2.StereoBM_create(numDisparities=16 * 8, blockSize=15)
    disparity = stereo.compute(left_img, right_img).astype(np.float32) / 16.0

    # Compute depth map
    depth = (focal_length * baseline) / (disparity + 1e-6)
    return disparity, depth

def generate_point_cloud(disparity, left_img, Q):
    # Reproject to 3D
    points_3D = cv2.reprojectImageTo3D(disparity, Q)
    colors = cv2.cvtColor(left_img, cv2.COLOR_BGR2RGB)
    mask = disparity > disparity.min()

    points = points_3D[mask]
    colors = colors[mask]

    # Create point cloud
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors / 255.0)
    return point_cloud

if __name__ == "__main__":
    # Load calibration data
    mtx_left = np.load("calibration_data/camera_matrix_left.npy")
    dist_left = np.load("calibration_data/dist_coeffs_left.npy")
    params = np.load("calibration_data/stereo_params.npy", allow_pickle=True).item()
    R, T = params["R"], params["T"]

    # Load stereo images
    left_img = cv2.imread("input_images/left/left1.jpg", cv2.IMREAD_GRAYSCALE)
    right_img = cv2.imread("input_images/right/right1.jpg", cv2.IMREAD_GRAYSCALE)

    # Undistort images
    left_img = cv2.undistort(left_img, mtx_left, dist_left)
    right_img = cv2.undistort(right_img, mtx_left, dist_left)

    # Compute baseline and focal length
    baseline = np.linalg.norm(T)
    focal_length = mtx_left[0, 0]

    # Compute disparity and depth map
    disparity, depth = compute_disparity_and_depth(left_img, right_img, mtx_left, baseline, focal_length)

    # Save disparity and depth map
    os.makedirs("output", exist_ok=True)
    cv2.imwrite("output/disparity_map.jpg", (disparity / disparity.max() * 255).astype(np.uint8))
    cv2.imwrite("output/depth_map.jpg", (depth / depth.max() * 255).astype(np.uint8))

    # Generate point cloud
    Q = np.eye(4)  # Simplified for aligned stereo
    point_cloud = generate_point_cloud(disparity, cv2.imread("input_images/left/left1.jpg"), Q)
    o3d.io.write_point_cloud("output/pointcloud.ply", point_cloud)

    print("Disparity map, depth map, and point cloud saved in 'output/'.")

