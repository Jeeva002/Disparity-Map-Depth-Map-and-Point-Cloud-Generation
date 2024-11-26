
# Stereo Vision: Disparity Map, Depth Map, and Point Cloud Generation

This project demonstrates stereo vision techniques, including **camera calibration**, **disparity map computation**, **depth map estimation**, and **point cloud generation**. The project uses OpenCV for image processing and Open3D for 3D visualization.

---

## **Features**
1. **Camera Calibration**:
   - Calibrates left and right cameras using a chessboard pattern.
   - Calculates intrinsic and extrinsic parameters.
2. **Disparity Map**:
   - Computes the pixel-wise disparity between stereo image pairs.
3. **Depth Map**:
   - Derives depth information from disparity and baseline.
4. **Point Cloud**:
   - Generates a 3D point cloud of the scene from the depth map.

---

## **Folder Structure**
```
stereo_vision/
├── stereo_depth_pointcloud.py      # Generates disparity map, depth map, and point cloud
├── stereo_calibration.py           # Calibrates stereo camera
├── input_images/
│   ├── left/                       # Folder for left camera images
│   ├── right/                      # Folder for right camera images
├── calibration_data/               # Stores calibration results
├── output/                         # Stores generated outputs
├── requirements.txt                # Project dependencies
└── README.md                       # Project documentation
```

---

## **Setup**

### **1. Clone the Repository**
```bash
git clone https://github.com/your_username/stereo_vision.git
cd stereo_vision
```

### **2. Install Dependencies**
Install required Python libraries using pip:
```bash
pip install -r requirements.txt
```

### **3. Collect Input Images**
- Place **left camera images** in `input_images/left/`.
- Place **right camera images** in `input_images/right/`.
- Use a chessboard pattern for calibration and stereo image pairs for depth estimation.

---

## **Usage**

### **1. Camera Calibration**
Calibrate the stereo camera using chessboard images:
```bash
python stereo_calibration.py
```
This step generates calibration files in the `calibration_data/` folder.

### **2. Generate Disparity, Depth Map, and Point Cloud**
Run the script to compute disparity, depth, and 3D point cloud:
```bash
python stereo_depth_pointcloud.py
```
Results are saved in the `output/` folder:
- `disparity_map.jpg`: Visualizes pixel differences between stereo images.
- `depth_map.jpg`: Shows pixel-wise depth.
- `pointcloud.ply`: 3D point cloud of the scene.

---

## **Outputs**

### **1. Disparity Map**
A grayscale image where pixel intensity represents disparity between left and right views.

### **2. Depth Map**
A grayscale image where pixel intensity corresponds to depth (distance) from the camera.

### **3. Point Cloud**
A `.ply` file visualizing the 3D structure of the scene, which can be viewed in 3D tools like Open3D or Meshlab.

---

## **Examples**

### Input Stereo Images
| Left Image                  | Right Image                 |
|-----------------------------|-----------------------------|
| ![Left Image](output/left.jpg) | ![Right Image](output/right.jpg) |

### Outputs
| Disparity Map               | Depth Map                  |
|-----------------------------|-----------------------------|
| ![Disparity Map](output/disparity_map.jpg) | ![Depth Map](output/depth_map.jpg) |

| **Point Cloud**             |
|-----------------------------|
| ![Point Cloud](output/pointcloud.png) |

---

## **Requirements**
- Python 3.8+
- OpenCV 4.8+
- NumPy 1.26+
- Open3D 0.17+

Install the requirements using:
```bash
pip install -r requirements.txt
```

---

## **Troubleshooting**
- Ensure left and right images are well-aligned and have overlapping fields of view.
- Use a properly sized chessboard pattern for calibration.
- If the disparity map appears noisy, adjust `numDisparities` and `blockSize` in the `StereoBM` parameters.

---
