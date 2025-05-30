# 360° Panorama Object Detection Node for ROS 2

## Overview

`panoramic_object_detector` is a ROS 2 package developed for real-time object detection on 360° equirectangular panoramas captured using the **Ricoh Theta Z1** camera.

It leverages stereographic sub-projections to divide the full panorama into four overlapping views, performs detection independently on each, and accurately reprojects the bounding boxes back into the original panoramic image.

The system is explicitly designed to address the issue of duplicate detections along the horizontal seams of 360° imagery — especially at the 0°/360° boundary, where objects like people or vehicles often appear split and detected multiple times. A custom merging strategy is applied to robustly unify overlapping detections, as illustrated below.

<p align="center">
<img src="./readme_images/duplication_problem.png" alt="Duplicated Object Problem" width="600">
</p>

This implementation is inspired by the paper:

> Wenyan Yang, Yanlin Qian, Joni-Kristian Kämäräinen, "Object Detection in Equirectangular Panorama," CVPR Workshops, 2018.

## Installation

### 1. System Prerequisites

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-cv-bridge \
  libopencv-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  libglib2.0-dev \
  libusb-1.0-0-dev
```

### 2. Python Dependencies

```bash
pip3 install --user \
  ultralytics \
  opencv-python \
  numpy \
  torch torchvision
```

### 3. Clone & Build

```bash
cd ~/ros2_ws/src
git clone (https://github.com/RubenCasal/Object-detection-360.git
cd ~/ros2_ws
colcon build --packages-select panoramic_object_detector
source install/setup.bash
```

## Detailed Workflow

### 1. Subscribe to Panorama

- Subscribes to `/stitched_image` (`sensor_msgs/Image`) containing an equirectangular 360° panorama.

### 2. Precompute LUTs (Lookup Tables)

On node initialization, four Lookup Tables (LUTs) are generated — one for each yaw angle (0°, 90°, 180°, 270°).  
Each LUT encodes how pixels in a stereographic projection view map to sampling locations in the original equirectangular panorama.

#### How LUTs are generated:

**For each yaw direction:**

- A virtual tangent plane is defined, centered on the unit sphere at the desired yaw and pitch angles.
- For every pixel (x,y) in this planar view:
  - It is mapped to a point (θ, Φ) on the sphere using the inverse stereographic projection of the paper.
  - (θ, Φ) represent spherical coordinates: latitude and longitude.
  - These spherical coordinates are then converted to pixel coordinates in the panorama using:

  <div align="center">
  <img src="readme_images/LUT_equation.png" alt="LUT Equations" width="300">
</div>

  where W and H are the panorama width and height.

- The resulting (u, v) values are stored in a LUT, LUT(θ, Φ) = (u,v), so that `cv2.remap()` can efficiently sample the correct pixels from the 360° panorama.
  
  <div align="center">
  <img src="readme_images/stereo_projection.png" alt="Stereo Projection Representation" width="550">
</div>

At runtime, applying `cv2.remap()` with this LUT generates a stereographic view centered at the desired yaw and pitch, with an approximate 180° horizontal Field of View (FOV).

Neighboring projections are spaced 90° apart to ensure full coverage and generate an effective overlap of ~90° between adjacent views.

This method concentrates detection on local undistorted areas, significantly reducing the visual distortions common in raw equirectangular images, especially near horizontal seams.


### 3. Apply Projections (Parallelized)

- When a frame arrives, it is simultaneously projected into four stereographic views using the precomputed LUTs.
- Projections are computed in parallel using `ThreadPoolExecutor`.

<p align="center">
    <img src="./readme_images/four_projection_corrected.png" alt="4 projections" width="650">
    </p>

### 4. YOLO Object Detection (Batch Parallelizable)

- All four stereographic projections are batched together and passed through a YOLO model.
- This batch inference improves overall GPU utilization and minimizes latency.
- Each detection outputs bounding boxes (center x, center y, width, height), confidence scores, and class IDs.

<p align="center">
    <img src="./readme_images/projections_detection.png" alt="4 projections detections" width="700">
    </p>

### 5. Reprojection of Bounding Boxes to Panorama

After detection on the stereographic projections, bounding boxes must be accurately mapped back to the original panorama.

- First, the bounding box corners (top-left and bottom-right) are rescaled relative to the stereographic view.
- These corners are inverse-mapped to spherical coordinates using `map_to_sphere`.
- Spherical coordinates are then converted to UV pixel coordinates in the panorama.
- Special logic corrects wrap-around at the 0°/360° seam to ensure correct bounding box representation.

### 6. Global NMS (Greedy Merge)

- After reprojection, bounding boxes from all projections are grouped.
- The custom greedy_merge_largest_box() algorithm is applied:
    - Detections with IoU > 0.2 are clustered together.
    - For each cluster, the box with the largest score-weighted area is kept.
This method effectively removes duplicated detections from overlapping regions and projection seams. The following image illustrates the duplication problem without applying the custom NMS:

<p align="center">
    <img src="./readme_images/bboxes_overlap.png" alt="4 Bounding Boxes Overlap Problem" width="600">
    </p>


### 7. Draw and Publish Annotated Image

- After selecting the final set of detections through the custom NMS, the bounding boxes are drawn onto a copy of the original panorama.

- Each detection is annotated with its class label and confidence score.

- Standard YOLO color coding is used for consistency and clarity.

- The final annotated panorama is then published on the ROS 2 topic /annotated_panorama.

<div align="center">
  <img src="readme_images/demo_360_object_detection.gif" alt="Demo 360 Object Detector" width="600">
</div>

## Processing Pipeline Overview

<div align="center">
  <img src="readme_images/panormic_detector_diagram.png" alt="Pipeline Overview" width="700">
</div>

## ROS 2 Usage

### Run Node Directly

```bash
ros2 run panoramic_object_detector theta_node
```
```bash
ros2 run panoramic_object_detector panoramic_detector.py
```

### Run with Launch File

```bash
ros2 launch panoramic_object_detector panoramic_detector_launch.py
```



### Topics

| Topic Name             | Type                | Direction | Description                    |
| ---------------------- | ------------------- | --------- | ------------------------------ |
| `/stitched_image`      | `sensor_msgs/Image` | Sub       | Input panorama image           |
| `/annotated_panorama`  | `sensor_msgs/Image` | Pub       | Final panorama with detections |


## Limitations & Future Work

- Slow inference speed due to multiple sequential steps in the pipeline. There is room for improvement through greater parallelization.

- Duplicate bounding boxes may appear when objects are very close to the camera or occupy a large portion of one or more projections.

- Aggressive NMS filtering may suppress valid detections of nearby objects belonging to the same class, leading to missed detections in crowded scenes.

- Projection overlap could be tuned (reduced) to assess whether it helps prevent duplication artifacts for objects near the camera.

## References

- Wenyan Yang et al., "Object Detection in Equirectangular Panorama," CVPR Workshops, 2018.
- Ultralytics YOLO: [https://github.com/ultralytics/ultralytics](https://github.com/ultralytics/ultralytics)

---


