# tracker_360: 360° Panorama Object Detection Node for ROS 2

## Overview

tracker_360 is a ROS 2 package that enables real-time object detection on 360° equirectangular panorama images. It uses stereographic sub-projections to divide the panorama into four views, detects objects individually, and reprojects the detections onto the panorama.

This package has been specifically designed to solve the common problem of duplicate detections at the horizontal seams of 360° images. Objects crossing the 0°/360° boundary — such as people partially visible on each side of the panorama — can often be detected twice. Our method uses custom reprojection and merging logic to robustly unify duplicated detections, as illustrated below:

<p align="center">
<img src="./readme_images/duplication_problem.png" alt="Live Mode" width="300">
</p>
Inspired by:

> Wenyan Yang, Yanlin Qian, Joni-Kristian Kämäräinen, "Object Detection in Equirectangular Panorama," CVPR Workshops, 2018.

---
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
git clone https://github.com/YourOrg/tracker_360.git
cd ~/ros2_ws
colcon build --packages-select tracker_360
source install/setup.bash
```

## Detailed Workflow

### 1. Subscribe to the Panorama Stream

- The node subscribes to `/stitched_image` (`sensor_msgs/msg/Image`), which must publish a **stitched equirectangular 360° image** (2:1 width/height ratio).
- These images typically come from a panoramic camera (e.g., Ricoh Theta Z1).

### 2. LUT (Lookup Table) Precomputation

- On startup, the node precomputes **4 LUTs** (pixel remappings) using `precompute_stereo_luts()`:
  - Each LUT corresponds to a yaw angle: **0°, 90°, 180°, 270°**.
  - This defines how each pixel in the stereographic projection maps back to the original panorama.
- This is done **once** to avoid recalculating mappings every frame (critical for real-time performance).

### 3. Panorama Projection into Stereographic Views (Parallelized)

- When a panorama frame arrives, `apply_stereo_luts_parallel()` is called:
  - Each LUT is used to remap the panorama to a stereographic view using `cv2.remap()`.
  - Projections are computed **in parallel** using a `ThreadPoolExecutor`, one thread per LUT.
- Result: **4 stereographic views** (one for each yaw) are created **simultaneously**.

### 4. YOLOv8 Object Detection on Projections (Parallel Inference)

- Each stereographic view is passed independently to the YOLOv8 model using `detect_on_projections()`.
- Each detection produces:
  - Bounding boxes (center x, center y, width, height)
  - Confidence scores
  - Class IDs
- This step can also be parallelized further (future optimization) to maximize GPU/CPU usage.

### 5. Reprojection of Bounding Boxes onto the Panorama

- Each detection's bounding box (from the stereographic plane) is **reprojected back** onto the panorama.
- This involves:
  - Applying the inverse of the stereographic transformation using `map_to_sphere()`.
  - Mapping the spherical coordinates to 2D UV coordinates of the panorama.
  - Correcting for **wrap-around** (boxes that cross 0°/360° horizontal seam).

### 6. Global NMS across Projections (Greedy Merge by Area)

- After reprojecting all detections:
  - Overlapping boxes from different projections are grouped based on Intersection over Union (IoU > 0.2).
  - Inside each group, only the **largest box** (in area) is kept.
- This custom `greedy_merge_largest_box()` algorithm avoids multiple detections of the same object appearing on panorama seams.

### 7. Drawing the Final Annotated Panorama

- The surviving boxes are drawn on the panorama using:
  - YOLOv8 standard colors for class IDs (`ultralytics.utils.plotting.colors()`).
  - Class names and confidence scores over each box.
- Visualization is clean and consistent with YOLOv8 styling.

### 8. Publishing the Final Outputs

- The final **annotated panorama** is published to `/annotated_panorama`.
- The four **stereographic projections** are also published to `/image_projection1` to `/image_projection4` for debugging purposes.

---

## ROS 2 Usage

### Run Node Directly

```bash
ros2 run tracker_360 multi_person_tracker
```

### Run with Launch File

```bash
ros2 launch tracker_360 multi_person_tracker.launch.py \
  stereo_image_size:=448x448 \
  confidence_threshold:=0.5 \
  model_path:=/path/to/yolov8n.pt
```

Launch arguments:
- `stereo_image_size`: Size (W x H) for each stereographic projection.
- `confidence_threshold`: Minimum score to accept detections.
- `model_path`: Path to your YOLOv8 model weights.

---

## ROS 2 Topics

| Topic Name              | Type                 | Direction | Description                                |
|------------------------- |----------------------|-----------|--------------------------------------------|
| `/stitched_image`        | `sensor_msgs/msg/Image` | Sub      | Input panorama image                       |
| `/annotated_panorama`    | `sensor_msgs/msg/Image` | Pub      | Final annotated panorama with detections   |
| `/image_projection1-4`   | `sensor_msgs/msg/Image` | Pub      | Intermediate stereographic projected views |

---

## Example Commands

```bash
# Launch the node manually
ros2 run tracker_360 multi_person_tracker --ros-args \
  -p stereo_image_size:="448x448" \
  -p confidence_threshold:=0.6 \
  -p model_path:="~/models/yolov8n.pt"

# Visualize the result
rqt_image_view /annotated_panorama
```

---

## References

- Wenyan Yang, Yanlin Qian, Joni-Kristian Kämäräinen,  
  ["Object Detection in Equirectangular Panorama," CVPR Workshops, 2018](https://openaccess.thecvf.com/content_cvpr_2018_workshops/w15/html/Yang_Object_Detection_in_CVPR_2018_paper.html)
- [Ultralytics YOLOv8 Official Repository](https://github.com/ultralytics/ultralytics)

---

*Created for high-performance 360° object perception using ROS 2.*



