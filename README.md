# tracker\_360: 360° Panorama Object Detection Node for ROS 2

## Overview

`tracker_360` is a ROS 2 package that enables real-time object detection on 360° equirectangular panorama images. It uses stereographic sub-projections to divide the panorama into four views, detects objects individually, and reprojects the detections onto the panorama.

This implementation is inspired by the paper:

> Wenyan Yang, Yanlin Qian, Joni-Kristian Kämäräinen, "Object Detection in Equirectangular Panorama," CVPR Workshops, 2018.

## Key Features

- **Stereographic Sub-Projections**: Reduces panorama distortion by splitting it into four rectified views at yaw angles 0°, 90°, 180°, and 270°.
- **Parallelized Remapping**: Applies precomputed LUTs in parallel to remap the panorama into stereographic views.
- **Batch Inference with YOLOv8**: Performs batch detection (batch size = 4) on the four projections using a YOLOv8 model.
- **Bounding Box Reprojection**: Projects detected boxes back to the panorama coordinates, adjusting for spherical distortion and wrap-around.
- **Global NMS (Custom)**: Merges overlapping detections from different projections into single detections using a custom NMS that prioritizes the largest bounding box.
- **ROS 2 Native Integration**: Operates through standard ROS 2 topics with flexibility to tune parameters.

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

1. **Subscribe to Panorama**
   - Subscribes to `/stitched_image` (`sensor_msgs/Image`) which should contain an equirectangular 360° panorama.

2. **Precompute LUTs (Lookup Tables)**
   - At node initialization, four remapping LUTs are generated (for yaw 0°, 90°, 180°, 270°).
   - These LUTs define how to warp the panorama into stereographic projections efficiently.

3. **Apply Projections (Parallelized)**
   - The incoming panorama frame is simultaneously projected into four stereographic images using the LUTs.
   - Parallelism is achieved via a `ThreadPoolExecutor`, significantly reducing remapping latency.

4. **Batch Object Detection**
   - The four stereographic images are stacked and passed as a batch into the YOLOv8 model.
   - The model returns bounding boxes (center_x, center_y, width, height), confidence scores, and class IDs.

5. **Reprojection of Bounding Boxes**
   - For each detection:
     - The box coordinates are mapped from stereographic plane back to the spherical UV coordinates.
     - Proper handling for wrap-around (objects near 0°/360° seams) is implemented.

6. **Custom Global NMS (Greedy Merge)**
   - Detections across projections are merged by:
     - Grouping boxes with IoU > 0.2.
     - Keeping the box with the largest area in overlapping groups.
   - This reduces duplicate detections caused by overlapping projections.

7. **Draw Annotated Image**
   - The final accepted detections are drawn over the panorama.
   - Class names and confidence scores are rendered on top of bounding boxes using YOLOv8's color scheme.

8. **Publish Result**
   - Publishes the final annotated panorama on `/annotated_panorama`.

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
- `stereo_image_size`: Size of each stereographic view.
- `confidence_threshold`: Minimum score for object acceptance.
- `model_path`: Path to your YOLOv8 model.

### Topics

| Topic Name             | Type                | Direction | Description                          |
| ---------------------- | ------------------- | --------- | ------------------------------------ |
| `/stitched_image`      | `sensor_msgs/Image` | Sub       | Input panorama image                 |
| `/image_projection1-4` | `sensor_msgs/Image` | Pub       | Projected views for debugging        |
| `/annotated_panorama`  | `sensor_msgs/Image` | Pub       | Final panorama with detections       |

## Example Commands

```bash
# Launch the node
ros2 run tracker_360 multi_person_tracker --ros-args \
  -p stereo_image_size:="448x448" \
  -p confidence_threshold:=0.6 \
  -p model_path:="~/models/yolov8m.pt"

# Visualize the output
rqt_image_view /annotated_panorama
```

## Limitations & Future Work

- **Seam Issues**: Objects crossing view borders may still produce small misalignment artifacts.
- **Inference Speed**: Future work could include ONNX+TensorRT acceleration and dynamic batch management.
- **Automatic FOV Adjustment**: Dynamically tuning FOV based on object density could enhance accuracy.

## References

- Wenyan Yang et al., "Object Detection in Equirectangular Panorama," CVPR Workshops, 2018.
- Ultralytics YOLOv8: [https://github.com/ultralytics/ultralytics](https://github.com/ultralytics/ultralytics)

---

*Created by Your Name — for advanced ROS 2 perception applications.*

