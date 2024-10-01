
---

# Computer Vision for Arctos robotic Arm

## Overview

This project enables a robotic arm to utilize computer vision for object detection, specifically turning to face cats and dogs using YOLO v3 (You Only Look Once). The robotic arm uses ROS (Robot Operating System) for communication and control.

## Features

- Detects and identifies objects (cats and dogs) using YOLO v3.
- Commands the robotic arm to turn based on the detected object.
- Easily customizable for additional object recognition and actions.

## Requirements

- ROS (Robot Operating System) installed.
- MoveIt! for motion planning and control.
- Darknet and YOLO v3 model for object detection.
- Python 3 and required Python packages.

## Installation

### Step 1: Set Up Arctos Robotic Arm in ROS

Follow the instructions provided in the [Arctos Robotic Arm ROS Setup Guide](https://github.com/ArctosRobotics/ROS) to set up your robotic arm in ROS.

### Step 2: [Install Darknet and YOLO v3](https://github.com/leggedrobotics/darknet_ros)

Clone the `darknet_ros` repository:

```bash
git clone https://github.com/leggedrobotics/darknet_ros.git
```

Follow the instructions in the `darknet_ros` repository to build and configure YOLO v3.

### Step 3: Set Up the Computer Vision Control Package

This repository will only contain the `object_follower.py` script. Follow these steps:

1. **Clone this repository** (replace `<YOUR_REPOSITORY_URL>` with your actual repository URL):

   ```bash
   git clone https://github.com/ArctosRobotics/Computer-vision
   ```

2. **Copy the `object_follower.py` script** into the `/ROS/arctos_moveit/scripts` folder:

   ```bash
   cp <YOUR_PATH_TO_OBJECT_FOLLOWER>/object_follower.py ~/ROS/src/arctos_moveit/scripts/
   ```

3. **Build the package**:

   ```bash
   cd ~/ROS
   catkin build
   ```

### Step 4: Source the Workspace

After building your workspace, make sure to source it:

```bash
source devel/setup.bash
```

### Step 5: Run the YOLO Node

Before running the object follower script, you need to start the YOLO node. In a new terminal, run:

```bash
roslaunch darknet_ros darknet_ros.launch
```

### Step 6: Run Camera Preview or Bounding Box

To visualize the camera feed or bounding boxes, you can run the following commands in separate terminals:

- For camera feed:

   ```bash
   rostopic echo /usb_cam/image_raw
   ```

- For bounding boxes:

   ```bash
   rostopic echo /darknet_ros/bounding_boxes
   ```

## Usage

Once the computer vision script is running, the robotic arm will respond to detected objects. It will turn to face cats and turn back to face dogs. You can customize the behavior by modifying the `object_follower.py` script.

## Troubleshooting

- Ensure that all dependencies are installed and correctly configured.
- If the arm does not respond as expected, check the output of the ROS nodes for error messages.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---
