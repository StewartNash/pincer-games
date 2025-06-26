# Pincer Games

+ Tic-Tac-Toe
+ Checkers
+ Chess
+ Backgammon
+ Go

## Introduction

This repository is a fork of the Arctos robotics computer vision repository for its robotic arm. We call our project which utilizes the arm 'Pincer'. The repository is intended to offer several popular games that the robot can play against a human (or even another robot) opponent using computer vision.

### Caveats

The repository is currently under development. It is provided as is without warranty or guarantees. Some or all of the games are not yet functioning. I will update this document when the repository has been finalized. Feel free to make any contributions you would like.

The repository is intended to be run in conjunction with darknet_ros. However, we'll have to create our own darknet_ros fork in the future in order to customize it to use custom game pieces. We'll also have to create the custom game pieces.

## Games

This repository provides several popular games. The games are currently under development. The games will use computer vision with the YOLOv3 library to play against an opponent. CAD files for the game pieces may be included in the future.

### Tic-Tac-Toe

This game is based off of the tictac.py script provided by Arctos.

### Checkers

The checkers game is under development. It is adapted from the Python checkers game developed by 'Tech With Tim' which is available in the examples folder. That implementation, as well as this one, uses the minimax algorithm. Possible additions include:
- Alpha-beta pruning
- Iterative deepening
- Transposition tables

### Chess

This game is under development. Links and information on algorithm sources will be provided.

### Backgammon

This game is under development. Links and information on algorithm sources will be provided.

### Go

This game is under development. Links and information on algorithm sources will be provided.

## Requirements

TBD - Requirements are to be determined. They may include the following:

- A computer with ROS
- A camera
- An Arduino

---

## Setup

Setup instructions will be provided in the future. The setup will hopefully be limited to the following steps:

1. Clone the repository
2. Build the ROS code
3. Ensure all hardware is connected and operating
3. Run the launch file
   
### Emulator

An emulator and test scripts for each game are under development. It is hoped that we will have emulators that can be run in place of physical hardware.

### Manual Launch

Previous manual launch instructions were as follows
1. **Open Terminal** and source ROS environment:
   ```bash
   source /opt/ros/melodic/setup.bash
   source ~/catkin_workspace/devel/setup.bash
   ```
2. **Launch YOLOv3** for object detection:
   ```bash
   roslaunch darknet_ros darknet_ros.launch
   ```
3. **Launch the USB Camera** node:
   ```bash
   roslaunch usb_cam usb_cam-test.launch
   ```
4. **Run the Tic Tac Toe script**:
   ```bash
   python3 ~/catkin_workspace/src/darknet_ros/darknet/scripts/tictac.py
   ```

## Troubleshooting

### Customizing the Script

#### Custom include files for Arduino

Include files for the Arduino must end in '.hpp'. If the include file is to be included in only one sketch, it can be included in the same directory as the sketch. It can also be created by creating a new tab in the Arduino IDE and naming the file appropriately. If the include file is meant to be shared by multiple sketches, then it is preferable to create a library by doing the following:
- Close the Arduino IDE
- Navigate to the {Arduino}\hardware\libraries directory
- Create a sub-directory with the name of your library
- Open the header file and insert your code
- Open the Arduino IDE
- Create or open a Sketch
- Add an #include to the Sketch that references the new include file
    - I believe that the included file may need to be in quotes

## References
- **Darknet YOLOv3:** Insert link here
- **ROS Jazzy Jalisco:** Insert link here
- **Arctos Robotics:** Insert link here

