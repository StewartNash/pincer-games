# Pincer Games

+ Tic Tac Toe
+ Checkers
+ Chess
+ Backgammon
+ Go

Games for our variant of the Arctos robotic arm which we call 'Pincer'. These games are developed from a fork of the tic tac toe computer vision repository provided by Arctos robotics. And using other available algorithms which we will document.

The system is ready to run inside a virtual machine (VM) with **ROS (Robot Operating System)** and **darknet_ros** already set up.

---

# 🎮 YOLOv3 Tic Tac Toe Game

This repository provides a pre-configured Tic Tac Toe game where the computer uses **YOLOv3** for computer vision to play Tic Tac Toe against a human player.

---

## ⚙️ Requirements

- **Computer with ROS**
- **USB Camera**
- **Arduino Board**

---

## 📥 Setup Instructions

Follow these steps to run the any game:

### 1. Clone the repository

1. Clone the github repository
2. Build the ROS code.

### 2. Connect Devices

1. Open the Virtual Machine.
2. In the top menu, go to **Devices > USB >** and select:
   - Your connected **USB Camera**.
   - Your connected **Arduino** board.
   
### 3. Emulator

Emulators can be run in place of physical hardware.

---

## ▶️ Run the Game

The game can be launched automatically through the desktop app provided. Follow these steps:

### Method 1: Run the Desktop App (Automatic)

1. Double-click the desktop shortcut **"Arctos CV"** on the virtual machine's desktop.

This shortcut will:
- Launch all necessary ROS nodes:
   - YOLOv3 for object detection.
   - USB Camera driver.
- Start the Tic Tac Toe script `tictac.py`.

### Method 2: Run Manually

If you prefer to run the game manually, follow these steps:

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

---

## 🧩 Script Details: `tictac.py`

The `tictac.py` script is responsible for:

1. **Listening** to YOLOv3 bounding boxes for move detection.
2. **Controlling the robotic arm** to place the "X" marker at the correct position.
3. Managing the Tic Tac Toe game logic and updating the board state.

### Key Features:
- **ASCII Art:** The script prints ASCII art messages to indicate the winner (Robot, Human, or Tie).
- **Reset Option:** Press `r` and Enter to restart the game at any time.
- **Move Validation:** Detects and validates moves using bounding boxes.

---

## 🔧 Customize the Script

If you want to customize the game or change behavior, edit the `tictac.py` script located here:

```bash
~/catkin_workspace/src/darknet_ros/darknet/scripts/tictac.py
```

For example:
- Modify board positions.
- Change robot move strategies.
- Add new visual effects.

---

## 🎉 Enjoy Playing!

Once everything is running, enjoy challenging the robot to a Tic Tac Toe game powered by **computer vision**!

---

## 💬 Troubleshooting

If you encounter issues:
- Make sure all devices (camera, Arduino) are correctly connected to the virtual machine.
- Verify the ROS nodes are running properly using `rosnode list`.
- Check for errors in the terminal when running `tictac.py`.

---

### 🤝 Credits
- **Darknet YOLOv3:** For real-time object detection.
- **ROS Jazzy Jalisco:** For handling robotic commands and environment.
- **Arctos Robotics:** For providing robots.
---

