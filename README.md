# Forbal
Forbal is the series of **For**ce **Bal**anced Manipulators built out of **Fo**u**r** **Ba**r **L**inkages. There are two variants:
- Forbal2: a 2-DOF planar Force Balanced Four Bar Linkage
- Forbal5: a 5-DOF extension of Forbal2 with additional degrees of freedom at the base and end-effector. It is still Force Balanced!

## Setup configuration
- Ubuntu 22 / Ubuntu 24
- ROS Hunble / Jazzy
- ROS2 Control
- Dynamixel Hardware

## Installation Instructions
### Initial Setup
1. Clean installation of Ubuntu 22 or 24. I recommend a partition, Windows Subsystem for Linux not tested yet.
2. Install ROS2 [Humble](https://docs.ros.org/en/humble/Installation.html) or [Jazzy](https://docs.ros.org/en/jazzy/Installation.html) following the instructions
3. Install ROS2 control for [Humble](https://control.ros.org/humble/doc/getting_started/getting_started.html) or [Jazzy](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
4. Install the dynamixel hardware plugin using following the instructions for the branch corresponding to your ROS installation ([Humble](https://github.com/dynamixel-community/dynamixel_hardware/tree/humble) or [Jazzy](https://github.com/dynamixel-community/dynamixel_hardware/tree/jazzy)) **Note**: you can also install this in your local workspace (in the next set of instructions) by cloning to the `~/forbal_ws/src/` directory

### Workspace Setup
1. Set up [ssh keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) for your github account if you haven't done so
2. Create a new workspace in your home directory:
  ```
mkdir -p ~/forbal_ws/src/forbal
cd ~/forbal_ws/src
```
2. Clone this repository `git@github.com:yjvyas/forbal.git`
3. Build
```
cd ~/forbal_ws
colcon build --symlink-install
```
All the packages should build successfully, if not, one of your earlier dependencies didn't build properly.

