# Forbal
Forbal is the series of **For**ce **Bal**anced Manipulators built out of **Fo**u**r** **Ba**r **L**inkages. There are two variants:
- Forbal2: a 2-DOF planar Force Balanced Four Bar Linkage
- Forbal5: a 5-DOF extension of Forbal2 with additional degrees of freedom at the base and end-effector. It is still Force Balanced!

## Setup configuration
### Hardware
- A workstation with at least 4GB RAM, 1.6GHz
- Dynamixel U2D2 + 12V Power Hub
- Dynamixel XC430-240T x 2 (For Forbal2)
- Dynamixel XM430-W350-T + 2XL430-W250-T (for Forbal5)
- ROBOTOUS 100N Force Torque Sensor (optional)
- Mechanical Components, as outlined:
  - Base and mounts: Base, Mount_base, Mount bars, mount_connectors (see CAD assemblies for reference)
  - Links and connectors: Link11_motor Link 21_motor, Link12, Link21, EE, EE_imp (see CAD assemblies for reference)
  - M2.5 screws (mount motor to base / links)
  - M2 screws (varying lengths, to screw into motors and also idlers)
  - M3 screw kit (use as required) + M3 countersunk screws (customized)
  - Teflon bearings
  - M5 Shaft screw connectors
  - M6 Filleted Bars
  - M5 nuts
  - M6 nuts
  - M6x20mm Diameter washers
  - Counter mass rings: M6x40mmx1.6mm, M8x32mmx2mm washers etc.

### Software
- Ubuntu 22 / Ubuntu 24
- ROS Humble / Jazzy
- ROS2 Control
- Dynamixel Hardware

## Assembly Instructions
### Forbal2
Use the Solidworks CAD Assembly as a reference on how to install. A diagram with the parts is shown here. In general, follow this order:
1. (If F/T sensor) Attach F/T sensor to the wooden base using M3 screws, and the Base to the F/T sensor using the M3 countersink screws
2. First screw the motor mount connectors together with the motors and bars
3. Calibrate the motor. To do this, download Dynamixel Wizard 2.0, and connect the motors separately. Turn on Torque enabled, and set the angle to 0 degrees using Goal Position. Leave it on to ensure the motor doesn't slip, and screw the link on at the correct angle where the link's x-faces along the long side of the motor with the positive x-axis towards the shorter end of the joint.
4. Attach the Motors mounts to the Base, make sure the correct one is in the correct position and orientation.
5. Then connect Links11 and 21 to the motor joints using the short 2.5mm screws, and also the idler at the back using the longer screws (use nuts to hold the link in place).
6. Connect Link12 to Link11 using the Teflon bearing and M5 shaft screw connector, held in place with a M6x20mm washer and M5 screw.
7. Screw the End Effector (EE) to Link21 using M2 screws and nuts.
8. Connect Link22 to Link22 using the Teflon bearings, M5 shaft screw, washer and M5 nut (same as Link12).
9. Close the loop by linking the large connection holes in the EE and Link12 using the Teflon bearing, M5 shaft screw, washer, and M5 nut.
10. Attach the filleted bars at the counter mass locations for Link11, Link21 and Link22, using the appropriate lengths (shorter one for Link22). Attach the counter masses as specified.

### Forbal5
Also follow the CAD assembly as a reference for this design.
1. Follow Steps 1-3 for Forbal 2
2. Calibrate the base joint (Joint0) (Dynamixel XM430-W350-T) and attach to the Base using the base joint connectors
3. Attach the Mount Base to Joint0, make sure it is calibrated before attachment.
4. Attach the Motors mounts to the mount base.
5. Assemble the links (steps 5-6 for Forbal2).
6. Attach the motor to the end-effector (calibrate both first!).
7. Attach the End-effector implement to the second motor.
8. Close the loop of the linkages with the Teflon bearing + M6x20mm washer + M5 nut.

## Installation Instructions
### Initial Setup
1. Clean installation of Ubuntu 22 or 24. I recommend a partition, Windows Subsystem for Linux not tested yet.
2. Install ROS2 [Humble](https://docs.ros.org/en/humble/Installation.html) or [Jazzy](https://docs.ros.org/en/jazzy/Installation.html) following the instructions. Add this line at the end of your `~/.bashrc` file (using `vim` or any other editor): `source /opt/ros/humble/setup.bash` (replace humble with jazzy if that's your version)
3. Install ROS2 control for [Humble](https://control.ros.org/humble/doc/getting_started/getting_started.html) or [Jazzy](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
4. Install xacro `sudo apt install ros-${ROS_DISTRO}-xacro`
5. Workspace Setup:
  5.1. Set up [ssh keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) for your github account if you haven't done so.
  5.2. Create a new workspace in your home directory:
  ```
  mkdir -p ~/forbal_ws/src
  cd ~/forbal_ws/src
  ```
6. Install the Dynamixel drivers in this workspace:
```
git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
git clone https://github.com/youtalk/dynamixel_hardware.git
cd ~/forbal_ws/
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Forbal Setup
1. Set up [ssh keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) for your github account if you haven't done so
2. Create a new workspace in your home directory:
  ```
mkdir -p ~/forbal_ws/src/forbal
cd ~/forbal_ws/src
```
2. Clone this repository
```
cd ~/forbal_ws/src/
git clone git@github.com:yjvyas/forbal.git
```
3. Build
```
cd ~/forbal_ws
colcon build --symlink-install
```
All the packages should build successfully, if not, one of your earlier dependencies didn't build properly.

### Recommended Tools
- [Plotjuggler](https://github.com/facontidavide/PlotJuggler)  `sudo apt install ros-$ROS_DISTRO-plotjuggler-ros`
- Colcon clean workspace `sudo apt install python3-colcon-clean` you can clean workspaces with the command `colcon clean workspace -a` which removes all the `build/` `log/` and `install/` folders.
- Terminator (install from Apps utility) - super useful for multiple terminals in one window

## Running the code
#### Forbal2
1. First, traverse to the workspace folder: `cd ~/forbal_ws/` (remember, `tab` is your friend :-) )
2. Source the workspace `source ./install/setup.bash`
3. Launch the controllers, two options available:
  3.1. If you have the actual hardware, connect it in the USB port (BEFORE the F/T sensor, so that it is connected as `/dev/ttyUSB0`). Then use the launch file `ros2 launch forbal_controllers forbal2.launch.py`.
  3.2. If you only want to run it in simulation mode, run `ros2 launch forbal_controllers forbal2.launch.py sim:=true`. Note that you will not have a valid pose until you send a trajectory reference.
4. In another terminal (remember to `cd ~/forbal_ws/` and `source ./install/setup.bash` first!), you can send two types of position trajectory commands. These are in the `~/forbal_ws/src/forbal/forbal_controllers/scripts` folder, and you should copy paste them:
  4.1. To send to joint-space interpolation only, publish to the topic `/position_trajectory`:
```
ros2 topic pub -1 /position_trajectory forbal_interfaces/msg/PositionTrajectory "{ 
  header: { 
    stamp: { sec: 0, nanosec: 0 }, 
    frame_id: 'world' 
  },
  x: [0.22],
  y: [0.0],
  z: [0.22],
  time: [1.0]
}"
```
  4.2. To send position interpolated splines, use the action `.follow_position_trajectory`, the trajectories are in the txt files. For example:
```
ros2 action send_goal /follow_position_trajectory forbal_interfaces/action/FollowPositionTrajectory "{
   type: 'constant_waypoints', 
   dt: 0.01, 
   x: [0.22, 0.32, 0.42, 0.32, 0.22], 
   y: [], 
   z: [0.22, 0.32, 0.22, 0.12, 0.22], 
   time: [0.0, 1.0, 2.0, 3.0, 4.0], 
   acc_time: 1.0
}"
```
6. (If F/T sensor) Connect the sensor in the adjacent USB port (it should be `/dev/ttyUSB1`) in another terminal run the sensor driver `ros2 launch rft_sensor_serial rft_sensor_launch.py`.
7. To visualize, use plotjuggler `ros2 run plotjuggler plotjuggler` and load the config file in `src/forbal/forbal_description/launch/pj_viz.xml`.



