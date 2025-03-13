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
3. Attach the Mount Base to Joint0, make sure it is facing the correct direction.
4. Attach the Motors mounts to the mount base
5. Assemble the links (steps 5-6 for Forbal2).
6. 

## Installation Instructions
### Initial Setup
1. Clean installation of Ubuntu 22 or 24. I recommend a partition, Windows Subsystem for Linux not tested yet.
2. Install ROS2 [Humble](https://docs.ros.org/en/humble/Installation.html) or [Jazzy](https://docs.ros.org/en/jazzy/Installation.html) following the instructions. Add this line at the end of your `~/.bashrc` file (using `vim` or any other editor): `source /opt/ros/humble/setup.bash` (replace humble with jazzy if that's your version)
3. Install ROS2 control for [Humble](https://control.ros.org/humble/doc/getting_started/getting_started.html) or [Jazzy](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
4. Install the [dynamixel hardware](https://github.com/dynamixel-community/dynamixel_hardware) plugin using following the instructions for the branch corresponding to your ROS installation. First, follow the link and switch branch to 'humble' or 'jazzy'. Use these commands:
```
mkdir -p ~/ros/${ROS_DISTRO} && cd ~/ros/${ROS_DISTRO}/src
git clone https://github.com/youtalk/dynamixel_hardware.git
git clone https://github.com/youtalk/dynamixel_hardware.git
cd ~/ros/${ROS_DISTRO}/src/dynamixel_hardware
git checkout --track origin/${ROS_DISTRO}
cd ~/ros/${ROS_DISTRO}/src/
sudo rosdep init
rosdep update
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
. install/setup.bash
```

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

