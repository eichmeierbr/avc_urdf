# USU AVC URDF

This is the setup for simulation with rviz and gazebo of the USU AVC Mintaur.

## How to execute code:

### Run launch file with either gazebo or rviz

Control with rviz and gazebo

```
$ roslaunch avc_urdf control.launch
```

Display with rviz

```
$ roslaunch avc_urdf display.launch
```

Display with gazebo

```
$ roslaunch avc_urdf gazebo.launch
```
## Melodic Changes
If running melodic, you must go into the urdf folder

```
$ cd avc_urdf/urdf
```
Then through your favorite editor go into links.xacro and change line 14 to only have two // on the mesh url

```
<mesh filename="file://$(find avc_urdf)/meshes/chassis2.DAE" scale="${chassisScale} ${chassisScale} ${chassisScale}"/>
```
and then again in geometries.xacro on lines 7, and 27 remove one of the / on the mesh urls so that there are only two.

```
7  <mesh filename="file://$(find avc_urdf)/meshes/chassis2.DAE" scale="${chassisScale} ${chassisScale} ${chassisScale}"/>
14 <mesh filename="file://$(find avc_urdf)/meshes/chassis2.DAE" scale="${chassisScale} ${chassisScale} ${chassisScale}"/>
```

## Robot Description

The four components of the robot are the frame, wheels, imu, and lidar.

### Frame
Description: The frame is the body of the robot where all the other parts are mounted. It is modeled using a 3D mesh of the frame.

The frame is a mesh over the base link.

Where in Code: Instantiated in bvc.xacro line: 9.




### Wheels

Description: The wheels are attached to the frame and can rotate. A motor powers each wheel, and the front two wheels can twist in order to steer the vehicle. Each one is modeled using a 3D mesh of the wheel.

Each wheel has a continuous wheel joint that allows the wheels to roll.
The {left or right}_{front or back}_wheel_link connects to the base link via the {left or right}_{front or back}_wheel_joint

Where in Code: Instantiated in bvc.xacro lines: 11-14.


### Imu
Description: The inertial measurement unit (imu) is fixed to the frame. It is modeled using a simple red box.

The imu_link is fixed to the base_link through the imu_joint

Where in Code: Instantiated in bvc.xacro line: 16.

### Lidar
Description: The lidar is fixed to the top frame via the base link. It is modeled using a blue cylinder.

The lidar_link is fixed to the base_link through the lidar_joint.

Where in Code: Instantiated in bvc.xacro line: 17.

![This is a pic of the robot](https://github.com/eichmeierbr/avc_urdf/blob/master/real_robot.jpg)

