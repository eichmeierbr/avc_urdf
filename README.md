# USU AVC URDF

## How to execute code:
### build urdf file: (NOTE: This step is optional)

```
$ xacro --inorder bvc.xacro > build/<desired_file_name.urdf>

```
### Then launch using either gazebo or rviz
```
$ roslaunch avc_urdf display.launch

$ roslaunch avc_urdf gazebo.launch
```

## Robot Description

The four components of the robot are the frame, wheels, imu, and lidar.

### Frame
The frame is the body of the robot where all the other parts are mounted. It is modeled using a 3D mesh of the frame.

### Wheels
The wheels are attached to the frame and can rotate. A motor powers each wheel, and the front two wheels can twist in order to steer the vehicle. Each one is modeled using a 3D mesh of the wheel.

### Imu
The inertial measurement unit (imu) is fixed to the frame. It is modeled using a simple red box.

### Lidar
The lidar is fixed to the top frame. It is modeled using a blue cylinder.
