# USU AVC MOTION

## Downloading using rosinstall

```
$ cd <catkin_ws>/src
$ rosinstall . avc.rosinstall
```

## How to execute code:

### Run launch file with either gazebo or rviz

Control with rviz and gazebo

```
$ roslaunch avc_motion proj2_avc.launch
```

Display with rviz

```
$ roslaunch avc_motion avc_rvizSim.launch
```

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

## Robot Control

The robot is controlled through the w,a,s,d keys w is forward, s is backwards, and a and d change the angular velocity for rotation.



![This is a pic of the robot](https://github.com/eichmeierbr/avc_urdf/blob/master/real_robot.jpg)


