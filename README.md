# USU AVC URDF

## How to execute code:
### build urdf file:

```
$ xacro --inorder bvc.xacro > <file_Name>

```
### Then launch using either gazebo or rviz
```
$ roslaunch avc_urdf display.launch model:=<file_Name>

$ roslaunch avc_urdf gazebo.launch model:=<file_Name>
```
