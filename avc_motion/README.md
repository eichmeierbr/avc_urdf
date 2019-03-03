# USU AVC MOTION

## How to execute code:

### Run launch file 

```
$ roslaunch avc_motion proj2_avc.launch
```

## Robot Control

We used the better bicycle control model to simulate our bot. This is because the two front wheels of our bot twist.

We implemented the code by taking the unicycle model that was provided us and turning it into a simple bicycle model by changing how angular velocity is being computed. Then we made it into a smooth bicycle by adding translational velocity and angular velocity as states.

Then we had to import the URDF from our other miniproject. We also made it so that the wheels spin acording to the change in odometry.

We made changes to the original URDF so the the wheels could twist like they do in real life. We put in the steering angle limits as well as velocity limits to match those of the actual vehicle. We had to change the base_link frame so that we could control the robot between the back wheels rather than the center of mass based on the maths of the control model.

The robot is controlled by controlling the translational acceleration as well as steering angle acceleration. This is done through keyboard input. When no input is being recieved zero acceleration is being commanded.

![Simulation](https://github.com/eichmeierbr/avc_urdf/blob/master/sim.png)

## Help - Melodic Changes
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