# USU AVC MOTION PROJECT 2

## How to execute code:

### Run launch file 

#### Project 2
Run
```
$ roslaunch avc_motion proj2_avc.launch
```
#### Project 3
Run
```
$ roslaunch avc_motion proj3.launch
```
for the main part of the project.
To run the extra credit goals model, run
```
$ roslaunch avc_motion list_goal.launch
```

## Robot Control

The robot is controlled by controlling the translational acceleration as well as steering angle acceleration. This is done through keyboard input. Positive and negative linear acceleration is controlled using 'w' and 's' respectively. Positive and negative angular rotation around the z-axis is controlled with 'a' and 'd' respectively. Pressing an input button sends an acceleration value according to avc_teleop_key lines 38-39. When no input is being recieved zero acceleration is being commanded.


## Model Used

We used the continuous-steering car control model to simulate our bot. We chose this model because the two front wheels of our bot twist as modeled by the model presented in LaVelle-Planning Algorithms Section (13.2.4.2). The kinematic equations for this model include:

<a href="https://www.codecogs.com/eqnedit.php?latex=\\&space;Inputs:u_a,u_\alpha\\&space;\dot{x}&space;=&space;v&space;*&space;cos($$\theta$$)\\&space;\dot{y}&space;=&space;v&space;*&space;sin($$\theta$$)\\&space;\dot{\theta}&space;=&space;v/L&space;*&space;tan($$\phi$$)\\&space;\dot{v}&space;=&space;u_a&space;\\&space;\dot{\phi}&space;=&space;$$\omega$$\\&space;\dot{\omega}=&space;u_\alpha" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\\&space;Inputs:u_a,u_\alpha\\&space;\dot{x}&space;=&space;v&space;*&space;cos($$\theta$$)\\&space;\dot{y}&space;=&space;v&space;*&space;sin($$\theta$$)\\&space;\dot{\theta}&space;=&space;v/L&space;*&space;tan($$\phi$$)\\&space;\dot{v}&space;=&space;u_a&space;\\&space;\dot{\phi}&space;=&space;$$\omega$$\\&space;\dot{\omega}=&space;u_\alpha" title="\\ Inputs:u_a,u_\alpha\\ \dot{x} = v * cos($$\theta$$)\\ \dot{y} = v * sin($$\theta$$)\\ \dot{\theta} = v/L * tan($$\phi$$)\\ \dot{v} = u_a \\ \dot{\phi} = $$\omega$$\\ \dot{\omega}= u_\alpha" /></a>

We implemented the code by taking the unicycle model that was provided us and turning it into a simple bicycle model by changing how angular velocity is being computed (bicycle.cpp line 107). Then we made it into a smooth bicycle by adding translational acceleration and angular acceleration as states (bicycle.h lines 64-65) and updating the model states using the commanded acceleration (bicycle.cpp lines 88-103).

Then we had to import the URDF from our other miniproject (avc_description.launch.xml and avc_rvizSim.launch line 14). We also made it so that the wheels spin according to the change in odometry.

## Visual Changes from URDF Project

We made changes to the original URDF so the the wheels could twist like they do in real life, the previous mini-project had implemented a simple differential drive model. We put in the steering angle limits as well as velocity limits to match those of the actual vehicle. We had to change the base_link frame so that we could control the robot between the back wheels rather than the center of mass based on the maths of the control model.

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

# AVC VELOCITY CONTROL PROJECT 3
