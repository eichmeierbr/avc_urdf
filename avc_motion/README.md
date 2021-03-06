# USU AVC MOTION

## How to execute code:

### Run launch file 

#### Project 2
Drive the vehicle using a keyboard controller
```
$ roslaunch avc_motion proj2_avc.launch
```
#### Project 3
Drive the vehicle using "2D Nav Goal" button in RVIZ
```
$ roslaunch avc_motion proj3.launch
```
Vehicle drives autonomously in an hourglass shape following preset goal points
```
$ roslaunch avc_motion list_goal.launch
```

## Robot Control

### Project 2: Keyboard Control
The robot is controlled by controlling the translational acceleration as well as steering angle acceleration. This is done through keyboard input. Positive and negative linear acceleration is controlled using 'w' and 's' respectively. Positive and negative angular rotation around the z-axis is controlled with 'a' and 'd' respectively. Pressing an input button sends an acceleration value according to avc_teleop_key lines 38-39. When no input is being recieved zero acceleration is being commanded.

### Project 3: Go to Goal Control
By using the go-to-goal controller, the robot is given a goal by the user through the 2D Nav goal button on RVIZ. The velocity control that was developed for use in this control was described by x_dot = (A - Bk)x. This velocity controller was implemented in bicycle.cpp. The k controller matrix is found on lines 25-30. The k matrix was calculated by using the matlab lqr function. Then k was used to evalute u_v and u_phi_dot on lines 94-101. These control inputs were then limited through velLimit, and phiLimit on lines 113-123. As the robot gets closer to the goal point, the desired velocity decreases, becoming zero when the robot reaches the point. A new goal can be sent at any time through the go-to-goal controller.

#### List Control
When the list_goal.launch file is used, the robot, will update the desired position through a hard-coded list found in list_goal_generator.cpp. The velocity controller used for this goal system is the same.

## Model Used

We used the continuous-steering car control model to simulate our bot. We chose this model because the two front wheels of our bot twist as modeled by the model presented in LaVelle-Planning Algorithms Section (13.2.4.2). The kinematic equations for this model include:

<a href="https://www.codecogs.com/eqnedit.php?latex=\\&space;Inputs:u_v,u_\dot{\phi}\\&space;\dot{x}&space;=&space;v&space;*&space;cos($$\theta$$)\\&space;\dot{y}&space;=&space;v&space;*&space;sin($$\theta$$)\\&space;\dot{\theta}&space;=&space;v/L&space;*&space;tan($$\phi$$)\\&space;\dot{v}&space;=&space;u_v&space;\\&space;\dot{\phi}&space;=&space;$$\omega$$\\&space;\dot{\omega}=&space;u_\dot{\phi}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\\&space;Inputs:u_v,u_\phi\\&space;\dot{x}&space;=&space;v&space;*&space;cos($$\theta$$)\\&space;\dot{y}&space;=&space;v&space;*&space;sin($$\theta$$)\\&space;\dot{\theta}&space;=&space;v/L&space;*&space;tan($$\phi$$)\\&space;\dot{v}&space;=&space;u_v&space;\\&space;\dot{\phi}&space;=&space;$$\omega$$\\&space;\dot{\omega}=&space;u_\dot{\phi}" title="\\ Inputs:u_v,u_\dot{\phi}\\ \dot{x} = v * cos($$\theta$$)\\ \dot{y} = v * sin($$\theta$$)\\ \dot{\theta} = v/L * tan($$\phi$$)\\ \dot{v} = u_v \\ \dot{\phi} = $$\omega$$\\ \dot{\omega}= u_\phi" /></a>

We implemented the code by taking the unicycle model that was provided us and turning it into a simple bicycle model by changing how angular velocity is being computed (Project 2: bicycle2.cpp line 107, Project 3: line 127). Then we made it into a smooth bicycle by adding translational acceleration and angular acceleration as states (Poject 2: bicycle2.h lines 64-65, Project 3: bicycle.h lines 69-70) and updating the model states using the commanded acceleration (Project 2: bicycle2.cpp lines 88-103, Project 3: bicycle.cpp lines 104-123).

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