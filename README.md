## Robotics Project: Kinematics Pick & Place
### Name: Zhanrui Liao

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)
[image_strc]: ./misc_images/img_strc.png
[image_ik]: ./misc_images/img_ik.png
[image_ik1]: ./misc_images/img_ik1.png
[image_debug]: ./misc_images/debug.png
[image_results]: ./misc_images/results.png
[image1]: ./misc_images/img_p1.png
[image2]: ./misc_images/img_p2.png
[image3]: ./misc_images/img_p3.png
[image4]: ./misc_images/img_p4.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is the overall structure of the kr210 robot arm.

![alt text][image_strc]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | q2-pi/2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

Create Modified DH parameters
```
DH = {alpha0:     0,  a0:      0,   d1:  0.75,
	      alpha1: -pi/2,  a1:   0.35,   d2:     0,   q2: q2-pi/2,
	      alpha2:     0,  a2:   1.25,   d3:     0,
	      alpha3: -pi/2,  a3: -0.054,   d4:  1.50,
	      alpha4: -pi/2,  a4:      0,   d5:     0,
	      alpha5: -pi/2,  a5:      0,   d6:     0,
	      alpha6:     0,  a6:      0,   d7: 0.303,   q7:       0}
```
Define Modified DH Transformation matrix. (First I created the transformation matrices individually. Then I found that defining a transformation function and substituting with the DH parameters is more efficient.)
```
def TF_Matrix(alpha, a, d, q):
        TF = Matrix([[	    cos(q),           -sin(q),           0,             a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                0,                 0,           0,             1]])
    	return TF

# base_link to link1
T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH)

# link1 to link2
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH)

# link2 to link3
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH)

# link3 to link4
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH)

# link4 to link5
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH)

# link5 to link6
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH)

# link6 to gripper_link
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH)

# base_link to gripper_link
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 
![alt text][image_ik1]

From the figure above, we can derive `theta1`:
```
theta1 = atan2(WC[1],WC[0])
```

![alt text][image_ik]
From the figure above, we can derive the `theta2` and `theta3`: 
```
angle_a = acos((pow(l25,2) + pow(a2.subs(DH),2) - pow(l35,2))/(2*l25*a2.subs(DH)))
angle_b = acos((pow(l35,2) + pow(a2.subs(DH),2) - pow(l25,2))/(2*l35*a2.subs(DH)))
angle_c = acos((pow(l35,2) + pow(l25,2) - pow(a2.subs(DH),2))/(2*l35*l25))
angle_d = atan2(WC[1]- d1.subs(DH),WC[0]-a1.subs(DH))

theta2 = pi/2 - angle_a - angle_d
```
```
angle_e = pi/2 - theta2
angle_f = atan2(a3.subs(DH),d4.subs(DH))

theta3 = angle_e - angle_f - pi/2
```

We can find the rotation matrices:
```
R_x = Matrix([[1,      0,       0],
	          [0, cos(r), -sin(r)],
	          [0, sin(r),  cos(r)]]) # Roll

R_y = Matrix([[ cos(p), 0,  sin(p)],
	          [     0,  1,       0],
	          [-sin(p), 0,  cos(p)]]) #Pitch

R_z = Matrix([[cos(y), -sin(y), 0],
	          [sin(y),  cos(y), 0],
                  [     0,       0, 1]]) # Yaw

R0_3 = T0_1[0:3,0:3]*T1_2[0:3,0:3]*T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})

R3_6 = R0_3.inv("LU")*R_EE
```

Then we can find Euler angles `theta4`, `theta5` and `theta6` from rotation matrix:
```
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


With the method above, the IK code runs withour error.

As sometimes the gripper can't grasp the object successfully, I fix the "CAN't Grasp" problem by using suggestions from Slack:
adding "ros::Duration(5.0).sleep();" in line 327 of "~/RoboND-Kinematics-Project/kuka_arm/src/trajectory_sampler.cpp".

Here are 4 screenshots that illustrate the project process:
![alt text][image1]
![alt text][image2]
![alt text][image3]
![alt text][image4]

My IK code completes 10/10 pick and place tasks. The process is quiet smoothly and efficiently. Here is the results screenshot, where 10 objects are in the basket:
![alt text][image_results]

I will work on optimized solution based on several suggestions after the project is passed.


# Appendix: How to Run
Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.
