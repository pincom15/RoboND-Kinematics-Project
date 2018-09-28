## Project: Kinematics Pick & Place

---

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.


We can find the URDF configuration in `kr210.urdf.xacro` file. From the URDF file, we can extract the position and orientation of each joint.

| no | joint | parent | child | x | y | z | r | p | y |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | fixed_base_joint | base_footprint | base_link | 0 | 0 | 0 | 0 | 0 | 0 |
| 1 | joint_1 | base_link | link_1 | 0 | 0 | 0.33 | 0 | 0 | 0 |
| 2 | joint_2 | link_1 | link_2 | 0 .35| 0 | 0.42 | 0 | 0 | 0 |
| 3 | joint_3 | link_2 | link_3 | 0 | 0 | 1.25 | 0 | 0 | 0 |
| 4 | joint_4 | link_3 | link_4 | 0.96 | 0 | -0.054 | 0 | 0 | 0 |
| 5 | joint_5 | link_4 | link_5 | 0.54 | 0 | 0 | 0 | 0 | 0 |
| 6 | joint_6 | link_5 | link_6 | 0.193 | 0 | 0 | 0 | 0 | 0 |
| EE | gripper_joint | link_6 | gripper_link | 0.11 | 0 | 0 | 0 | 0 | 0 |

We can derive our modified DH table.

| Links | i | alpha(i-1) | a(i-1) | d(i) | theta(i) |
| ---- | --- | --- | --- | --- | --- |
| 0->1 | 1 | 0 | 0 | 0.75 | q1 |
| 1->2 | 2 | -pi/2 | 0.35 | 0 | -pi/2+q2 |
| 2->3 | 3 | 0 |  | 1.25 | q3 |
| 3->4 | 4 | -pi/2 | -0.05 | 1.5 | q4 |
| 4->5 | 5 | pi/2 | 0 | 0 | q5 |
| 5->6 | 6 | -pi | 0 | 0 | q6 |
| 6->7 | 7 | 0 | 0 | 0.303 | q7 |


![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

We can create individual transformation matrices using our modified DH table.

First, 
```python
# Define Modified DH Transformation matrix
def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[      cos(q),     -sin(q),        0,      a],
        [ sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
        [ sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),   cos(alpha)*d],
        [                 0,                 0,            0,              1]])
    return TF
```


```python
# Create individual transformation matrices
T0_1 =  TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2 =  TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3 =  TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4 =  TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5 =  TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6 =  TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
```


```python
	# Extract rotation matrices from the transformation matrices
	T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Inverse position Kinematics

Correct rotation. Than, calculate the wrist conter position.

```python
Rot_error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

ROT_x = Matrix([[       1,       0,       0],
                [       0,  cos(r), -sin(r)],
                [       0,  sin(r),  cos(r)]])

ROT_y = Matrix([[  cos(p),       0,  sin(p)],
                [       0,       1,       0],
                [ -sin(p),       0,  cos(p)]])

ROT_z = Matrix([[  cos(y), -sin(y),       0],
                [  sin(y),  cos(y),       0],
                [       0,       0,       1]])

ROT_EE = ROT_z * ROT_y * ROT_x

Rot_error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

ROT_EE = simplify(ROT_EE * Rot_error)
```

```python
WC = EE - (0.303) * ROT_EE[:,2]

# Calculate joint angles using Geometric IK method
theta1 = atan2(WC[1],WC[0])

# SSS triangle for theta2 and theta3
side_a = 1.501
side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
side_c = 1.25

angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
theta3 = pi / 2 - (angle_b + 0.036)

R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3] 
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3}) 

R3_6 = R0_3.transpose() * ROT_EE # transpose() works better than inv(method="LU") 

# Euler angles from rotation matrix 
theta4 = atan2(R3_6[2,2], -R3_6[0,2]) 
theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2]) 
theta6 = atan2(-R3_6[1,1], R3_6[1,0]) 
```


Inverse Orientation Kinematics

```python
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3] 
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3}) 
R3_6 = R0_3.transpose() * ROT_EE # transpose() works better than inv(method="LU") 

# Euler angles from rotation matrix 
theta4 = atan2(R3_6[2,2], -R3_6[0,2]) 
theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2]) 
theta6 = atan2(-R3_6[1,1], R3_6[1,0]) 
```

![alt text][image2] 

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Here is my result video. https://youtu.be/ugLVY9zCm9I

![alt text][image3]


