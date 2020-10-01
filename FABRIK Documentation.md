# Forward and Backward Reaching Inverse Kinematics
- Inverse Kinematics is to determine a set of appropriate joint configurations for which the end effectors move to desired positions as smoothly, rapidly, and as accurately as possible.
- FABRIK describes that avoid the use of rotational angles or matrices, and instead finds each joint position via locating a point on a line. It converges in few iterations, has low computational cost and produces visually realistic poses.

<br />
Function *out_of_range_condition()* is to get new postions of joints with reference to target and determine whether the position of each joint or joint angle is out of range.   
1. If it is out of range, it will be pushed back within the range.  
2. If the target is too far to be reached, robotic arm will straigten and point towards the target.

<br />
Function *iteration()* will let the position of wrist be the position of target and calculate out the position of each joint with repect to the position of wrist when target is within the reachable range.  
1. It requires input values of ***target position*** and tolerance.  
2. The value of **tolerance** can be changed acoording to reality demands.

<br />
Function *inverse()* is used to determine:  
1. whether the straight-line distance between target and shoulder is greater than the total length of robotic arm (57.1 cm)  
2. whether the straight-line distance between target and wrist is greater than the tolerance.  
and will return corresponding values of joint angles.

