# Forward And Backward Reaching Inverse Kinematics

- Inverse Kinematics is to determine a set of appropriate joint configurations for which the end effectors move to desired positions as smoothly, rapidly, and as accurately as possible.
- FABRIK describes that avoid the use of rotational angles or matrices, and instead finds each joint position via locating a point on a line. It converges in few iterations, has low computational cost and produces visually realistic poses.

Function *out_of_range_condition()* is to get new postions of joints with reference to target and determine whether the position of each joint or joint angle is out of range. 

If it is out of range, it will be pushed back within the range.

If the target is too far to be reached, robotic arm will straigten and point towards the target.





