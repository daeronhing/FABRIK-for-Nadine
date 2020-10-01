# Forward Kinematics

- Refers to the use of kinematic equations of a robot to compute the position of the end-effector from specified values for the joint parameters.
- Obtained using rigid transformation to characterize the relative movement allowed at each joint and define the dimensions of each link.
- Result is a sequence of rigid transformations of alternating joints and link transformations from the base to its end link.

Use *getTransformMatrix()* to get the matrix of rigid transformation of each link by inputting required DH parameters.

Function *get_0T3()* will return result matrix of transformation from base to 3rd link.

The rest are used to obtain the position of elbow or wrist.
