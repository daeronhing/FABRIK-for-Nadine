# Forward Kinematics

Forward kinematics refers to the use of kinematic equations of a robot to compute the position of the end-effector from specified values for the joint parameters.
The kinematics equations for the series chain of a robot are obtained using rigid transformation to characterize the relative movement allowed at each joint and define the dimensions of each link. The result is a sequence of rigid transformations alternating joint and link transformations from the base of the chain to its end link.
To get the transformation matrix of each link, simply input the required DH parameters of each joint into the function *getTransformMatrix()*.
Function *get_0T3()* will return result matrix of transformation from base to 3rd link.
The rest are used to obtain the position of elbow or wrist.
