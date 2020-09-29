from numpy import cos as cos
from numpy import sin as sin
from numpy import radians as rad
from numpy import array as arr
from numpy import pi as pi
from vectormath import Vector3 as vector

oc = rad(72)
half = pi/2

def getTransformMatrix(theta, d, a, alpha):
    T = arr([[cos(theta) , -sin(theta)*cos(alpha) ,  sin(theta)*sin(alpha) , a*cos(theta)],
             [sin(theta) ,  cos(theta)*cos(alpha) , -cos(theta)*sin(alpha) , a*sin(theta)],
             [0          ,  sin(alpha)            ,  cos(alpha)            , d           ],
             [0          ,  0                     ,  0                     , 1           ]
            ])
    return T

def get_0T3(params):
    t_12 = getTransformMatrix(params[0]+half ,    0 , 0 , half)
    t_23 = getTransformMatrix(params[1]+pi   , 13.7 , 0 , half)
    t_34 = getTransformMatrix(params[2]-half ,    0 , 0 , oc)

    return (t_12.dot(t_23).dot(t_34))

def get_elbow_position(params):
    # Create the transformation matrices for the respective joints
    t_12 = getTransformMatrix(params[0]+half ,    0 , 0 , half)
    t_23 = getTransformMatrix(params[1]+pi   , 13.7 , 0 , half)
    t_34 = getTransformMatrix(params[2]-half ,    0 , 0 , oc)
    t_45 = getTransformMatrix(         -half ,  -31 , 0 , half)

    # Get the overall transformation matrix
    end_tip_m = t_12.dot(t_23).dot(t_34).dot(t_45)

    # The coordinates of the end tip are the 3 upper entries in the 4th column
    pos = vector(end_tip_m[0,3],end_tip_m[1,3],end_tip_m[2,3])
    return pos

def get_wrist_position(params):
    # Create the transformation matrices for the respective joints
    t_12 = getTransformMatrix(params[0]+half ,     0 , 0 , half)
    t_23 = getTransformMatrix(params[1]+pi   ,  13.7 , 0 , half)
    t_34 = getTransformMatrix(params[2]-half ,     0 , 0 , oc)
    t_45 = getTransformMatrix(params[3]-half ,   -31 , 0 , half)
    t_56 = getTransformMatrix(params[4]+pi   ,     0 , 0 , half)
    t_67 = getTransformMatrix(          0    , -26.1 , 0 , half)

    # Get the overall transformation matrix
    end_tip_m = t_12.dot(t_23).dot(t_34).dot(t_45).dot(t_56).dot(t_67)

    # The coordinates of the end tip are the 3 upper entries in the 4th column
    pos = vector(end_tip_m[0,3],end_tip_m[1,3],end_tip_m[2,3])
    return pos
