import numpy as np
from vectormath import Vector3 as vector

def getTransformMatrix(theta, d, a, alpha):
    T = np.array([[np.cos(theta) , -np.sin(theta)*np.cos(alpha) ,  np.sin(theta)*np.sin(alpha) , a*np.cos(theta)],
                  [np.sin(theta) ,  np.cos(theta)*np.cos(alpha) , -np.cos(theta)*np.sin(alpha) , a*np.sin(theta)],
                  [0             ,  np.sin(alpha)               ,  np.cos(alpha)               , d              ],
                  [0             ,  0                           ,  0                           , 1              ]
                 ])
    return T

def get_0T3(params):
    t_12 = getTransformMatrix(np.radians(90)+params[0],0,0,np.radians(90))
    t_23 = getTransformMatrix(np.radians(180)+params[1],13.7,0,np.radians(90))
    t_34 = getTransformMatrix(np.radians(-90)+params[2],0,0,np.radians(72))

    end_tip_m = t_12.dot(t_23).dot(t_34)

    return end_tip_m

def get_elbow_position(params):
    # Create the transformation matrices for the respective joints
    t_12 = getTransformMatrix(np.radians(90)+params[0],0,0,np.radians(90))
    t_23 = getTransformMatrix(np.radians(180)+params[1],13.7,0,np.radians(90))
    t_34 = getTransformMatrix(np.radians(-90)+params[2],0,0,np.radians(72))
    t_45 = getTransformMatrix(np.radians(-90)+params[3],-31,0,np.radians(90))

    # Get the overall transformation matrix
    end_tip_m = t_12.dot(t_23).dot(t_34).dot(t_45)

    # The coordinates of the end tip are the 3 upper entries in the 4th column
    pos = vector(end_tip_m[0,3],end_tip_m[1,3],end_tip_m[2,3])
    return pos

def get_wrist_position(params):
    # Create the transformation matrices for the respective joints
    t_12 = getTransformMatrix(np.radians(90)+params[0],0,0,np.radians(90))
    t_23 = getTransformMatrix(np.radians(180)+params[1],13.7,0,np.radians(90))
    t_34 = getTransformMatrix(np.radians(-90)+params[2],0,0,np.radians(72))
    t_45 = getTransformMatrix(np.radians(-90)+params[3],-31,0,np.radians(90))
    t_56 = getTransformMatrix(np.radians(180)+params[4],0,0,np.radians(90))
    t_67 = getTransformMatrix(0,-26.1,0,np.radians(90))

    # Get the overall transformation matrix
    end_tip_m = t_12.dot(t_23).dot(t_34).dot(t_45).dot(t_56).dot(t_67)

    # The coordinates of the end tip are the 3 upper entries in the 4th column
    pos = vector(end_tip_m[0,3],end_tip_m[1,3],end_tip_m[2,3])
    return pos