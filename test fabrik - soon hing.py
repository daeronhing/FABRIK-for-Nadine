import numpy as np
from vectormath import Vector3 as vector

joint_positions = {
                    'origin'     : vector(0, 0, 0),
                    'shoulder'   : vector(13.7, 0, 0),
                    'elbow'      : vector(13.7, -29.4828, -9.5795),
                    'wrist'       : vector(13.7, -54.3053, -17.6449)
                  }



tolerance = 0.1


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
    pos = np.array([end_tip_m[0,3],end_tip_m[1,3],end_tip_m[2,3]])
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
    pos = np.array([end_tip_m[0,3],end_tip_m[1,3],end_tip_m[2,3]])
    return pos



def iterate(target, tolerance):
    target = vector(target)
    tolerance = tolerance
    iterations = 0
    previous_iteration = vector(0,0,0)
    current_iteration = vector(0,0,0)
    vector_1 = vector(0,0,0)
    vector_2 = vector(0,0,0)

    if((target - joint_positions['shoulder']).length >= 57.1):
        len_share = 13.7 / (joint_positions['origin'] - target).length
        joint_positions['shoulder'] = (1-len_share)*joint_positions['origin'] + len_share*target

    # ____________________________________________________________________________________________
    # __________________test if shoulder is away from x-y plane___________________________________
    # ____________________________________________________________________________________________
        if(joint_positions['shoulder'][2] != 0):
            # rotate shoulder about y-axis, so that it stays in x-y plane
            rotation = np.arctan(joint_positions['shoulder'][2]/joint_positions['shoulder'][0])
            x = joint_positions['shoulder'][0]*np.cos(rotation) + joint_positions['shoulder'][2]*np.sin(rotation)
            z = joint_positions['shoulder'][2]*np.cos(rotation) - joint_positions['shoulder'][0]*np.sin(rotation)
            y = joint_positions['shoulder'][1]
            joint_positions['shoulder'] = vector(x,y,z)

        theta_shoulder = np.arcsin(joint_positions['shoulder'][1]/13.7)

    # ____________________________________________________________________________________________
    # __________________test if shoulder is out of the boundary___________________________________
    # ____________________________________________________________________________________________
        if((np.degrees(theta_shoulder) > 9) or (np.degrees(theta_shoulder) < 0)):
            if(np.degrees(theta_shoulder) > 9): theta_shoulder = np.radians(9)
            elif(np.degrees(theta_shoulder) < 0): theta_shoulder = np.radians(0)
            # rotate about z-axis, so that it stays within hardware limit
            x = 13.7*np.cos(theta_shoulder)
            y = 13.7*np.sin(theta_shoulder)
            z = 0
            joint_positions['shoulder'] = vector(x,y,z)

    # _______________________________________________________________________________________________
    # ____________________________________Adjustment of shoulder ends here___________________________
    # _______________________________________________________________________________________________
    # ____________________________________Starts adjustment of elbow_________________________________
    # _______________________________________________________________________________________________

        len_share = 31 / (joint_positions['shoulder'] - target).length
        joint_positions['elbow'] = (1-len_share)*joint_positions['shoulder'] + len_share*target

        theta3 = -1*np.arccos((joint_positions['elbow'].length**2 - 1148.69) / (849.4*np.sin(np.radians(72))))
        theta_arm_oc = theta3 + np.pi/2

        R = np.sqrt((np.sin(np.radians(72))*np.sin(theta3))**2 + (np.cos(np.radians(72)))**2)
        phi = np.arctan(1/(np.sin(theta3)*np.tan(np.radians(72))))

        theta2 = np.arcsin(joint_positions['elbow'][2] / ((-31)*R)) + phi
        theta_arm_ud = theta2

    # _______________________________________________________________________________________________
    # _________________________________Test if position of elbow is out of boundary__________________
    # _______________________________________________________________________________________________

        if((np.degrees(theta_arm_ud) > 32) or (np.degrees(theta_arm_ud) < 0) or (np.degrees(theta_arm_oc) > 30) or (np.degrees(theta_arm_oc) < 0)):
            if(np.degrees(theta_arm_ud) > 32):
                theta_arm_ud = np.radians(32)
            elif(np.degrees(theta_arm_ud < 0)):
                theta_arm_ud = 0
            if(np.degrees(theta_arm_oc) > 30):
                theta_arm_oc = np.radians(30)
            elif(np.degrees(theta_arm_oc) < 0):
                theta_arm_oc = 0

            arr = get_elbow_position([theta_shoulder, theta_arm_ud, theta_arm_oc, 0])
            joint_positions['elbow'] = vector(arr)

            theta3 = -1*np.arccos((joint_positions['elbow'].length**2 - 1148.69) / (849.4*np.sin(np.radians(72))))
            theta_arm_oc = theta3 + np.pi/2

            R = np.sqrt((np.sin(np.radians(72))*np.sin(theta3))**2 + (np.cos(np.radians(72)))**2)
            phi = np.arctan(1/(np.sin(theta3)*np.tan(np.radians(72))))

            theta2 = np.arcsin(joint_positions['elbow'][2] / ((-31)*R)) + phi
            theta_arm_ud = theta2

    # _______________________________________________________________________________________________
    # _________________________________Adjustment of elbow ends here_________________________________
    # _______________________________________________________________________________________________
    # _________________________________Starts adjustment of wrist____________________________________
    # _______________________________________________________________________________________________

        len_share = 26.1 / (joint_positions['elbow'] - target).length
        joint_positions['wrist'] = (1-len_share)*joint_positions['elbow'] + len_share*target

        theta5 = np.arccos((1642.21 - ((joint_positions['wrist']-joint_positions['shoulder']).length)**2)/1618.2)
        theta_elbow = np.pi - theta5

        a = get_0T3([theta_shoulder, theta_arm_ud, theta_arm_oc])
        a11, a12 = a[0,0], a[0,1]
        a21, a22 = a[1,0], a[1,1]
        a31, a32 = a[2,0], a[2,1]
        zw, yw, xw = joint_positions['wrist'][2], joint_positions['wrist'][1], joint_positions['wrist'][0]
        ys, xs = joint_positions['shoulder'][1], joint_positions['shoulder'][0]

        lala1 = a11*(xw-xs) + a21*(yw-ys) + a31*zw
        lala2 = a12*(xw-xs) + a22*(yw-ys) + a32*zw
        denominator = -26.1*np.sin(theta_elbow + np.pi)

        test_1 = -1*np.arccos(lala1/denominator)
        test_2 = np.arcsin(lala2/denominator)

        theta_uturn_c = test_1 + np.pi/2
        theta_uturn_s = test_2 + np.pi/2
        test_list = [theta_uturn_c, theta_uturn_s, 'stop']

        for i in test_list:
            if(i == 'stop'):
                theta_uturn = np.radians(-361)
            else:
                testing = [theta_shoulder, theta_arm_ud, theta_arm_oc, i, theta_elbow]
                test = vector(get_wrist_position(testing))
                if ((test - target).length < 1):
                    theta_uturn = i
                    break

        if ((np.degrees(theta_uturn) > 90) or (np.degrees(theta_uturn < 0))):
            if(np.degrees(theta_uturn) > 90):
                theta_uturn = np.radians(90)
            else:
                theta_uturn = 0

        arr = get_wrist_position([theta_shoulder, theta_arm_ud, theta_arm_oc, theta_uturn, theta_elbow])
        joint_positions['wrist'] = vector(arr)

    else:
        while(joint_positions['wrist'] - target).length > tolerance:
            # ___________________________________________________________________________________________
            #________________________ Forward reaching, from target to reference_________________________
            # ___________________________________________________________________________________________
            joint_positions['wrist'] = target

            len_share = 26.1 / (joint_positions['wrist'] - joint_positions['elbow']).length
            joint_positions['elbow'] = (1-len_share)*joint_positions['wrist'] + len_share*joint_positions['elbow']

            len_share = 31 / (joint_positions['elbow'] - joint_positions['shoulder']).length
            joint_positions['shoulder'] = (1-len_share)*joint_positions['elbow'] + len_share*joint_positions['shoulder']

            len_share = 13.7 / (joint_positions['shoulder'] - joint_positions['origin']).length
            joint_positions['origin'] = (1-len_share)*joint_positions['shoulder'] + len_share*joint_positions['origin']

        # ________________________________________________________________________________________________
        # ____________________________Forward reaching ends here__________________________________________
        # ________________________________________________________________________________________________


        #_________________________________________________________________________________________________
        #_________________________________Backward reaching_______________________________________________
        #_____________________________from reference to target____________________________________________
        # ________________________________________________________________________________________________
            joint_positions['origin'] = vector(0,0,0)

            len_share = 13.7 / (joint_positions['origin'] - joint_positions['shoulder']).length
            joint_positions['shoulder'] = (1-len_share)*joint_positions['origin'] + len_share*joint_positions['shoulder']

        # ____________________________________________________________________________________________
        # __________________test if shoulder is away from x-y plane___________________________________
        # ____________________________________________________________________________________________
            if(joint_positions['shoulder'][2] != 0):
            # rotate shoulder about y-axis, so that it stays in x-y plane
            rotation = np.arctan(joint_positions['shoulder'][2]/joint_positions['shoulder'][0])
            x = joint_positions['shoulder'][0]*np.cos(rotation) + joint_positions['shoulder'][2]*np.sin(rotation)
            z = joint_positions['shoulder'][2]*np.cos(rotation) - joint_positions['shoulder'][0]*np.sin(rotation)
            y = joint_positions['shoulder'][1]
            joint_positions['shoulder'] = vector(x,y,z)

            theta_shoulder = np.arcsin(joint_positions['shoulder'][1]/13.7)

        # ____________________________________________________________________________________________
        # __________________test if shoulder is out of the boundary___________________________________
        # ____________________________________________________________________________________________
            if((np.degrees(theta_shoulder) > 9) or (np.degrees(theta_shoulder) < 0)):
            if(np.degrees(theta_shoulder) > 9): theta_shoulder = np.radians(9)
            elif(np.degrees(theta_shoulder) < 0): theta_shoulder = np.radians(0)
            # rotate about z-axis, so that it stays within hardware limit
            x = 13.7*np.cos(theta_shoulder)
            y = 13.7*np.sin(theta_shoulder)
            z = 0
            joint_positions['shoulder'] = vector(x,y,z)

        # _______________________________________________________________________________________________
        # ____________________________________Adjustment of shoulder ends here___________________________
        # _______________________________________________________________________________________________
        # ____________________________________Starts adjustment of elbow_________________________________
        # _______________________________________________________________________________________________

            len_share = 31 / (joint_positions['shoulder'] - joint_positions['elbow']).length
            joint_positions['elbow'] = (1-len_share)*joint_positions['shoulder'] + len_share*joint_positions['elbow']

            theta3 = -1*np.arccos((joint_positions['elbow'].length**2 - 1148.69) / (849.4*np.sin(np.radians(72))))
            theta_arm_oc = theta3 + np.pi/2

            R = np.sqrt((np.sin(np.radians(72))*np.sin(theta3))**2 + (np.cos(np.radians(72)))**2)
            phi = np.arctan(1/(np.sin(theta3)*np.tan(np.radians(72))))

            theta2 = np.arcsin(joint_positions['elbow'][2] / ((-31)*R)) + phi
            theta_arm_ud = theta2

        # _______________________________________________________________________________________________
        # _________________________________Test if position of elbow is out of boundary__________________
        # _______________________________________________________________________________________________

            if((np.degrees(theta_arm_ud) > 32) or (np.degrees(theta_arm_ud) < 0) or (np.degrees(theta_arm_oc) > 30) or (np.degrees(theta_arm_oc) < 0)):
            if(np.degrees(theta_arm_ud) > 32):
                theta_arm_ud = np.radians(32)
            elif(np.degrees(theta_arm_ud < 0)):
                theta_arm_ud = 0
            if(np.degrees(theta_arm_oc) > 30):
                theta_arm_oc = np.radians(30)
            elif(np.degrees(theta_arm_oc) < 0):
                theta_arm_oc = 0

            arr = get_elbow_position([theta_shoulder, theta_arm_ud, theta_arm_oc, 0])
            joint_positions['elbow'] = vector(arr)

            theta3 = -1*np.arccos((joint_positions['elbow'].length**2 - 1148.69) / (849.4*np.sin(np.radians(72))))
            theta_arm_oc = theta3 + np.pi/2

            R = np.sqrt((np.sin(np.radians(72))*np.sin(theta3))**2 + (np.cos(np.radians(72)))**2)
            phi = np.arctan(1/(np.sin(theta3)*np.tan(np.radians(72))))

            theta2 = np.arcsin(joint_positions['elbow'][2] / ((-31)*R)) + phi
            theta_arm_ud = theta2

        # ______________________________________________________________________________________________
        # _____________________________To make sure theta_uturn does not go negative____________________
        # ______________________________________________________________________________________________
            if(joint_positions['elbow'][0] < target[0]):
                theta_arm_oc = np.radians(30)
                arr = get_elbow_position([theta_shoulder, theta_arm_ud, theta_arm_oc, 0])
                joint_positions['elbow'] = vector(arr)

        # _______________________________________________________________________________________________
        # _________________________________Adjustment of elbow ends here_________________________________
        # _______________________________________________________________________________________________
        # _________________________________Starts adjustment of wrist____________________________________
        # _______________________________________________________________________________________________

            len_share = 26.1 / (joint_positions['elbow'] - joint_positions['wrist']).length
            joint_positions['wrist'] = (1-len_share)*joint_positions['elbow'] + len_share*joint_positions['wrist']

            theta5 = np.arccos((1642.21 - ((joint_positions['wrist']-joint_positions['shoulder']).length)**2)/1618.2)
            theta_elbow = np.pi - theta5

            a = get_0T3([theta_shoulder, theta_arm_ud, theta_arm_oc])
            a11, a12 = a[0,0], a[0,1]
            a21, a22 = a[1,0], a[1,1]
            a31, a32 = a[2,0], a[2,1]
            zw, yw, xw = joint_positions['wrist'][2], joint_positions['wrist'][1], joint_positions['wrist'][0]
            ys, xs = joint_positions['shoulder'][1], joint_positions['shoulder'][0]

            lala1 = a11*(xw-xs) + a21*(yw-ys) + a31*zw
            lala2 = a12*(xw-xs) + a22*(yw-ys) + a32*zw
            denominator = -26.1*np.sin(theta_elbow + np.pi)

            test_1 = -1*np.arccos(lala1/denominator)
            test_2 = np.arcsin(lala2/denominator)

            theta_uturn_c = test_1 + np.pi/2
            theta_uturn_s = test_2 + np.pi/2
            test_list = [theta_uturn_c, theta_uturn_s, 'stop']

            for i in test_list:
                if(i == 'stop'):
                    theta_uturn = np.radians(-361)
                else:
                    testing = [theta_shoulder, theta_arm_ud, theta_arm_oc, i, theta_elbow]
                    test = vector(get_wrist_position(testing))
                    if ((test - target).length < 1):
                        theta_uturn = i
                        break

        # _______________________________________________________________________________________________
        # ________________________________Test if wrist is out of boundary_______________________________
        # _______________________________________________________________________________________________

            if ((np.degrees(theta_uturn) > 90) or (np.degrees(theta_uturn < 0))):
                if(np.degrees(theta_uturn) > 90):
                    theta_arm_ud = 0
                    theta_uturn = 0
                else:
                    theta_arm_oc = np.radians(30)
                    theta_uturn = np.pi/2

            arr = get_wrist_position([theta_shoulder, theta_arm_ud, theta_arm_oc, theta_uturn, theta_elbow])
            joint_positions['wrist'] = vector(arr)

            iterations += 1
            if(iterations > 200):
                raise ValueError('Target is unreachable')
                break

            current_iteration = joint_positions['wrist']
            vector_2 = current_iteration - previous_iteration
            if((vector_2 - vector_1).length == 0):
                raise ValueError('Caught in loop')
                break
            else:
                previous_iteration = current_iteration
                vector_1 = vector_2