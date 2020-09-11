import numpy as np
from vectormath import Vector3 as vector
from Classic_FK import get_0T3
from Classic_FK import get_elbow_position
from Classic_FK import get_wrist_position

joint_positions = {
                    'origin'     : vector(0, 0, 0),
                    'shoulder'   : vector(13.7, 0, 0),
                    'elbow'      : vector(13.7, -29.4828, -9.5795),
                    'wrist'       : vector(13.7, -54.3053, -17.6449)
                  }

def get_oc_angle():
    theta3 = -1*np.arccos((joint_positions['elbow'].length**2 - 1148.69) / (849.4*np.sin(np.radians(72))))
    theta_arm_oc = theta3 + np.pi/2
    return theta3, theta_arm_oc

def get_ud_angle(t3):
    R = np.sqrt((np.sin(np.radians(72))*np.sin(t3))**2 + (np.cos(np.radians(72)))**2)
    phi = np.arctan(1/(np.sin(t3)*np.tan(np.radians(72))))

    theta_arm_ud = np.arcsin(joint_positions['elbow'][2] / ((-31)*R)) + phi
    return theta_arm_ud

def get_elbow_angle():
    intermediate = (1642.21 - ((joint_positions['wrist']-joint_positions['shoulder']).length)**2)/1618.2
    if(intermediate + 1) < 0.0001: theta5 = np.pi
    else: theta5 = np.arccos((1642.21 - ((joint_positions['wrist']-joint_positions['shoulder']).length)**2)/1618.2)
    theta_elbow = np.pi - theta5
    return theta_elbow

def get_uturn_angle(ts,tud,toc,te,t):
    a = get_0T3([ts, tud, toc])
    a11, a12 = a[0,0], a[0,1]
    a21, a22 = a[1,0], a[1,1]
    a31, a32 = a[2,0], a[2,1]
    zw, yw, xw = joint_positions['wrist'][2], joint_positions['wrist'][1], joint_positions['wrist'][0]
    ys, xs = joint_positions['shoulder'][1], joint_positions['shoulder'][0]

    lala1 = a11*(xw-xs) + a21*(yw-ys) + a31*zw
    lala2 = a12*(xw-xs) + a22*(yw-ys) + a32*zw
    denominator = -26.1*np.sin(te + np.pi)

    test_1 = -1*np.arccos(lala1/denominator)
    test_2 = np.arcsin(lala2/denominator)

    theta_uturn_c = test_1 + np.pi/2
    theta_uturn_s = test_2 + np.pi/2
    test_list = [theta_uturn_c, theta_uturn_s, 'stop']

    for i in test_list:
        if(i == 'stop'):
            theta_uturn = np.radians(-361)
        else:
            testing = [ts, tud, toc, i, te]
            test = vector(get_wrist_position(testing))
            if ((test - t).length < 1):
                theta_uturn = i
                break

    return theta_uturn

def out_of_range_condition(t):
    a = 31
    b = (t - joint_positions['shoulder']).length
    c = (a/b)*(t - joint_positions['shoulder']) + joint_positions['shoulder']  # new_elbow
    joint_positions['elbow'] = c

    theta_shoulder = np.arcsin(joint_positions['shoulder'][1]/13.7) # theta_shoulder = 0

# new_elbow judgement        
    theta3, theta_arm_oc = get_oc_angle()
    theta_arm_ud = get_ud_angle(theta3)

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
        joint_positions['elbow']= vector(arr)
        c = joint_positions['elbow']

        theta3, theta_arm_oc = get_oc_angle()
        theta_arm_ud = get_ud_angle(theta3)

    d = 26.1
    e = (t - c).length 
    f = (d/e)*(t - c) + c  # new_wrist
    joint_positions['wrist'] = f

# new_wrist judgement        
    theta_elbow = get_elbow_angle()
    theta_uturn = get_uturn_angle(theta_shoulder, theta_arm_ud, theta_arm_oc, theta_elbow, t)
    if ((np.degrees(theta_uturn) > 90) or (np.degrees(theta_uturn < 0))):
        if(np.degrees(theta_uturn) > 90):
            theta_uturn = np.radians(90)
        else:
            theta_uturn = 0

        arr = get_wrist_position([theta_shoulder, theta_arm_ud, theta_arm_oc, theta_uturn, theta_elbow])
        f = vector(arr)
        joint_positions['wrist'] = f

    angle_rad = [theta_shoulder, theta_arm_ud, theta_arm_oc, theta_uturn, theta_elbow]
    angle_deg = np.degrees(angle_rad)
    return(angle_deg)

def iteration(t,tolerance):
    iterations = 0
    previous_iteration = vector(0,0,0)
    current_iteration = vector(0,0,0)
    vector_1 = vector(0,0,0)
    vector_2 = vector(0,0,0)
    need_test = False

    if (((t - joint_positions['elbow']).cross(joint_positions['wrist'] - joint_positions['elbow']).length <= 0.1) or ((t - joint_positions['elbow']).length <= 0.1)):
        arr = get_elbow_position([0, 0.1, 0.1])
        joint_positions['elbow'] = vector(arr)
        arr = get_wrist_position([0, 0.1, 0.1, 0.1, 1])
        joint_positions['wrist'] = vector(arr)

    while((need_test) or ((joint_positions['wrist'] - t).length > tolerance)):
    # _______________________________________________________________________________________________
    #________________________ Forward reaching, from target to reference_____________________________
    # _______________________________________________________________________________________________
        joint_positions['wrist'] = t

        len_share = 26.1 / (joint_positions['wrist'] - joint_positions['elbow']).length
        joint_positions['elbow'] = (1-len_share)*joint_positions['wrist'] + len_share*joint_positions['elbow']

        len_share = 31 / (joint_positions['elbow'] - joint_positions['shoulder']).length
        joint_positions['shoulder'] = (1-len_share)*joint_positions['elbow'] + len_share*joint_positions['shoulder']

        len_share = 13.7 / (joint_positions['shoulder'] - joint_positions['origin']).length
        joint_positions['origin'] = (1-len_share)*joint_positions['shoulder'] + len_share*joint_positions['origin']

    # ________________________________________________________________________________________________
    # ____________________________Forward reaching ends here__________________________________________
    # ________________________________________________________________________________________________

# -------------------------------------------------------------------------------------------------------------------------------

    #_________________________________________________________________________________________________
    #_________________________Backward reaching, from reference to target_____________________________
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

# --------------------------------------------------------------------------------------------------------------------------------

    # _______________________________________________________________________________________________
    # ____________________________________Starts adjustment of elbow_________________________________
    # _______________________________________________________________________________________________

        len_share = 31 / (joint_positions['shoulder'] - joint_positions['elbow']).length
        joint_positions['elbow'] = (1-len_share)*joint_positions['shoulder'] + len_share*joint_positions['elbow']

        theta3, theta_arm_oc = get_oc_angle()

        theta_arm_ud = get_ud_angle(theta3)

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

        theta3, theta_arm_oc = get_oc_angle()

        theta_arm_ud = get_ud_angle(theta3)

    # _______________________________________________________________________________________________
    # _________________________________Adjustment of elbow ends here_________________________________
    # _______________________________________________________________________________________________
    # _________________________________Starts adjustment of wrist____________________________________
    # _______________________________________________________________________________________________

        len_share = 26.1 / (joint_positions['elbow'] - joint_positions['wrist']).length
        joint_positions['wrist'] = (1-len_share)*joint_positions['elbow'] + len_share*joint_positions['wrist']

        theta_elbow = get_elbow_angle()

        theta_uturn = get_uturn_angle(theta_shoulder, theta_arm_ud, theta_arm_oc, theta_elbow, t)
        need_test = False

    # _______________________________________________________________________________________________
    # ________________________________Test if wrist is out of boundary_______________________________
    # _______________________________________________________________________________________________

        if(np.degrees(theta_uturn)<0):
            V = joint_positions['elbow'] - joint_positions['wrist']
            k = joint_positions['shoulder'] - joint_positions['elbow']
            k = k/(k.length)
            Vrot = V*np.cos(-theta_uturn) + (k.cross(V))*np.sin(-theta_uturn) + k*(k.dot(V))*(1-np.cos(-theta_uturn))
            joint_positions['elbow'] = joint_positions['wrist'] + Vrot
            need_test = True

        if(np.degrees(theta_uturn)>90):
            theta_uturn = 0
            theta_arm_ud = 0

        iterations += 1
        if(iterations > 200):
            raise ValueError('Target is unreachable (iterations > 200)')
            break

        current_iteration = joint_positions['wrist']
        vector_2 = current_iteration - previous_iteration
        if((vector_2 - vector_1).length == 0):
            raise ValueError('Caught in loop hole')
            break

    angle_rad = [theta_shoulder, theta_arm_ud, theta_arm_oc, theta_uturn, theta_elbow]
    angle_deg = np.degrees(angle_rad)
    print('Iterations = ',iterations)
    return(angle_deg)

def inverse(t,tol=0.1):
    target = vector(t)
    tolerance = tol

# ____________________________________________________________________________________________________________
# _______________________________________For Unreachable Targets______________________________________________
# ____________________________________________________________________________________________________________

    if((t - joint_positions['shoulder']).length > 57.1):
        angle = out_of_range_condition(target)
        print(angle)
        print('Target is Unreachable')

    else:
        angle = iteration(target,tolerance)
        print(angle)
