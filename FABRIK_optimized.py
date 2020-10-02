import numpy as np
from numpy import cos as cos
from numpy import sin as sin
from numpy import arccos as arccos
from numpy import arcsin as arcsin
from numpy import radians as rad
from numpy import degrees as deg
from vectormath import Vector3 as vector
from Classic_FK_optimized import get_0T3
from Classic_FK_optimized import get_elbow_position
from Classic_FK_optimized import get_wrist_position

joint_positions = {
                    'origin'     : vector(0, 0, 0),
                    'shoulder'   : vector(13.7, 0, 0),
                    'elbow'      : vector(13.7, -29.48275201, -9.579526826),
                    'wrist'       : vector(13.7, -54.30532708, -17.64487038)
                  }



def get_oc_angle():
    theta3 = -arccos((joint_positions['elbow'].length**2 - 1148.69) / (849.4*sin(rad(72))))
    return theta3, (theta3 + np.pi/2)

def get_ud_angle(t3):
    R = np.sqrt((sin(rad(72))*sin(t3))**2 + (cos(rad(72)))**2)
    phi = np.arctan(1/(sin(t3)*np.tan(rad(72))))

    return (arcsin(joint_positions['elbow'][2] / ((-31)*R)) + phi)

def get_elbow_angle():
    intermediate = (1642.21 - ((joint_positions['wrist']-joint_positions['shoulder']).length)**2)/1618.2
    if((intermediate<-1) and (intermediate>-1.01)): theta5 = np.pi
    else: theta5 = arccos((1642.21 - ((joint_positions['wrist']-joint_positions['shoulder']).length)**2)/1618.2)

    return (np.pi - theta5)

def get_uturn_angle(ts,tud,toc,te):
    a = get_0T3([ts, tud, toc])
    a11, a12 = a[0,0], a[0,1]
    a21, a22 = a[1,0], a[1,1]
    a31, a32 = a[2,0], a[2,1]
    zw, yw, xw = joint_positions['wrist'][2], joint_positions['wrist'][1], joint_positions['wrist'][0]
    ys, xs = joint_positions['shoulder'][1], joint_positions['shoulder'][0]

    lala1 = a11*(xw-xs) + a21*(yw-ys) + a31*zw
    lala2 = a12*(xw-xs) + a22*(yw-ys) + a32*zw
    denominator = -26.1*sin(te + np.pi)

    try:
        test_1 = -arccos(lala1/denominator)
    except:
        test_1 = 0

    try:
        test_2 = arcsin(lala2/denominator)
    except:
        test_2 = 0

    theta_uturn_c = test_1 + np.pi/2
    theta_uturn_s = test_2 + np.pi/2
    test_list = [theta_uturn_c, theta_uturn_s, 'stop']


    for i in test_list:
        if(i == 'stop'):
            theta_uturn = rad(-361)
        else:
            testing = [ts, tud, toc, i, te]
            test = vector(get_wrist_position(testing))
            if ((test - joint_positions['wrist']).length < 1):
                theta_uturn = i
                break

    return theta_uturn

def out_of_range_condition(t):
    b = (t - joint_positions['shoulder']).length
    c = (31/b)*(t - joint_positions['shoulder']) + joint_positions['shoulder']  # new_elbow
    joint_positions['elbow'] = c

    theta_shoulder = arcsin(joint_positions['shoulder'][1]/13.7) # theta_shoulder = 0

# new_elbow judgement        
    theta3, theta_arm_oc = get_oc_angle()
    theta_arm_ud = get_ud_angle(theta3)

    if((deg(theta_arm_ud) > 32) or (deg(theta_arm_ud) < 0) or (deg(theta_arm_oc) > 30) or (deg(theta_arm_oc) < 0)):
        if(deg(theta_arm_ud) > 32):
            theta_arm_ud = rad(32)
        elif(deg(theta_arm_ud < 0)):
            theta_arm_ud = 0
        if(deg(theta_arm_oc) > 30):
            theta_arm_oc = rad(30)
        elif(deg(theta_arm_oc) < 0):
            theta_arm_oc = 0

        arr = get_elbow_position([theta_shoulder, theta_arm_ud, theta_arm_oc])
        joint_positions['elbow']= vector(arr)
        c = joint_positions['elbow']

    e = (t - c).length 
    f = (26.1/e)*(t - c) + c  # new_wrist
    joint_positions['wrist'] = f

# new_wrist judgement        
    theta_elbow = get_elbow_angle()
    theta_uturn = get_uturn_angle(theta_shoulder, theta_arm_ud, theta_arm_oc, theta_elbow)
    if ((deg(theta_uturn) > 90) or (deg(theta_uturn < 0))):
        if(deg(theta_uturn) > 90):
            theta_uturn = rad(90)
        else:
            theta_uturn = 0

        arr = get_wrist_position([theta_shoulder, theta_arm_ud, theta_arm_oc, theta_uturn, theta_elbow])
        f = vector(arr)
        joint_positions['wrist'] = f

    return(deg([theta_shoulder, theta_arm_ud, theta_arm_oc, theta_uturn, theta_elbow]))

def iteration(t,tolerance):
    iterations = 0
    need_test = False

    if ((t - joint_positions['elbow']).length < 0.001):
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

    # ________________________________________________________________________________________________
    # ____________________________Forward reaching ends here__________________________________________
    # ________________________________________________________________________________________________

# -------------------------------------------------------------------------------------------------------------------------------

    #_________________________________________________________________________________________________
    #_________________________Backward reaching, from reference to target_____________________________
    # ________________________________________________________________________________________________

        len_share = 13.7 / (joint_positions['origin'] - joint_positions['shoulder']).length
        joint_positions['shoulder'] = (1-len_share)*joint_positions['origin'] + len_share*joint_positions['shoulder']

    # ____________________________________________________________________________________________
    # __________________test if shoulder is away from x-y plane___________________________________
    # ____________________________________________________________________________________________
        if(joint_positions['shoulder'][2] != 0):
            # rotate shoulder about y-axis, so that it stays in x-y plane
            rotation = np.arctan(joint_positions['shoulder'][2]/joint_positions['shoulder'][0])
            x = joint_positions['shoulder'][0]*cos(rotation) + joint_positions['shoulder'][2]*sin(rotation)
            z = joint_positions['shoulder'][2]*cos(rotation) - joint_positions['shoulder'][0]*sin(rotation)
            y = joint_positions['shoulder'][1]
            joint_positions['shoulder'] = vector(x,y,z)

        theta_shoulder = arcsin(joint_positions['shoulder'][1]/13.7)

    # ____________________________________________________________________________________________
    # __________________test if shoulder is out of the boundary___________________________________
    # ____________________________________________________________________________________________
        if((deg(theta_shoulder) > 9) or (deg(theta_shoulder) < 0)):
            if(deg(theta_shoulder) > 9): theta_shoulder = rad(9)
            elif(deg(theta_shoulder) < 0): theta_shoulder = rad(0)
            # rotate about z-axis, so that it stays within hardware limit
            x = 13.7*cos(theta_shoulder)
            y = 13.7*sin(theta_shoulder)
            joint_positions['shoulder'] = vector(x,y,0)

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

        if((deg(theta_arm_ud) > 32) or (deg(theta_arm_ud) < 0) or (deg(theta_arm_oc) > 30) or (deg(theta_arm_oc) < 0)):
            if(deg(theta_arm_ud) > 32):
                theta_arm_ud = rad(32)
            elif(deg(theta_arm_ud < 0)):
                theta_arm_ud = 0
            if(deg(theta_arm_oc) > 30):
                theta_arm_oc = rad(30)
            elif(deg(theta_arm_oc) < 0):
                theta_arm_oc = 0

        arr = get_elbow_position([theta_shoulder, theta_arm_ud, theta_arm_oc])
        joint_positions['elbow'] = vector(arr)

    # _______________________________________________________________________________________________
    # _________________________________Adjustment of elbow ends here_________________________________
    # _______________________________________________________________________________________________
    # _________________________________Starts adjustment of wrist____________________________________
    # _______________________________________________________________________________________________

        len_share = 26.1 / (joint_positions['elbow'] - joint_positions['wrist']).length
        joint_positions['wrist'] = (1-len_share)*joint_positions['elbow'] + len_share*joint_positions['wrist']

        theta_elbow = get_elbow_angle()

        theta_uturn = get_uturn_angle(theta_shoulder, theta_arm_ud, theta_arm_oc, theta_elbow)
        need_test = False

    # _______________________________________________________________________________________________
    # ________________________________Test if wrist is out of boundary_______________________________
    # _______________________________________________________________________________________________

        if(deg(theta_uturn)<0 and abs(deg(theta_uturn))>0.00001):
            V = joint_positions['elbow'] - joint_positions['wrist']
            k = joint_positions['shoulder'] - joint_positions['elbow']
            k = k/(k.length)
            Vrot = V*cos(-theta_uturn) + (k.cross(V))*sin(-theta_uturn) + k*(k.dot(V))*(1-cos(-theta_uturn))
            joint_positions['elbow'] = joint_positions['wrist'] + Vrot
            need_test = True

        if(deg(theta_uturn)>90):
            theta_uturn = np.pi/2
            theta_arm_ud = theta_arm_ud - rad(1)

        iterations += 1
        if(iterations > 100):
            print('Target is unreachable (iterations > 100)')
            return(deg([theta_shoulder, theta_arm_ud, theta_arm_oc, theta_uturn, theta_elbow]))

    print('Iterations = ',iterations)
    return(deg([theta_shoulder, theta_arm_ud, theta_arm_oc, theta_uturn, theta_elbow]))


# ____________________________________________________________________________________________________
# _______________________________________Main Function________________________________________________
# ____________________________________________________________________________________________________
def inverse(t,tol=0.1):
    target = vector(t)

    if((t - joint_positions['shoulder']).length > 57.1):
        print('Out of range')
        return out_of_range_condition(target)

    elif((t - joint_positions['wrist']).length < tol):
        print('Iterations = 0')
        try:
            return(deg([theta_shoulder, theta_arm_ud, theta_arm_oc, theta_uturn, theta_elbow]))
        except NameError:
            return([0,0,0,0,0])

    else:
        return iteration(target,tol)
