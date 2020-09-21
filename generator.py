test = {
        'Eyelid'                :   128,
        'Eyebrows'              :   128,
        'Eye up/down'           :   0,
        'Eye left/right'        :   128,
        'Mouth'                 :   0,
        'O'                     :   0,
        'Cheek up/dowmn'        :   0,
        'Neck twist left'       :   0,
        'Neck twist right'      :   128,
        'Neck left/right'       :   128,
        'Body twist left'       :   128,
        'Body twist right'      :   128,
        'Body left/right'       :   128,
        'Left shoulder up/down' :   0,
        'Right shoulder up/down':   0,
        'Left arm up/down'      :   0,
        'Right arm up/down'     :   0,
        'Left arm open/close'   :   0,
        'Right arm open/close'  :   0,
        'Left upperarm turn'    :   0,
        'Right upperarm turn'   :   0,
        'Left elbow'            :   0,
        'Right elbow'           :   0,
        'Left forearm turn'     :   0,
        'Right forearm turn'    :   0,
        'Left wrist'            :   0,
        'Right wrist'           :   0,
        'Ch217'                 :   0,
        }

joints = ['Right shoulder up/down','Right arm up/down','Right arm open/close','Right upperarm turn','Right elbow','Right forearm turn','Right wrist']

import numpy as np
import os.path

def modify(x,y):
    test[x] = y
    return None

def get(x):
    if (type(x) != list): 
        return test[x]
    else:
        current_joints = []
        for joint in x:
            value = get(joint)
            current_joints.append(value)
        return(current_joints)

def dict_values():
    a = list(test.values())
    a = str(a)
    a = a.replace(' ','')
    a = a.replace('[', '')
    a = a.replace(']','')
    return a

def convertor(params):
    constraints = [9, 30, 32, 90, 105, 120, 70]
    converted_params = []
    for i in range(len(params)):
        bin = round(255*params[i]/constraints[i])
        if bin>255: bin = 255
        elif bin<0 : bin = 0
        converted_params.append(bin)
    return converted_params

def mjtg(current, setpoint, frequency, move_time):
    traj = []
    timefreq = int(move_time * frequency)

    for time in range(1, timefreq+1):
        value = round(current + (setpoint - current) * (10.0*(time/timefreq)**3 - 15.0*(time/timefreq)**4 + 6.0*(time/timefreq)**5))
        traj.append(value)
    return traj

def create_trajectory(setpoint, frequency, move_time):
    trajectory_list = []
    current = ['Right shoulder up/down', 'Right arm up/down', 'Right arm open/close', 'Right upperarm turn', 'Right elbow', 'Right forearm turn', 'Right wrist']
    current = get(current)
    for i in range(len(setpoint)):
        traj = mjtg(current[i], setpoint[i], frequency, move_time)
        arr = np.array(traj)
        trajectory_list.append(arr)
    return(trajectory_list)

def generate_line(h,m,s,f):
    time_stamp = str('{:02}:{:02}:{:02}.{:02}').format(h,m,s,f)
    line = time_stamp + ',' + dict_values()
    return line

def generate(setpoint, frequency = 30, move_time = 2):
    trajectory = create_trajectory(setpoint, frequency, move_time)
    listfile = open("C:\\Users\\IMI-Intern-Soon Hing\\Desktop\\Nadine\\listfile.txt", 'w', newline = '')
    scriptpath = os.path.realpath("listfile.txt")
    listfile.write(scriptpath)
    listfile.write('\n')
    h, m, s, f = 0, 0, 0, 0

    for i in range(frequency*move_time):
        shoulder = trajectory[0][i]
        arm_ud   = trajectory[1][i]
        arm_oc   = trajectory[2][i]
        u_turn   = trajectory[3][i]
        elbow    = trajectory[4][i]
        f_turn   = trajectory[5][i]
        wrist    = trajectory[6][i]

        num = [shoulder, arm_ud, arm_oc, u_turn, elbow, f_turn, wrist]
        global joints
        for j in range(len(num)):
            modify(joints[j],num[j])

        line = generate_line(h,m,s,f)
        listfile.writelines(line)
        listfile.write('\n')

        if (f<29):
            f += 1
        else:
            f = 0
            s += 1

    listfile.close()