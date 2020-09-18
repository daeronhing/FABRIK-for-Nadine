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

import numpy as np

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

def generate():
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
        bin = 255*params[i]/constraints[i]
        converted_params.append(bin)
    return converted_params

def mjtg(current, setpoint, frequency, move_time):
    trajectory = []
    timefreq = int(move_time * frequency)

    for time in range(1, timefreq+1):
        trajectory.append(
            current + (setpoint - current) *
            (10.0 * (time/timefreq)**3
             - 15.0 * (time/timefreq)**4
             + 6.0 * (time/timefreq)**5))
    return trajectory

def trajectory(setpoint, frequency = 30, move_time = 2):
    trajectory = []
    current = ['Right shoulder up/down', 'Right arm up/down', 'Right arm open/close', 'Right upperarm turn', 'Right elbow', 'Right forearm turn', 'Right wrist']
    current = get(current)
    for i in range(len(setpoint)):
        traj = mjtg(current[i], setpoint[i], frequency, move_time)
        arr = np.array(traj)
        arr = arr.reshape(1,(frequency*move_time))
        trajectory.append(arr)
    return(trajectory)