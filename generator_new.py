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
        'Left arm up/down'      :   128,
        'Right arm up/down'     :   128,
        'Left arm open/close'   :   128,
        'Right arm open/close'  :   128,
        'Left upperarm turn'    :   128,
        'Right upperarm turn'   :   128,
        'Left elbow'            :   128,
        'Right elbow'           :   128,
        'Left forearm turn'     :   0,
        'Right forearm turn'    :   0,
        'Left wrist'            :   0,
        'Right wrist'           :   0,
        'Ch217'                 :   0,
        }

joints = ['Right shoulder up/down','Right arm up/down','Right arm open/close','Right upperarm turn','Right elbow','Right forearm turn','Right wrist']
via_points = []

import os.path
import matplotlib.pyplot as plt
from FABRIK_optimized import out_of_range_condition

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
    constraints = [9, 32, 30, 90, 105, 120, 70]
    converted_params = []
    for i in range(len(params)):
        bin = round(255*params[i]/constraints[i])
        if bin>255: bin = 255
        elif bin<0: bin = 0
        converted_params.append(bin)
    return converted_params

def convert_to_angle(params):
    constraints = [9, 32, 30, 90, 105, 120, 70]
    converted_params = []
    for i in range(len(params)):
        deg = (params[i]/255)*constraints[i]
        converted_params.append(deg)
    return converted_params

def move(angle, time = 3):
    bit = convertor(angle)
    while (len(bit)<7):
        bit.append(0)
    bit.append([time])
    global via_points
    via_points.append(bit)

def hold(time):
    global via_points
    via_points.append(['hold', time])

def mjtg(current, setpoint, frequency, move_time):
    traj = []
    timefreq = int(move_time * frequency)

    for time in range(1, timefreq+1):
        value = round(current + (setpoint - current) * (10.0*(time/timefreq)**3 - 15.0*(time/timefreq)**4 + 6.0*(time/timefreq)**5))
        traj.append(value)
    return traj

def create_trajectory(setpoints, frequency):
    trajectory_list = []
    current = ['Right shoulder up/down', 'Right arm up/down', 'Right arm open/close', 'Right upperarm turn', 'Right elbow', 'Right forearm turn', 'Right wrist']
    current = get(current)

    for via_point in setpoints:
        temp_list = []
        if via_point[0] == 'hold':
            for i in range(len(current)):
                traj = []
                for t in range(frequency*via_point[1]):
                    traj.append(current[i])
                temp_list.append(traj)
        else:
            for i in range(len(via_point)-1):
                traj = mjtg(current[i], via_point[i], frequency, via_point[-1][0])
                temp_list.append(traj)
                current[i] = temp_list[i][-1]

        if not trajectory_list:
            trajectory_list = temp_list
        else:
            for j in range(len(temp_list)):
                trajectory_list[j] = trajectory_list[j] + temp_list[j]
    global via_points
    via_points = []
    return (trajectory_list)

def generate_line(h,m,s,f):
    time_stamp = str('{:02}:{:02}:{:02}.{:02}').format(h,m,s,f)
    line = time_stamp + ',' + dict_values()
    return line

def generate(frequency = 30):
    global via_points
    trajectory = create_trajectory(via_points, frequency)
    listfile = open("C:\\Users\\IMI-Intern-Soon Hing\\Desktop\\Nadine\\listfile.txt", 'w', newline = '')
    scriptpath = os.path.realpath("listfile.txt")
    listfile.write(scriptpath)
    listfile.write('\n')
    h, m, s, f = 0, 0, 0, 0

    for i in range(len(trajectory[0])):
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

    trajectory = create_trajectory([[0,128,128,128,128,128,0,[2]]], frequency)

    for i in range(len(trajectory[0])):
        shoulder = trajectory[0][i]
        arm_ud   = trajectory[1][i]
        arm_oc   = trajectory[2][i]
        u_turn   = trajectory[3][i]
        elbow    = trajectory[4][i]
        f_turn   = trajectory[5][i]
        wrist    = trajectory[6][i]

        num = [shoulder, arm_ud, arm_oc, u_turn, elbow, f_turn, wrist]
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




def plot(current = 0, frequency = 30, move_time = 2):
    m = out_of_range_condition([9.976023462,-42.6523038,-31.19549077])
    joint_new_positions_bin = convertor(m)

    for k in range(7):
        setpoint = joint_new_positions_bin[k]
    
        traj = mjtg(current, setpoint, frequency, move_time)
        # Create plot.
        xaxis = [i / frequency for i in range(1, int(move_time * frequency))]
        
        plt.plot(xaxis, traj)
        plt.title("Minimum jerk trajectory")
        plt.xlabel("Time [s]")
        plt.ylabel("Angle [bits]")
        plt.show()
        
# print(plot())



