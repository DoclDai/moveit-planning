from utils import *
from socket import *
import numpy as np
import time
import csv
from compute_jnt_siganl import *
from other_codes.stream_motion_func import *
import numpy as np
import smoothtrajectory

UDP_IP = "192.168.50.7"
UDP_PORT = 60015

### Clock to monitor communication speed ###
start_time = time.time()

### Connect to socket ###
sock = socket(AF_INET,SOCK_DGRAM)
sock.connect((UDP_IP,UDP_PORT))
print('SOCKET CONNECTED: {:.2f}ms'.format(1000*(time.time()-start_time)))

### Send init pack ###
data = initpack()
sock.sendto(data,(UDP_IP,UDP_PORT))
print('INIT PACK SENT: {:.2f}ms'.format(1000*(time.time()-start_time)))

resp = sock.recv(132)
resp = explainRobData(resp)

### Extract joint data ###
current_jnt_data = resp[18:27]
print(f'JOINT DATA: {current_jnt_data}')
print('({:.2f}ms)'.format(1000*(time.time()-start_time)))

### Send end pack ###  
data = endpack()
sock.sendto(data,(UDP_IP,UDP_PORT))
print('END PACK SENT')

####################################################################################################################

### ---------- Get joint signal ---------- ###
jnt_traj = []
with open("smoothsignal_dipping.csv", mode ='r')as file:
  csvFile = csv.reader(file)
  for lines in csvFile:
    jnt_traj.append(lines)
dipsignal = np.asarray(jnt_traj, dtype=float)
### -------------------------------------- ###

### ---------- Get joint signal ---------- ###
# strokeNo = [139,140,141,142,143,144] # 102
startNo = 0
strokeNo = [startNo,startNo+1,startNo+2,startNo+3,startNo+4]
jnt_traj = []
with open("ALL_Dynamic_joint_trajectory20240426_1726.csv", mode ='r')as file:
  csvFile = csv.reader(file)
  for lines in csvFile:
    # print(f"type = {lines[0]}, value = {lines[0]}")
    for k in strokeNo:
        if int(lines[0]) == k:
            lineele = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            for i in range(len(lines)):
                if i == 3:
                    lineele[i-1] = (float(lines[i]) - float(lines[2]))* 57.3
                elif i != 0:
                    lineele[i-1] = float(lines[i]) * 57.3
            jnt_traj.append(lineele)
### -------------------------------------- ###
print("Drawing:")
print(*strokeNo, sep = ", ") 

smoothfunc = smoothtrajectory.initialize()
matsignal, maxjerk, numsample = smoothfunc.smoothtrajectory(jnt_traj, nargout=3)
signal1 = np.array(matsignal)
print(f"numsample = {numsample}, maxjerk = {maxjerk}")

print(f"Initiate second part:")

### ---------- Get joint signal ---------- ###
# strokeNo = [139,140,141,142,143,144] # 102
startNo_new = startNo + 5
strokeNo = [startNo_new,startNo_new+1,startNo_new+2,startNo_new+3,startNo_new+4]
jnt_traj = []
with open("ALL_Dynamic_joint_trajectory20240426_1726.csv", mode ='r')as file:
  csvFile = csv.reader(file)
  for lines in csvFile:
    # print(f"type = {lines[0]}, value = {lines[0]}")
    for k in strokeNo:
        if int(lines[0]) == k:
            lineele = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            for i in range(len(lines)):
                if i == 3:
                    lineele[i-1] = (float(lines[i]) - float(lines[2]))* 57.3
                elif i != 0:
                    lineele[i-1] = float(lines[i]) * 57.3
            jnt_traj.append(lineele)
### -------------------------------------- ###
print("Drawing:")
print(*strokeNo, sep = ", ") 

matsignal, maxjerk, numsample = smoothfunc.smoothtrajectory(jnt_traj, nargout=3)
signal2 = np.array(matsignal)
print(f"numsample = {numsample}, maxjerk = {maxjerk}")

smoothfunc.terminate()

if maxjerk > 7e-4:
    print(f"Jerk = {maxjerk} TOO BIG! Terminate script")
    # exit(0)

####################################################################################################################

# time.sleep(5)
start_time = time.time()

signalDnD1 = np.append(dipsignal,signal1,axis=0)
signalDnD2 = np.append(dipsignal,signal2,axis=0)
signal = np.append(signalDnD1,signalDnD2,axis=0)

### Connect to socket ###
sock = socket(AF_INET,SOCK_DGRAM)
sock.connect((UDP_IP,UDP_PORT))
print('SOCKET CONNECTED: {:.2f}ms'.format(1000*(time.time()-start_time)))

### Send init pack ###
data = initpack()
sock.sendto(data,(UDP_IP,UDP_PORT))
print('INIT PACK SENT: {:.2f}ms'.format(1000*(time.time()-start_time)))

resp = sock.recv(132)
resp = explainRobData(resp)

### Extract joint data ###
current_jnt_data = resp[18:27]
print(f'JOINT DATA: {current_jnt_data}')
print('({:.2f}ms)'.format(1000*(time.time()-start_time)))

for i, value in enumerate(signal):

    jnt_data = value
    # print(f"value = {value}, i = {i}")

    if (i == 0):
        data = commandpack([1, 0, 1, jnt_data])
        print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in jnt_data])
        print('Sent Seq No:', [1,0,1])
        
    elif (i < len(signal)-1):
        data = commandpack([resp[2], 0, 1, jnt_data])
        # print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in jnt_data])
        # print('Sent Seq No:', [resp[2],0,1])
    
    else:
        data = commandpack([resp[2], 1, 1, jnt_data])
        print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in jnt_data])
        print('Sent Seq No:', [resp[2], 1, 1])
        
    sock.sendto(data, (UDP_IP, UDP_PORT))
    # print('({:.2f}ms)'.format(1000*(time.time()-start_time)))

    resp = sock.recv(132)
    resp = explainRobData(resp) 
    current_jnt_data = resp[18:27]
    # print('current_jnt_data',current_jnt_data)

    # sleeptime =1e-3
    # time.sleep(sleeptime)
  
### Send end pack ###  
data = endpack()
sock.sendto(data,(UDP_IP,UDP_PORT))
print('END PACK SENT')

sock.close()