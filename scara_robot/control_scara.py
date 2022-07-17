import time
from time import perf_counter

import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
from numpy import cos, pi, sin
from roboticstoolbox import *

import sim

np.set_printoptions(suppress=True)

#######################
# CRIANDO O ROBÔ      #
#######################

# parametros DH

# revolução
a1 = 0.475
d1 = 0
# theta1 variável
alpha1 = 0

# revolução
a2 = 0.4
d2 = 0
# theta2 variável
alpha2 = pi

# prismática
a3 = 0
# variável 0 <= d3 <= 0.1
d3_lower_limit = 0
d3_upper_limit = 0.1
theta3 = 0
alpha3 = 0

# revolução
a4 = 0
d4 = 0
# theta4 variável
alpha4 = 0

# código abaixo monta o robô a partir dos parâmetros de DH
robot = DHRobot([
    RevoluteDH(a=a1, d=d1, alpha=alpha1),
    RevoluteDH(a=a2, d=d2, alpha=alpha2),
    PrismaticDH(a=a3, theta=theta3, alpha=alpha3, qlim=[d3_lower_limit, d3_upper_limit]),
    RevoluteDH(a=a4, d=d4, alpha=alpha4)
])

print("Tabela DH do robô:")
print(robot)

#######################
# LOOP PRINCIPAL      #
#######################

print ('Program started')
sim.simxFinish(-1) 
clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID != -1:
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
    print ('Connected to remote API server')
    
    sim.simxAddStatusbarMessage(clientID,'Connection working...',sim.simx_opmode_oneshot_wait)
    time.sleep(0.02)
    
    # Criando handlers
    erro, baseJoint = sim.simxGetObjectHandle(clientID, '/MTB/axis', sim.simx_opmode_oneshot_wait)
    erro, armJoint = sim.simxGetObjectHandle(clientID, '/MTB/link/axis', sim.simx_opmode_oneshot_wait)
    erro, armJoint2 = sim.simxGetObjectHandle(clientID, '/MTB/link/axis/link/axis', sim.simx_opmode_oneshot_wait)
    erro, pad = sim.simxGetObjectHandle(clientID, '/MTB/suctionPad', sim.simx_opmode_oneshot_wait)
    erro, dummy = sim.simxGetObjectHandle(clientID, '/Dummy', sim.simx_opmode_oneshot_wait)
    
    # Inicializando stream de dados do coppelia
    value = sim.simxGetJointPosition(clientID, baseJoint, sim.simx_opmode_streaming)
    value = sim.simxGetJointPosition(clientID, armJoint, sim.simx_opmode_streaming)
    value = sim.simxGetJointPosition(clientID, armJoint2, sim.simx_opmode_streaming)
    erro, d = sim.simxGetObjectPosition(clientID, dummy, -1, sim.simx_opmode_streaming)
    erro, X = sim.simxGetObjectPosition(clientID, pad, -1, sim.simx_opmode_streaming)
    time.sleep(1)
    
    delta_t = 0.5 # intervalo de tempo usado para a integração numérica
    error1 = []
    error2 = []
    error3 = []
    error4 = []
        
    while True:
        # lendo valores das juntas
        erro, q0 = sim.simxGetJointPosition(clientID, baseJoint, sim.simx_opmode_buffer)
        erro, q1 = sim.simxGetJointPosition(clientID, armJoint, sim.simx_opmode_buffer)
        erro, q2 = sim.simxGetJointPosition(clientID, armJoint2, sim.simx_opmode_buffer)
        time.sleep(0.05)
        q = [q0, q1, q2, 0]
                
        # lendo posição do atuador e do dummy
        erro, d = sim.simxGetObjectPosition(clientID, dummy, -1, sim.simx_opmode_buffer)    
        erro, X = sim.simxGetObjectPosition(clientID, pad, sim.sim_handle_parent, sim.simx_opmode_buffer)
        time.sleep(0.05)
        
        # aplicando resolved-rate
        
        pad_pose = robot.fkine(q)
        dummy_pose = sm.SE3(d[0], d[1], d[2])
                
        v, arrived = rtb.p_servo(pad_pose, dummy_pose, 1)
        
        J = robot.jacobe(q)
        J_inv = np.linalg.pinv(J)
        
        q_dot = J_inv @ v                
        q += q_dot*(delta_t)
        
        # saving errors for plotting
                
        error1.append(q_dot[0])
        error2.append(q_dot[1])
        error3.append(q_dot[2])
        error4.append(q_dot[3])
        
        np.save('scara_errors.npy', np.array([error1, error2, error3, error4]))
        
        sim.simxSetJointPosition(clientID, baseJoint, q[0], sim.simx_opmode_oneshot)
        time.sleep(0.05)
        
        sim.simxSetJointPosition(clientID, armJoint, q[1], sim.simx_opmode_oneshot)
        time.sleep(0.05)
        
        sim.simxSetJointPosition(clientID, armJoint2, q[1], sim.simx_opmode_oneshot)
        time.sleep(0.05)
        
        time.sleep(0.05)
                
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
