import pybullet as p
import pybullet_data
import time
import numpy as np


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0.1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

cfId = p.loadURDF("/home/igor-czudy/code/simulation/simple_quad.urdf",
                  startPos, startOrientation, useFixedBase=False)

prop_ids = []
for i in range(p.getNumJoints(cfId)):
    joint_info = p.getJointInfo(cfId, i)
    joint_name = joint_info[1].decode("utf-8")
    if "prop" in joint_name.lower():
        prop_ids.append(i)

print("Found props:", prop_ids)

prop_ids_dic = {}
for name, prop in zip(["FL", "RL", "RR", "FR"], prop_ids):
    prop_ids_dic[name] = prop


class PIDcontroller():
    def __init__(self):
        self.Kp = 3.0
        self.Ki = 0.001
        self.Kd = 3.5

        self.hover_thrust = 0.27 / 4 
    
        self.prev_error = 0
        self.integral_error = 0

    def move(self, current_height, target_height, dt):
        error = target_height - current_height
        
        P = self.Kp * error

        self.integral_error += (error * dt) 
        I = self.Ki * self.integral_error

        D = self.Kd * ((error - self.prev_error)/dt)
        self.prev_error = error
        
        #thrust_z = self.hover_thrust + P+I+D
        thrusts = P+I+D
        thrusts[2] += self.hover_thrust
        return thrusts


pidController = PIDcontroller()

dt = 1.0 / 40.0
for i in range(2000):

    pos, orn = p.getBasePositionAndOrientation(cfId)
    # orn to kwaternion (x, y, z, w)
    lin_vel, ang_vel = p.getBaseVelocity(cfId)
    # ang_vel = (wx, wy, wz) [rad/s]


    rpy = p.getEulerFromQuaternion(orn)
    # rpy = (roll, pitch, yaw)

    
    # current_height = pos[2]
    # target_height = 2.0
    
    current_pos = pos
    target_pos = np.array([0.0, 0.0, 3.0])
    thrust = pidController.move(np.array(current_pos), target_pos, dt)

    # U_th, U_p, U_r, U_y są znormalizowane, najczęściej do zakresu [-1, +1]
    U_th = 0.27 / 4 
    U_p = 0.0
    U_r = 0.0
    U_y = -0.1
    FL = U_th - U_p + U_r - U_y  #// Silnik CCW (thrust_)
    FR = U_th - U_p - U_r + U_y  #// Silnik CW (thrust_)
    RL = U_th + U_p + U_r + U_y  #// Silnik CW (thrust_)
    RR = U_th + U_p - U_r - U_y  #// Silnik CCW (thrust_)
    thrust={"FL": FL, "RL": RL, "RR": RR, "FR": FR}


    for n, prop in prop_ids_dic.items():
        tras = thrust[n]
        tras = max(0, min(tras, 1)) #clipping from 0 to 1 
        p.applyExternalForce(
            objectUniqueId=cfId,
            linkIndex=prop,
            forceObj=[0, 0, tras],     # siła do góry
            posObj=[0, 0, 0],       # w środku śmigła
            flags=p.LINK_FRAME      # w lokalnym układzie wirnika
        )
    

    """Oblicz momenty obrotowe na podstawie ciągów silników"""
    # Moment yaw (obrót wokół osi Z)
    K_M = 0.27 / 4 
    tau_z = K_M * (thrust["FL"] - thrust["FR"] - thrust["RL"] + thrust["RR"])
    
    # Dla uproszczenia, zakładamy że pitch i roll są kontrolowane przez różnice w ciągu
    tau_x = (thrust["RL"] + thrust["RR"] - thrust["FL"] - thrust["FR"]) * 0.1
    tau_y = (thrust["FL"] + thrust["RL"] - thrust["FR"] - thrust["RR"]) * 0.1

    # Zastosuj moment obrotowy na całym korpusie
    p.applyExternalTorque(
        objectUniqueId=cfId,
        linkIndex=-1,  # -1 = base link
        torqueObj=[tau_x, tau_y, tau_z],
        flags=p.LINK_FRAME
    )



    p.stepSimulation()
    time.sleep(dt)

# Final position
pos, orn = p.getBasePositionAndOrientation(cfId)
print("Final pos:", pos)
print("Final orn:", orn)

p.disconnect()
