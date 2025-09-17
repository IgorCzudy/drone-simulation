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
        
        thrust_z = self.hover_thrust + P+I+D

        return [0, 0, thrust_z]


pidController = PIDcontroller()

dt = 1.0 / 40.0
for i in range(2000):
    pos, orn = p.getBasePositionAndOrientation(cfId)
    lin_vel, ang_vel = p.getBaseVelocity(cfId)
    
    current_height = pos[2]
    target_height = 2.0
    
    thrust = pidController.move(current_height, target_height, dt)

    for prop_id in prop_ids:
        p.applyExternalForce(
            objectUniqueId=cfId,
            linkIndex=prop_id,
            forceObj=thrust,     # siła do góry
            posObj=[0, 0, 0],       # w środku śmigła
            flags=p.LINK_FRAME      # w lokalnym układzie wirnika
        )



    p.stepSimulation()
    time.sleep(dt)

# Final position
pos, orn = p.getBasePositionAndOrientation(cfId)
print("Final pos:", pos)
print("Final orn:", orn)

p.disconnect()
