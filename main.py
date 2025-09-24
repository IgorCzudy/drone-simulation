import pybullet as p
import pybullet_data
import numpy as np
from PIDcontroller import PIDcontroller
from PossitionController import PossitionController



def get_properels(cfId):
    prop_ids = []
    prop_positions = []
    for i in range(p.getNumJoints(cfId)):
        joint_info = p.getJointInfo(cfId, i)
        joint_name = joint_info[1].decode("utf-8")
        if "prop" in joint_name.lower():
            prop_ids.append(i)
            link_state = p.getLinkState(cfId, i)
            joint_pos = link_state[0]
            prop_positions.append(joint_pos)

    prop_ids_dic = {}
    for i, prop_id in enumerate(prop_ids):
        x, y, z = prop_positions[i]
        if x > 0 and y > 0:
            name = "FR" # Front-Right
        elif x > 0 and y < 0:
            name = "FL" # Front-Left
        elif x < 0 and y > 0:
            name = "RR" # Rear-Right
        elif x < 0 and y < 0:
            name = "RL" # Rear-Left
        else:
            name = f"UNK{i}"
        prop_ids_dic[name] = prop_id
        print(f"Assigned {name} to joint {prop_id} at position ({x:.3f}, {y:.3f}, {z:.3f})")

    return prop_ids_dic


def set_debag_line(cfId):
    p.setDebugObjectColor(cfId, -1, [1, 0, 0]) 
    p.addUserDebugLine([0, 0, 0], [0.2, 0, 0], [1, 0, 0], parentObjectUniqueId=cfId) # Oś X (czerwona)
    p.addUserDebugLine([0, 0, 0], [0, 0.2, 0], [0, 1, 0], parentObjectUniqueId=cfId) # Oś Y (zielona)
    p.addUserDebugLine([0, 0, 0], [0, 0, 0.2], [0, 0, 1], parentObjectUniqueId=cfId) # Oś Z (niebieska)


def velocity_to_attitude(desired_velocity, curr_yaw):
    """
    Converts the desired speed in the global coordinate system
    into roll and pitch angles in the local drone coordinate system.
    """
    # 1. Obróć żądaną prędkość do lokalnego układu drona
    #    (uwzględniając aktualny yaw)
    yaw = curr_yaw
    rotation_matrix = np.array([
        [np.cos(yaw), np.sin(yaw)],
        [-np.sin(yaw), np.cos(yaw)]
    ])

    desired_velocity_local = rotation_matrix @ desired_velocity
    
    # 2. Ogranicz maksymalną prędkość (dla stabilności)
    max_speed = 50.0  # m/s
    speed = np.linalg.norm(desired_velocity_local[:2])
    if speed > max_speed:
        desired_velocity_local[0] = desired_velocity_local[0] / speed * max_speed
        desired_velocity_local[1] = desired_velocity_local[1] / speed * max_speed
    
    # 3. Przekształć żądaną prędkość poziomą na kąty przechylenia
    #    Im większa żądana prędkość, tym większe przechylenie
    max_angle = 0.45  # rad (około 30 stopni - maksymalny bezpieczny kąt)
    
    # Współczynnik: jak bardzo się przechylić, aby osiągnąć żądaną prędkość
    velocity_gain = 0.5
    
    desired_pitch = desired_velocity_local[0] * velocity_gain #GFDSGFS zmiana znaków
    desired_roll = -desired_velocity_local[1] * velocity_gain
    
    # Ogranicz kąty do bezpiecznego zakresu
    desired_pitch = np.clip(desired_pitch, -max_angle, max_angle)
    desired_roll = np.clip(desired_roll, -max_angle, max_angle)
    
    return desired_roll, desired_pitch


def debug_visualization(pos, desired_velocity, target_pos):
    p.addUserDebugLine(pos, target_pos+[0], [1, 0, 0], lineWidth=2, lifeTime=0.1)
    
    vel_end = [pos[0] + desired_velocity[0], 
               pos[1] + desired_velocity[1],
               0]
    p.addUserDebugLine(pos, vel_end, [0, 1, 0], lineWidth=2, lifeTime=0.1)


def init_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    planeId = p.loadURDF("plane.urdf")
    startPos = [0, 0, 1.0]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])

    cfId = p.loadURDF("/home/igor-czudy/code/simulation/simple_quad.urdf",
                    startPos, startOrientation, useFixedBase=False)
    return cfId

cfId = init_simulation()
prop_ids_dic = get_properels(cfId)
set_debag_line(cfId)


pidControllerRoll = PIDcontroller(Kp = 0.5, Ki = 0.001, Kd = 0.1)
pidControllerPitch = PIDcontroller(Kp = 0.5, Ki = 0.001, Kd = 0.1)
pidControllerYaw = PIDcontroller(Kp = 0.01, Ki = 0.0, Kd = 0.01)

pidControllerHigh = PIDcontroller(Kp=0.4, Ki=0.02, Kd=5.5)
possitionController = PossitionController(Kp=1.0, Ki=0.0, Kd=5.5)

target_x, target_y, target_z = 1, 2, 3
target_yaw = 0.9


dt = 1.0 / 40.0
for i in range(2000):

    pos, orn = p.getBasePositionAndOrientation(cfId) #GLOBAL orientacja(obrót), position 
    curr_pos_x, curr_pos_y, curr_pos_z= pos
    
    curr_lin_vel, curr_ang_vel = p.getBaseVelocity(cfId)
    # ang_vel = (wx, wy, wz) [rad/s]
    
    rpy = p.getEulerFromQuaternion(orn) # Quaternion -> Euler angles (roll, pitch, yaw) - in radians
    curr_roll, curr_pitch, curr_yaw = rpy # rpy = (roll, pitch, yaw)

    desired_velocity = possitionController.compute(pos[:2], [target_x, target_y], dt)

    debug_visualization(pos, desired_velocity, [target_x, target_y])

    target_roll, target_pitch = velocity_to_attitude(desired_velocity, curr_yaw)
    
    if i % 10 == 0:
        print(f"{pos}")
        print(f"{target_roll=:.3f}, {target_pitch=:.3f}")
        print(f"{curr_roll=:.3f}, {curr_pitch=:.3f}")
    

    U_th = pidControllerHigh.compute(curr_pos_z, target_z, dt)
    U_th = max(0, min(U_th, 1.0))  # Limit thrust

    U_p = pidControllerPitch.compute(curr_pitch, target_pitch, dt)
    U_r = pidControllerRoll.compute(curr_roll, target_roll, dt)
    U_y = pidControllerYaw.compute(curr_yaw, target_yaw, dt)

    # Forces for each properels
    FR = U_th - U_p + U_r - U_y
    FL = U_th - U_p - U_r + U_y
    RL = U_th + U_p - U_r - U_y
    RR = U_th + U_p + U_r + U_y
    thrust={"FL": FL, "RL": RL, "RR": RR, "FR": FR}

    for n, prop in prop_ids_dic.items():
        tras = thrust[n]
        tras = max(0, tras) #clipping from 0 to 1 

        p.applyExternalForce(
            objectUniqueId=cfId,
            linkIndex=prop,
            forceObj=[0, 0, tras],     # siła do góry
            posObj=[0, 0, 0],       # w środku śmigła
            flags=p.LINK_FRAME # w lokalnym układzie wirnika
        )
    
    #for yaw
    tau_z = 0.27 * (thrust["FL"] - thrust["FR"] - thrust["RL"] + thrust["RR"])
    p.applyExternalTorque(
        objectUniqueId=cfId,
        linkIndex=-1,  # Apply to base, not individual propellers
        torqueObj=[0, 0, tau_z],
        flags=p.LINK_FRAME
    )
    p.stepSimulation()


pos, orn = p.getBasePositionAndOrientation(cfId)
print("Final pos:", pos)
print("Final orn:", orn)

p.disconnect()
