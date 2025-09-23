import pybullet as p
import pybullet_data
import time
import numpy as np


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0.9]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

cfId = p.loadURDF("/home/igor-czudy/code/simulation/simple_quad.urdf",
                  startPos, startOrientation, useFixedBase=False)



prop_ids = []
prop_positions = [] # NOWA LISTA do przechowywania pozycji
for i in range(p.getNumJoints(cfId)):
    joint_info = p.getJointInfo(cfId, i)
    joint_name = joint_info[1].decode("utf-8")
    if "prop" in joint_name.lower():
        prop_ids.append(i)
        # Pobierz pozycję kotwicy jointa (relative to base)
        link_state = p.getLinkState(cfId, i)
        joint_pos = link_state[0]
        prop_positions.append(joint_pos)

prop_ids_dic = {}
for i, prop_id in enumerate(prop_ids):
    x, y, z = prop_positions[i]
    # Określ pozycję na podstawie współrzędnych
    if x > 0 and y > 0:
        name = "FR" # Front-Right
    elif x > 0 and y < 0:
        name = "FL" # Front-Left
    elif x < 0 and y > 0:
        name = "RR" # Rear-Right
    elif x < 0 and y < 0:
        name = "RL" # Rear-Left
    else:
        name = f"UNK{i}" # Dla bezpieczeństwa
    prop_ids_dic[name] = prop_id
    print(f"Assigned {name} to joint {prop_id} at position ({x:.3f}, {y:.3f}, {z:.3f})")


p.setDebugObjectColor(cfId, -1, [1, 0, 0])  # Pokazuj środek masy (czerwona kropka)
p.addUserDebugLine([0, 0, 0], [0.2, 0, 0], [1, 0, 0], parentObjectUniqueId=cfId) # Oś X (czerwona)
p.addUserDebugLine([0, 0, 0], [0, 0.2, 0], [0, 1, 0], parentObjectUniqueId=cfId) # Oś Y (zielona)
p.addUserDebugLine([0, 0, 0], [0, 0, 0.2], [0, 0, 1], parentObjectUniqueId=cfId) # Oś Z (niebieska)


class PIDcontroller():
    def __init__(self, Kp = 3.0, Ki = 0.001, Kd = 3.5):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
    
        self.prev_error = 0
        self.integral_error = 0

    def compute(self, current, target, dt):
        error = target - current
        
        P = self.Kp * error

        # TODO 
        # self.integral_error = np.clip(self.integral_error, -max_val, max_val)
        self.integral_error += (error * dt)
        max_integral = 10.0  # lub odpowiednia wartość
        self.integral_error = np.clip(self.integral_error, -max_integral, max_integral)

        I = self.Ki * self.integral_error

        D = self.Kd * ((error - self.prev_error)/dt)
        self.prev_error = error
        
        output = P+I+D
        return output


class PossitionController():
    def __init__(self, Kp = 3.0, Ki = 0.001, Kd = 3.5):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
    
        self.prev_error = 0
        self.integral_error = 0

    def compute(self, current, target, dt):
        target = np.array(target)
        current = np.array(current)
        error = target - current
        
        P = self.Kp * error

        # TODO 
        # self.integral_error = np.clip(self.integral_error, -max_val, max_val)
        self.integral_error += (error * dt)
        max_integral = 10.0  # lub odpowiednia wartość
        self.integral_error = np.clip(self.integral_error, -max_integral, max_integral)

        I = self.Ki * self.integral_error

        D = self.Kd * ((error - self.prev_error)/dt)
        self.prev_error = error
        
        desired_velocity = P+I+D
        return desired_velocity # żądana prędkość w globalnym układzie współrzędnych [m/s]


# przekształcanie żądanej prędkości na kąty przechylenia
def velocity_to_attitude(desired_velocity, curr_yaw):
    """
    Przekształca żądaną prędkość w globalnym układzie współrzędnych
    na kąty przechylenia (roll, pitch) w lokalnym układzie drona.
    """
    # 1. Obróć żądaną prędkość do lokalnego układu drona
    #    (uwzględniając aktualny yaw)
    yaw = curr_yaw
    # rotation_matrix = np.array([
    #     [np.cos(yaw), np.sin(yaw), 0],
    #     [-np.sin(yaw), np.cos(yaw), 0],
    #     [0, 0, 1]
    # ])
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
    # Wyświetl aktualną pozycję i cel
    p.addUserDebugLine(pos, target_pos+[0], [1, 0, 0], lineWidth=2, lifeTime=0.1)
    
    # Wyświetl żądaną prędkość
    vel_end = [pos[0] + desired_velocity[0], 
               pos[1] + desired_velocity[1],
               0]
    p.addUserDebugLine(pos, vel_end, [0, 1, 0], lineWidth=2, lifeTime=0.1)



pidControllerRoll = PIDcontroller(Kp = 0.5, Ki = 0.001, Kd = 0.1)
pidControllerPitch = PIDcontroller(Kp = 0.5, Ki = 0.001, Kd = 0.1)
pidControllerYaw = PIDcontroller(Kp = 0.5, Ki = 0.001, Kd = 0.1)  # Yaw zwykle łagodniejszy

pidControllerHigh = PIDcontroller(Kp=0.4, Ki=0.02, Kd=5.5) #0.55
possitionController = PossitionController(Kp=1.0, Ki=0.0, Kd=5.5)

# target_roll, target_pitch, target_yaw = 0.3, 0.3, 0.0
target_x, target_y, target_z = 2, 3, 4
target_yaw = 0.0

dt = 1.0 / 40.0
for i in range(2000):

    pos, orn = p.getBasePositionAndOrientation(cfId)
    curr_pos_x, curr_pos_y, curr_pos_z= pos
    if i % 10 == 0:
        print(f"{pos}")
    # orn to kwaternion (x, y, z, w)
    curr_lin_vel, curr_ang_vel = p.getBaseVelocity(cfId)
    # ang_vel = (wx, wy, wz) [rad/s]
    
    #opory powietrza

    rpy = p.getEulerFromQuaternion(orn) # rpy = (roll, pitch, yaw)
    curr_roll, curr_pitch, curr_yaw = rpy

    desired_velocity = possitionController.compute(pos[:2], [target_x, target_y], dt)

    # if i % 100 == 0:
    #     print(f"Position error: {np.array([target_x, target_y, target_z]) - np.array(pos)}")
    #     print(f"Desired velocity: {desired_velocity}")
    
    # Wizualizacja debug
    debug_visualization(pos, desired_velocity, [target_x, target_y])

    target_roll, target_pitch = velocity_to_attitude(desired_velocity, curr_yaw)
    if i % 10 == 0:
            print(f"{target_roll=:.3f}, {target_pitch=:.3f}")
            print(f"{curr_roll=:.3f}, {curr_pitch=:.3f}")
        

    U_th = pidControllerHigh.compute(curr_pos_z, target_z, dt)
    U_th = max(0, min(U_th, 1.0))  # Ogranicz thrust
    
    U_p = pidControllerPitch.compute(curr_pitch, target_pitch, dt)
    U_r = pidControllerRoll.compute(curr_roll, target_roll, dt)
    U_y = pidControllerYaw.compute(curr_yaw, target_yaw, dt)

    FR = U_th - U_p + U_r + U_y  #// Silnik CCW (thrust_)
    FL = U_th - U_p - U_r - U_y  #// Silnik CW (thrust_)
    RL = U_th + U_p - U_r + U_y  #// Silnik CW (thrust_)
    RR = U_th + U_p + U_r - U_y  #// Silnik CCW (thrust_)


    thrust={"FL": FL, "RL": RL, "RR": RR, "FR": FR}

    for n, prop in prop_ids_dic.items():
        tras = thrust[n]
        tras = max(0, tras) #clipping from 0 to 1 
        # tras = max(0, min(tras, 1))
        p.applyExternalForce(
            objectUniqueId=cfId,
            linkIndex=prop,
            forceObj=[0, 0, tras],     # siła do góry
            posObj=[0, 0, 0],       # w środku śmigła
            flags=p.LINK_FRAME #LINK_FRAME      # w lokalnym układzie wirnika
        )
    
    # tau_z = 0.27 * (thrust["FL"] - thrust["FR"] - thrust["RL"] + thrust["RR"])
    # p.applyExternalTorque(
    #     objectUniqueId=cfId,
    #     linkIndex=-1,  # Apply to base, not individual propellers
    #     torqueObj=[0, 0, tau_z],
    #     flags=p.LINK_FRAME
    # )
    p.stepSimulation()
    # time.sleep(dt)

# Final position
pos, orn = p.getBasePositionAndOrientation(cfId)
print("Final pos:", pos)
print("Final orn:", orn)

p.disconnect()
