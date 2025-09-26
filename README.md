# Quadcopter Simulation in PyBullet

A quadcopter was simulated in PyBullet using a crazyfly URDF model. The drone was controlled with PID controllers for roll, pitch, yaw, and altitude, combined with a position controller that generated velocity commands.

The overall control structure is illustrated in the following block diagram:

<img title="Controllers flow chart" alt="Alt text" src="Flow Chart.png">

This diagram shows how setpoints (position, yaw, altitude) are compared with the drone’s current state (from sensors), processed by the position and height controllers, and regulated by PID loops before being translated into forces for individual motors.

Drone successfully reached the target position and stabilized around it.
