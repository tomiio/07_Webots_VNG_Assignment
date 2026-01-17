<img width="1697" height="809" alt="image" src="https://github.com/user-attachments/assets/28154a75-5945-4894-9383-6a0fa9808d7c" />

Autonomous Drone Controller with Obstacle Avoidance
This controller manages a quadcopter drone in a Webots environment. It features an intelligent path-planning system that analyzes environmental obstacles to determine an optimal flight altitude and generates detour waypoints to avoid tall structures.

1. System Features
- Intelligent Altitude Selection: Calculates the average height of all obstacles to set a "Cruise Altitude," ensuring efficient flight without simply flying over the tallest object.
- Dynamic Path Planning: Analyzes the trajectory between the start and destination. If a "Tall Obstacle" (higher than cruise altitude) blocks the path, it calculates a 2D tangent detour (half-round curve).
- Multi-Stage Mission: 1. Takeoff to cruise altitude. 2. Fly to Destination (via avoidance waypoints). 3. Soft Landing: Smoothly decreases altitude for a gentle touchdown. 4. Wait & Return: Waits 5 seconds on the ground, re-plans the path, and returns to the initial home position.
- Precise Navigation: Uses GPS, IMU, and Gyro sensors with PID control for position, heading (Yaw), and stabilization.

2. Architecture Overview
The system operates on a Finite State Machine (FSM). The transition logic ensures that the drone only moves to the next phase of the mission once specific physical tolerances (distance or altitude) are met.
STATE_TAKEOFF	        Ascends to the calculated cruise altitude.
STATE_FLYING	        Follows generated waypoints to the target.
STATE_LANDING	        Decreases target_altitude incrementally for a soft touch.
STATE_LANDED_WAIT	    Motors idle on the ground for 5 seconds.
STATE_RETURN_FLYING	  Re-calculates path from Destination back to Home.

3. Obstacle Avoidance Logic
- The drone identifies obstacles from a .csv file. The avoidance logic follows two rules:
- Fly Over: If an obstacle is shorter than the cruise altitude (Avg + Margin), the drone ignores it and flies directly over.
- Fly Around: If an obstacle is taller than the cruise altitude, the drone calculates a perpendicular detour point based on the obstacle's radius and a safety margin.

4. How to Use
- Obstacle Data: Ensure obstacles.csv is located in the data folder. The file should contain x, y, z, width, depth, height.
- Set Destination: Update the dest_pt coordinates in the main() function to your desired target.
- Compilation: Compile the controller in Webots. The drone will automatically capture its starting position as "Home."

5. Control Constants (Tuning)
- LANDING_SPEED: Adjusts how slowly the drone descends during the soft landing phase.
- OBSTACLE_SAFETY_MARGIN: The horizontal distance (in meters) the drone maintains from the edge of a tall obstacle during a detour.
- k_pos_p / k_pos_d: Proportional and Derivative gains for horizontal position holding.
