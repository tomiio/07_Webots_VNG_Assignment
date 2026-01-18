IMPORTANT NOTE!:
Eventhough I submitted this repo link, but the video files was large and everything was moved to this drive instead: https://drive.google.com/drive/folders/1c03Tf3Qp-t65CH9fiYV2eRgCC-F4jFGt?usp=sharing

Submitted video: 07_Webots_VNG_Assignment\video

Submitted sourcecode: 07_Webots_VNG_Assignment\controllers\mavic2pro\mavic2pro.c 

1.
- To counteract the high-frequency sensor noise, I implemented a Simple Moving Average (SMA) filter using a circular buffer of size 15. This acts as a low-pass filter, attenuating the random noise spikes to reveal the underlying trend.
  
- For the altitude stable, I only introduced a P gain controller, since the filter has filter the noise with small lag, my P controller start with big Kp (2.0) gain to encounter this lag (but this will also make the system jitter a little bit)
  
- Kp = 2.0 was only used for hovering the altitude above 0.5m. For lower 0.5m (when landing), I decreased the Kp to 0.5. This should make the system has delay in response, but for stable and soft landing, this would not making any drawback (the assignment did not mention on archieving the small rise time)

2.
The theoretical delay introduced by a Simple Moving Average filter is calculated as roughly half the window size times the timestep

Window Size ($N$): 15 samples
Timestep ($dt$): 8 ms
Delay = (N-1)/2 * dt
Calculation: 14/2 * 8ms = 7*8ms =56ms

Therefore, the system is reacting to the state of the drone approximately 56 milliseconds in the past."

3.
The lag was accounted for in the tuning of the vertical controller parameters.

In this specific C code, the vertical controller uses a Cubic Proportional logic (pow(alt_err, 3.0)) and does not utilize an explicit Derivative (D) term for altitude (unlike the horizontal position controller which uses k_pos_d).

- Proportional (P) Adjustment: As mentioned, k_vertical_p was reduced to 2.0. While lowering P usually makes a system sluggish, the Cubic Error Curve (pow(3.0)) compensates for this. It provides very low gain for small errors (ignoring residual noise) but ramps up gain rapidly for larger errors. This allows the drone to be "gentle" near the target (stable despite lag) but still responsive enough to climb during takeoff.

- Since there was no vertical D term to increase (which is the standard way to fight lag), the strategy relied entirely on dampening the response via the P-term and the cubic curve to tolerate the 56ms delay.

4.
I also created a full delivery system, with path generation, path tracking and return home functionality. you can check it out (you can choose any destination you want by adjusting destination point in dest_pt (for the given assignment, I set this to (0.0, 0.0, 0.2)

Use source code controllers\mavic2pro\mavic2pro_backup.c for this task

<img width="1697" height="809" alt="image" src="https://github.com/user-attachments/assets/28154a75-5945-4894-9383-6a0fa9808d7c" />

Autonomous Drone Controller with Obstacle Avoidance

This controller manages a quadcopter drone in a Webots environment. It features an intelligent path-planning system that analyzes environmental obstacles to determine an optimal flight altitude and generates detour waypoints to avoid tall structures.
<img width="5564" height="3144" alt="image" src="https://github.com/user-attachments/assets/8970321c-9a04-4260-83d5-dd4e0c79e3c1" />


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
