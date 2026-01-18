/*
 * Drone Mission: 
 * Start -> Smart Avoidance Path -> Target -> Land (Soft) -> Wait 5s -> Return Home -> Land
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>

// --- Constants ---
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define MAX_OBSTACLES 50
#define MAX_WAYPOINTS 10

// Tuning for altitude, roll, pitch, yaw and position controller
const double k_vertical_thrust = 68.5;  
const double k_vertical_offset = 0.6;   
const double k_vertical_p = 3.0;        

const double k_roll_p = 50.0;           
const double k_pitch_p = 30.0;      

const double k_yaw_p = 2.0;             
const double k_pos_p = 2.0;        
const double k_pos_d = 5.0; 

const double TARGET_TOLERANCE = 0.5;   
const double WAIT_TIME = 5.0;          
const double OBSTACLE_SAFETY_MARGIN = 3.0; // Avoidance margin in meters
const double LANDING_SPEED = 0.005; // Meters per step (Controls soft landing speed)

// --- Data Structures for Obstacles ---
typedef struct {
  double x, y, z, w, d, h;
  char name[64];
} Obstacle;

// --- Data Structures for Waypoints ---
typedef struct {
  double x, y, z;
} Point;

typedef enum {
  STATE_TAKEOFF,       // Initial takeoff
  STATE_FLYING,        // Flying to target
  STATE_HOVER_WAIT,    // Hover and wait before landing
  STATE_LANDING,       // Soft landing at destination
  STATE_LANDED_WAIT,   // Wait 5s on ground
  STATE_RETURN_TAKEOFF,// Take off again
  STATE_RETURN_FLYING, // Fly back
  STATE_FINAL_LANDING, // Land at home
  STATE_FINISHED     
} FlightState;

// --- Global Data ---
Obstacle obstacles[MAX_OBSTACLES];
int obstacle_count = 0;

Point waypoints[MAX_WAYPOINTS];
int waypoint_count = 0;
int current_wp_index = 0;

double cruise_altitude = 12.0; // Global variable to hold the calculated average obstacles height

// --------------- Helper Functions ---------------

// This function loads obstacles from a CSV file
void load_obstacles(const char *filename) {
  FILE *fp = fopen(filename, "r");
  if (!fp) return;
  char line[256];
  fgets(line, sizeof(line), fp); 
  while (fgets(line, sizeof(line), fp) && obstacle_count < MAX_OBSTACLES) {
    Obstacle o;
    int parsed = sscanf(line, "%63[^,],%lf,%lf,%lf,%lf,%lf,%lf",
           o.name, &o.x, &o.y, &o.z, &o.w, &o.d, &o.h);
    if (parsed == 7) obstacles[obstacle_count++] = o;
  }
  fclose(fp);
}

// Calculate 2D distance
double dist2d(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

// Check if line segment intersects with obstacle (cylinder approximation)
int check_intersection(Point start, Point end, Obstacle o, double clearance) {
    if (start.z > o.h + 0.5) return 0; 
    double l2 = pow(end.x - start.x, 2) + pow(end.y - start.y, 2);
    if (l2 == 0) return 0;
    double t = ((o.x - start.x) * (end.x - start.x) + (o.y - start.y) * (end.y - start.y)) / l2;
    t = CLAMP(t, 0.0, 1.0);
    double closest_x = start.x + t * (end.x - start.x);
    double closest_y = start.y + t * (end.y - start.y);
    double dist = dist2d(o.x, o.y, closest_x, closest_y);
    double radius = (o.w > o.d ? o.w : o.d) / 2.0;
    if (dist < radius + clearance) return 1; 
    return 0;
}

// Plan flight path with obstacle avoidance
void plan_flight_path(Point start, Point target) {
    // 1. Calculate Average Obstacle Height
    double total_h = 0;
    for(int i=0; i<obstacle_count; i++) total_h += obstacles[i].h;
    double avg_h = (obstacle_count > 0) ? total_h / obstacle_count : 5.0;
    
    // 2. Set Cruise Altitude (The logic you requested)
    cruise_altitude = avg_h + 4.0; // Margin 4.0 meters above average obstacle height
    
    // Safety check: Don't fly lower than destination
    if (cruise_altitude < target.z) cruise_altitude = target.z;
    if (cruise_altitude < start.z) cruise_altitude = start.z;
    
    printf("Calculated Safe Cruise Altitude: %.2f meters (Avg Obs Height: %.2f)\n", cruise_altitude, avg_h);

    // Set Z for path planning
    double fly_z = cruise_altitude;

    printf("Planning Path: %.2f,%.2f -> %.2f,%.2f (Alt: %.2f)\n", start.x, start.y, target.x, target.y, fly_z);

    waypoint_count = 0;
    start.z = fly_z;
    target.z = fly_z; 

    // Check collision
    Obstacle *hit_obs = NULL;
    double min_dist_to_start = 100000.0;

    for(int i=0; i<obstacle_count; i++) {
        if (obstacles[i].h > fly_z) {
            if (check_intersection(start, target, obstacles[i], OBSTACLE_SAFETY_MARGIN)) {
                double d = dist2d(start.x, start.y, obstacles[i].x, obstacles[i].y);
                if (d < min_dist_to_start) {
                    min_dist_to_start = d;
                    hit_obs = &obstacles[i];
                }
            }
        }
    }

    if (hit_obs) {
        printf("-> Avoiding %s. Generating Curve.\n", hit_obs->name);
        double dx = target.x - start.x;
        double dy = target.y - start.y;
        double cross_product = dx * (hit_obs->y - start.y) - dy * (hit_obs->x - start.x);
        double dir = (cross_product > 0) ? -1.0 : 1.0; 
        
        double len = sqrt(dx*dx + dy*dy);
        double perp_x = -dy / len * dir;
        double perp_y = dx / len * dir;
        double avoid_radius = (hit_obs->w > hit_obs->d ? hit_obs->w : hit_obs->d) / 2.0 + OBSTACLE_SAFETY_MARGIN;

        Point detour;
        detour.x = hit_obs->x + perp_x * avoid_radius;
        detour.y = hit_obs->y + perp_y * avoid_radius;
        detour.z = fly_z;
        
        waypoints[0] = detour;
        waypoints[1] = target;
        waypoint_count = 2;
    } else {
        waypoints[0] = target;
        waypoint_count = 1;
    }
}

// ------------------ Main ------------------

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Devices
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");
  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");

  WbDeviceTag motors[4] = {
      wb_robot_get_device("front left propeller"),
      wb_robot_get_device("front right propeller"),
      wb_robot_get_device("rear left propeller"),
      wb_robot_get_device("rear right propeller")};
  for (int m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  load_obstacles("../../data/obstacles.csv");

  // Wait one second.
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0) break;
  }
  
  // --- Capture Home Position ---
  const double *start_pos_gps = wb_gps_get_values(gps);
  const double *start_rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
  
  Point home_pt = {start_pos_gps[0], start_pos_gps[1], start_pos_gps[2]};
  double home_yaw = start_rpy[2]; // Initial heading
  
  // Define Destination
  Point dest_pt = {-50.1438, 11.6753, 10.0657}; 

  // Plan Initial Path
  plan_flight_path(home_pt, dest_pt);

  FlightState state = STATE_TAKEOFF;
  
  // Control Variables
  double target_altitude = home_pt.z;
  double target_x = home_pt.x; 
  double target_y = home_pt.y;
  double target_yaw = home_yaw; // Variable to store where the heading should be
  
  double wait_start_time = 0.0;
  
  double prev_x = home_pt.x;
  double prev_y = home_pt.y;
  double prev_time = wb_robot_get_time();

  printf("Mission: Deliver to position (%.2f, %.2f, %.2f)\n", dest_pt.x, dest_pt.y, dest_pt.z);

  while (wb_robot_step(timestep) != -1) {
    const double time = wb_robot_get_time();
    const double dt = time - prev_time;

    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    const double altitude = wb_gps_get_values(gps)[2];
    const double roll_velocity = wb_gyro_get_values(gyro)[0];
    const double pitch_velocity = wb_gyro_get_values(gyro)[1];
    const double *pos = wb_gps_get_values(gps);
    double x = pos[0];
    double y = pos[1];

    double vel_x = (x - prev_x) / dt;
    double vel_y = (y - prev_y) / dt;
    prev_x = x; prev_y = y; prev_time = time;

    double roll_disturbance = 0, pitch_disturbance = 0, yaw_disturbance = 0;

    switch (state) {
      
      case STATE_TAKEOFF:
        target_altitude = cruise_altitude;
        target_x = x; // Lock X when taking off
        target_y = y; // Lock Y when taking off
        
        if (altitude > target_altitude - 0.5) {
           printf("Takeoff Complete. Flying to Destination...\n");
           state = STATE_FLYING;
           current_wp_index = 0;
        }
        break;

      case STATE_FLYING:
        if (current_wp_index < waypoint_count) {
            Point wp = waypoints[current_wp_index];
            target_x = wp.x;
            target_y = wp.y;
            target_altitude = wp.z; // Cruise altitude

            // Face the waypoint
            double dx = target_x - x;
            double dy = target_y - y;
            target_yaw = atan2(dy, dx);

            if (sqrt(dx*dx + dy*dy) < TARGET_TOLERANCE) current_wp_index++;
        } else {
            printf("Reached Destination. Hovering...\n");
            wait_start_time = time;
            state = STATE_HOVER_WAIT;
        }
        break;

      case STATE_HOVER_WAIT:
        // Hover briefly before landing to stabilize
        target_x = dest_pt.x;
        target_y = dest_pt.y;
        target_altitude = dest_pt.z + 1.5; // Hover slightly above landing spot
        
        if (time - wait_start_time > 2.0) {
           printf("Initiating Soft Landing...\n");
           // Initialize target altitude for landing to current altitude to prevent sudden drops
           target_altitude = altitude; 
           state = STATE_LANDING;
        }
        break;

      case STATE_LANDING:
        target_x = dest_pt.x;
        target_y = dest_pt.y;
        
        // SOFT LANDING
        // Slowly decrease target altitude until we reach destination_z
        if (target_altitude > dest_pt.z) {
            target_altitude -= LANDING_SPEED; 
        } else {
            target_altitude = dest_pt.z;
        }

        // Check if we are physically close to ground target
        if (fabs(altitude - dest_pt.z) < 0.1) {
            printf("Landed. Waiting 5s...\n");
            wait_start_time = time;
            // Turn off motors (optional) or keep idle? 
            // We will keep idle here to take off faster
            state = STATE_LANDED_WAIT; 
        }
        break;

      case STATE_LANDED_WAIT:
         target_altitude = dest_pt.z; // Stay on desitination z
         
         if (time - wait_start_time > WAIT_TIME) {
             printf("Wait Complete. Computing Return Path...\n");
             
             // RE-PLAN PATH: Start = Dest, End = Home
             Point current_loc = {x, y, altitude};
             plan_flight_path(current_loc, home_pt);
             current_wp_index = 0;
             
             state = STATE_RETURN_TAKEOFF;
         }
         break;

      // === RETURN ===
      case STATE_RETURN_TAKEOFF:
         target_altitude = cruise_altitude;
         target_x = x; // Lock X when taking off
         target_y = y; // Lock Y when taking off
         
         if (altitude > target_altitude - 0.5) {
             printf("Return Takeoff Complete. Flying Home...\n");
             state = STATE_RETURN_FLYING;
         }
         break;

      // Return Flying
      case STATE_RETURN_FLYING:
         if (current_wp_index < waypoint_count) {
            Point wp = waypoints[current_wp_index];
            target_x = wp.x;
            target_y = wp.y;
            target_altitude = wp.z;

            double dx = target_x - x;
            double dy = target_y - y;
            target_yaw = atan2(dy, dx);

            if (sqrt(dx*dx + dy*dy) < TARGET_TOLERANCE) current_wp_index++;
         } else {
            printf("Arrived Home. Aligning Heading and Landing...\n");
            target_yaw = home_yaw;      // Restore initial pose
            target_altitude = altitude; // Prepare for soft landing
            state = STATE_FINAL_LANDING;
         }
         break;

      // Final Landing at Home
      case STATE_FINAL_LANDING:
         target_x = home_pt.x;
         target_y = home_pt.y;
         target_yaw = home_yaw;         // Keep heading to initial direction

         // Soft Landing again
         if (target_altitude > home_pt.z) {
            target_altitude -= LANDING_SPEED;
         }

         if (fabs(altitude - home_pt.z) < 0.1) {
             printf("Mission Complete. Motors Off.\n");
             state = STATE_FINISHED;
         }
         break;

      // Set velocity to 0 for all motors
      case STATE_FINISHED:
        wb_motor_set_velocity(motors[0], 0);
        wb_motor_set_velocity(motors[1], 0);
        wb_motor_set_velocity(motors[2], 0);
        wb_motor_set_velocity(motors[3], 0);
        wb_robot_step(timestep);
        continue;
    }

    // ------ Heading P Controller ------
    // This keeps the drone oriented towards the target yaw (heading to target point)
    double yaw_err = target_yaw - yaw;
    while(yaw_err > M_PI) yaw_err -= 2*M_PI;
    while(yaw_err < -M_PI) yaw_err += 2*M_PI;
    yaw_disturbance = k_yaw_p * yaw_err;
    yaw_disturbance = CLAMP(yaw_disturbance, -1.0, 1.0);

    // ------ Position PD Controller ------
    // This keep the drone on track to the target position, and also hover when reach target
    double error_x = target_x - x;
    double error_y = target_y - y;
    double body_fwd_err = cos(yaw) * error_x + sin(yaw) * error_y;
    double body_side_err = -sin(yaw) * error_x + cos(yaw) * error_y;
    double body_fwd_vel = cos(yaw) * vel_x + sin(yaw) * vel_y;
    double body_side_vel = -sin(yaw) * vel_x + cos(yaw) * vel_y;

    pitch_disturbance = -(k_pos_p * body_fwd_err) + (k_pos_d * body_fwd_vel);
    roll_disturbance = (k_pos_p * body_side_err) - (k_pos_d * body_side_vel);
    
    pitch_disturbance = CLAMP(pitch_disturbance, -2.0, 2.0);
    roll_disturbance = CLAMP(roll_disturbance, -2.0, 2.0);

    // ------ Actuation ------
    // This remains mostly unchanged compared to the original low-level controller of Webots sample
    const bool led = ((int)time) % 2;
    wb_led_set(front_left_led, led);
    wb_led_set(front_right_led, !led);
    wb_motor_set_position(camera_roll_motor, -0.115 * roll_velocity);
    wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_velocity);

    double roll_in = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
    double pitch_in = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
    double yaw_in = yaw_disturbance;
    double alt_err = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    double vert_in = k_vertical_p * pow(alt_err, 3.0);

    double m1 = k_vertical_thrust + vert_in - roll_in + pitch_in - yaw_in;
    double m2 = k_vertical_thrust + vert_in + roll_in + pitch_in + yaw_in;
    double m3 = k_vertical_thrust + vert_in - roll_in - pitch_in + yaw_in;
    double m4 = k_vertical_thrust + vert_in + roll_in - pitch_in - yaw_in;

    wb_motor_set_velocity(motors[0], m1);
    wb_motor_set_velocity(motors[1], -m2);
    wb_motor_set_velocity(motors[2], -m3);
    wb_motor_set_velocity(motors[3], m4);
  }

  wb_robot_cleanup();
  return EXIT_SUCCESS;
}