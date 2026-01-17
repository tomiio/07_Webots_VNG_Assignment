// DroneController.h
#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

// Finite State Machine states for the drone controller
typedef enum {
  STATE_IDLE,
  STATE_TAKEOFF,
  STATE_HOVER,
  STATE_FLYING,
  STATE_LANDING
} FlightState;

typedef struct {
  double x;
  double y;
  double z;
  double yaw;
} Position3D;

typedef struct {
  double roll;
  double pitch;
  double yaw;
} Disturbances;

// Thresholds and targets (in meters/seconds)
#define TARGET_ALTITUDE 2.0       // Desired takeoff altitude
#define ALTITUDE_TOLERANCE 0.05   // ± tolerance for altitude
#define HOVER_DURATION 5.0       // Hover time in seconds
#define GOAL_TOLERANCE 0.5       // 3D distance tolerance to goal
#define DESCENT_RATE 0.2         // Descent rate (m/s)
#define TARGET_X  5.0            // Target horizontal X coordinate
#define TARGET_Z  5.0            // Target horizontal Z coordinate

#define MAX_PITCH_ANGLE 1.57  // radians
#define MAX_ROLL_ANGLE  1.57  // radians
#define MAX_YAW_RATE   1.5   // radians/second

// 
#define POS_EPS   0.5   // meters
#define YAW_EPS   0.05  // radians (~3 deg)


Disturbances calculate_disturbance(Position3D current_state, Position3D next_state);

#endif // DRONE_CONTROLLER_H
