

#include "controller.h"
#include <math.h>

#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))


// Disturbances calculate_disturbance(Position3D current_state, Position3D next_state)
// {
//     Disturbances disturbances = {0.0, 0.0, 0.0};
//     static double k_pos_p = 3.0; // position gain
//     static double k_yaw_p = 2;   // yaw gain

//     // Simple Euclidean distance calculation
//     float dx = next_state.x - current_state.x;
//     float dy = next_state.y - current_state.y;
//     // float dz = next_state.z - current_state.z;
//     next_state.yaw = atan2f(dy, dx);
//     float dyaw = next_state.yaw - current_state.yaw;

//     // Handle tollerance if waypoints are too close
//     if (fabs(dx) < POS_EPS &&
//         fabs(dy) < POS_EPS &&
//         fabs(dyaw) < YAW_EPS) {
//         return disturbances;
//     }
//     // Handle yaw tollerance if waypoints are too close
//     else if (fabs(dx) < POS_EPS &&
//              fabs(dy) < POS_EPS) {
//         k_yaw_p = 0.1;
//     }

//     if (fabs(dx) < 2.0 &&
//         fabs(dy) < 2.0) {
//         k_pos_p = 0.5;
//     }


//     // Body-frame errors
//     double cy = cos(current_state.yaw);
//     double sy = sin(current_state.yaw);
//     double error_x =  cy * dx + sy * dy;
//     double error_y = -sy * dx + cy * dy;

//     // Normalize to [-pi, pi]
//     while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
//     while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

//     // disturbances.pitch = CLAMP(-k_pos_p * error_x, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
//     // disturbances.roll = CLAMP(-k_pos_p * error_y, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
//     disturbances.yaw = CLAMP(k_yaw_p * dyaw, -MAX_YAW_RATE, MAX_YAW_RATE);

//     if (fabs(dx) < POS_EPS &&
//              fabs(dy) < POS_EPS) {
//         disturbances.roll = 0.0;
//         disturbances.pitch = 0.0;
//         disturbances.yaw = 0.0;
//     }

//     return disturbances;
// }

Disturbances calculate_disturbance(Position3D current, Position3D next)
{
    Disturbances out = {0.0, 0.0, 0.0};

    /* Tunables */
    const double KP_POS = 0.6;       // position → velocity
    const double KP_YAW = 2.0;       // yaw P
    const double G      = 9.81;
    const double MAX_TILT = 0.35;    // rad (~20 deg)

    /* Position error (world frame) */
    double dx = next.x - current.x;
    double dy = next.y - current.y;

    /* Distance check */
    double dist = hypot(dx, dy);

    /* Desired yaw (only when far) */
    double target_yaw = atan2(dy, dx);
    double yaw_err = target_yaw - current.yaw;

    while (yaw_err > M_PI)  yaw_err -= 2*M_PI;
    while (yaw_err < -M_PI) yaw_err += 2*M_PI;

    /* Dead-zone */
    if (dist < POS_EPS) {
        out.roll  = 0.0;
        out.pitch = 0.0;
        out.yaw   = 0.0;
        return out;
    }

    /* Position → velocity */
    double vx = KP_POS * dx;
    double vy = KP_POS * dy;

    vx = CLAMP(vx, -2.0, 2.0);
    vy = CLAMP(vy, -2.0, 2.0);

    /* World → body frame */
    double cy = cos(current.yaw);
    double sy = sin(current.yaw);

    double vx_body =  cy * vx + sy * vy;
    double vy_body = -sy * vx + cy * vy;

    /* Velocity → tilt */
    out.pitch =  CLAMP(-vx_body / G, -MAX_TILT, MAX_TILT);
    out.roll  =  CLAMP(-vy_body / G, -MAX_TILT, MAX_TILT);

    /* Yaw control (freeze near target) */
    if (dist > 1.0)
        out.yaw = CLAMP(KP_YAW * yaw_err, -MAX_YAW_RATE, MAX_YAW_RATE);
    else
        out.yaw = 0.0;

    return out;
}