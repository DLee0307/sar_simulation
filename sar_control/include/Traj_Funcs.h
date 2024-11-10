#ifndef TRAJ_FUNCS_H
#define TRAJ_FUNCS_H

#include <math.h>
#include "math3d.h"

#include "Shared_Lib.h"
#include "stabilizer.h"
#include "Controller_GTC.h"

// =================================
//     TRAJECTORY INITIALIZATION
// =================================

typedef enum {
    NONE = 0,
    P2P = 1,
    CONST_VEL = 2,
    GZ_CONST_VEL = 3,
}Trajectory_Type;
extern Trajectory_Type Traj_Type;

typedef enum {
    x_axis = 0, 
    y_axis = 1,
    z_axis = 2
} axis_direction;
extern axis_direction axis;

struct traj_vec{
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float idx[3];
  };
};

extern bool Traj_Active[3];
extern float s_0_t[3];              // Traj Start Point [m]
extern float s_f_t[3];              // Traj End Point [m]
extern float v_t[3];                // Traj Vel [m/s]
extern float a_t[3];                // Traj Accel [m/s^2]
extern float j_t[3];                // Traj Jerk [m/s^3]
extern float T[3];                  // Traj completion time [s]
extern float t_traj[3];             // Traj time counter [s]


void set_vec_element(struct vec *v, int index, float value);
void resetTraj_Vals(uint8_t axis);
void point2point_Traj();
void const_velocity_Traj();
void const_velocity_GZ_Traj();

#endif // TRAJ_FUNCS_H