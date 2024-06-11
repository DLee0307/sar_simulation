#ifndef CONTROLLER_H
#define CONTROLLER_H

// STANDARD LIBRARIES
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// CF LIBARARIES
#include "app.h"
#include "stabilizer_types.h"
#include "controller.h"
#include "console.h"
#include "math3d.h"

// CUSTOM LIBRARIES
#include "Shared_Lib.h"
//#include "Traj_Funcs.h"
//#include "stabilizer.h"

void controllerOutOfTreeReset();
void controllerOutOfTreeInit();
void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick);

#endif // CONTROLLER_H