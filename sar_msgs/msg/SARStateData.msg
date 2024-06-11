builtin_interfaces/Time time

## STATES WRT ORIGIN
geometry_msgs/Pose pose_b_o
geometry_msgs/Twist twist_b_o
geometry_msgs/Accel accel_b_o
geometry_msgs/Vector3 eul_b_o
float64 accel_b_o_mag

## STATES WRT PLANE
geometry_msgs/Pose pose_p_b
geometry_msgs/Twist twist_b_p
geometry_msgs/Vector3 eul_p_b

float64 vel_mag_b_p
float64 vel_angle_b_p
float64 d_perp
float64 d_perp_pad_min
float64 d_perp_pad
float64 d_perp_cr



## OPTICAL FLOW STATES
geometry_msgs/Vector3 optical_flow
geometry_msgs/Vector3 optical_flow_cam
float64 tau_cr

## STATE SETPOINTS
geometry_msgs/Vector3 x_d
geometry_msgs/Vector3 v_d
geometry_msgs/Vector3 a_d

## CONTROL ACTIONS
float64[4] fm
float64[4] motorthrusts
uint32[4] motor_cmd

## POLICY ACTIONS
float64[4] nn_output
float64 a_trg
float64 a_rot
