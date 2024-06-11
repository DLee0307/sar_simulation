bool trg_flag
builtin_interfaces/Time time_trg

## STATES WRT ORIGIN
geometry_msgs/Pose pose_b_o_trg
geometry_msgs/Twist twist_b_o_trg
geometry_msgs/Vector3 eul_b_o_trg
float64 vel_mag_b_o_trg
float64 vel_angle_b_o_trg

## STATES WRT PLANE
geometry_msgs/Pose pose_p_b_trg
geometry_msgs/Twist twist_b_p_trg
geometry_msgs/Vector3 eul_p_b_trg
float64 vel_mag_b_p_trg
float64 vel_angle_b_p_trg
float64 d_perp_trg
float64 d_perp_cr_trg

## OPTICAL FLOW STATES
geometry_msgs/Vector3 optical_flow_trg
float64 tau_cr_trg

## POLICY ACTIONS
float64[4] nn_output_trg
float64 a_trg_trg
float64 a_rot_trg
