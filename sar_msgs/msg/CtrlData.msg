uint32 tick

# =========================
## CONTROLLER STATES
# =========================
geometry_msgs/Pose pose_b_o
geometry_msgs/Twist twist_b_o
geometry_msgs/Accel accel_b_o
float64 accel_b_o_mag

geometry_msgs/Pose pose_p_b
geometry_msgs/Twist twist_b_p
float64 vel_mag_b_p
float64 vel_angle_b_p
float64 d_perp
float64 d_perp_cr

## PLANE STATES
geometry_msgs/Vector3 plane_pos
float64 plane_angle_deg

## OPTICAL FLOW STATES
geometry_msgs/Vector3 optical_flow
geometry_msgs/Vector3 optical_flow_cam
float64 tau_cr

geometry_msgs/Vector3 x_d
geometry_msgs/Vector3 v_d
geometry_msgs/Vector3 a_d

float64[4] fm
float64[4] motorthrusts
uint32[4] motor_cmd

float64[4] nn_output
float64 a_trg
float64 a_rot

bool optical_flow_flag

# ==========================
#  STATES AT POLICY TRIGGER
# ==========================
bool trg_flag

geometry_msgs/Pose pose_b_o_trg
geometry_msgs/Twist twist_b_o_trg

geometry_msgs/Pose pose_p_b_trg
geometry_msgs/Twist twist_b_p_trg
float64 vel_mag_b_p_trg
float64 vel_angle_b_p_trg
float64 d_perp_trg
float64 d_perp_cr_trg

geometry_msgs/Vector3 optical_flow_trg
float64 tau_cr_trg
#float64 tau_dh_trg

float64[4] nn_output_trg
float64 a_trg_trg
float64 a_rot_trg

# =========================
#      STATES AT IMPACT
# =========================

bool impact_flag_ob

geometry_msgs/Pose pose_b_o_impact_ob
geometry_msgs/Twist twist_b_p_impact_ob
float64 vel_mag_b_p_impact_ob
float64 vel_angle_b_p_impact_ob
float64 domega_b_o_y_impact_ob
