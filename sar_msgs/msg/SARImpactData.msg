bool impact_flag

## ONBOARD IMPACT DETECTION
bool impact_flag_ob
builtin_interfaces/Time time_impact_ob
geometry_msgs/Pose pose_b_o_impact_ob
geometry_msgs/Vector3 eul_b_o_impact_ob

geometry_msgs/Twist twist_b_p_impact_ob
geometry_msgs/Vector3 eul_p_b_impact_ob

## EXTERNAL IMPACT DETECTION
bool impact_flag_ext
builtin_interfaces/Time time_impact_ext

bool bodycontact_flag
bool forelegcontact_flag
bool hindlegcontact_flag

geometry_msgs/Pose pose_b_o_impact_ext
geometry_msgs/Vector3 eul_b_o_impact_ext

geometry_msgs/Twist twist_b_p_impact_ext
geometry_msgs/Vector3 eul_p_b_impact_ext
float64 rot_sum_impact_ext

## IMPACT FORCES
geometry_msgs/Vector3 force_impact
float64 impact_magnitude

## STICKY PAD CONTACTS
uint8 pad_connections
uint8 pad1_contact
uint8 pad2_contact
uint8 pad3_contact
uint8 pad4_contact
