Header header
float64 vmax_ec # [m/s]
float64 t_traj_curr
geometry_msgs/Twist Xd_traj
geometry_msgs/Twist Xdd_traj
geometry_msgs/Twist X_err
geometry_msgs/Twist Xd_control
sensor_msgs/JointState qd_des
bool play_traj_
bool positioning_
bool tune_gains_
float64 distance_to_contact_