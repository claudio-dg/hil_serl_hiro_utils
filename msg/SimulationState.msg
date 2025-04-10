std_msgs/Float64 gripper_command
geometry_msgs/PoseStamped[] obj_poses        # Array di pose dei cubi
geometry_msgs/TwistStamped[] obj_velocities # Array di velocità dei cubi
geometry_msgs/PoseStamped tcp_pose
geometry_msgs/TwistStamped tcp_velocity
geometry_msgs/WrenchStamped tcp_force_torque
sensor_msgs/Image[] camera_images            # Array di immagini delle telecamere