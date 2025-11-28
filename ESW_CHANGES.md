
# Changes needed for ESW integration
[link to slides](https://docs.google.com/presentation/d/1qTI7UU9cB1_I8BpI9w1791hx_FbAomup_Tq8lQPWTK4/edit?slide=id.g38eb6fed52f_0_8#slide=id.g38eb6fed52f_0_8)

## Arm
- `/ee_pos_cmd` -> `/arm_position_cmd`, use mrover/Position
- `/ee_vel_cmd` -> `/arm_velocity_cmd`, use mrover/Velocity
- ARM has a copy of `/controller_cmd_vel`, move to drive

## Drive
- now has two pubs for drive, `/controller_cmd_vel` and `/joystick_cmd_vel`

## Science
- `/sa_gear_diff_position` integrated into `/sp_joint_state`
- `/sa_gear_diff_set_position` -> `/sp_funnel_set_position`
- `/sa_throttle_cmd` -> `/sp_throttle_cmd`, move to science_consumer
- `/sa_controller_state` -> `/sp_controller_state`, move to science_consumer

## Mast
- `/mast_gimbal_throttle_cmd` -> SERVICE: `/gimbal_set_position`, type `mrover/ServoSetPos`
- preset positions for pitch
- yaw requires precise control
- subscribe to `/gimbal_joint_state` for state, type `std_msgs/msg/JointState`
- MAKE FRONTEND WITH BUTTONS TO ADJUST