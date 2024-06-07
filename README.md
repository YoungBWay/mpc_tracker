// 同kr simulation的一个demo
roslaunch kr_mav_launch rviz.launch
roslaunch kr_mav_launch demo.launch sim:=true vicon:=false mav_name:=quadrotor
rosrun kr_trackers waypoints_to_action.py __ns:=quadrotor
rosrun rqt_mav_manager rqt_mav_manager

// 运行MPC的包
roslaunch rpg_mpc mpc_controller_only.launch 
// 在GUI中motor on，rviz中选择waypoint，然后publish waypoint
