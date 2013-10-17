rosrun xacro xacro.py ../youbot.urdf.xacro > youbot.urdf
rosrun collada_urdf urdf_to_collada youbot.urdf youbot.dae
rosrun moveit_ikfast round_collada_numbers.py youbot.dae youbot_rounded_5.dae 5
openrave youbot_rounded_5.dae
