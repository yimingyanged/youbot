###############3
# instructions to install youbot_camera on hydro:
##

echo "installing ar_tools"
#Install ar_tools for hydro:
git clone https://github.com/IHeartEngineering/ar_tools.git
add path to ROS_PACKAGE_PATH
rosmake

# Install youbot_camera
git clone git@github.com:andywolff/youbot_camera.git


