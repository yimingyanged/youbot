execute_process(COMMAND "/home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/linemod/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/linemod/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
