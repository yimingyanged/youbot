## *********************************************************
## 
## File autogenerated for the kdl_kinematics_plugin_upgrade package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

##**********************************************************
## Software License Agreement (BSD License)
##
##  Copyright (c) 2008, Willow Garage, Inc.
##  All rights reserved.
##
##  Redistribution and use in source and binary forms, with or without
##  modification, are permitted provided that the following conditions
##  are met:
##
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above
##     copyright notice, this list of conditions and the following
##     disclaimer in the documentation and/or other materials provided
##     with the distribution.
##   * Neither the name of the Willow Garage nor the names of its
##     contributors may be used to endorse or promote products derived
##     from this software without specific prior written permission.
##
##  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
##  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
##  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
##  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
##  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
##  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
##  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
##  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
##  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
##  POSSIBILITY OF SUCH DAMAGE.
##**********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 231, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 260, 'description': 'IK type', 'max': 1, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'ik_type', 'edit_method': "{'enum_description': 'An enum to set IK type', 'enum': [{'srcline': 9, 'description': 'IKPositionOrientation6D', 'srcfile': '/home/alex/ros_workspace/youbot/workspaces/yiming/src/kdl_kinematics_plugin_upgrade/cfg/kdl_ik_params.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'ParamIKPositionOrientation6D'}, {'srcline': 10, 'description': 'IKPosition3D', 'srcfile': '/home/alex/ros_workspace/youbot/workspaces/yiming/src/kdl_kinematics_plugin_upgrade/cfg/kdl_ik_params.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'ParamIKPosition3D'}]}", 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 260, 'description': 'First number.', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'use_nullspace', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 260, 'description': "If true, 'weights' will be used", 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'use_weighting', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 260, 'description': 'Robot-specific settings', 'max': 6, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'robot_settings', 'edit_method': "{'enum_description': 'An enum to set robot specific presets', 'enum': [{'srcline': 13, 'description': 'Custom', 'srcfile': '/home/alex/ros_workspace/youbot/workspaces/yiming/src/kdl_kinematics_plugin_upgrade/cfg/kdl_ik_params.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'ParamAatlasCustom'}, {'srcline': 14, 'description': 'Atlas left arm and back', 'srcfile': '/home/alex/ros_workspace/youbot/workspaces/yiming/src/kdl_kinematics_plugin_upgrade/cfg/kdl_ik_params.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'ParamAatlasLeftArmAndBack'}, {'srcline': 15, 'description': 'Atlas left arm', 'srcfile': '/home/alex/ros_workspace/youbot/workspaces/yiming/src/kdl_kinematics_plugin_upgrade/cfg/kdl_ik_params.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'ParamAtlasLeftArm'}, {'srcline': 16, 'description': 'Atlas right arm and back', 'srcfile': '/home/alex/ros_workspace/youbot/workspaces/yiming/src/kdl_kinematics_plugin_upgrade/cfg/kdl_ik_params.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'ParamAtlasRightArmAndBack'}, {'srcline': 17, 'description': 'Atlas right arm', 'srcfile': '/home/alex/ros_workspace/youbot/workspaces/yiming/src/kdl_kinematics_plugin_upgrade/cfg/kdl_ik_params.cfg', 'cconsttype': 'const int', 'value': 4, 'ctype': 'int', 'type': 'int', 'name': 'ParamAtlasRightArm'}, {'srcline': 18, 'description': 'Left leg', 'srcfile': '/home/alex/ros_workspace/youbot/workspaces/yiming/src/kdl_kinematics_plugin_upgrade/cfg/kdl_ik_params.cfg', 'cconsttype': 'const int', 'value': 5, 'ctype': 'int', 'type': 'int', 'name': 'ParamLeftLeg'}, {'srcline': 19, 'description': 'Right leg', 'srcfile': '/home/alex/ros_workspace/youbot/workspaces/yiming/src/kdl_kinematics_plugin_upgrade/cfg/kdl_ik_params.cfg', 'cconsttype': 'const int', 'value': 6, 'ctype': 'int', 'type': 'int', 'name': 'ParamRightLeg'}]}", 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 260, 'description': 'Joint weighting of the least squares solution. Values should be from interval (0,1] (smaller = less influence). The vector must have the same size and the number of joints in the group.', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'weights', 'edit_method': '', 'default': '', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 260, 'description': 'Comfortable pose used for null-space resolution', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'q_comfortable', 'edit_method': '', 'default': '', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 260, 'description': 'Regularisation term (smaller = less regularisation)', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'lambda', 'edit_method': '', 'default': 0.001, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 260, 'description': 'Convergence threshold (smaller= higher accuracy)', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'epsilon', 'edit_method': '', 'default': 0.01, 'level': 0, 'min': 1e-10, 'type': 'double'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

kdl_ik_params_ParamIKPositionOrientation6D = 0
kdl_ik_params_ParamIKPosition3D = 1
kdl_ik_params_ParamAatlasCustom = 0
kdl_ik_params_ParamAatlasLeftArmAndBack = 1
kdl_ik_params_ParamAtlasLeftArm = 2
kdl_ik_params_ParamAtlasRightArmAndBack = 3
kdl_ik_params_ParamAtlasRightArm = 4
kdl_ik_params_ParamLeftLeg = 5
kdl_ik_params_ParamRightLeg = 6
