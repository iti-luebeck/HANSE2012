## *********************************************************
## 
## File autogenerated for the hanse_pidcontrol package 
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

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 233, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 259, 'description': 'how often to send data to server', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'publish_frequency', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 259, 'description': 'p part of pid control', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'k_p', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 259, 'description': 'i part of pid control', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'k_i', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 259, 'description': 'd part of pid control', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'k_d', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 259, 'description': 'bias value for output', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'bias', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 259, 'description': 'maximum value for integral part', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'i_max', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 259, 'description': 'minimal output value', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'output_min', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 259, 'description': 'maximal output value', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'output_max', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 259, 'description': 'should error be calculated as a circle value', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'angular', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}], 'type': '', 'id': 0}

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

