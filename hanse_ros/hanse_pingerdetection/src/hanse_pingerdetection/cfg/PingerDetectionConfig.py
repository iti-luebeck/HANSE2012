## *********************************************************
## 
## File autogenerated for the hanse_pingerdetection package 
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

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 228, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 254, 'description': 'Desired frequency to detect', 'max': 25000.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'frequency', 'edit_method': '', 'default': 15000.0, 'level': 0, 'min': 1000.0, 'type': 'double'}, {'srcline': 254, 'description': 'Sliding window for goertzel', 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'window', 'edit_method': '', 'default': 480, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 254, 'description': 'Sliding window for goertzel', 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'minWindow', 'edit_method': '', 'default': 240, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 254, 'description': 'Threshold for the detection of a ping', 'max': 10000.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'thresholdHigh', 'edit_method': '', 'default': 0.5, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 254, 'description': 'Threshold for the detection of a ping', 'max': 10000.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'thresholdLow', 'edit_method': '', 'default': 0.25, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 254, 'description': 'Average', 'max': 3, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'analyseSource', 'edit_method': "{'enum_description': 'Valid analyse values', 'enum': [{'srcline': 16, 'description': 'Normal', 'srcfile': '../cfg/PingerDetection.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Normal'}, {'srcline': 17, 'description': 'MinFilter', 'srcfile': '../cfg/PingerDetection.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'MinFilter'}, {'srcline': 18, 'description': 'Average', 'srcfile': '../cfg/PingerDetection.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'Average'}]}", 'default': 3, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 254, 'description': 'Plot samples per pixel', 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'samplesPerPixel', 'edit_method': '', 'default': 100, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 254, 'description': 'Max value for goertzel plot', 'max': 10000.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'plotScaleGoertzel', 'edit_method': '', 'default': 2.0, 'level': 0, 'min': 1.0, 'type': 'double'}, {'srcline': 254, 'description': 'Max value for goertzel plot', 'max': 10000.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'plotScaleMin', 'edit_method': '', 'default': 2.0, 'level': 0, 'min': 1.0, 'type': 'double'}, {'srcline': 254, 'description': 'Max value for goertzel plot', 'max': 10000.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'plotScaleAverageGoertzel', 'edit_method': '', 'default': 2.0, 'level': 0, 'min': 1.0, 'type': 'double'}, {'srcline': 254, 'description': 'Plot update rate', 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'counter', 'edit_method': '', 'default': 4800, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 254, 'description': 'Plot raw data', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'plotRaw', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 254, 'description': 'Plot goertzel', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'plotGoertzel', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 254, 'description': 'Plot goertzel', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'plotMin', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 254, 'description': 'Factor for scaling left micro', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'stereoAdjustment', 'edit_method': '', 'default': 0.4846, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 254, 'description': 'Factor for scaling left micro', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'baseline', 'edit_method': '', 'default': 0.4, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 254, 'description': 'Factor for scaling left micro', 'max': 10000.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'speedOfSound', 'edit_method': '', 'default': 1500.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 254, 'description': 'Factor for scaling left micro', 'max': 10000.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'timeout', 'edit_method': '', 'default': 0.1, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 254, 'description': 'Plot raw update rate', 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'averageWindowGoertzelSamples', 'edit_method': '', 'default': 1000, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 254, 'description': 'Plot goertzelaverage', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'plotGoertzelAverage', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 254, 'description': 'Average magnitude window', 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/fuerte/stacks/dynamic_reconfigure/src/dynamic_reconfigure/parameter_generator.py', 'name': 'averageMagnitudeWindow', 'edit_method': '', 'default': 10, 'level': 0, 'min': 1, 'type': 'int'}], 'type': '', 'id': 0}

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

PingerDetection_Normal = 1
PingerDetection_MinFilter = 2
PingerDetection_Average = 3
