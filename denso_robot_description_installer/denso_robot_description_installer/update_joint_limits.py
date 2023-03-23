'''
 Software License Agreement (MIT License)

 @copyright Copyright (c) 2021 DENSO WAVE INCORPORATED

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
'''


import os
import re
import sys
from ament_index_python.packages import get_package_share_directory
import shutil
import xml.etree.ElementTree as ET
from lxml import etree

AMENT_PREFIX_PATH_ENV_VAR = 'AMENT_PREFIX_PATH'

def get_search_paths():
    """
    Get the paths from the environment variable '{AMENT_PREFIX_PATH_ENV_VAR}'.
    :returns: list of paths
    :raises: :exc:`EnvironmentError`
    """.format(AMENT_PREFIX_PATH_ENV_VAR=AMENT_PREFIX_PATH_ENV_VAR)
    ament_prefix_path = os.environ.get(AMENT_PREFIX_PATH_ENV_VAR)
    if not ament_prefix_path:
        raise EnvironmentError(
            "Environment variable '{}' is not set or empty".format(AMENT_PREFIX_PATH_ENV_VAR))

    paths = ament_prefix_path.split(os.pathsep)
    return [p for p in paths if p and os.path.exists(p)]


def main(args=None):

    args = sys.argv

    # Check num of arguments
    if len(args) < 3:
        print('Usage: update_joint_limits "robot name" "path of joint_limits.yaml"')
        sys.exit()

    rob_name = args[1]
    path_limits = args[2]

    # Check path exists
    if not os.path.isfile(path_limits):
        print(path_limits + " does not exists")
        sys.exit()

    # Get package directory
    try:
        descs_pkg = get_package_share_directory('denso_robot_descriptions')
    except LookupError:
        raise PackageNotFoundError(
            'package "denso_robot_descriptions" not found,' \
            ' searching: {}'.format(get_search_paths()))
        sys.exit()

    try:
        conf_pkg  = get_package_share_directory('denso_robot_moveit_config')
    except LookupError:
        raise PackageNotFoundError(
            'package "denso_robot_moveit_config" not found,' \
            ' searching: {}'.format(get_search_paths()))
        sys.exit()

    path_desc = descs_pkg + '/robots/' + rob_name
    if not os.path.isdir(path_desc):
        print(rob_name + ' does not exist')
        sys.exit()

    path_limits_xacro = path_desc + '/urdf/denso_robot_joint_limits.xacro'
    if not os.path.isfile(path_limits_xacro):
        print(rob_name + ' xacro file for velocity limits does not exist')
        sys.exit()

    path_kinematics_xacro = path_desc + '/urdf/denso_robot_kinematics.xacro'
    if not os.path.isfile(path_kinematics_xacro):
        print(rob_name + ' xacro file for kinematics does not exist')
        sys.exit()

    path_conf = conf_pkg + '/robots/' + rob_name + '/config'
    if not os.path.isdir(path_conf):
        print(rob_name + ' config folder does not exist')
        sys.exit()

    et_limits_xacro = etree.parse(path_limits_xacro)
    elem_joints = et_limits_xacro.getroot()[0].findall('joint')

    r_joint = re.compile('(joint_\d+):')
    r_has_vel_limit = re.compile('has_velocity_limits: (\w+)')
    r_max_vel = re.compile('max_velocity: ([\d\.]+)')

    cur_elem = None
    cur_elem_kinematics = None
    new_joint = False

    f_limits = open(path_limits, 'r+')

    # Insert space after colon.
    data_lines = f_limits.read()
    index = data_lines.find(': ')
    if index == -1:
        new_data = re.sub(r':([\w\.]+)', r': \1', data_lines)
        f_limits.seek(0)
        f_limits.write(new_data)
        f_limits.truncate()

    f_limits.seek(0)
    line = f_limits.readline()
    while line:
        m = r_joint.search(line)
        if not m == None:
            for elem in elem_joints:
                if m.group(1) in elem.attrib['name']:
                    cur_elem = elem
                    new_joint = True
                    break

        if new_joint:
            m = r_has_vel_limit.search(line)
            if (not m == None):
                interfaces = cur_elem.findall('command_interface')
                for interf in interfaces:
                    if 'velocity' == interf.attrib['name']:
                        parameters = interf.findall('param')
                        for param in parameters:
                            if 'max' == param.attrib['name']:
                                if (m.group(1) == 'false'):
                                    param.text = '1'
                                else:
                                    line = f_limits.readline()
                                    m = r_max_vel.search(line)
                                    if not m == None:
                                        param.text = m.group(1)
                new_joint = False

        line = f_limits.readline()

    # Update limits xacro file
    etree.ElementTree(
        et_limits_xacro.getroot()).write(
            path_limits_xacro,
            encoding="UTF-8",
            xml_declaration=True)

    et_kinematics_xacro = etree.parse(path_kinematics_xacro)
    elem_joints = et_kinematics_xacro.getroot()[0].findall('joint')
    f_limits.seek(0)
    line = f_limits.readline()
    while line:
        m = r_joint.search(line)
        if not m == None:
            for elem in elem_joints:
                if m.group(1) in elem.attrib['name']:
                    cur_elem = elem
                    new_joint = True
                    break

        if new_joint:
            m = r_has_vel_limit.search(line)
            if (not m == None):
                if (m.group(1) == 'false'):
                    cur_elem.find('limit').attrib['velocity'] = '1'
                else:
                    line = f_limits.readline()
                    m = r_max_vel.search(line)
                    if not m == None:
                        cur_elem.find('limit').attrib['velocity'] = m.group(1)
                new_joint = False

        line = f_limits.readline()
    f_limits.close()

    # Update kinematics xacro file
    etree.ElementTree(
        et_kinematics_xacro.getroot()).write(
            path_kinematics_xacro,
            encoding="UTF-8",
            xml_declaration=True)

    # Move joint_limits.yaml
    shutil.copy(path_limits, path_conf)
    # os.remove(path_limits)


if __name__ == '__main__':
    main()
