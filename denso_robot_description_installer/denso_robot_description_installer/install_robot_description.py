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
    if len(args) < 2:
        print('Usage: install_robot_description "path of description folder"')
        sys.exit()

    path_desc = args[1]

    # Check path format
    m = re.search('([\w\d_]+)_ROS2_descriptions/?$', path_desc)
    if m == None:
        print('Invalid path format: it have to be *_ROS2_descriptions')
        sys.exit()

    rob_name = m.group(1)

    # Check path exists
    if not os.path.isdir(path_desc):
        print(path_desc + ' does not exists')
        sys.exit()

    # Check descriptions directory
    desc_dir = path_desc
    if desc_dir[-1] != '/':
        desc_dir += '/'
    desc_dir += 'descriptions'
    if not os.path.isdir(desc_dir):
        print(desc_dir + ' does not exists')
        sys.exit()

    # Check config directory
    conf_dir = path_desc
    if conf_dir[-1] != '/':
        conf_dir += '/'
    conf_dir += 'moveit_config'
    if not os.path.isdir(conf_dir):
        print(conf_dir + ' does not exists')
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

    if os.path.isdir(descs_pkg + '/robots/' + rob_name):
        print(
            rob_name + ' is already in the denso_robot_descriptions package')
        sys.exit()

    if os.path.isdir(conf_pkg + '/robots/' + rob_name):
        print(
            rob_name + ' is already in the denso_robot_moveit_config package')
        sys.exit()

    # Copy description directory
    shutil.copytree(desc_dir, descs_pkg + '/robots/' + rob_name)

    # Copy config directory
    shutil.copytree(conf_dir, conf_pkg + '/robots/' + rob_name)


if __name__ == '__main__':
    main()
