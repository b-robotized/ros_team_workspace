#!/bin/bash
#
# Copyright 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

usage="setup-robot-moveit CELL_NAME CONTROL_PKG_NAME CONTROL_LAUNCH_FILE"

# Load Framework defines
script_own_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
source $script_own_dir/../setup.bash
check_and_set_ros_distro_and_version "${ROS_DISTRO}"

CELL_NAME=$1
if [ -z "$CELL_NAME" ]; then
  print_and_exit "ERROR: You should provide robot/cell/scenario name! Nothing to do 😯" "$usage"
fi

CONTROL_PKG_NAME=$2
if [ -z "$CONTROL_PKG_NAME" ]; then
  print_and_exit "ERROR: You should provide control package name! Nothing to do 😯" "$usage"
fi

CONTROL_LAUNCH_FILE=$3
if [ -z "$CONTROL_LAUNCH_FILE" ]; then
  print_and_exit "ERROR: You should provide control launch file name! Nothing to do 😯" "$usage"
fi

echo "Which launchfiles should be added? Choose from the following options:"
echo "1) xml"
echo "2) python"
echo "3) both"

read -p "Enter your choice:" choice

LAUNCH_FILE_TYPES=()

case $choice in
1)
  LAUNCH_FILE_TYPES+=(".xml")
  ;;
2)
  LAUNCH_FILE_TYPES+=(".py")
  ;;
3)
  LAUNCH_FILE_TYPES+=(".xml" ".py")
  ;;
*)
  print_and_exit "Invalid choice. Exiting."
  ;;
esac

if [ ! -f "package.xml" ]; then
  print_and_exit "ERROR: 'package.xml' not found. You should execute this script at the top level of your package folder. Nothing to do 😯" "$usage"
fi
PKG_NAME="$(grep -Po '(?<=<name>).*?(?=</name>)' package.xml | sed -e 's/[[:space:]]//g')"

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}ATTENTION: Setting up moveit package for robot '$CELL_NAME' in package '$PKG_NAME' in folder '$(pwd)' with robot control package '$CONTROL_PKG_NAME' and control launch file '$CONTROL_LAUNCH_FILE'.${TERMINAL_COLOR_NC}"
echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}If correct press <ENTER>, otherwise <CTRL>+C and start the script again from the package folder and/or with correct robot name.${TERMINAL_COLOR_NC}"
read

# Remove include and src folders - in this package should be no source
RM_FOLDERS=("src")

for FOLDER in "${RM_FOLDERS[@]}"; do
  if [[ -d $FOLDER && ! "$(ls -A $FOLDER)" ]]; then
    rm -r $FOLDER
  fi
done

# Copy rviz files
mkdir -p rviz
ROBOT_RVIZ="rviz/moveit.rviz"
cp --update=none "$MOVEIT_TEMPLATES/moveit.rviz" $ROBOT_RVIZ

# Copy config files
MOVEIT_CONFIG_FOLDER="moveit_config"
mkdir -p $MOVEIT_CONFIG_FOLDER
MOVEIT_JOINT_LIMITS="$MOVEIT_CONFIG_FOLDER/joint_limits.yaml"
MOVEIT_KINEMATICS="$MOVEIT_CONFIG_FOLDER/kinematics.yaml"
MOVEIT_CONTROLLERS="$MOVEIT_CONFIG_FOLDER/moveit_controllers.yaml"
MOVEIT_MOVE_GROUP="$MOVEIT_CONFIG_FOLDER/move_group_config.yaml"

MOVEIT_PLANNER_CHOMP="$MOVEIT_CONFIG_FOLDER/chomp_planning.yaml"
MOVEIT_PLANNER_OMPL="$MOVEIT_CONFIG_FOLDER/ompl_planning.yaml"
MOVEIT_PLANNER_PILZ="$MOVEIT_CONFIG_FOLDER/pilz_planning.yaml"
MOVEIT_PLANNER_PILZ_CARTESIAN_LIMITS="$MOVEIT_CONFIG_FOLDER/pilz_cartesian_limits.yaml"
MOVEIT_PLANNER_STOMP="$MOVEIT_CONFIG_FOLDER/stomp_planning.yaml"

# MOVEIT_SENSORS_3d="$MOVEIT_CONFIG_FOLDER/sensors_3d.yaml"  # not used right now

cp --update=none "$MOVEIT_TEMPLATES/config/joint_limits.yaml" $MOVEIT_JOINT_LIMITS
cp --update=none "$MOVEIT_TEMPLATES/config/kinematics.yaml" $MOVEIT_KINEMATICS
cp --update=none "$MOVEIT_TEMPLATES/config/moveit_controllers.yaml" $MOVEIT_CONTROLLERS
cp --update=none "$MOVEIT_TEMPLATES/config/move_group_config.yaml" $MOVEIT_MOVE_GROUP

cp --update=none "$MOVEIT_TEMPLATES/config/chomp_planning.yaml" $MOVEIT_PLANNER_CHOMP
cp --update=none "$MOVEIT_TEMPLATES/config/ompl_planning.yaml" $MOVEIT_PLANNER_OMPL
cp --update=none "$MOVEIT_TEMPLATES/config/pilz_planning.yaml" $MOVEIT_PLANNER_PILZ
cp --update=none "$MOVEIT_TEMPLATES/config/pilz_cartesian_limits.yaml" $MOVEIT_PLANNER_PILZ_CARTESIAN_LIMITS
cp --update=none "$MOVEIT_TEMPLATES/config/stomp_planning.yaml" $MOVEIT_PLANNER_STOMP

# cp --update=none "$MOVEIT_TEMPLATES/config/sensors_3d.yaml" $MOVEIT_SENSORS_3d

# Copy rviz file
mkdir -p rviz
cp --update=none "$MOVEIT_TEMPLATES/rviz/moveit_config.rviz" "rviz/"

# Copy SRDF/xacro files
mkdir -p srdf
CELL_SRDF="srdf/${CELL_NAME}.srdf.xacro"
CELL_SRDF_MACRO="srdf/${CELL_NAME}_macro.srdf.xacro"
cp --update=none "$MOVEIT_TEMPLATES/srdf/robot.srdf.xacro" $CELL_SRDF
cp --update=none "$MOVEIT_TEMPLATES/srdf/robot_macro.srdf.xacro" $CELL_SRDF_MACRO


# Copy launch files
mkdir -p launch
for file_type in "${LAUNCH_FILE_TYPES[@]}"; do
  # Construct the file paths
  MOVEIT_LAUNCH="launch/${CELL_NAME}.launch${file_type}"

  # Copy the templates to the destination with the specified file type
  cp --update=none "$MOVEIT_TEMPLATES/moveit.launch${file_type}" "${MOVEIT_LAUNCH}"

  # sed all needed files
  FILES_TO_SED=($MOVEIT_LAUNCH $CELL_SRDF $CELL_SRDF_MACRO $MOVEIT_JOINT_LIMITS $MOVEIT_KINEMATICS $MOVEIT_CONTROLLERS $MOVEIT_MOVE_GROUP $MOVEIT_PLANNER_CHOMP $MOVEIT_PLANNER_OMPL $MOVEIT_PLANNER_PILZ $MOVEIT_PLANNER_PILZ_CARTESIAN_LIMITS $MOVEIT_PLANNER_STOMP)

  for SED_FILE in "${FILES_TO_SED[@]}"; do
    sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" $SED_FILE
    sed -i "s/\\\$CELL_NAME\\\$/${CELL_NAME}/g" $SED_FILE
    sed -i "s/\\\$CONTROL_PKG_NAME\\\$/${CONTROL_PKG_NAME}/g" $SED_FILE
    sed -i "s/\\\$CONTROL_LAUNCH_FILE\\\$/${CONTROL_LAUNCH_FILE}/g" $SED_FILE
  done
done

# package.xml: Add dependencies if they not exist
DEP_PKGS=(
  "xacro"
  "$CONTROL_PKG_NAME"
  "moveit_ros_move_group"
  "moveit_kinematics"
  "moveit_planners"
  "moveit_simple_controller_manager"
  )

for DEP_PKG in "${DEP_PKGS[@]}"; do
  if $(grep -q $DEP_PKG package.xml); then
    echo "'$DEP_PKG' is already listed in package.xml"
  else
    append_to_string="<buildtool_depend>ament_cmake<\/buildtool_depend>"
    sed -i "s/$append_to_string/$append_to_string\\n\\n  <exec_depend>${DEP_PKG}<\/exec_depend>/g" package.xml
  fi
done

# CMakeLists.txt: Add install paths of the files
prepend_to_string="if(BUILD_TESTING)"
sed -i "s/$prepend_to_string/install\(\\n  DIRECTORY config launch rviz srdf\\n  DESTINATION share\/\$\{PROJECT_NAME\}\\n\)\\n\\n$prepend_to_string/g" CMakeLists.txt

# extend README with general instructions
if [ -f README.md ]; then
  cat $MOVEIT_TEMPLATES/append_to_README.md >>README.md
  sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" README.md
  sed -i "s/\\\$CELL_NAME\\\$/${CELL_NAME}/g" README.md
fi

# TODO: Add license checks

# skip compilation, let the user install MoveIt manually, as it can introduce
# breaking changes often.

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}FINISHED: You can test the configuration by first launching the ros2_control bringup, followed by
'ros2 launch $PKG_NAME moveit.launch${LAUNCH_FILE_TYPES[*]}'${TERMINAL_COLOR_NC}"
