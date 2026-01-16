# Copyright (c) 2026 b»robotized
# All rights reserved.
#
# Proprietary License
#
# Unauthorized copying of this file, via any medium is strictly prohibited.
# The file is considered confidential.


usage="setup-ikfast-plugin.bash ROBOT_NAME [CLASS_NAME]"

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../../setup.bash
# Find exact location of the plugin templates
TEMPLATE_DIR="$script_own_dir/../../templates/ikfast_plugin"

check_and_set_ros_distro_and_version "${ROS_DISTRO}"

ROBOT_NAME=$1
if [ -z "$1" ]; then
  print_and_exit "You should provide the robot name! Nothing to do 😯" "$usage"
fi
if [ -f src/"${ROBOT_NAME}_ikfast.cpp" ]; then
  print_and_exit "ERROR:The file '${ROBOT_NAME}_ikfast.cpp' already exist! 😱!" "$usage"
fi

if [ ! -f "package.xml" ]; then
  print_and_exit "ERROR: 'package.xml' not found. You should execute this script at the top level of your package folder. Nothing to do 😯" "$usage"
fi

FILE_NAME="${ROBOT_NAME}_ikfast"


echo "" # Adds empty line

CLASS_NAME=$2
if [ -z "$2" ]; then
  delimiter='_'
  s="$ROBOT_NAME$delimiter"
  CLASS_NAME=""
  while [[ $s ]]; do
    part="${s%%"$delimiter"*}"
    s=${s#*"$delimiter"}
    CLASS_NAME="$CLASS_NAME${part^}"
  done
  CLASS_NAME="${CLASS_NAME}Kinematics"
  echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}ClassName guessed from the '$ROBOT_NAME': '$CLASS_NAME'. Is this correct? If not provide it as the second parameter.${TERMINAL_COLOR_NC}"
fi

#License
echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Which license-header do you want to use? [1]"
echo "(0) None"
echo "(1) Apache 2.0 License"
echo "(2) Proprietary"
echo -n -e "${TERMINAL_COLOR_NC}"
read choice
choice=${choice:="1"}

if [ "$choice" != 0 ]; then
  echo -n -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Insert your company or personal name (copyright): ${TERMINAL_COLOR_NC}"
  read NAME_ON_LICENSE
  NAME_ON_LICENSE=${NAME_ON_LICENSE:="User"}
  YEAR_ON_LICENSE=`date +%Y`
fi

LICENSE_HEADER=""
case "$choice" in
"1") LICENSE_HEADER="$LICENSE_TEMPLATES/default_cpp.txt" ;;
"2") LICENSE_HEADER="$LICENSE_TEMPLATES/propriatery_company_cpp.txt" ;;
esac

echo -n -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Is package already configured (CMake/Package.xml updated)? (yes/no) [no]: ${TERMINAL_COLOR_NC}"
read package_configured
package_configured=${package_configured:="no"}

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}ATTENTION: Setting up IKFast plugin with: robot '$ROBOT_NAME', class '$CLASS_NAME', package '$FILE_NAME'. Directory: '`pwd`'.${TERMINAL_COLOR_NC}"
echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}If correct press <ENTER>, otherwise <CTRL>+C.${TERMINAL_COLOR_NC}"
read

# Add folders if deleted
PKG_NAME="$(grep -Po '(?<=<name>).*?(?=</name>)' package.xml | sed -e 's/[[:space:]]//g')"
ADD_FOLDERS=("include/$PKG_NAME" "src")
for FOLDER in "${ADD_FOLDERS[@]}"; do
    mkdir -p $FOLDER
done

# Set file constants
PLUGIN_CPP="src/${FILE_NAME}_plugin.cpp"
PLUGIN_XML="$FILE_NAME.xml"

# copy the files and rename/rewrite it
if [ ! -f "$TEMPLATE_DIR/dummy_ikfast_plugin.cpp" ]; then
    echo -e "${TERMINAL_COLOR_USER_ERROR}ERROR: Template not found in $TEMPLATE_DIR${TERMINAL_COLOR_NC}"
    exit 1
fi
cp --update=none "$TEMPLATE_DIR/dummy_ikfast_plugin.cpp" "$PLUGIN_CPP"
cp --update=none "$TEMPLATE_DIR/dummy_ikfast_plugin.xml" "$PLUGIN_XML"

echo -e "${TERMINAL_COLOR_USER_NOTICE}Template files copied from ${TEMPLATE_DIR}.${TERMINAL_COLOR_NC}"

# Add license header to the files
FILES_TO_LICENSE=("$PLUGIN_CPP")
TMP_FILE=".f_tmp"
if [[ "$LICENSE_HEADER" != "" && -f "$LICENSE_HEADER" ]]; then
  for FILE_TO_LIC in "${FILES_TO_LICENSE[@]}"; do
    cat "$LICENSE_HEADER" > $TMP_FILE
    cat "$FILE_TO_LIC" >> $TMP_FILE
    mv $TMP_FILE "$FILE_TO_LIC"
    sed -i "s/\\\$YEAR\\\$/${YEAR_ON_LICENSE}/g" "$FILE_TO_LIC"
    sed -i "s/\\\$NAME_ON_LICENSE\\\$/${NAME_ON_LICENSE}/g" "$FILE_TO_LIC"
  done
fi

# sed all needed files
FILES_TO_SED=("$PLUGIN_CPP" "$PLUGIN_XML")
# declare -p FILES_TO_SED
for SED_FILE in "${FILES_TO_SED[@]}"; do
  sed -i "s/dummy_ikfast_plugin/$FILE_NAME/g" $SED_FILE
  sed -i "s/dummy_ikfast/$FILE_NAME/g" $SED_FILE
  sed -i "s/DummyKinematics/$CLASS_NAME/g" $SED_FILE
  sed -i "s/Dummy/$ROBOT_NAME/g" $SED_FILE
  sed -i "s/ikfast_dummy.cpp/${FILE_NAME}.cpp/g" $SED_FILE
done

DEP_PKGS=("pluginlib" "kinematics_interface" "kinematics_interface_ikfast" "rclcpp")

# package.xml
for DEP_PKG in "${DEP_PKGS[@]}"; do
  if ! grep -q "<depend>${DEP_PKG}</depend>" package.xml; then
    sed -i "/<buildtool_depend>ament_cmake<\/buildtool_depend>/a \  <depend>${DEP_PKG}<\/depend>" package.xml
  fi
done

# 9. Manage CMakeLists.txt (If not configured)
if [[ "$package_configured" == "no" ]]; then
  echo -e "${TERMINAL_COLOR_USER_NOTICE}Updating CMakeLists.txt...${TERMINAL_COLOR_NC}"

  # Remove old comments
  DEL_STRINGS=("# uncomment the following" "# further" "# find_package(<dependency>")
  for DEL_STR in "${DEL_STRINGS[@]}"; do
    sed -i "/$DEL_STR/d" CMakeLists.txt
  done

  touch $TMP_FILE
  # ament_cmake satırından öncesini al
  TEST_LINE=`awk '$1 == "find_package(ament_cmake" { print NR }' CMakeLists.txt`
  let CUT_LINE=$TEST_LINE-1
  head -$CUT_LINE CMakeLists.txt >> $TMP_FILE

  # Define specific IKFast dependencies
  echo "set(THIS_PACKAGE_INCLUDE_DEPENDS" >> $TMP_FILE
  echo "  pluginlib" >> $TMP_FILE
  echo "  kinematics_interface" >> $TMP_FILE
  echo "  kinematics_interface_ikfast" >> $TMP_FILE
  echo "  rclcpp" >> $TMP_FILE
  echo ")" >> $TMP_FILE

  echo "" >> $TMP_FILE
  echo "find_package(ament_cmake REQUIRED)" >> $TMP_FILE
  echo "foreach(Dependency IN ITEMS \${THIS_PACKAGE_INCLUDE_DEPENDS})" >> $TMP_FILE
  echo "  find_package(\${Dependency} REQUIRED)" >> $TMP_FILE
  echo "endforeach()" >> $TMP_FILE
  echo "" >> $TMP_FILE

  # IKFast Library and Export rules
  echo "# IKFast Library Definition" >> $TMP_FILE
  echo "add_library($FILE_NAME SHARED $PLUGIN_CPP)" >> $TMP_FILE
  echo "target_include_directories($FILE_NAME PRIVATE" >> $TMP_FILE
  echo '  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"' >> $TMP_FILE
  echo '  "$<INSTALL_INTERFACE:include>")' >> $TMP_FILE
  echo "ament_target_dependencies($FILE_NAME \${THIS_PACKAGE_INCLUDE_DEPENDS})" >> $TMP_FILE
  echo "" >> $TMP_FILE
  echo "pluginlib_export_plugin_description_file(kinematics_interface $PLUGIN_XML)" >> $TMP_FILE

  # Installation
  echo "install(TARGETS $FILE_NAME" >> $TMP_FILE
  echo "  EXPORT export_\${PROJECT_NAME}" >> $TMP_FILE
  echo "  RUNTIME DESTINATION bin" >> $TMP_FILE
  echo "  ARCHIVE DESTINATION lib" >> $TMP_FILE
  echo "  LIBRARY DESTINATION lib" >> $TMP_FILE
  echo ")" >> $TMP_FILE
  echo "ament_export_include_directories(include)" >> $TMP_FILE
  echo "ament_export_libraries($FILE_NAME)" >> $TMP_FILE
  echo "ament_export_dependencies(\${THIS_PACKAGE_INCLUDE_DEPENDS})" >> $TMP_FILE
  echo "ament_package()" >> $TMP_FILE

  mv $TMP_FILE CMakeLists.txt
fi

# Manage package.xml (If not configured)
if [[ "$package_configured" == "no" ]]; then
  echo -e "${TERMINAL_COLOR_USER_NOTICE}Updating package.xml dependencies...${TERMINAL_COLOR_NC}"
  DEP_PKGS=("pluginlib" "kinematics_interface" "kinematics_interface_ikfast" "rclcpp")
  for DEP_PKG in "${DEP_PKGS[@]}"; do
    if ! grep -q "<depend>${DEP_PKG}</depend>" package.xml; then
      sed -i "/<buildtool_depend>ament_cmake<\/buildtool_depend>/a \  <depend>${DEP_PKG}<\/depend>" package.xml
    fi
  done
fi
# extend README with general instructions

if [ -f README.md ]; then
  echo -e "\n### IKFast Kinematics Plugin\n* **Library:** $FILE_NAME\n* **Plugin Class:** $FILE_NAME/$CLASS_NAME" >> README.md
fi


echo -e "${TERMINAL_COLOR_USER_NOTICE}Template files were adjusted.${TERMINAL_COLOR_NC}"

# Github initialization
IS_GIT_REPO=$(git rev-parse --is-inside-work-tree 2>/dev/null)

if [ "$IS_GIT_REPO" == "true" ]; then
    echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}Github repository is detected. Changes are being added...${TERMINAL_COLOR_NC}"
    git add .
    # git commit -m "RosTeamWS: ros2_control skeleton files for $ROBOT_NAME generated."
else
    echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}This package is not a Git repository. Initialize Git (git init)? [y/N]${TERMINAL_COLOR_NC}"
    read -r git_init_choice
    if [[ "$git_init_choice" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        git init
        git add .
        echo -e "${TERMINAL_COLOR_USER_NOTICE}Git repository initialized and files added.${TERMINAL_COLOR_NC}"
    else
        echo -e "${TERMINAL_COLOR_USER_NOTICE}Git operations skipped.${TERMINAL_COLOR_NC}"
    fi
fi
# git commit -m "RosTeamWS: ros2_control skeleton files for $ROBOT_NAME generated."

# Compile and add new package the to the path
compile_and_source_package $PKG_NAME "yes"

echo ""

echo -e "${TERMINAL_COLOR_USER_NOTICE}FINISHED: IKFast package for $ROBOT_NAME is ready.${TERMINAL_COLOR_NC}"