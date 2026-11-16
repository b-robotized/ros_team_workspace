#!/bin/bash
#
# Copyright 2026 b»robotized group
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


usage='setup-docker-publish-ghcr.bash ["dockerfile_path"]'

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../setup.bash

dockerfile=${1:-"Dockerfile"}
if [ ! -f "$dockerfile" ]; then
  print_and_exit "Dockerfile '$dockerfile' does not exist!" "$usage"
fi

read -p "${RAW_TERMINAL_COLOR_BROWN}Name of the ROS distro for the image? Supported distributions(${rtw_supported_ros_distributions[*]})${RAW_TERMINAL_COLOR_NC} [default: rolling] " default_ros_distro
default_ros_distro=${default_ros_distro:="rolling"}
check_and_set_ros_distro_and_version "$default_ros_distro"
read -p "${RAW_TERMINAL_COLOR_BROWN}Name of the repository default branch?${RAW_TERMINAL_COLOR_NC} [default: master]: " default_branch
default_branch=${default_branch:="master"}

CI_FILE=".github/workflows/docker-publish-ghcr.yml"
mkdir -p .github/workflows
cp --update=none ${PACKAGE_TEMPLATES}/CI-github_docker-publish-ghcr.yml $CI_FILE
sed -i 's/\$ros_distro\$/'${ros_distro}'/g' $CI_FILE
sed -i 's/\$branch\$/'${default_branch}'/g' $CI_FILE
sed -i 's#\$DOCKERFILE\$#'${dockerfile}'#g' $CI_FILE

echo ""
echo -e "${TERMINAL_COLOR_BLUE}FINISHED: Check '$CI_FILE'. After the first push, set the package visibility in the GitHub package settings.${TERMINAL_COLOR_NC}"
