#!/bin/bash
#
# Copyright 2021 Denis Stogl (Stogl Robotics Consulting)
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

script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../../setup.bash

user_decision "Do you want to automatically source RosTeamWs? This is going to add a .ros_team_ws file to your home directory. Additionally your .bashrc is modified to automatically source RosTeamWs."
if [[ " ${negative_answers[*]} " =~ " ${user_answer} " ]]; then
    print_and_exit "Aborting the setup of autosourcing. Exiting..."
fi

echo "Copying .ros_team_ws_rc to your home folder."

rtw_file=".ros_team_ws_rc"
rtw_file_location=~/"$rtw_file"
template_location="$FRAMEWORK_BASE_PATH/templates/$rtw_file"
if ! [ -f "$HOME/$rtw_file" ]; then
    cp "${template_location}" ~/.
fi
sed -i "s|source <PATH TO ros_team_workspace>/setup.bash|source $FRAMEWORK_BASE_PATH/setup.bash|g" "$rtw_file_location"

echo ""
user_decision "Do you want to add add colored output to your shell? This will show the current git branche, ros workspace and time in terminal. If you are unsure you can test this later by uncommenting the \"export PS1=\" in the .ros_team_ws_rc file or rerun the setup-auto-sourcing command."
if [[ " ${positive_answers[*]} " =~ " ${user_answer} " ]]; then
    echo "You can revert this by uncommenting the \"export PS1=\" line in the .ros_team_ws_rc file. Or rerun the setup script."
    # uncomment line with export PS1
    # find line number
    export_ps1_line_number=$(grep -n "#.*export PS1=" .ros_team_ws_rc | grep -Eo '^[^:]+')
    # uncomment
    if ! [ -z "$export_ps1_line_number" ]; then
        sed -i "${export_ps1_line_number},${export_ps1_line_number}s|#||g" "$rtw_file_location"
    fi
else
    echo "Skipping. "
    # comment line with export PS1
    # find line number
    export_ps1_line_number=$(grep -n ".*export PS1=" .ros_team_ws_rc | grep -Eo '^[^:]+')
    # comment
    if ! [ -z "$export_ps1_line_number" ]; then
        sed -i "${export_ps1_line_number},${export_ps1_line_number}s|^|#|g" "$rtw_file_location"
    fi
fi


echo "Updating your .bashrc file."
bashrc_location=~/.bashrc
if ! ( grep -q '\..*\.ros_team_ws_rc' $bashrc_location || grep -q 'source.*\.ros_team_ws_rc' $bashrc_location ); then
    echo "" >> $bashrc_location
    echo "# automatically source RosTeamWorkspace if the .ros_team_ws file is present in your home folder." >> $bashrc_location
    echo "if [ -f ~/.ros_team_ws_rc ]; then" >> $bashrc_location
    echo "    . ~/.ros_team_ws_rc" >> $bashrc_location
    echo "fi" >> $bashrc_location
fi

echo "Done! Please open a new terminal now."
