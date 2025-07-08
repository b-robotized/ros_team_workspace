#!/usr/bin/env bash
# Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

# install_software_22.bash [computer_type:{office, robot, default:basic}]

script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SCRIPT_PATH=$script_own_dir
source "${SCRIPT_PATH}"/../../setup.bash

if [ -z "$supported_computer_types" ]; then
  readonly supported_computer_types=("basic" "office" "robot")
fi

computer_type=$1
# check if given computer_type is supported. If this is not the case, then inform user about supported types and
# let user chose which typ he would like.
if ! [[ " ${supported_computer_types[*]} " =~ " ${computer_type} " ]]; then
  notify_user "The computer type ${computer_type} you have given is not supported."
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION} Chose one of the following:"
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION} basic    -Standard pc. Basic utilities like development utilities (git, vscode, ...) and some additional tools."
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION} office   -Used for office pcs. Basic utilities, additional tools and office related stuff."
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION} robot    -Used for robot platforms. Basic utilities like development utilities (git, vscode, ...)."
  select computer_type in basic office robot;
  do
    case "$computer_type" in
          basic)
              computer_type="basic"
              break
            ;;
          office)
              computer_type="office"
              break
            ;;
          robot)
              computer_type="robot"
              break
            ;;
    esac
  done
  echo -n -e "${TERMINAL_COLOR_NC}"
fi

echo "Installing software for $computer_type computers. Press <ENTER> to continue."
read

ROS2_VERSIONS=( "jazzy" "rolling" )

### BACKPORTS ###
# KDE Backports
sudo apt-add-repository -y ppa:kubuntu-ppa/backports
sudo apt update
sudo apt -y dist-upgrade
sudo apt -y autoremove

### CORE TOOLS ###
sudo apt update
sudo apt -y install ca-certificates curl gnupg gnupg2 lsb-release

# Nala - better apt frontend
sudo apt -y install nala
nala --install-completion bash

# git stable upstream version
sudo add-apt-repository -y ppa:git-core/ppa
sudo apt update

### BASIC TOOLS ###
sudo nala install -y neovim ssh git qgit trash-cli htop unrar screen finger ksshaskpass kompare filelight tldr tree pre-commit tmux apt-transport-https curl

## Enable flatpak
sudo nala install -y plasma-discover-backend-flatpak kde-config-flatpak
sudo flatpak remote-add --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo

# Useful libraries
sudo nala install -y libxml2-dev libvlc-dev libmuparser-dev libudev-dev

### DEVELOPMENT TOOLS ###
# gh - Github CLI (it seems that gh needs to be install via apt to get access to .ssh keys)
sudo nala install -y gh
gh completion -s bash | tee "$HOME"/.local/share/bash-completion/completions/gh.bash > /dev/null

# visual studio code
sudo snap install --classic code
# install all plugins for visual studio code
vs_code_plugin_file=$RosTeamWS_FRAMEWORK_OS_CONFIGURE_PATH/vs-code_plugins.txt
while read extension; do
  # ignore empty lines or lines starting with "#"
  [[ $extension =~ ^#.* ]] || [ -z "$extension" ] && continue
  code --install-extension "${extension}"
done < "${vs_code_plugin_file}"

# Docker
sudo apt-get update
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo nala install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
sudo groupadd docker
sudo usermod -aG docker "$(whoami)"

# Python tools
sudo nala install -y python3-pip \
  python3-colcon-common-extensions \
  python3-colcon-cd \
  python3-colcon-argcomplete \
  python3-flake8 \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-docstrings \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-rosdep \
  python3-rosdep2 \
  python3-setuptools \
  python3-vcstool \
  python3-pip \
  python3-virtualenv \
  python3-virtualenvwrapper \
  python3-notebook \
pip3 install --upgrade pip

## ROS
# ROS2 Packages
for ROS2_VERSION in "${ROS2_VERSIONS[@]}"
do
  bash $SCRIPT_PATH/install_software_ros2.bash $ROS2_VERSION
done

sudo rosdep init
rosdep update


# setup bash
cat "$OS_CONFIGURE_TEMPLATES/extend_to_bashrc" >> "$HOME/.bashrc"
cat "$OS_CONFIGURE_TEMPLATES/extend_to_bash_aliases" >> "$HOME/.bash_aliases"
cat "$OS_CONFIGURE_TEMPLATES/extend_to_bash_commands" >> "$HOME/.bash_commands"

# setup git
commit_template_path="$HOME/.config/git"
template_name="commit-template.txt"
mkdir -p "$commit_template_path"
cp "$OS_CONFIGURE_TEMPLATES/$template_name" "$commit_template_path/."
git config --global core.editor "vim"
git config --global commit.template "$commit_template_path/$template_name"


########################## END BASIC SETUP ##########################

if [[ "${computer_type}" != "robot" ]]
then
  sudo nala install -y recordmydesktop peek rdesktop gimp meshlab inkscape pdfposter unrar
fi

if [[ "${computer_type}" == "office" ]]
then
  # Nextcloud
  sudo nala install -y nextcloud-desktop
  #flatpak install app/com.gitlab.j0chn.nextcloud_password_client/x86_64/stable
  wget https://github.com/nextcloud-releases/talk-desktop/releases/latest/download/Nextcloud.Talk-linux-x64.flatpak
  sudo flatpak install -y Nextcloud.Talk-linux-x64.flatpak

  # Dolphin Plugins
  sudo nala install -y kdesdk-scripts dolphin-nextcloud

  # VirtualBox
  sudo nala install -y virtualbox dkms virtualbox-guest-utils virtualbox-ext-pack

  ##  Network monitoring and debugging
  # Wireshark
  sudo DEBIAN_FRONTEND=noninteractive apt -y install wireshark
  # debugging
  sudo apt install nethogs nload net-tools

  ### ADDITIONAL DEVELOPMENT TOOLS ###
  # Code optimization and profiling
  sudo nala install -y valgrind kcachegrind hotspot heaptrack-gui
  sudo nala install -y linux-tools-generic linux-cloud-tools-generic

  # Kernel tools
  sudo nala install -y flac

  # 3D software
  flatpak install -y app/org.freecad.FreeCAD/x86_64/stable

  # Logitech tools
  sudo add-apt-repository ppa:solaar-unifying/stable
  sudo nala update
  sudo nala install -y solaar

  # Tracing
  sudo nala install -y lttng-tools lttng-modules-dkms liblttng-ust-dev
  sudo nala install -y python3-babeltrace python3-lttng python3-lttngust
  sudo usermod -aG tracing "$(whoami)"

  ### MANAGE PERSONAL INFORMATION
  # KDE-PIM
  sudo nala install -y kontact korganizer kmail kjots kaddressbook kdepim*

  ### OFFICE TOOLS ###
  # Notes-taking
  sudo add-apt-repository ppa:pbek/qownnotes
  sudo apt update
  sudo nala install -y qownnotes

fi

# Setting up protools
read -p "${TERMINAL_COLOR_USER_INPUT_DECISION}}Do you want to setup pro-tool that Dr. Denis uses?${RAW_TERMINAL_COLOR_NC} (yes/no) [no]: " protools
protools=${protools:="no"}

if  [[ "$protools" == "yes" ]]; then
  ### CRYPTOPGRAPHY AND PASSWORD MANAGEMENT ###
  # Cryptography
  sudo nala install -y kleopatra scdaemon
  #  Passwordmanager
  sudo nala install -y pass

  ### NETWORKING ###
  # Network manager
  sudo nala install -y network-manager-openvpn network-manager-vpnc network-manager-ssh network-manager-openconnect

  # Remote desktop
  sudo nala install -y krdc

  ### LANGUAGES ###
  # Language packs
  sudo nala install -y language-pack-de language-pack-de-base language-pack-kde-de aspell-de hunspell-de-de hyphen-de wogerman
  sudo nala install -y language-pack-hr language-pack-hr-base language-pack-kde-hr aspell-hr hunspell-hr hyphen-hr

fi

sudo apt update
sudo apt dist-upgrade
sudo apt autoremove
# log is created somehow
trash "${RosTeamWS_FRAMEWORK_OS_CONFIGURE_PATH}/log/"

# Get Corporate Identity package from b»robotized and install them
cd ~/Downloads
NAME="br-ci-package-computer"
ZIP_NAME="${name}.zip"
wget https://cloud.b-robotized.com/s/447BJD7tXrd3nDA/download -O ${ZIP_NAME}
unzip ${ZIP_NAME} -d ${NAME}
rm ${ZIP_NAME}
sudo mv ${NAME} /usr/share/
# Create your Theme or change files in: /usr/share/sddm/themes/kubuntu/theme.conf
