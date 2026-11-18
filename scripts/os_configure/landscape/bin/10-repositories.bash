#!/usr/bin/env bash
# Copyright (c) 2026, b»robotized group
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

set -Eeuo pipefail

source "$(dirname -- "${BASH_SOURCE[0]}")/common.bash"
require_root
export DEBIAN_FRONTEND=noninteractive
work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT

log 'Configure APT repositories'
apt-get update
apt-get -o DPkg::Lock::Timeout=300 install -y ca-certificates curl gnupg software-properties-common locales
add-apt-repository -y universe
add-apt-repository -y multiverse
if [[ "$RTW_KDE_BACKPORTS" == 1 ]]; then
  [[ "${ID:-}" != neon ]] || { echo 'Kubuntu Backports must not be enabled on KDE neon.' >&2; exit 2; }
  add-apt-repository -y ppa:kubuntu-ppa/backports
fi
add-apt-repository -y ppa:git-core/ppa

install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o "$work/docker.asc"
gpg --batch --yes --dearmor -o /etc/apt/keyrings/docker.gpg "$work/docker.asc"
chmod 0644 /etc/apt/keyrings/docker.gpg
printf 'deb [arch=%s signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu %s stable\n' \
  "$(dpkg --print-architecture)" "$RTW_UBUNTU_CODENAME" > /etc/apt/sources.list.d/docker.list

# ROS repository through the official ros2-apt-source package; it keeps the key up to date.
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
  version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')
  curl -fsSL -o "$work/ros2-apt-source.deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.${RTW_UBUNTU_CODENAME}_all.deb"
  rm -f /etc/apt/sources.list.d/ros2.list /usr/share/keyrings/ros-archive-keyring.gpg
  dpkg -i "$work/ros2-apt-source.deb"
fi

locale-gen en_US.UTF-8
apt-get update
log 'Repositories configured. Next: add the Landscape tags for the package profiles.'
