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

RTW_BUNDLE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RTW_TARGET_USERS=()
source "$RTW_BUNDLE_DIR/device.conf"
RTW_ROLE=${RTW_ROLE:-basic}
RTW_GIT_REF=${RTW_GIT_REF:-master}
RTW_INSTALL_DIR=/opt/ros_team_workspace
RTW_VENV_DIR=/opt/ros_team_workspace_venv
RTW_PRO_TOOLS=${RTW_PRO_TOOLS:-0}
RTW_KDE_BACKPORTS=${RTW_KDE_BACKPORTS:-0}
RTW_INSTALL_ROLLING=${RTW_INSTALL_ROLLING:-0}
RTW_INSTALL_BRANDING=${RTW_INSTALL_BRANDING:-0}
RTW_REPLACE_LIBREOFFICE=${RTW_REPLACE_LIBREOFFICE:-0}
RTW_GRANT_DOCKER=${RTW_GRANT_DOCKER:-0}
RTW_GRANT_SUDO=${RTW_GRANT_SUDO:-0}
RTW_CTRLD_BOX_HOST=${RTW_CTRLD_BOX_HOST:-}
RTW_CTRLD_BOX_MAC=${RTW_CTRLD_BOX_MAC:-}
[[ -z "$RTW_CTRLD_BOX_HOST" || "$RTW_CTRLD_BOX_HOST" =~ ^[0-9]+$ && "$RTW_CTRLD_BOX_HOST" -ge 2 && "$RTW_CTRLD_BOX_HOST" -le 254 ]] \
  || { echo 'RTW_CTRLD_BOX_HOST must be 2..254' >&2; exit 2; }
[[ -z "$RTW_CTRLD_BOX_MAC" || "$RTW_CTRLD_BOX_MAC" =~ ^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$ ]] \
  || { echo 'RTW_CTRLD_BOX_MAC must be a MAC address' >&2; exit 2; }

case "$RTW_ROLE" in basic|office|demo) ;; *) echo "Invalid RTW_ROLE: $RTW_ROLE" >&2; exit 2;; esac
for flag in RTW_PRO_TOOLS RTW_KDE_BACKPORTS RTW_INSTALL_ROLLING RTW_INSTALL_BRANDING \
  RTW_REPLACE_LIBREOFFICE RTW_GRANT_DOCKER RTW_GRANT_SUDO; do
  case "${!flag}" in 0|1) ;; *) echo "$flag must be 0 or 1" >&2; exit 2;; esac
done

source /etc/os-release
case "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}" in
  noble) RTW_UBUNTU_CODENAME=noble; RTW_ROS_DISTRO=jazzy ;;
  resolute) RTW_UBUNTU_CODENAME=resolute; RTW_ROS_DISTRO=lyrical ;;
  *) echo 'Supported Ubuntu bases: 24.04 noble, 26.04 resolute.' >&2; exit 2 ;;
esac
ROS2_VERSIONS=("$RTW_ROS_DISTRO")
if [[ "$RTW_INSTALL_ROLLING" == 1 ]]; then ROS2_VERSIONS+=(rolling); fi

log() { printf '\n[RTW] %s\n' "$*"; }
require_root() { [[ $EUID == 0 ]] || { echo 'Run this phase as root.' >&2; exit 2; }; }
require_user() { [[ $EUID != 0 ]] || { echo 'Run this phase as the user, not root.' >&2; exit 2; }; }
trap 'printf "RTW failed at %s:%s (exit %s)\n" "${BASH_SOURCE[0]}" "$LINENO" "$?" >&2' ERR

validate_accounts() {
  local account
  [[ $# -gt 0 ]] || { echo 'Set RTW_TARGET_USERS in device.conf.' >&2; exit 2; }
  for account in "$@"; do
    [[ "$account" =~ ^[a-z_][a-z0-9_-]*$ ]] && id "$account" >/dev/null 2>&1 \
      && [[ "$(id -u "$account")" -ge 1000 ]] || {
      echo "Invalid or missing non-system account: $account" >&2; exit 2;
    }
  done
}

# Landscape package profile names for this machine.
required_profiles() {
  local distro
  echo br-base
  for distro in "${ROS2_VERSIONS[@]}"; do echo "br-ros-$distro"; done
  case "$RTW_ROLE" in
    basic) echo br-graphics ;;
    office) echo br-graphics; echo br-office ;;
    demo) echo br-demo ;;
  esac
  if [[ "$RTW_PRO_TOOLS" == 1 ]]; then echo br-pro-tools; fi
}

profile_packages() {
  local profile=$1 file
  case "$profile" in
    br-ros-*) file=br-ros.txt ;;
    *) file="$profile.txt" ;;
  esac
  grep -Ev '^[[:space:]]*(#|$)' "$RTW_BUNDLE_DIR/profiles/$file" | sed "s/DISTRO/${profile#br-ros-}/"
}

# Phases 20 to 40 run only after Landscape has applied all package profiles.
require_profiles() {
  local profile package missing=()
  for profile in $(required_profiles); do
    for package in $(profile_packages "$profile"); do
      dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'install ok installed' \
        || missing+=("$profile:$package")
    done
  done
  if [[ ${#missing[@]} -gt 0 ]]; then
    printf 'Package profiles are not compliant yet. Missing:\n' >&2
    printf '  %s\n' "${missing[@]}" >&2
    exit 3
  fi
  log "Package profiles compliant: $(required_profiles | tr '\n' ' ')"
}
