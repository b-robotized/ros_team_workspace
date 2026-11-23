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

# APT packages come from the Landscape package profiles. This phase does the rest.
set -Eeuo pipefail

source "$(dirname -- "${BASH_SOURCE[0]}")/common.bash"
require_root
require_profiles
work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT

log 'Flatpak and snap applications'
flatpak remote-add --system --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo
if ! snap list code >/dev/null 2>&1; then snap install --classic code; fi
if [[ "$RTW_REPLACE_LIBREOFFICE" == 1 ]]; then
  apt-get remove -y 'libreoffice-*'
  flatpak install --system --noninteractive -y flathub org.libreoffice.LibreOffice
fi
if [[ "$RTW_ROLE" == office ]]; then
  if ! flatpak info --system com.nextcloud.talk >/dev/null 2>&1; then
    curl -fL https://github.com/nextcloud-releases/talk-desktop/releases/latest/download/Nextcloud.Talk-linux-x64.flatpak \
      -o "$work/Nextcloud.Talk-linux-x64.flatpak"
    flatpak install --system --noninteractive -y "$work/Nextcloud.Talk-linux-x64.flatpak"
  fi
  flatpak install --system --noninteractive -y --or-update flathub org.freecad.FreeCAD
fi

log 'rosdep'
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi

if [[ -n "$RTW_CTRLD_BOX_HOST" ]]; then
  log 'Network b-controlled box'
  connection='b-controlled box'
  h=$RTW_CTRLD_BOX_HOST
  nmcli -t -f NAME connection show | grep -qxF "$connection" \
    || nmcli connection add type ethernet con-name "$connection" ifname '*'
  nmcli connection modify "$connection" ipv4.method manual ipv4.never-default yes ipv6.method ignore \
    ipv4.addresses "10.28.28.$h/24,192.168.28.$h/24,192.168.1.$h/24,10.50.50.$h/24"
  if [[ -n "$RTW_CTRLD_BOX_MAC" ]]; then
    nmcli connection modify "$connection" connection.interface-name '' 802-3-ethernet.mac-address "$RTW_CTRLD_BOX_MAC" connection.autoconnect yes
  else
    nmcli connection modify "$connection" connection.autoconnect no
  fi
fi

log "RTW $RTW_GIT_REF in $RTW_INSTALL_DIR"
if [[ ! -d "$RTW_INSTALL_DIR/.git" ]]; then
  git clone https://github.com/b-robotized/ros_team_workspace.git "$RTW_INSTALL_DIR"
fi
git -C "$RTW_INSTALL_DIR" fetch --tags origin "$RTW_GIT_REF"
git -C "$RTW_INSTALL_DIR" checkout --force --detach FETCH_HEAD
git config --system --replace-all safe.directory "$RTW_INSTALL_DIR" "^$RTW_INSTALL_DIR\$"
python3 -m venv "$RTW_VENV_DIR"
(cd "$RTW_INSTALL_DIR/rtwcli" && "$RTW_VENV_DIR/bin/pip" install --quiet -r requirements.txt)
ln -sfn "$RTW_VENV_DIR/bin/rtw" /usr/local/bin/rtw

if [[ "$RTW_ROLE" == demo ]]; then
  log 'Realtime scheduling for group realtime'
  getent group realtime >/dev/null || groupadd --system realtime
  printf '@realtime - rtprio 99\n@realtime - memlock unlimited\n' > /etc/security/limits.d/90-realtime.conf
fi

if [[ "$RTW_INSTALL_BRANDING" == 1 ]]; then
  log 'b»robotized KDE design'
  install -m 0755 "$RTW_BUNDLE_DIR/bin/kde-design-update.bash" /usr/local/sbin/rtw-kde-design-update
  /usr/local/sbin/rtw-kde-design-update
  printf '[Unit]\nDescription=Install the latest b»robotized KDE design release\nAfter=network-online.target\nWants=network-online.target\n\n[Service]\nType=oneshot\nExecStart=/usr/local/sbin/rtw-kde-design-update\n' \
    > /etc/systemd/system/rtw-kde-design-update.service
  printf '[Unit]\nDescription=Daily b»robotized KDE design update\n\n[Timer]\nOnCalendar=daily\nRandomizedDelaySec=1h\nPersistent=true\n\n[Install]\nWantedBy=timers.target\n' \
    > /etc/systemd/system/rtw-kde-design-update.timer
  systemctl daemon-reload
  systemctl enable --now rtw-kde-design-update.timer
fi

if [[ -n "$RTW_TIMEZONE" ]]; then
  log "Time zone $RTW_TIMEZONE"
  timedatectl set-timezone "$RTW_TIMEZONE"
fi
if [[ -n "$RTW_FORMATS" ]]; then
  log "Regional formats $RTW_FORMATS"
  locale-gen en_US.UTF-8 "$RTW_FORMATS"
  update-locale LANG=en_US.UTF-8 LC_NUMERIC="$RTW_FORMATS" LC_TIME="$RTW_FORMATS" LC_MONETARY="$RTW_FORMATS" \
    LC_PAPER="$RTW_FORMATS" LC_NAME="$RTW_FORMATS" LC_ADDRESS="$RTW_FORMATS" LC_TELEPHONE="$RTW_FORMATS" \
    LC_MEASUREMENT="$RTW_FORMATS" LC_IDENTIFICATION="$RTW_FORMATS"
fi

log 'Desktop defaults for all users'
install -m 0755 "$RTW_BUNDLE_DIR/bin/desktop-defaults.bash" /usr/local/libexec/rtw-desktop-defaults
printf '[Desktop Entry]\nType=Application\nName=b»robotized desktop defaults\nExec=/usr/local/libexec/rtw-desktop-defaults\nOnlyShowIn=KDE;\nNoDisplay=true\nX-KDE-autostart-phase=2\n' \
  > /etc/xdg/autostart/rtw-desktop-defaults.desktop

if [[ -n "$RTW_ADMIN_USER" ]] && id "$RTW_ADMIN_USER" >/dev/null 2>&1; then
  log "Admin account $RTW_ADMIN_USER"
  usermod -c "$RTW_ADMIN_NAME" "$RTW_ADMIN_USER"
fi

logo=/usr/share/icons/hicolor/scalable/apps/b-robotized.svg
if [[ "$RTW_INSTALL_BRANDING" == 1 && -f "$logo" ]] && command -v magick >/dev/null; then
  log 'User pictures'
  picture=/usr/share/b-robotized-kde-design/user-picture.png
  install -d /usr/share/b-robotized-kde-design
  magick -background none -density 600 "$logo" -resize 256x256 "$picture"
  for account in $RTW_ADMIN_USER "${RTW_TARGET_USERS[@]}"; do
    id "$account" >/dev/null 2>&1 || continue
    user_path=/org/freedesktop/Accounts/User$(id -u "$account")
    current=$(busctl get-property org.freedesktop.Accounts "$user_path" org.freedesktop.Accounts.User IconFile | cut -d'"' -f2)
    if [[ -z "$current" || ! -s "$current" || "$current" == /var/lib/AccountsService/icons/$account.png ]]; then
      busctl call org.freedesktop.Accounts "$user_path" org.freedesktop.Accounts.User SetIconFile s "$picture"
    fi
  done
fi

log 'System setup complete. Next: phase 30 user settings.'
