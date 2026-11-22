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
  install -m 0755 "$RTW_BUNDLE_DIR/bin/realtime-grub-default.bash" /etc/kernel/postinst.d/zzz-rtw-realtime-default
  if ! grep -q '^GRUB_DEFAULT=saved$' /etc/default/grub; then
    sed -i 's/^GRUB_DEFAULT=.*/GRUB_DEFAULT=saved/' /etc/default/grub
    update-grub
  fi
  /etc/kernel/postinst.d/zzz-rtw-realtime-default
  if [[ "$(uname -r)" != *-realtime ]]; then
    printf '[RTW] WARNING: running kernel %s is not the Ubuntu realtime kernel. Select it in GRUB and reboot.\n' \
      "$(uname -r)" >&2
  fi
fi

if [[ "$RTW_INSTALL_BRANDING" == 1 ]]; then
  log 'b»robotized KDE design'
  release=https://code.b-robotized.com/api/v4/projects/b_public%2Finfra%2Fb_kde_design/releases/permalink/latest
  deb_url=$(curl -fsSL "$release" | python3 -c '
import json, sys
print(next(l["url"] for l in json.load(sys.stdin)["assets"]["links"] if l["url"].endswith(".deb")))')
  curl -fsSL "$deb_url" -o "$work/b-robotized-kde-design.deb"
  apt-get -o DPkg::Lock::Timeout=300 install -y "$work/b-robotized-kde-design.deb"
  grep -ls br-ci-package-computer /usr/share/sddm/themes/*/theme.conf.user | xargs -r rm -f
fi

if [[ -n "$RTW_ADMIN_USER" ]] && id "$RTW_ADMIN_USER" >/dev/null 2>&1; then
  log "Admin account $RTW_ADMIN_USER"
  usermod -c "$RTW_ADMIN_NAME" "$RTW_ADMIN_USER"
  logo=/usr/share/icons/hicolor/scalable/apps/b-robotized.svg
  if [[ -f "$logo" ]] && command -v magick >/dev/null; then
    picture=/var/lib/AccountsService/icons/$RTW_ADMIN_USER.png
    install -d -m 0775 /var/lib/AccountsService/icons
    magick -background none -density 600 "$logo" -resize 256x256 "$picture"
    busctl call org.freedesktop.Accounts "/org/freedesktop/Accounts/User$(id -u "$RTW_ADMIN_USER")" \
      org.freedesktop.Accounts.User SetIconFile s "$picture"
  fi
fi

log 'System setup complete. Next: phase 30 user settings.'
