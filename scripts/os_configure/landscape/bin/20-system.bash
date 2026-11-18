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
  curl -fL https://github.com/nextcloud-releases/talk-desktop/releases/latest/download/Nextcloud.Talk-linux-x64.flatpak \
    -o "$work/Nextcloud.Talk-linux-x64.flatpak"
  flatpak install --system --noninteractive -y "$work/Nextcloud.Talk-linux-x64.flatpak"
  flatpak install --system --noninteractive -y flathub org.freecad.FreeCAD
fi

log 'rosdep'
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi

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
  if [[ "$(uname -r)" != *-realtime ]]; then
    printf '[RTW] WARNING: running kernel %s is not the Ubuntu realtime kernel. Select it in GRUB and reboot.\n' \
      "$(uname -r)" >&2
  fi
fi

if [[ "$RTW_INSTALL_BRANDING" == 1 ]]; then
  log 'Branding'
  curl -fL https://cloud.b-robotized.com/s/447BJD7tXrd3nDA/download -o "$work/branding.zip"
  python3 - "$work/branding.zip" <<'BRANDING'
import pathlib, sys, zipfile
out = pathlib.Path('/usr/share/br-ci-package-computer')
out.mkdir(parents=True, exist_ok=True)
with zipfile.ZipFile(sys.argv[1]) as z:
    for item in z.infolist():
        p = pathlib.PurePosixPath(item.filename)
        if not item.is_dir() and len(p.parts) == 1:
            dest = out / p.name
            if dest.is_symlink():
                raise RuntimeError(f'Refusing symlink: {dest}')
            dest.write_bytes(z.read(item))
            dest.chmod(0o644)
BRANDING
fi

log 'System setup complete. Next: phase 30 user settings.'
