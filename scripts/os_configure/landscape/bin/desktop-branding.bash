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

# Runs once in the Plasma session of the user (autostart from phase 30):
# b»robotized wallpaper and application menu icon.
set -uo pipefail

ci=/usr/share/br-ci-package-computer
wallpaper="$ci/Primary - SDDM Background - rgb.svg"
icon="$ci/Primary - Icon - rgb - no bg - no margin.svg"
autostart="$HOME/.config/autostart/rtw-desktop-branding.desktop"

for _ in $(seq 1 30); do
  qdbus6 org.kde.plasmashell /PlasmaShell >/dev/null 2>&1 && break
  sleep 2
done

[[ -f "$wallpaper" ]] && plasma-apply-wallpaperimage "$wallpaper"
[[ -f "$icon" ]] && qdbus6 org.kde.plasmashell /PlasmaShell org.kde.PlasmaShell.evaluateScript "
panels().forEach(function (panel) {
  panel.widgets().forEach(function (widget) {
    if (['org.kde.plasma.kickoff', 'org.kde.plasma.kicker', 'org.kde.plasma.kickerdash'].indexOf(widget.type) >= 0) {
      widget.currentConfigGroup = ['General'];
      widget.writeConfig('icon', '$icon');
    }
  });
});"

rm -f "$autostart"
