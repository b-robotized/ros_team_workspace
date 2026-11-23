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

# Runs once per user in the Plasma session (system autostart from phase 20):
# regional formats from the system locale, team holidays and time zones in the clock.
set -uo pipefail

version="1 $(md5sum < /etc/default/locale 2>/dev/null | cut -c1-12)"
marker="${XDG_STATE_HOME:-$HOME/.local/state}/rtw/desktop-defaults"
[[ "$(cat "$marker" 2>/dev/null)" == "$version" ]] && exit 0

time_zones='Local,Africa/Casablanca,Africa/Cairo,Europe/Berlin,America/New_York,Asia/Singapore'
holiday_regions='de_de,de-bw_de,hr_hr,si_sl,eg_ar,ma_ar,us_en-us'
calendar_plugins='holidaysevents,astronomicalevents,pimevents'

if [[ -f /etc/default/locale ]]; then
  for key in LC_NUMERIC LC_TIME LC_MONETARY LC_MEASUREMENT LC_PAPER LC_ADDRESS LC_NAME LC_TELEPHONE; do
    value=$(sed -n "s/^$key=//p" /etc/default/locale | tr -d '"')
    [[ -n "$value" ]] && kwriteconfig6 --file plasma-localerc --group Formats --key "$key" "$value"
  done
fi
kwriteconfig6 --file plasma_calendar_holiday_regions --group General --key selectedRegions "$holiday_regions"

for _ in $(seq 1 30); do
  qdbus6 org.kde.plasmashell /PlasmaShell >/dev/null 2>&1 && break
  sleep 2
done
qdbus6 org.kde.plasmashell /PlasmaShell org.kde.PlasmaShell.evaluateScript "
panels().forEach(function (panel) {
  panel.widgets('org.kde.plasma.digitalclock').forEach(function (clock) {
    clock.currentConfigGroup = ['Appearance'];
    clock.writeConfig('selectedTimeZones', '$time_zones'.split(','));
    clock.writeConfig('enabledCalendarPlugins', '$calendar_plugins'.split(','));
    clock.writeConfig('showWeekNumbers', true);
    clock.writeConfig('wheelChangesTimezone', true);
    clock.reloadConfig();
  });
});" || exit 1

mkdir -p "$(dirname "$marker")"
echo "$version" > "$marker"
