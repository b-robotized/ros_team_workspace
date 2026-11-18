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

# Installs the packages of the package profiles directly, for a computer
# where Landscape has not applied the profiles yet.
set -Eeuo pipefail
source /opt/rtw-provisioning/current/bin/common.bash
require_root
export DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=a
packages=()
for profile in $(required_profiles); do
  mapfile -t -O "${#packages[@]}" packages < <(profile_packages "$profile")
done
log "Install ${#packages[@]} packages: $(required_profiles | tr '\n' ' ')"
apt-get -o DPkg::Lock::Timeout=600 update
apt-get -o DPkg::Lock::Timeout=600 install -y "${packages[@]}"
require_profiles
