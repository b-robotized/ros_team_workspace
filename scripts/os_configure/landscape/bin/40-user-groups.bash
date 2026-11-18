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
require_profiles
validate_accounts "${RTW_TARGET_USERS[@]}"

groups=()
if [[ "$RTW_GRANT_SUDO" == 1 ]]; then groups+=(sudo); fi
if [[ "$RTW_GRANT_DOCKER" == 1 ]]; then groups+=(docker); fi
if [[ "$RTW_ROLE" == demo ]]; then groups+=(realtime); fi
if [[ "$RTW_ROLE" == office ]] && getent group tracing >/dev/null; then groups+=(tracing); fi

for account in "${RTW_TARGET_USERS[@]}"; do
  for group in "${groups[@]}"; do
    getent group "$group" >/dev/null || { echo "Group $group is missing. Run phase 20 first." >&2; exit 2; }
    usermod -aG "$group" "$account"
  done
  log "$account: $(id -nG "$account")"
done
log 'Groups set. Users must log out and in again.'
