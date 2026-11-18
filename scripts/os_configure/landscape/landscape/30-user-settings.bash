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

# Runs phase 30 as each target user with the home of that user.
set -Eeuo pipefail
source /opt/rtw-provisioning/current/bin/common.bash
require_root
validate_accounts "${RTW_TARGET_USERS[@]}"
for account in "${RTW_TARGET_USERS[@]}"; do
  runuser -u "$account" -- env -i HOME="$(getent passwd "$account" | cut -d: -f6)" \
    USER="$account" LOGNAME="$account" LANG=C.UTF-8 \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin \
    bash /opt/rtw-provisioning/current/bin/30-user.bash
done
