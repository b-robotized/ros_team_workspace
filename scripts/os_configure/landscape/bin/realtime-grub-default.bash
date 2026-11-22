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

# Kernel postinst hook for demo PCs: boot the newest Ubuntu realtime kernel by default.
set -euo pipefail

version=$(linux-version list | grep -- '-realtime$' | linux-version sort | tail -1)
[[ -n "$version" && -f /boot/grub/grub.cfg ]] || exit 0
entry=$(awk -v v="$version" -F"'" '
  /^submenu / { submenu = $2 }
  /^\tmenuentry / && $2 ~ ("with Linux " v "$") { print submenu ">" $2; exit }
' /boot/grub/grub.cfg)
[[ -n "$entry" ]] || { echo "No GRUB entry for $version" >&2; exit 0; }
grub-set-default "$entry"
echo "GRUB default: $entry"
