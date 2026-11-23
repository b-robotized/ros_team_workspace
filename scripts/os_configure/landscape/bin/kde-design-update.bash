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

# Installs the latest release of the b»robotized KDE design package when it is newer.
set -euo pipefail

release=https://code.b-robotized.com/api/v4/projects/b_public%2Finfra%2Fb_kde_design/releases/permalink/latest
deb_url=$(curl -fsSL "$release" | python3 -c '
import json, sys
print(next(l["url"] for l in json.load(sys.stdin)["assets"]["links"] if l["url"].endswith(".deb")))')
latest=$(basename "$deb_url" | sed -E 's/^b-robotized-kde-design_(.*)_all\.deb$/\1/')
installed=$(dpkg-query -W -f='${Version}' b-robotized-kde-design 2>/dev/null || true)
if [[ -n "$installed" ]] && dpkg --compare-versions "$installed" ge "$latest"; then
  exit 0
fi
work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT
curl -fsSL "$deb_url" -o "$work/b-robotized-kde-design.deb"
DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=300 -o Dpkg::Options::=--force-confnew install -y "$work/b-robotized-kde-design.deb"
echo "b-robotized-kde-design $installed -> $latest"
