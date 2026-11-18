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

# Copies the attached bundle to /opt/rtw-provisioning/releases/<sha256> and points "current" to it.
set -Eeuo pipefail
[[ $EUID == 0 ]] || { echo 'Run as root.' >&2; exit 2; }
: "${LANDSCAPE_ATTACHMENTS:?Attach rtw-provisioning.tar.gz to this script.}"
archive="$LANDSCAPE_ATTACHMENTS/rtw-provisioning.tar.gz"
[[ -f "$archive" ]] || { echo "Missing attachment: $archive" >&2; exit 2; }
release=$(sha256sum "$archive" | cut -d' ' -f1)
base=/opt/rtw-provisioning
install -d -m 0755 "$base/releases"
staging=$(mktemp -d "$base/releases/.staging.XXXXXX")
trap 'rm -rf -- "$staging"' EXIT
python3 - "$archive" "$staging" <<'EXTRACT'
import pathlib, sys, tarfile
with tarfile.open(sys.argv[1], 'r:gz') as archive:
    for member in archive.getmembers():
        p = pathlib.PurePosixPath(member.name)
        if p.is_absolute() or '..' in p.parts or not (member.isfile() or member.isdir()):
            raise ValueError(f'Unsafe archive member: {member.name}')
    archive.extractall(sys.argv[2], filter='data')
EXTRACT
[[ -f "$staging/bin/common.bash" && -f "$staging/device.conf" ]] || { echo 'Incomplete bundle.' >&2; exit 2; }
chown -R root:root "$staging"
chmod -R u=rwX,go=rX "$staging"
if [[ ! -d "$base/releases/$release" ]]; then mv "$staging" "$base/releases/$release"; fi
ln -sfn "$base/releases/$release" "$base/current"
echo "Staged bundle $release:"
cat "$base/current/device.conf"
