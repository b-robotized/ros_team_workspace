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

# Usage: make-attachment.bash <devices folder with <hostname>.conf files> <output.tar.gz>
set -Eeuo pipefail
[[ $# == 2 && -d "$1" ]] || { echo 'Usage: make-attachment.bash <devices folder> <output.tar.gz>' >&2; exit 2; }
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
rtw="$(cd -- "$here/../../.." && pwd)"
work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT
cp -r "$here/bin" "$here/profiles" "$work/"
mkdir "$work/devices"
for conf in "$1"/*.conf; do
  bash -n "$conf"
  cp "$conf" "$work/devices/"
done
[[ -n "$(ls "$work/devices")" ]] || { echo "No .conf files in $1" >&2; exit 2; }
cp "$here/../vs-code_plugins.txt" "$work/"
mkdir "$work/templates"
cp "$rtw/templates/os_configure/"{extend_to_bashrc,extend_to_bash_commands,gitconfig,gitconfig-ssh,gitignore} "$work/templates/"
tar --owner=0 --group=0 --sort=name -czf "$2" -C "$work" .
sha256sum "$2"
