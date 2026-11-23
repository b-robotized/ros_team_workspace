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
require_user
require_profiles
actual_home=$(getent passwd "$(id -un)" | cut -d: -f6)
[[ "$HOME" == "$actual_home" && -d "$HOME" && -w "$HOME" ]] || { echo 'HOME must be the home of the user.' >&2; exit 2; }
[[ -f "$RTW_INSTALL_DIR/setup.bash" ]] || { echo "RTW is missing in $RTW_INSTALL_DIR. Run phase 20 first." >&2; exit 2; }
cd "$HOME"
templates="$RTW_BUNDLE_DIR/templates"
log "Configure user $(id -un) in $HOME"

# Writes a block between markers; reruns replace it. Other content stays unchanged.
managed_block() {
  python3 - "$1" "$2" "$3" <<'BLOCK'
import pathlib, re, sys
path, position, body = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3]
if path.is_symlink():
    print(f'[RTW] Skip {path}: it is a symbolic link.', file=sys.stderr)
    sys.exit(0)
text = path.read_text() if path.exists() else ''
text = re.sub(r'(?ms)^# BEGIN RTW MANAGED\n.*?^# END RTW MANAGED\n?', '', text).strip('\n')
block = f'# BEGIN RTW MANAGED\n{body.rstrip()}\n# END RTW MANAGED\n'
text = block + ('\n' + text + '\n' if text else '') if position == 'top' else (text + '\n\n' if text else '') + block
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text(text)
BLOCK
}

log 'Shell'
managed_block "$HOME/.bash_commands" end "$(cat "$templates/extend_to_bash_commands")"
managed_block "$HOME/.bashrc" end "$(cat "$templates/extend_to_bashrc")
if [ -f ~/.ros_team_ws_rc ]; then
    . ~/.ros_team_ws_rc
fi
export PATH=\"\$HOME/.local/bin:\$PATH\""
if [[ ! -f "$HOME/.ros_team_ws_rc" ]]; then
  sed -e "s|<PATH TO ros_team_workspace>|$RTW_INSTALL_DIR|g" \
      -e "s|<Path to ros_team_workspace>|$RTW_INSTALL_DIR|g" \
      "$RTW_INSTALL_DIR/templates/.ros_team_ws_rc" > "$HOME/.ros_team_ws_rc"
fi
mkdir -p "$HOME/.local/share/bash-completion/completions"
gh completion -s bash > "$HOME/.local/share/bash-completion/completions/gh"

log 'Git'
install -D -m 0644 "$templates/gitconfig" "$HOME/.config/git/rtw.gitconfig"
git_include="[include]
	path = ~/.config/git/rtw.gitconfig"
if [[ "$RTW_ROLE" != demo ]]; then
  install -D -m 0644 "$templates/gitconfig-ssh" "$HOME/.config/git/rtw-ssh.gitconfig"
  git_include="$git_include
	path = ~/.config/git/rtw-ssh.gitconfig"
fi
managed_block "$HOME/.gitconfig" top "$git_include"
managed_block "$HOME/.config/git/ignore" end "$(cat "$templates/gitignore")"

log 'Desktop'
kwriteconfig6 --file kdeglobals --group KDE --key AutomaticLookAndFeel true
rm -f "$HOME/.config/autostart/rtw-desktop-branding.desktop"

log 'VS Code extensions'
while IFS= read -r extension || [[ -n "$extension" ]]; do
  [[ -z "$extension" || "$extension" =~ ^[[:space:]]*# ]] && continue
  /snap/bin/code --install-extension "$extension"
done < "$RTW_BUNDLE_DIR/vs-code_plugins.txt"

log 'colcon mixins and rosdep'
if ! colcon mixin list | grep -Eq '^default([[:space:]]|:)'; then
  colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
fi
colcon mixin update default
rosdep update

log 'User setup complete. Open a new shell. Git identity stays personal: see ~/.gituser-b-robotized in the RTW docs.'
