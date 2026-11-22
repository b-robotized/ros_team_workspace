# RTW provisioning with Landscape

Set up Ubuntu 24.04 (noble, ROS Jazzy) and 26.04 (resolute, ROS Lyrical) computers
with Landscape SaaS: package profiles for APT packages, scripts for everything else.

## Roles

| Role | Use | Package profiles |
|---|---|---|
| `basic` | Developer computer | `br-base`, `br-ros-<distro>`, `br-graphics` |
| `office` | Developer and office computer | `br-base`, `br-ros-<distro>`, `br-graphics`, `br-office` |
| `demo` | Shared demo and development PC | `br-base`, `br-ros-<distro>`, `br-demo` |

`br-pro-tools` is optional for every role (`RTW_PRO_TOOLS=1`).
`br-ros-<distro>` is `profiles/br-ros.txt` with `DISTRO` replaced by the ROS distro.

Rules for `demo` PCs:
- Kernel: the precompiled Ubuntu realtime kernel (`linux-realtime`). No custom kernels.
- Phase 20 sets `GRUB_DEFAULT=saved` and installs the kernel hook
  `/etc/kernel/postinst.d/zzz-rtw-realtime-default`: GRUB boots the newest
  realtime kernel, also after kernel updates.
- Users are in the groups `realtime` (rtprio 99, unlimited memlock), `docker` and `sudo`.
- Shared user: no HTTPS-to-SSH rewrite in Git (`gitconfig-ssh` is not included).

## Phases

| Landscape script | Runs | Needs |
|---|---|---|
| `00-stage-bundle.bash` | Copies the attached bundle to `/opt/rtw-provisioning/current` and selects `devices/<hostname>.conf` | Attachment `rtw-provisioning.tar.gz` |
| `10-repositories.bash` | Universe, multiverse, git PPA, Docker and ROS repositories | Phase 00 |
| `15-install-profile-packages.bash` | Installs the packages of the package profiles | Phase 10 |
| `20-system.bash` | Flatpak, VS Code snap, `rosdep init`, network connection `b-controlled box`, RTW in `/opt/ros_team_workspace` with `rtw` CLI, realtime limits and GRUB default (demo), CI branding | Phase 15 |
| `30-user-settings.bash` | Per user: shell, `~/.ros_team_ws_rc`, Git defaults, VS Code extensions, colcon mixins, rosdep | Phase 20 |
| `40-user-groups.bash` | Group memberships from `device.conf` | Phase 20 |

Phases 20, 30 and 40 stop with exit code 3 and list the missing packages when a
profile package is missing.

Add the profile tags to the computer after phase 40. Landscape applies package
profiles only after the first full package sync of a new computer, which can
take a day. From then on, the profiles keep the packages installed and add new
ones from the profile lists.

All scripts run as root. Phase 30 switches to each user.
Run one phase at a time and check its activity before the next.

## Prepare a computer

1. Install Ubuntu, create the local user accounts.
2. Attach Ubuntu Pro and register the computer in Landscape.
3. Allow root scripts in `/etc/landscape/client.conf`, section `[client]`, then
   `sudo systemctl restart landscape-client`:

   ```ini
   include_manager_plugins = ScriptExecution
   script_users = root
   ```

4. Copy `device.conf.example` to `<devices folder>/<hostname>.conf`, set role,
   users and flags. One file per computer, named after its short hostname.
5. Build one attachment for all computers:

   ```bash
   ./make-attachment.bash <devices folder> rtw-provisioning.tar.gz
   ```

## Network connection "b-controlled box"

With `RTW_CTRLD_BOX_HOST` in the device configuration, phase 20 creates the
NetworkManager connection `b-controlled box`: static addresses with this host
number in `10.28.28.0/24`, `192.168.28.0/24`, `192.168.1.0/24` and
`10.50.50.0/24`, no default route. With `RTW_CTRLD_BOX_MAC`, the connection is
pinned to that network card and connects automatically; without it, the user
selects it by hand. Wi-Fi passwords are not part of the bundle.

## Desktop

With `RTW_INSTALL_BRANDING=1`, phase 20 installs the b»robotized CI package to
`/usr/share/br-ci-package-computer` and sets the login screen background.
Phase 30 turns on automatic light and dark theme (Plasma follows the day and
night times of the computer location) and adds a one-time autostart
`bin/desktop-branding.bash`: wallpaper and application menu icon at the next
login.

## Git defaults

Phase 30 adds `templates/os_configure/gitconfig` as `~/.config/git/rtw.gitconfig`
and includes it at the top of `~/.gitconfig`. Personal settings in `~/.gitconfig`
override it. For roles other than `demo`, it also includes `gitconfig-ssh`, which
rewrites GitHub, GitLab and code.b-robotized.com HTTPS URLs to SSH; these users
need an SSH key for these servers.
Git identity stays personal: copy `templates/os_configure/gituser` to
`~/.gituser-b-robotized`.

## Not covered

- OS upgrades and reboots. Use separate Landscape actions.
- Flatpaks and VS Code extensions are not pinned.
