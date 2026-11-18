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
- Users are in the groups `realtime` (rtprio 99, unlimited memlock), `docker` and `sudo`.

## Phases

| Landscape script | Runs | Needs |
|---|---|---|
| `00-stage-bundle.bash` | Copies the attached bundle to `/opt/rtw-provisioning/current` | Attachment `rtw-provisioning.tar.gz` |
| `10-repositories.bash` | Universe, multiverse, git PPA, Docker and ROS repositories | Phase 00 |
| `15-install-profile-packages.bash` | Installs the packages of the package profiles | Phase 10 |
| `20-system.bash` | Flatpak, VS Code snap, `rosdep init`, RTW in `/opt/ros_team_workspace` with `rtw` CLI, realtime limits | Phase 15 |
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

4. Copy `device.conf.example`, set role, users and flags.
5. Build the attachment:

   ```bash
   ./make-attachment.bash <device.conf> rtw-provisioning.tar.gz
   ```

## Git defaults

Phase 30 adds `templates/os_configure/gitconfig` as `~/.config/git/rtw.gitconfig`
and includes it at the top of `~/.gitconfig`. Personal settings in `~/.gitconfig`
override it. The template rewrites GitHub, GitLab and code.b-robotized.com HTTPS
URLs to SSH, so every user needs an SSH key for these servers.
Git identity stays personal: copy `templates/os_configure/gituser` to
`~/.gituser-b-robotized`.

## Not covered

- GRUB default kernel. Select the realtime kernel once by hand when the
  computer has other kernels. Phase 20 warns when it does not run.
- OS upgrades and reboots. Use separate Landscape actions.
- Flatpaks and VS Code extensions are not pinned.
