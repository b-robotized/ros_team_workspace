# Copyright (c) 2023-2026, b»robotized group
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import grp
from typing import List, Union

from rtwcli import logger
from rtwcli.constants import DISPLAY_MANAGER_WAYLAND, OS_ID_DEBIAN
from rtwcli.utils import get_display_manager, get_os_id, run_command


def generate_rocker_flags(
    disable_nvidia: bool,
    container_name: str,
    hostname: str,
    enable_ipc: bool,
    ssh_abs_path: str,
    ssh_abs_path_in_docker: str,
    final_image_name: str,
    ws_volumes: Union[List[str], None] = None,
    user_override_name: Union[str, None] = None,
    env_file: Union[str, None] = None,
    devices: Union[List[str], None] = None,
    enable_input: bool = False,
    enable_realtime: bool = False,
    use_groups: Union[List[str], None] = None,
) -> List[str]:
    # rocker flags have order, see rocker --help
    rocker_flags = ["--nocache", "--nocleanup", "--git"]

    rocker_flags.extend(["-e", "QT_QPA_PLATFORM=xcb"])
    if get_os_id() == OS_ID_DEBIAN:
        rocker_flags.extend(["-e", "QT_X11_NO_MITSHM=1"])
        rocker_flags.extend(["-e", "LIBGL_ALWAYS_SOFTWARE=1"])
        rocker_flags.extend(["--device", "/dev/dri"])
    if not disable_nvidia:
        rocker_flags.extend(["-e", "__GLX_VENDOR_LIBRARY_NAME=nvidia"])
        rocker_flags.extend(["-e", "__NV_PRIME_RENDER_OFFLOAD=1"])
    if get_display_manager() == DISPLAY_MANAGER_WAYLAND:
        waylad_display = os.environ.get("WAYLAND_DISPLAY", None)
        if not waylad_display:
            raise RuntimeError("WAYLAND_DISPLAY is not set.")
        rocker_flags.extend(["-e", "XDG_RUNTIME_DIR=/tmp"])
        rocker_flags.extend(["-e", f"WAYLAND_DISPLAY={waylad_display}"])
        rocker_flags.extend(
            [
                "--volume",
                f"{os.environ['XDG_RUNTIME_DIR']}/{waylad_display}:/tmp/{waylad_display}",
            ]
        )

    rocker_flags.extend(["--hostname", hostname])
    rocker_flags.extend(["--name", container_name])
    rocker_flags.extend(["--network", "host"])

    if enable_ipc:
        rocker_flags.extend(["--ipc", "host"])

    if not disable_nvidia:
        rocker_flags.extend(["--nvidia", "gpus"])

    rocker_flags.extend(["--user", "--user-preserve-home"])
    if user_override_name:
        rocker_flags.extend(["--user-override-name", user_override_name])

    if env_file:
        rocker_flags.extend(["--env-file", env_file])

    # rocker volumes
    rocker_flags.append("--volume")
    rocker_flags.append(f"{ssh_abs_path}:{ssh_abs_path_in_docker}:ro")
    if ws_volumes:
        rocker_flags.extend(ws_volumes)

    rocker_flags.append("--rtw-tmpfs")
    rocker_flags.append("--rtw-update")
    rocker_flags.append("--x11")

    # --- DEVICE MOUNTS ---
    if devices:
        for device in devices:
            rocker_flags.extend(["--device", device])

    # --- GROUPS ---
    groups_to_add = use_groups.copy() if use_groups else []

    # --- INPUT HARDWARE BINDINGS ---
    # off-your-rocker rocker extension, to add docker run args rocker does not support out of the box.
    oyr_run_args = []

    if enable_input:
        groups_to_add.append("input")
        rocker_flags.extend(["--volume", "/dev/input:/dev/input"])
        # In Linux hardware management, devices are categorized by a "Major" number.
        # The Major number 13 is permanently assigned by the Linux kernel to the entire Input Subsystem.
        # This 13 is a static kernel constant.
        # We use it to grant the container access to all joysticks, mice, and keyboards at once.
        # read this: https://docs.docker.com/reference/cli/docker/container/run/#device-cgroup-rule
        oyr_run_args.append("--device-cgroup-rule='c 13:* rmw'")

    if enable_realtime:
        groups_to_add.append("realtime")
        rocker_flags.extend(
            [
                "--ulimit",
                "rtprio=99",
                "--ulimit",
                "memlock=-1",
            ]
        )
        oyr_run_args.append("--cap-add=sys_nice")

    # Resolve and add GIDs for all requested groups
    for group_name in set(groups_to_add):
        try:
            gid = grp.getgrnam(group_name).gr_gid
            rocker_flags.extend(["--group-add", str(gid)])
            logger.info(f"SUCCESS: Added '{group_name}' group (GID: {gid}) to rocker flags.")
        except KeyError:
            # Abort if the user requested a group they don't actually have on the host system
            raise RuntimeError(
                f"Requested group '{group_name}' does not exist on the host system. Please create it or remove the flag."
            )

    if oyr_run_args:
        raw_docker_args = " ".join(oyr_run_args)
        rocker_flags.extend(["--oyr-run-arg", raw_docker_args])

    rocker_flags.extend(["--mode", "interactive"])
    rocker_flags.extend(["--image-name", f"{final_image_name}"])

    return rocker_flags


def execute_rocker_cmd(rocker_flags: List[str], rocker_base_image_name: str) -> bool:
    rocker_cmd = ["rocker"] + rocker_flags + [rocker_base_image_name]
    rocker_cmd_str = " ".join(rocker_cmd)
    logger.info(f"Executing rocker command '{rocker_cmd_str}'")
    return run_command(rocker_cmd)
