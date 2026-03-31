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

import argparse

import rich
from rich.table import Table
from rtwcli import logger
from rtwcli.constants import WORKSPACES_PATH
from rtwcli.verb import VerbExtension
from rtwcli.workspace_utils import load_workspaces_config_from_yaml_file


class ListVerb(VerbExtension):
    """List all known ROS workspaces."""

    def add_arguments(self, parser: argparse.ArgumentParser, cli_name: str):
        pass

    def main(self, *, args):
        workspaces_config = load_workspaces_config_from_yaml_file(WORKSPACES_PATH)
        if not workspaces_config.workspaces:
            logger.info(f"No workspaces found in config file '{WORKSPACES_PATH}'")
            return

        table = Table(title="Workspaces")
        table.add_column("Name", style="bold cyan")
        table.add_column("Distro")
        table.add_column("Folder")
        table.add_column("Docker", justify="center")

        for ws in workspaces_config.workspaces.values():
            table.add_row(
                ws.ws_name,
                ws.distro,
                ws.ws_folder or "-",
                "yes" if ws.ws_docker_support else "no",
            )

        rich.print(table)
