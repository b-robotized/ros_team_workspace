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
import questionary
from rtwcli import logger
from rtwcli.constants import WORKSPACES_PATH
from rtwcli.verb import VerbExtension
from rtwcli.workspace_utils import (
    load_workspaces_config_from_yaml_file,
    save_workspaces_config,
    workspace_name_completer,
)


def add_rtw_workspace_edit_args(parser: argparse.ArgumentParser):
    arg = parser.add_argument(
        "workspace_name",
        help="The workspace name",
        nargs="?",
    )
    arg.completer = workspace_name_completer  # type: ignore
    parser.add_argument(
        "--env-vars",
        nargs="*",
        help="Environment variables to add or update (format: KEY=VALUE)",
        default=[],
    )
    parser.add_argument(
        "--remove-env-vars",
        nargs="*",
        help="Environment variable keys to remove",
        default=[],
    )


class EditVerb(VerbExtension):
    """Edit an existing ROS workspace configuration."""

    def add_arguments(self, parser: argparse.ArgumentParser, cli_name: str):
        add_rtw_workspace_edit_args(parser)

    def main(self, *, args):
        workspaces_config = load_workspaces_config_from_yaml_file(WORKSPACES_PATH)
        if not workspaces_config.workspaces:
            logger.info(f"No workspaces found in config file '{WORKSPACES_PATH}'")
            return

        ws_names = workspaces_config.get_ws_names()
        ws_name = args.workspace_name

        # Interactive selection if workspace name is missing
        if not ws_name:
            ws_name = questionary.autocomplete(
                "Choose workspace to edit",
                ws_names,
                qmark="'Tab' to see all workspaces, type to filter, 'Enter' to select\n",
                meta_information=workspaces_config.ws_meta_information,
                validate=lambda ws_choice: ws_choice in ws_names,
                style=questionary.Style([("answer", "bg:ansiwhite")]),
                match_middle=True,
            ).ask()
            if not ws_name:  # Cancelled by user
                return

        workspace = workspaces_config.workspaces.get(ws_name)
        if not workspace:
            logger.error(f"Workspace '{ws_name}' not found.")
            return

        # Ensure env_vars dict exists
        if not hasattr(workspace, "env_vars") or workspace.env_vars is None:
            workspace.env_vars = {}

        updates_made = False

        # Add/Update env vars
        if args.env_vars:
            for env_var in args.env_vars:
                if "=" not in env_var:
                    logger.warning(
                        f"Skipping invalid environment variable format: '{env_var}'. "
                        "Expected format KEY=VALUE"
                    )
                    continue
                key, value = env_var.split("=", 1)
                workspace.env_vars[key] = value
                logger.info(f"Set environment variable '{key}' = '{value}'")
                updates_made = True

        # Remove env vars
        if args.remove_env_vars:
            for key in args.remove_env_vars:
                if key in workspace.env_vars:
                    del workspace.env_vars[key]
                    logger.info(f"Removed environment variable '{key}'")
                    updates_made = True
                else:
                    logger.warning(f"Environment variable '{key}' not found in workspace config.")

        # Interactive mode if no flags provided
        if not args.env_vars and not args.remove_env_vars:
            logger.info(f"Current environment variables for '{ws_name}': {workspace.env_vars}")
            action = questionary.select(
                "What would you like to do?",
                choices=[
                    "Add/Update environment variable",
                    "Remove environment variable",
                    "Exit",
                ],
            ).ask()

            if action == "Add/Update environment variable":
                key = questionary.text("Enter variable name (KEY):").ask()
                if key:
                    value = questionary.text(f"Enter value for {key}:").ask()
                    if value is not None:
                        workspace.env_vars[key] = value
                        logger.info(f"Set environment variable '{key}' = '{value}'")
                        updates_made = True

            elif action == "Remove environment variable":
                if not workspace.env_vars:
                    logger.info("No environment variables to remove.")
                else:
                    keys_to_remove = questionary.checkbox(
                        "Select variables to remove:", choices=list(workspace.env_vars.keys())
                    ).ask()
                    if keys_to_remove:
                        for key in keys_to_remove:
                            del workspace.env_vars[key]
                            logger.info(f"Removed environment variable '{key}'")
                        updates_made = True

        if updates_made:
            # We use update_workspaces_config indirectly by saving the whole config
            # But the utility functions are designed to update single workspace entry logic
            # Here we just save the modified config object back to file
            # But we should backup first, similar to update_workspaces_config logic
            # However, update_workspaces_config takes a workspace object and reloads config
            # Let's reuse the save logic directly since we have the full config object
            import shutil
            import datetime
            from rtwcli.constants import WORKSPACES_PATH_BACKUP_FORMAT, BACKUP_DATETIME_FORMAT
            from rtwcli.utils import create_file_if_not_exists

            # Backup logic (duplicated from workspace_utils to be safe or we could refactor)
            current_date = datetime.datetime.now().strftime(BACKUP_DATETIME_FORMAT)
            backup_filename = WORKSPACES_PATH_BACKUP_FORMAT.format(current_date)
            create_file_if_not_exists(backup_filename)
            shutil.copy(WORKSPACES_PATH, backup_filename)
            logger.info(f"Backed up current workspaces config file to '{backup_filename}'")

            if save_workspaces_config(WORKSPACES_PATH, workspaces_config):
                logger.info(f"Successfully updated workspace '{ws_name}' configuration.")
            else:
                logger.error("Failed to save workspace configuration.")
        else:
            logger.info("No changes made to workspace configuration.")
