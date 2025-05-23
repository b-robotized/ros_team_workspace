==============
RTW CLI
==============

RTW has a command line interface (CLI) that provides a set of commands to
manage ROS workspaces incl. Docker workspaces. The CLI is designed to have a
more user-friendly overview of the available RTW commands and to provide a more
intuitive way to interact with RTW.

How to install the CLI
""""""""""""""""""""""""
.. _rtwcli-setup:

Follow the instructions in the ``README.md`` inside the ``rtwcli`` folder
`#here <https://github.com/b-robotized/ros_team_workspace/blob/master/rtwcli/README.md>`_.


How to use the CLI
""""""""""""""""""""
.. _rtwcli-usage:

The CLI currently supports the following commands:

* ``rtw workspace``: Various workspace related sub-commands
   * ``create``: Create a new ROS workspace (local or dockerized)
   * ``use``: Select and source an existing ROS workspace
   * ``port``: Port existing RTW workspace(s)

* ``rtw docker``: Various Docker related sub-commands
   * ``enter``: Enter a Docker container for a dockerized workspace

* ``rtw ws``: Alias for ``rtw workspace use``

.. note::
   For more detailed usage of each command or sub-command, run
   ``rtw <command> -h`` or ``rtw <command> <sub-command> -h``.


Setting up a new workspace
""""""""""""""""""""""""""""
.. _rtwcli-setup-workspace:

PR `#169 <https://github.com/b-robotized/ros_team_workspace/pull/169>`_
introduced a new feature to create a new local or dockerized workspace.
The workspace can additionally be created using ``.repos`` files in your
repository, streamlining the setup process for complex projects with multiple
repositories.

* Usage:
   * ``rtw workspace create``
     ``--ws-folder <path_to_workspace>``
     ``--ros-distro <distribution>``
     ``[options]``
   * Main ``[options]``:
      * ``--docker``: Create workspace(s) in a docker environment
      * ``--repos-containing-repository-url <url>``: URL of the repository
        containing the ``.repos`` files
        * Main ws repos format: ``{repo_name}.{ros_distro}.repos``
        * Upstream ws repos format: ``{repo_name}.{ros_distro}.upstream.repos``
      * ``--repos-branch <branch>``: Branch of the repository containing the
        ``.repos`` files

* Minimal example:

.. code-block:: bash

   rtw workspace create \
      --ws-folder dummy_minimal_ws \
      --ros-distro jazzy
..

   * This command will create an empty local workspace named ``dummy_minial_ws``
     with ROS distribution ``jazzy``.

* Example:

.. code-block:: bash

   rtw workspace create \
      --ws-folder dummy_docker_ws \
      --ros-distro jazzy \
      --docker \
      --repos-containing-repository-url \
         https://github.com/b-robotized/br_dummy_packages.git \
      --repos-branch dummy_demo_pkg
..

   * This command will create a new dockerized workspace named ``dummy_ws``
     with ROS distribution ``jazzy`` using the ``.repos`` files from the
     repository ``br_dummy_packages`` on branch ``dummy_demo_pkg``.

.. important::
   If you don't have nvidia graphics card or you don't want to use nvidia capabilits
   in the container add ``--disable-nvidia`` flag to the command.


* Example of a ``standalone`` workspace and ``robot`` user:

.. code-block:: bash

   rtw workspace create \
      --ws-folder dummy_docker_standalone_ws \
      --ros-distro jazzy \
      --docker \
      --repos-containing-repository-url \
         https://github.com/b-robotized/br_dummy_packages.git \
      --repos-branch dummy_demo_pkg \
      --standalone \
      --user-override-name robot
..

   * This command will create a new dockerized standalone workspace named
     ``dummy_ws`` with ROS distribution ``jazzy`` using the
     ``.repos`` files from the repository ``br_dummy_packages`` on branch
     ``dummy_demo_pkg``.

     However, for exporting the workspace docker image, the commit command must
     be executed first:

     .. code-block:: bash

         docker commit rtw_dummy_ws_final-instance rtw_dummy_ws_export

     When importing the workspace docker image, the following command must be
     executed:

     .. code-block:: bash

         rtw workspace import \
            --ws-name dummy_import_ws \
            --ros-distro jazzy \
            --standalone-docker-image rtw_dummy_ws_export \
            --user-override-name robot

     The ``--user-override-name`` flag is necessary to create the user with
     the same name as the one used in the exported workspace.

.. important::
   After PC restart, the ``.xauth`` cookie file will be removed. Therefore,
   before attaching VSCode, execute ``rtw ws <ws-name>`` and
   ``rtw docker enter`` to create the necessary ``.xauth`` cookie file.

.. note::
   After creating a new dockerized workspace, the rocker will start interactive
   bash session in the container.

   Only after exiting the container, the
   corresponding workspace config will be saved.

   This is done due to the fact that the setting up of the rocker container
   fails often.


How to setup ROS2 RTW for inter communication
"""""""""""""""""""""""""""""""""""""""""""""""
.. _rtwcli-ipc-usage:

The CLI provides a way to setup ROS2 RTW for inter communication between RTW
workspaces.

* Example:

.. code-block:: bash

   rtw workspace create \
      --ws-folder humble_ws \
      --ros-distro humble \
      --docker \
      --enable-ipc

   rtw workspace create \
      --ws-folder rolling_ws \
      --ros-distro rolling \
      --docker \
      --enable-ipc

   (humble_ws)$ ros2 run demo_nodes_cpp talker

   (rolling_ws)$ ros2 run demo_nodes_cpp listener


How to install rocker fork with the new features
""""""""""""""""""""""""""""""""""""""""""""""""""
.. _rtwcli-setup-rocker-fork:

Until rocker PR is merged you are encouraged to install your rocker fork with:

.. code-block:: bash

   pip3 uninstall rocker   # if you have installed it with 'sudo' use it here too
   git clone https://github.com/StoglRobotics-forks/rocker.git --branch <your-feature-branch>
   cd rocker && pip3 install -e . && cd -
