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
   * ``edit``: Edit an existing ROS workspace configuration
   * ``use``: Select and source an existing ROS workspace
   * ``port``: Port existing RTW workspace(s)

* ``rtw docker``: Various Docker related sub-commands
   * ``enter``: Enter a Docker container for a dockerized workspace (make sure to run ``rtw ws <ws-name>`` before)

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
      * ``--enable-local-updates``: Enable system updates (``apt-get update``
        and ``rosdep update``) for local workspaces. By default, these updates
        are only run for Docker workspaces.
      * ``--env-vars``: Additional environment variables to export in the
        workspace (format: ``KEY=VALUE``). Multiple variables can be specified
        separated by spaces. These variables are exported when the workspace is
        sourced.

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

.. warning::
   When using ``.repos`` files, ``rosdep install`` may fail if the package
   references in your repositories are outdated or if the rosdep database
   is not up-to-date. In such cases:

   * Ensure your ``.repos`` files point to the correct branches for your ROS distro
   * Run ``rosdep update`` before creating the workspace
   * Check that the packages in your repositories have valid ``package.xml`` files
     with correct dependencies
   * For Docker workspaces, the rosdep database is updated automatically during
     the build process

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


How to use proxy for Docker workspaces
""""""""""""""""""""""""""""""""""""""""
.. _rtwcli-proxy-usage:

When working behind a corporate proxy, the CLI provides options to configure
proxy settings for Docker workspaces. This ensures that apt, pip, and git
can access the internet through the proxy during the Docker image build process.

* Proxy options:
   * ``--proxy-server <url>``: Proxy server URL (e.g., ``http://proxy.company.com:8080``)
   * ``--proxy-ca-cert <path>``: Path to company CA certificate file for SSL inspection proxies

* What the proxy configuration does:
   * Sets environment variables (``http_proxy``, ``https_proxy``, ``HTTP_PROXY``, ``HTTPS_PROXY``, ``PIP_PROXY``, ``no_proxy``, ``NO_PROXY``)
   * Configures apt proxy in ``/etc/apt/apt.conf.d/95proxies``
   * Configures pip proxy and trusted hosts in ``/root/.config/pip/pip.conf``
   * Configures git proxy settings
   * If CA certificate is provided:
      * Copies the certificate to ``/usr/local/share/ca-certificates/``
      * Runs ``update-ca-certificates`` to register it
      * Sets ``REQUESTS_CA_BUNDLE`` environment variable for Python requests

* Example with proxy server only:

.. code-block:: bash

   rtw workspace create \
      --ws-folder my_docker_ws \
      --ros-distro jazzy \
      --docker \
      --proxy-server http://proxy.company.com:8080

* Example with proxy server and CA certificate:

.. code-block:: bash

   rtw workspace create \
      --ws-folder my_docker_ws \
      --ros-distro jazzy \
      --docker \
      --proxy-server http://proxy.company.com:8080 \
      --proxy-ca-cert /path/to/company-ca.crt

.. note::
   The CA certificate file will be temporarily copied to the Docker build context
   during the build process and automatically removed after the build completes.

.. important::
   If your company uses SSL inspection (MITM proxy), you must provide the
   company CA certificate using ``--proxy-ca-cert`` to avoid SSL verification
   errors during package installation.


How to install rocker fork with the new features
""""""""""""""""""""""""""""""""""""""""""""""""""
.. _rtwcli-setup-rocker-fork:

Until rocker PR is merged you are encouraged to install your rocker fork with:

.. code-block:: bash

   pip3 uninstall rocker   # if you have installed it with 'sudo' use it here too
   git clone https://github.com/b-robotized-forks/rocker.git --branch <your-feature-branch>
   cd rocker && pip3 install -e . && cd -

Modifying an existing workspace
"""""""""""""""""""""""""""""""
.. _rtwcli-workspace-edit:

You can update the configuration of an existing workspace (currently only
environment variables are supported) using the ``edit`` command.

* Usage:
   * ``rtw workspace edit [workspace_name] [options]``
   * Options:
      * ``--env-vars KEY=VALUE [KEY=VALUE ...]``: Add or update environment variables.
      * ``--remove-env-vars KEY [KEY ...]``: Remove environment variables.

* Examples:

  Update environment variables for a specific workspace:

  .. code-block:: bash

     rtw workspace edit my_ws --env-vars ROS_DOMAIN_ID=28 ROS_LOCALHOST_ONLY=0

  Remove a variable:

  .. code-block:: bash

     rtw workspace edit my_ws --remove-env-vars ROS_STATIC_PEERS

  Interactive mode (if no workspace name or options are provided):

  .. code-block:: bash

     rtw workspace edit

  This will prompt you to select a workspace and then choose an action (Add/Update or Remove variables).

How to set custom environment variables
"""""""""""""""""""""""""""""""""""""""
.. _rtwcli-env-vars-usage:

You can set custom environment variables that will be automatically exported whenever
you use the workspace (both local and Docker). This is useful for variables like
``ROS_DOMAIN_ID``, ``ROS_LOCALHOST_ONLY``, or ``RMW_IMPLEMENTATION``.

* Example:

.. code-block:: bash

   rtw workspace create \
      --ws-folder my_custom_ws \
      --ros-distro jazzy \
      --env-vars ROS_DOMAIN_ID=28 ROS_LOCALHOST_ONLY=1 ROS_STATIC_PEERS=10.28.28.28

When you later run ``rtw workspace use my_custom_ws`` (or entering the Docker container),
these variables will be set.
