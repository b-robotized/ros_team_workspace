==================================
Quick start with RTW
==================================
.. _tutorial-quick-start:

This tutorial shows the use of RosTeamWorkspace (RTW) for very common use-cases that can
be done without permanent changes to your environment.

.. note::

   This tutorial assumes that RTW is already set up and sourced.
   If that is not the case, please follow the :doc:`setting_up_rtw` tutorial first.

.. toctree::
   :maxdepth: 1

Creating a new workspace (RTW)
---------------------------------------------------
For more details on options and flags check :ref:`use-case description <rtwcli-setup-workspace>`.

Local, empty workspace:

.. code-block:: bash

   rtw workspace create --ros-distro jazzy --ws-folder my_workspace

Docker, empty workspace:

.. code-block:: bash

   rtw workspace create --ros-distro jazzy --docker --ws-folder my_workspace

Local workspace from ``.repos`` file:

.. code-block:: bash

   rtw workspace create --ros-distro jazzy --repos-containing-repository-url <my_git_url> --repos-branch <my_git_branch_with_repos> --ws-folder my_workspace

Docker workspace from ``.repos`` file:

.. code-block:: bash

   rtw workspace create --ros-distro jazzy --docker --repos-containing-repository-url <my_git_url> --repos-branch <my_git_branch_with_repos> --ws-folder my_workspace

.. warning::
   When using ``.repos`` files, ``rosdep install`` may fail if the package
   references in your repositories are outdated or if the rosdep database
   is not up-to-date. Ensure your ``.repos`` files point to the correct branches
   for your ROS distro and that packages have valid ``package.xml`` files with
   correct dependencies.

.. note::
   For more advanced options such as proxy configuration, standalone workspaces,
   IPC communication setup, and other features, check the :doc:`../rtwcli/index` page.

Create new package in an existing workspace
--------------------------------------------------------
For more details, check :ref:`use-case description <uc-new-package>`.

.. code-block:: bash

   source <path to your ROS workspace>/install/setup.bash
   cd <src folder of your ROS workspace>

   create-new-package <my_new_package_name> <"Some cool description of the package.">  # follow the instructions and remember to set a license

   cd .. && colcon build --symlink-install  # to compile your newly created package


Create robot description package
-------------------------------------------------
For more details, check
:ref:`use-case description <uc-setup-robot-description>`.

.. warning::

   You must have a <my_cool_robot_description_package_name> package of
   build type **ament_cmake** to hold the robot description.

.. code-block:: bash

   source <path to your ROS workspace>/install/setup.bash
   cd <src folder of your ROS workspace>/<my_cool_robot_description_package_name>

   setup-robot-description <my_cool_robot_name>


Create robot bringup package
-----------------------------------------------
For more details, check :ref:`use-case description <uc-setup-robot-bringup>`.

.. warning::
   You must have a <my_cool_robot_bringup_package_name> package of
   build type **ament_cmake** to hold the robot bringup.

.. code-block:: bash

   source <path to your ROS workspace>/install/setup.bash
   cd <src folder of your ROS workspace>/<my_cool_robot_bringup_package_name>

   setup-robot-bringup <my_cool_robot_name> <my_cool_robot_description_package_name>


Setup  ros2_control control hardware
-------------------------------------------------
For more details, check
:ref:`use-case description <uc-setup-ros2-control-hardware>`.

.. warning::
   You must have a <my_cool_robot_control_package_name> package of
   build type **ament_cmake** to hold the robot's ros2_control
   hardware interface.

.. code-block:: bash

   source <path to your ROS workspace>/install/setup.bash
   cd <src folder of your ROS workspace>/<my_cool_robot_control_package_name>

   ros2_control_setup-hardware-interface-package <my_cool_robot_hardware> [<MyCoolRobotHW>]


Setup  ros2_control controller
-----------------------------------------------
For more details, check :ref:`use-case description <uc-setup-ros2-controller>`.

.. warning::
   You must have a <my_cool_robot_controller_package_name> package of
   build type **ament_cmake** to hold the robot's ros2_control controller.

.. code-block:: bash

   source <path to your ROS workspace>/install/setup.bash
   cd <src folder of your ROS workspace>/<my_cool_robot_controller_package_name>

   ros2_control_setup-controller-package <my_controller_file_name>
