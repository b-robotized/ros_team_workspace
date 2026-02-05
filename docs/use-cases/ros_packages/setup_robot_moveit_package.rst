.. _uc-setup-moveit-package:

Setup Robot MoveIt Package
==========================

This script facilitates the setup of a ROS 2 package for robot motion planning using MoveIt 2.
It generates the necessary configuration files, launch files, and directory structure to use MoveIt with your robot.

Prerequisites
-------------

*   An existing ROS 2 package (create one using ``create-new-package`` if needed).
*   A robot description package (created with ``setup-robot-description``).
*   A robot control/bringup package (created with ``setup-robot-bringup``).

Usage
-----

Execute the script from the root of your target package:

.. code-block:: bash

   setup-robot-moveit CELL_NAME CONTROL_PKG_NAME CONTROL_LAUNCH_FILE DESCRIPTION_PKG_NAME

Arguments
~~~~~~~~~

*   ``CELL_NAME``: The name of the robot, cell, or scenario (e.g., ``myrobot``). This is used for naming files and parameters.
*   ``CONTROL_PKG_NAME``: The name of the package containing the robot's control launch files.
*   ``CONTROL_LAUNCH_FILE``: The name of the launch file (without extension) in the control package that brings up the robot hardware/simulation (e.g., ``start_offline``).
*   ``DESCRIPTION_PKG_NAME``: The name of the package containing the robot description (URDF/XACRO).

Interactive Steps
-----------------

1.  **Launch File Type Selection**: Choose whether to generate XML, Python, or both types of launch files.
2.  **Confirmation**: Review the configuration summary and press Enter to proceed.
3.  **SRDF Generation**: Choose whether to generate a template SRDF file in this package. If "no" (default), it assumes the SRDF is provided by the description package.

Generated Files and Structure
-----------------------------

The script creates the following structure in your package:

.. code-block:: text

   $PKG_NAME$/
   ├── config/
   │   └── moveit/
   │       ├── move_group_config.yaml         # Move group node configuration
   │       ├── moveit_controllers.yaml        # Controller manager configuration
   │       ├── joint_limits.yaml              # Joint limits overriding URDF
   │       ├── kinematics.yaml                # Kinematics solver configuration
   │       ├── sensors_3d.yaml                # Perception configuration
   │       └── <planner>_planning.yaml        # Planner configurations (OMPL, Pilz, CHOMP, STOMP)
   ├── launch/
   │   └── moveit.launch.xml                  # Main MoveIt launch file
   ├── srdf/                                  # (Optional, if selected)
   │   ├── $CELL_NAME$_macro.srdf.xacro       # Semantic robot description macro
   │   └── $CELL_NAME$.srdf.xacro             # Semantic robot description
   └── rviz/
       ├── moveit.rviz                        # RViZ config with MotionPlanning widget
       └── moveit_config.rviz                 # RViZ config for MoveIt Setup Assistant

Updates to Existing Files
-------------------------

The script automatically updates:

*   **package.xml**: Adds dependencies on ``moveit_ros_move_group``, ``moveit_kinematics``, ``moveit_planners``, and ``moveit_simple_controller_manager``.
*   **CMakeLists.txt**: Adds install rules for the new directories (`config`, `launch`, `rviz`, and optionally `srdf`).
*   **README.md**: Appends instructions on how to use the generated MoveIt configuration.

Post-Generation Steps
---------------------

1.  **Configure SRDF**: If you generated the SRDF templates, edit ``srdf/$CELL_NAME$.srdf.xacro`` to define your robot's groups, end-effectors, and virtual joints.
2.  **Configure Controllers**: Verify ``config/moveit/moveit_controllers.yaml`` matches your robot's controller configuration.
3.  **Configure Joint Limits**: Adjust `config/moveit/joint_limits.yaml` if needed.
4.  **Build**:
    .. code-block:: bash

       cb <your_package_name>

5.  **Run**:
    First, launch the robot bringup (simulation or real hardware):

    .. code-block:: bash

       ros2 launch <control_pkg> <control_launch_file>.launch.xml

    Then, launch MoveIt:

    .. code-block:: bash

       ros2 launch <your_package_name> moveit.launch.xml
