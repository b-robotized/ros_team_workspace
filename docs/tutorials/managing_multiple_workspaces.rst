=============================
Managing multiple workspaces
=============================
.. _tutorial-managing-multiple-workspaces:

Before learning how to manage multiple workspace with RTW be sure that you have set everything up as described in the :ref:`setting up RTW tutorial <tutorial-setting-up-rtw>`.

Also be sure that you have opened a new terminal after you RTW setup to be configured permanently.


Lets setup a new workspace called ``ws_rolling_ros2_control_demos`` inside a ``~/workspace`` folder.

.. code-block:: bash

   rtw workspace create \
      --ws-folder ~/workspace/ws_rolling_ros2c_demos \
      --ros-distro rolling

When asked for confirmation just press <ENTER>.
After a workspace is created, open a new terminal and execute ``rtw ws ws_rolling_ros2c_demos`` command for sourcing your newly created workspace.
Now you can start using *RTW* aliases, for example directly switch to the root folder of your workspace using ``rosd``.
Check other aliases for interaction with the workspace :ref:`here <uc-aliases>`.
Those can be used out of any folder you are in.

Let's now add a package to your workspace.

1. Make sure your workspace is sourced by executing ``rtw ws ws_rolling_ros2c_demos``.
   Then enter the source folder using ``rosds`` alias.

2. Lets clone ``ros2_control_demos`` repository for testing using:

.. code-block:: bash

   git clone git@github.com:ros-controls/ros2_control_demos.git # SSH
   # or if no ssh key is setup:
   git clone https://github.com/ros-controls/ros2_control_demos.git # HTTPS

3. Now install dependencies using *RTW* :ref:`alias <uc-aliases-dependencies>`:

.. code-block:: bash

   rosdepi  # Alias for: rosdep install -y -i -r --from-paths \$ROS_WS/src

.. note:: if ``rosdep`` command fails with a comment that binary packages can not be found by apt, try to update your rosdep index using ``rosdep update`` command or even your package index using ``sudo apt update``.

4. You can then build your workspace using the alias for colcon build:

.. code-block:: bash

   cb

Everything should now have been built successfully!

5. Next let's add another workspace

.. code-block:: bash

   rtw workspace create \
      --ws-folder ~/workspace/ws_rolling_gz_demos \
      --ros-distro rolling

6. Now repeat the above steps to add `gz_ros2_control <https://github.com/ros-controls/gz_ros2_control>`_ repository for testing and execute a demo from there.

.. note:: if ``rosdep`` commands fails with a comment on ros-rolling-ros-gz-sim not being installed successfully, maybe force the desired gazebo version with e.g. ``export GZ_VERSION=fortress``

.. important::
   Now each time you open a new terminal you can use either ``rtw ws ws_rolling_ros2c_demos`` or ``rtw ws ws_rolling_gz_demos`` to source needed workspace and use the same :ref:`aliases <uc-aliases>` without constantly thinking about exact workspace/folder you are working in.
