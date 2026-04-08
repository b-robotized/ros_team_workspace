============================
Zenoh Setup and Usage
============================

Zenoh is a data-centric communication layer which ROS 2 can use for node discovery and message transport.
It is used as an alternative to the default DDS-based middleware.

First make sure that Zenoh RMW is installed using:

.. code-block:: bash

   sudo apt install ros-<ros-distro>-rmw-zenoh-cpp

Replace ``<ros-distro>`` with your ROS distro, for example ``jazzy`` or ``rolling``.

Then, to use Zenoh as the RMW implementation, set the following environment variable:

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_zenoh_cpp

.. note:: You can uncomment the above line in your ``~/.ros_team_ws_rc`` file to make it permanent.

In one terminal, start the Zenoh router using `script <https://github.com/b-robotized/ros_team_workspace/blob/master/scripts/environment/start-local-zenoh.bash>`_ with the alias:

.. code-block:: bash

   start-local-zenoh


When using Zenoh, take care that the ``ros2`` commands are executed differently. More about this check the :ref:`ROS 2 Daemon Management <uc-aliases-ros2-daemon>` documentation.
