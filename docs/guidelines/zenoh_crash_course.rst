.. _zenoh_crash_course:

Zenoh Crash Course
==================

In the context of ROS 2, using the Zenoh RMW (``rmw_zenoh_cpp``) simplifies connecting multiple devices across different subnets.

Overview and Concepts
------------------------

When using Zenoh with ROS 2, the network architecture is slightly different from traditional DDS:

* **Router (zenohd):** Think of this as a ros2 daemon running on your host. It handles network discovery and host-to-host communication. Typically, you run one router per host. Routers must be configured to talk to each other across the network.
* **Sessions:** Every ROS 2 node creates a Zenoh "session" that communicates through the local loopback interface (``lo``). The router then picks up this traffic and handles the host-to-host routing.

The router is required for the initial discovery phase. In the common case, even if the router process dies, the established connection between ROS 2 nodes will persist, though new nodes will not be able to discover each other until the router is restarted.

For deeper technical details, refer to the `official rmw_zenoh documentation <https://github.com/ros2/rmw_zenoh>`_.


Workspace Configuration
--------------------------
To enable Zenoh, you need to set the appropriate environment variables. RTW expects these to be defined in your workspace configuration.

The best solution is to set zenoh variables per specific workspace either at the creation with `rtw workspace create` or later using the `rtw edit` command. In both cases the flag is `--env-vars RMW_IMPLEMENTATION=rmw_zenoh_cpp export RUST_LOG=zenoh=warn,zenoh_transport=warn`.
If you want to use zenoh to connecto to a remote computer append `ZENOH_CONNECT_IP=192.168.28.28`.

If you want to use `zenoh` as default in your system open your ``~/.ros_team_ws_rc`` file and uncomment the following lines.

.. code-block:: bash

    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export RUST_LOG=zenoh=warn,zenoh_transport=warn
    # Optional: Set a default target IP to connect to automatically
    export ZENOH_CONNECT_IP=192.168.28.28

Once added you will have to `source ~/.bashrc` again or better, open new terminal and then choose the workspace using `rtw ws <ws_name>`.
 

Starting the Router
----------------------
Every machine participating in the Zenoh network needs access to a Zenoh router (``rmw_zenohd``). RTW provides the ``rtw-zenoh-router`` alias to manage this.

**Local-Only mode**
If you are developing locally and do not need to bridge to an external device, start the router without arguments in one terminal:

.. code-block:: bash

    rtw-zenoh-router

*What it does:* This launches the router in ``listen`` mode, listening to all interfaces for incoming connections, but does not attempt to connect to other routers.

**Connected Mode**
If you need to connect your local machine to an external device also running a ``zenoh`` router:

.. code-block:: bash

    rtw-zenoh-router 192.168.28.28

If you need to connect your local machine to an external device also running a ``zenoh`` router, the script will automatically connect if you either:

1. Defined ``ZENOH_CONNECT_IP`` in your workspace configuration and run ``rtw-zenoh-router`` without arguments. How to set this variable per workspace or system-wide check the explanation above.
2. Explicitly pass the IP as an argument (this **overrides** the workspace variable, setting ``ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<IP>:7447"]'``):

Verifying the Connection
---------------------------
If you want to see detailed connection logs, you can increase the ``RUST_LOG`` verbosity in your terminal before running the script:

.. code-block:: bash

    export RUST_LOG=zenoh=info,zenoh_transport=info
    rtw-zenoh-router 192.168.28.28

When using Zenoh, take care that the ``ros2`` commands are executed differently. More about this check the :ref:`ROS 2 Daemon Management <uc-aliases-ros2-daemon>` documentation.
