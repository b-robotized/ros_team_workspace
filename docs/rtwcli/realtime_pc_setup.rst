Realtime PC Setup
====================================

This guide describes the recommended and simplest way to set up a
real-time (PREEMPT_RT) PC and prepare it for ROS 2 development using RTW.

We're using a pre-build binary on Linux Debian distribution and a docker container
with Ubuntu 24.04 with real-time permissions.

Operating System Installation
------------------------------

1. Download the newest Debian stable version.

   It is recommended to use the *complete installation image* from:
   https://www.debian.org/CD/

2. Create a bootable USB (e.g., using ``dd`` on Linux).

3. Boot from the USB and select ``Graphical Installer``.

4. Follow installation steps:

   * Be aware Debian uses a separate ``root`` user. Don't get confused with two passwords. Debian has root user that doesn't have name, and you can access it with ``su -`` command.
   * Install ``ssh`` during setup
   * Choose your preferred window manager

5. Configure networking:

   * Main network according to your company requirements
   * Set up a dedicated RT ethernet interface for your robot with a static IP.

Initial System Setup
---------------------

6. Connect via SSH from engineering PC.

7. Add your user to sudo:

.. code-block:: bash

   su -
   usermod -a -G sudo core
   exit

8. Re-login and verify sudo access.

9. Update system:

.. code-block:: bash

   sudo apt update

PREEMPT_RT Kernel Installation
-------------------------------

10. Check current kernel:

.. code-block:: bash

   uname -r

11. Install matching RT kernel:

Check the current kernel version with ``uname -r``, e.g., ``6.12.74+deb13+1-amd64``.
Then, install the exact same version of kernel with ``PREEMPT_RT`` flag

.. code-block:: bash

   sudo apt install linux-image-<version>-rt-amd64

12. Reboot:

.. code-block:: bash

   sudo reboot

13. Verify RT kernel:

.. code-block:: bash

   uname -a

   # should contain PREEMPT_RT

Realtime Permissions Setup
---------------------------

14. Install Docker (APT method):

   https://docs.docker.com/engine/install/debian/

15. Configure ``realtime`` and ``docker`` groups and configure limits:

.. code-block:: bash

   sudo usermod -aG docker $USER
   sudo addgroup realtime
   sudo usermod -aG realtime $USER

   sudo tee -a /etc/security/limits.conf <<EOF
   @realtime soft rtprio 99
   @realtime soft priority 99
   @realtime soft memlock unlimited
   @realtime hard rtprio 99
   @realtime hard priority 99
   @realtime hard memlock unlimited
   EOF

16. Logout/login and verify:

You should see ``docker`` and ``realtime`` groups in the output.

.. code-block:: bash

   groups

RTW and ROS Workspace Setup
----------------------------

17. Install RTW:

   https://rtw.b-robotized.com/master/tutorials/setting_up_rtw.html

18. Create RT-enabled workspace:

.. code-block:: bash

   rtw workspace create \
      --ws-folder <my-ws-name> \
      --ros-distro <my-distro> \
      --docker \
      --disable-nvidia \
      --repos-containing-repository-url \
         https://<repo-url>.git \
      --repos-branch master \
      --apt_packages <additional-pkg-1>  \
                    <additional-pkg-2>... \
      --enable-realtime

19. Exit container and source workspace:

Exiting the container saves the workspace configuration in RTW.

.. code-block:: bash

   rtw ws <my-ws-name>

20. Enter container:

.. code-block:: bash

   rtw docker enter

Verification and Testing
-------------------------

21. Verify RT kernel inside container:

.. code-block:: bash

   uname -a

   # should contain PREEMPT_RT
