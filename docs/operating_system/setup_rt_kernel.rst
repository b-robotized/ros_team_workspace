.. _uc-setup-rt-kernel:

Setup Real-Time Kernel
======================

This script automates the process of downloading, patching, compiling, and installing a Linux kernel with the PREEMPT_RT patch. This is essential for achieving real-time performance for robot control.

Usage
-----

Execute the script using the alias:

.. code-block:: bash

   setup-rt-kernel

What the script does
--------------------

1.  **Version Detection**:
    *   Checks your current running kernel version.
    *   Fetches available RT kernel versions from `kernel.org`.
    *   Checks for matching `linux-headers` packages in your Ubuntu repositories (important for DKMS modules like NVIDIA).

2.  **Selection**:
    *   Presents a list of compatible RT kernel versions.
    *   Highlights versions that have matching headers available (recommended).

3.  **Download and Verification**:
    *   Downloads the kernel source tarball and the RT patch.
    *   Downloads GPG signatures and verifies the authenticity of the downloaded files.

4.  **Configuration and Compilation**:
    *   Extracts the source and applies the RT patch.
    *   Configures the kernel using the current system's configuration as a base (`make defconfig`).
    *   Enables `CONFIG_PREEMPT_RT` and disables conflicting keys.
    *   Compiles the kernel (this step can take a significant amount of time, depending on your CPU).

5.  **Installation**:
    *   Installs the kernel headers and image (`make install`, `make modules_install`).
    *   Updates the GRUB bootloader.

6.  **NVIDIA Driver Check**:
    *   Detects if an NVIDIA GPU is present.
    *   Checks if the NVIDIA driver is installed.
    *   Attempts to build/install the NVIDIA kernel modules for the new RT kernel using DKMS.

Post-Installation
-----------------

After the script finishes successfully:

1.  **Reboot** your machine.
2.  In the GRUB menu (hold Shift or Esc during boot if it doesn't appear), select "Advanced options for Ubuntu".
3.  Select the new kernel version (it will have ``rt`` in the name).
4.  Verify the kernel is running with:

    .. code-block:: bash

       uname -r

    It should output something like `X.Y.Z-rtXX`.

5.  Verify PREEMPT_RT is active:

    .. code-block:: bash

       grep "CONFIG_PREEMPT_RT" /boot/config-$(uname -r)

    It should return `CONFIG_PREEMPT_RT=y`.

Notes
-----

*   **Compilation Time**: Compiling a Linux kernel takes time (from 15 minutes to over an hour depending on your hardware). Ensure you have a stable power supply (laptop plugged in).
*   **Disk Space**: Ensure you have at least 20-30 GB of free disk space for the build artifacts.
*   **Secure Boot**: If Secure Boot is enabled in your BIOS, the unsigned custom kernel might not boot. You may need to disable Secure Boot or sign the kernel manually.

References
----------

For more detailed information on Real-Time kernels in the context of robotics, check out these useful manuals:

*   `Universal Robots ROS2 Documentation: Real-time kernel setup <https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/real_time.html>`_
*   `Franka Robotics Documentation: Real-Time Kernel <https://frankarobotics.github.io/docs/libfranka/docs/real_time_kernel.html?highlight=realtime>`_
