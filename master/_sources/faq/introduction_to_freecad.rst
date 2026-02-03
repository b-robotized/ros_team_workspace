Introduction to FreeCAD
=======================

Installation
------------

First, make sure you have version 0.21 or higher. Versions lower than this import the .stp file as one entity, which is not what we want.

1. Debian packages are usually behind, so go to https://www.freecad.org/downloads.php and download the AppImage.
2. ``chmod +x FreeCAD-0.21.2-Linux-x86_64.AppImage``
3. ``./FreeCAD-0.21.2-Linux-x86_64.AppImage``

Install Useful Add-ons
^^^^^^^^^^^^^^^^^^^^^^

FreeCad provides some valuable add-ons. You can install them via the *Addon Manager*. (``Tools>Addon manager``).  You should install the following Add-ons:

- **Manipulator Workbench**
  Open *Add-on Manager* and select *Workbenches.* Then, search for the Manipulator Workbench and install it.

.. image:: images/install_addons_4.png
   :alt: install_addons (4).png

Viewing an object
-----------------

If you get lost by "zooming" around the object or if the object is not directly visible after opening a file. You can easily fit the screen by clicking on the small cube under the "view" cube in the right upper corner of the object view and selecting *Fit all* (or using the shortcut, by pressing first ``V`` and immediately after ``F``).

.. image:: images/slika.png
   :alt: Adjusting view to fit all

Navigation
----------

You can select different navigation styles by *right click> Navigation styles.* In my opinion, the **OpenInvetor** style is the most intuitive for navigating around. In this style you have to press *shift + left click* to select lines, planes or objects.

.. note::
   On the status bar (in the lower-right corner), you can also choose and switch fast between different Navigation styles as it suits you.

Orientation
-----------

If you select the isometric view

1. First, select start
2. Press isometric view

.. image:: images/freecad_isometric_view.png
   :alt: freecad_isometric_view.png

And the robot is not showing as expected, you can transform the model as following:

First set the navigation style to blender as this allows to rotate specific angles like 90°.

1. Select the model as shown
2. Right click and select transform
3. A coordinate system should pop up where you can rotate and translate.
4. Accept by pressing on ok in the task tab (right next to model in step one)

.. image:: images/transform_2.png
   :alt: transform (2).png

Measurements
------------

There are different measurement tools for different measurement tasks. There are tools for linear measurement and circular.

Linear (closest point) measurements
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Can be used to measure distance between two parallel lines/planes or the like.

1. Select the part workbench
2. The toolbar should show some measure tape symbols. The most left one in the picture is for linear measurements. The next one for measuring angles.

.. image:: images/linear_meassurments.png
   :alt: linear measurement tool

Lines-Line, Line-Plane, Plane-Plane:

1. Select the measure tape symbol.
2. In the task tab a selection and control box will pop up. You can then select a line/plane by clicking on it (shift + left click if you are in OpenInventor navigation style)
3. The selection box shows you if your selection was successful
4. Select first line/edge
5. Select second line/edge
6. The measurement will pop up, showing you the distances
7. With the control box you can deselect/delete/clear measurements

.. image:: images/linear-meassurment_3.png
   :alt: linear measurement

Circles
^^^^^^^

.. note::
   The **Manipulator** workbench add-on has to be installed.

1. Select the Manipulator workbench.
2. Select the caliper
3. The measure toolbox will open:

   a. The caliper symbol has to be clicked to take measurements

   b. This shows if your selections were successful

   c. You can select different meassurtypes. e.g. circle center, arcs, circles

   The selected one (2 blue lines and blue circle with dot) can be used to measure the distance between circle-circle or circle-line/plane

.. image:: images/circular_measurments.png
   :alt: circular_measurments.png

Radius:

First select the Manipulator workbench.

1. Select the Caliper Tool (press on the caliper symbol at the top)
2. This should open a small rectangular sub-window with different options like: measuring an ark or circle, get length of an edge, get angle...
3. Press on the *Get radius or arc or circle* option
4. Select an edge of a circular shape by pressing ``ctrl+<left mouse>``
5. Repeat for second point
6. The calculated middle point and radius should now be visible in the screen

   .. image:: images/meassure_circle_drawio_3.png
      :alt: How to measure a circular shape

Circle-Line, Circle-Plane, Circle-Circle:

.. image:: images/circlular_measurment.png
   :alt: circle circle measurement
