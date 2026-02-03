Creating Robot Description
==========================

.. note::
   **If you are unfamiliar with FreeCAD, check its introduction first.**

   :doc:`../faq/introduction_to_freecad`

   Also check out :doc:`../faq/moving_parts_to_different_reference_system`.

Create Description files
------------------------

Follow the RTW manual: `<https://rtw.stoglrobotics.de/master/use-cases/ros_packages/setup_robot_description_package.html>`_

Preparing robot meshes - workflow
---------------------------------

This workflow describes how to take the ``.step`` or ``.stp`` files of the robot in question and produce ``visual`` and ``collision`` meshes for robot use.

We do this for the following reasons:

1. To split the robot model into parts, with one mesh per part, so they can be included in the ``URDF`` files;
2. To reduce visual mesh size and adjust origins to match joint origins;
3. To generate simplified collision meshes (less vertices per part) for faster path planning.

.. warning::
   To achieve this, we use the following workflow and tools

   **FreeCAD** - for ``.step`` file manipulation *(delete unnecessary parts, group parts, measure joint origin)*

   **MeshLab** - for ``.dae`` mesh manipulation *(simplify mesh, create convex hull for collision)*

   **Blender** - for ``.dae`` origin manipulation *(move joint origin)*

Make sure you have the .step or .stp files and open them in FreeCAD
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The robot model should be split into parts already. Below is an example of such a file opened in FreeCAD.

.. note::
   ``.stp`` files can be really messy. They can have many parts that are unnecessary for prurposes of visualization, and the parts may not be grouped by sections we intend to use.

   Make sure to remove unnecessary parts and group them to have one part per link.

.. image:: images/1.png
   :alt: How a typical .stp file might look at first

.. image:: images/2.png
   :alt: 2.png

Exporting ``.dae`` files from FreeCAD
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Sort the elements according to colors. This means that the parts of the same color, may be in the same group (object/assembly).
2. Mark the objects you want to export to COLLADA mesh format.
3. Choose ``File --> Export`` and choose ``.dae`` as file format.

   .. error::
      COLLADA export options need to be checked, especially scaling. Make sure you export in ``m`` units, as this is the unit ROS 2 expects.

Moving the mesh origin
^^^^^^^^^^^^^^^^^^^^^^

It is likely that the robot parts will have the same origin, in the base of the robot.

Before exporting each part as ``.dae`` , we ideally want the origin of each mesh to align with the origin of the joint. This is done so that no additional origin transformations have to be applied in the ``URDF``, rather the mesh files can be imported directly.

.. note::
   Sometimes, measuring the parts and adjusting origins in the ``URDF`` can be easier than shifting origins, but produces a more cluttered ``URDF``.

This part does not have a workflow that will work in all cases, as it depends on the state of the received ``.stp`` files.

.. tip::
   For moving joint origins in ``.dae`` format, Blender is the software of choice. Make sure you have simplified the meshes and determined joint origins!

Blender resources
~~~~~~~~~~~~~~~~~

- You can move object, object origin only, you can fix global or local x, y, or z axis while doing so! `Experiment some <https://www.youtube.com/watch?v=8DbqvMOo_3s>`_, these are really useful when inputting exact values for joint origins.
- You might need to set up blender for working with exact ``mm`` values, so that the scale of the objects is preserved. `Check out this video <https://www.youtube.com/watch?v=8DbqvMOo_3s>`_
- Here is a useful `video on object origin manipulation <https://www.youtube.com/watch?v=9bxOKHx2VuU>`_

Simplify visual meshes
^^^^^^^^^^^^^^^^^^^^^^

Now we open MeshLab to simplify the visual mesh. In the resulting mesh, we are aiming for several thousand vertices, depending on the complexity of the part. This is usually enough for all of the visual information to be preserved without introducing unwanted visual artifacts

Manuel for the process of Simplifying Visual Meshes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Open the file in MeshLab and reduce its size
   ``Filters --> Remeshing, Simplification and Reconstruction --> Simplification: Quadratic Edge Collapse Decimation``
2. Apply and export as ``.dae``

   .. image:: images/5.png
      :alt: simplified visual mesh

Generate collision meshes
^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
   **It is also possible for you to export** ``.stl`` **directly from FreeCAD. This will not be transformed! Therefore, always first export** ``.dae`` **and then convert to** ``.stl`` **as needed.**

.. note::
   There is also a trick on how to export transformed. Use the ``Part_Fuse`` tool from ``Part`` workbench to get all the objects into one.

For collision meshes, a low vertex count is crucial for the speed of planning algorithms. To get a mesh that is simple but encapsulates the whole part, we do the following:

1. Open the visual mesh in MeshLab and generate a Convex Hull
   ``Filters --> Remeshing, Simplification and Reconstruction --> Convex Hull``
2. Export the collision mesh as ``.stl``
3. If possible, without significant loss of details, reduce the mesh size further. ``Filters --> Remeshing, Simplification and Reconstruction --> Simplification: Quadratic Edge Collapse Decimation``
4. Apply and export as ``.stl``

   .. image:: images/4.png
      :alt: Generated convex hull for the visual mesh

**All done!** You now have both visual and collision meshes  ``.stl``  for your robot.

Coloring the visual meshes
^^^^^^^^^^^^^^^^^^^^^^^^^^

After exporting from FreeCAD and decimating the mesh in Meshlab, color information is lost along the way. We can color the mesh manually using the following workflow:

1. Import ``.dae`` mesh to FreeCAD
2. Open ``MESH WORKBENCH``

   .. note::
      Sometimes, ``.dae`` mesh is already split to basic components. In the cases of more complex shapes, we can further separate mesh by components for finer control over colors of those components.

   .. image:: images/3.png
      :alt: 3.png

3. Select desired components and merge them by color. Here you can also delete some decimation artifacts if you see they don't impact the visual.

   .. image:: images/6.png
      :alt: 6.png

4. Convert the grouped mesh components to parts using ``PART WORKBENCH``

   .. image:: images/9.png
      :alt: 9.png

   1. Set the part colors

      .. image:: images/8.png
         :alt: 8.png

      .. image:: images/7.png
         :alt: 7.png

      All done! You can now export your parts as ``.dae`` once more and use them in your package.
