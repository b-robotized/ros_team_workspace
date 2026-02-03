Moving Parts (to different reference Systems)
=============================================

Sometimes, you have to move part of the global coordinate system to have the correct export. This can be done simply by ``drag 'n' drop`` removing the part from the parent object (assembly). However, depending on the coordinate system of the parent object, the part might change its position. This means you should put the part into the correct global coordinate system.

**FreeCAD's Python console is very helpful for doing this fast.**

.. note::
   To show the Python console use ``View --> Panels --> Python Console``

Moving the objects in FreeCAD from the local frame to the global frame
----------------------------------------------------------------------

1. Select the object that you want to move.
2. Get the global placement of the object:

   .. code-block:: python

      obj_placement = Gui.Selection.getSelection()[0].getGlobalPlacement()

3. Move the object the top level of the document (*drag 'n' drop*).
4. Set the placement of the object:

   .. code-block:: python

      Gui.Selection.getSelection()[0].Placement = obj_placement

Moving the objects in FreeCAD from their frame to some reference frame
----------------------------------------------------------------------

.. warning::
   This is only to give you an idea of the concept - needs polishing to be ``copy-pastable``

.. note::
   Enable visualization of models' ``origin`` in FreeCAD by default:
   ``Edit --> Preferences --> Display --> 3D View (tab) --> Show Axes Cross by default``

1. Select an object/coordinate system that should be the source of the joint
2. Get the inverse of the object placement:

   .. code-block:: python

      ref_object = Gui.Selection.getSelection()[0].Placement.inverse()

3. Move the object to the origin:

   .. code-block:: python

      ref_object = ref_object.inverse() * ref_object

4. Move other objects to the coordinate system of the object:

   *first select the second object*

   ``object = ref_object.inverse() * object``

5. Get the following joint by selecting the object/coordinate system and get its placement in the global coordinate system:

   ``object.getGlobalPlacement()``

**Commands:**

.. code-block:: python

   ref_obj_global = Gui.Selection.getSelection()[0].getGlobalPlacement()
   obj = Gui.Selection.getSelection()[0]
   obj.Label
   obj.Placement = ref_obj_global.inverse() * obj.Placement
