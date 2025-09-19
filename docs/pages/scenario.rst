.. _SCENARIO:

========
Scenario
========

--------
Overview
--------

The competition simulates an EV battery production factory. The factory uses battery cells of two types: Li-Ion (lithium ion) and Ni-MH (nickel metal hydride). 

.. figure:: /_static/images/li-ion_cell.png
  :width: 50%

  A Li-Ion cell

.. figure:: /_static/images/nimh_cell.png
  :width: 50%

  A Ni-MH cell

During a competition run, teams will be required to complete a certain number of kits and/or modules as determined by the trial. 

A **kit** is a grouping of four EV battery cells placed onto a tray. 

A **module** is a unit of four EV battery shells arranged with a top and bottom shells and electrically connected with welds.

.. figure:: /_static/images/module.png
  :width: 50%

  A fully constructed module


The scenario is broken down into two main tasks: 

* Task 1: Inspection and Kit Building
* Task 2: Module Construction

------
Task 1
------

Task 1 involves inspecting the individual cells and arranging them onto a tray on one of the environments AGVs.

.. figure:: /_static/images/task1_environment.png
  :width: 100%

  Task 1 annotated environment 

This task is broken into five steps:

* Step 1a: Physical Inspection 
* Step 1b: Conveyor Pickup
* Step 1c: Voltage Inspection
* Step 1d: AGV Placement
* Step 1e: Move AGV

Step 1a: Physical Inspection 
============================

The conveyor speed and feed rate for the inspection conveyor are set by the team in their configuration file. After the competition is started, a service will become available to control the cell feed type for the inspection conveyor. Available types are in :ref:`CellTypes.msg <cell_types_msg>`. Sending NONE will disable the cell feed.

For each cell, the team should read data from lidar sensors placed around the inspection conveyor to determine whether the cell is defective. After the inspection is complete, an inspection report is sent by the team. If the cell passes inspection the inspection door will open, if not the cell will drop into the bin next to the conveyor. 

.. figure:: /_static/images/task_1a.gif
  :width: 100%

  Inspecting a cell

.. warning::

  If a non-faulty cell drops into the bin, the team will incur a penalty


Step 1b: Conveyor Pickup 
========================

After a non-faulty cell passes through the inspection door it must be picked up by inspection robot 1, a UR3e equipped with a robotiq 2f-85 gripper. The gripper is controlled using a ROS action. After the cell is grasped it is placed into one of the two available voltage testers. 

.. figure:: /_static/images/task_1b.gif
  :width: 100%

  Picking a cell from the conveyor and placing into the voltage tester

.. warning::

  If the robot collides with any object in the environment other than a cell the team will incur a penalty.


Step 1c: Voltage Inspection
===========================

Once a cell is placed in the voltage tester, the tester publishes its voltage to a ROS topic.


.. note::

  The testers publish with a small amount of noise so it is best to take readings over a short time period and average the results. 

Step 1d: AGV Placement
======================

After voltage inspection, the cell should be picked by inspection robot 2 (a UR5e with robotiq 2f-85 gripper). If the cell voltage is outside the tolerance of :math:`\pm 0.2 V` it must be placed into the recycling bin. If a cell voltage is within spec it should be placed onto a tray on one of the available AGVs. 

Completed kits must meet a specified voltage tolerance:

.. math::

  V_{total} \in 4 \cdot V_{nom_cell} \pm 0.1


.. figure:: /_static/images/task_1d.gif
  :width: 100%

  Placing cells onto an AGV tray

.. note::

  Cells lock to the tray after they are placed and cannot be moved at the inspection station. 

.. warning::

  If two robots collide with each other the collision penalty is incurred twice. 


Step 1e: Move AGV
=================

There are four possible stations for the AGVs (Inspection, Assembly, Shipping, Recycling)

To complete a kitting order the AGV with a completed tray must be moved to the shipping station. When the AGV arrives the submit service should be called, at this point the kit will be evaluated to ensure that all four cells are non-defective, within voltage spec, of the same cell type, and the total kit voltage is correct. If the kit is accepted the tray will be cleared and the AGV can be sent back to the inspection station. If the kit is not accepted the AGV must be sent to the recycling station. When the AGV is at the recycling station a service can be called to recycle the tray which clears the cells and allows the AGV to be used again. 

To complete a module order the AGV should be moved the assembly station where task 2 can begin.

.. figure:: /_static/images/task_1e.gif
  :width: 100%

  Moving AGV1 to the shipping station and submitting the kit

.. warning::

  If two AGVs collide with each other the team will incur a penalty.

------
Task 2
------

Task 2 involves constructing a module from a completed kit.

.. figure:: /_static/images/task2_environment.png
  :width: 100%

  Task 2 annotated environment

This task is broken into six steps:

* Step 2a: Cell Installation
* Step 2b: Tool Change
* Step 2c: Top Shell Installation
* Step 2d: Top Welds
* Step 2e: Module Flip
* Step 2f: Bottom Welds

Step 2a: Cell Installation
==========================

When an AGV arrives at the assembly station, module construction can begin. The first step is to insert a new bottom shell into the environment using a service call. The shell will always spawn in the same place. 

Cells should be picked using assembly robot 1 (a UR5e equipped with a robotiq 2f-85 gripper) and placed into each of the four slots in the bottom shell. The cells must be properly oriented. The central hole in the AGV tray can optionally be used to perform a re-grasp of the cell.

.. figure:: /_static/images/shell_interior.png
  :width: 100%

  Shells are marked with + or - to indicate cell polarity direction 

.. figure:: /_static/images/task_2a.gif
  :width: 100%

  Installing cells into bottom shell

.. note::

  The central hole in the AGV tray will automatically center the cell when the cell contacts the bottom of the hole to make alignment easier. 

.. note::

  Cells lock to bottom shell when placed.


Step 2b: Tool Change
====================

Assembly robot 2 is equipped with a coupler that allows it to utilize one of two vacuum gripper tools (VG2 or VG4). To change tools the coupler should be moved to the location of the desired tool and a ROS service will lock the tool to the robot. For the next step assembly robot 2 must attach VG2 which is capable of picking up a top shell. The tool must then be maneuvered away from the tool stand without colliding. 

.. figure:: /_static/images/task_2b.gif
  :width: 100%

  Performing tool change operation

.. note::

  The arrows on the tool coupler and tool must be aligned in order to successfully pick up a tool. 


Step 2c: Top Shell Installation
===============================

Unlike the inspection conveyor which is constantly moving, the assembly conveyor is broken up into three sections that can be moved independently. Section 1 and 2 can be moved forward and section 3 can be moved forward and backwards. After all four cells are installed to the bottom shell the team should use the section 1 and section 2 conveyors to move the partially completed module until it reaches section 3. 

The next step is to insert a new top shell into the environment using a service call. The shell will spawn at a random location and orientation on the assembly table. Teams must use sensor(s) to determine its location before picking. Assembly robot 2 must then pick up the top shell from the assembly table and place it onto the partial module. 

.. figure:: /_static/images/task_2c.gif
  :width: 100%

  Installing top shell onto partial module

.. note::

  Both suction cups must be in contact with the top shell in order to grasp it.

.. note::

  The black and white bands on the top and bottom shell should be aligned.  

Step 2d: Top Welds
==================

With the top shell in place, teams must perform four welds to electrically connect the cells. The module should be moved to underneath the gantry welder using the section 3 conveyor. The gantry welder is a 3-DOF gantry with a set of welder electrodes at the tool. These electrodes must be in contact with the weld plate embedded in the shell before a proper weld can be performed through a service call. 

.. figure:: /_static/images/task_2d.gif
  :width: 100%

  Performing top welding operations

Step 2e: Module Flip
====================

After completing the top welds, the entire module must be flipped in order to provide access to perform the bottom welds. Assembly robot 2 must perform another tool change operation to return VG2 to the tool stand and pick up VG4. 

The module should be moved back to assembly robot 2 using the section 3 conveyor and grasped from the side. It should then be flipped and returned to the conveyor belt.  

.. figure:: /_static/images/task_2e.gif
  :width: 100%

  Flipping module to access bottom connections

.. note::

  Two of the suction cups must be in contact with the top shell and the other two must be in contact with the bottom shell in order to grasp the module. 

.. warning::

  If the module or any components of the module drop on the floor or any other invalid surface the team will incur a penalty


Step 2f: Bottom Welds
=====================

The final step involves performing welding operations on the bottom connections to complete the electrical circuit of the module. This completes the module construction process. The module can then be moved to the end of the conveyor to the submission area and the submit module service can be called. 

.. figure:: /_static/images/task_2f.gif
  :width: 100%

  Performing bottom welding operations

.. note::

  The module will be removed from the environment
