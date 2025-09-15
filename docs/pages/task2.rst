Task 2: Module Construction
===========================

Task 2 involves assembling four-cell kits into complete battery modules through six subtasks. This task focuses on precision assembly, tool management, and welding operations.

Environment Components
----------------------

* **Assembly Robot 1 & 2**: UR5e robots for manipulation tasks
* **Assembly Table**: Work surface for shell storage
* **Tool Stand**: Storage for VG-2 and VG-4 vacuum grippers
* **Assembly Conveyor**: Three-section conveyor system
* **Gantry Welder**: Automated welding system for electrical connections
* **Module Submission Area**: Final delivery zone

Task 2a: Cell Install
---------------------

**Objective**: Install battery cells from kit into module shell

Process
~~~~~~~

1. Insert bottom shell onto assembly conveyor section 1 via ROS service
2. Pick each cell from the AGV tray and install in correct orientation
3. **Optional**: Use center hole in AGV tray for regrasp operations
4. Cells lock in place after proper insertion

**Requirements**: Correct cell orientation and secure placement

Cell Installation Pattern
~~~~~~~~~~~~~~~~~~~~~~~~~

Cells must be installed in alternating orientations:

.. code-block:: text

   Bottom Shell Layout:
   +---+---+---+---+
   | + | - | + | - |  <- Cell orientations
   | 1 | 2 | 3 | 4 |  <- Cell positions
   +---+---+---+---+

Task 2b: Tool Change
--------------------

**Objective**: Attach appropriate vacuum gripper for shell handling

Process
~~~~~~~

1. Move Assembly Robot 2 to tool stand
2. Pick up VG-2 (vacuum gripper with two suction cups)
3. Align arrow on robot tool coupler with tool arrow
4. Attach tool with a service call
5. Remove tool from stand

**Tool Types**: 

* VG-2: Two suction cups for shell handling
* VG-4: Four suction cups for module manipulation

Tool Alignment
~~~~~~~~~~~~~

Critical alignment requirements:

* Robot tool coupler arrow must align with tool arrow
* Proper electrical connection for vacuum control
* Secure mechanical attachment before removing from stand

Task 2c: Top Shell Install
--------------------------

**Objective**: Complete module casing with top shell

Process
~~~~~~~

1. Move partial assembly to conveyor section 3 using ROS services
2. **Note**: Module teleports slightly when touching section 3 (physics workaround)
3. Insert top shell into environment via service
4. Pick top shell from assembly table using VG-2
5. Install on module (locks in place)

**Precision**: Requires accurate placement for proper fit

Conveyor Section Control
~~~~~~~~~~~~~~~~~~~~~~~

The three-section conveyor system requires coordinated control:

.. code-block:: text

   Section 1: Cell installation area
   Section 2: Transfer section  
   Section 3: Shell installation and welding area

Task 2d: Top Welds
------------------

**Objective**: Create electrical connections on module top

Process
~~~~~~~

1. Move module under gantry welder using conveyor
2. Position gantry above hole in top shell
3. Lower welder until electrodes contact weld plate
4. Complete weld via service call
5. **Repeat for all four top welds**

**Quality**: Proper electrode contact essential for weld integrity

Welding Sequence
~~~~~~~~~~~~~~~

Top shell welding pattern:

.. code-block:: text

   Top Shell Weld Locations:
   +---+---+---+---+
   | W |   | W |   |  <- W = Weld locations
   | 1 |   | 3 |   |
   +---+---+---+---+
   |   | W |   | W |
   |   | 2 |   | 4 |
   +---+---+---+---+

Weld Process Steps:

1. Position module under gantry
2. Move gantry to weld location
3. Lower electrodes to contact weld plate
4. Execute weld service call
5. Raise electrodes and move to next location

Task 2e: Module Flip
--------------------

**Objective**: Reorient module for bottom welding access

Process
~~~~~~~

1. Perform tool change: detach VG-2, attach VG-4
2. Move module within reach of Assembly Robot 2 using conveyor
3. Grasp module and perform flip operation
4. Place module back on conveyor

**Coordination**: Requires precise manipulation and conveyor control

Module Flip Mechanics
~~~~~~~~~~~~~~~~~~~~~

The flip operation requires careful coordination:

* VG-4 provides secure grip with four suction points
* 180-degree rotation about horizontal axis
* Gentle placement to avoid damage
* Verification of proper orientation after flip

Task 2f: Bottom Welds
---------------------

**Objective**: Complete electrical connections on module bottom

Process
~~~~~~~

1. Position module under gantry welder
2. Complete two required welds on bottom shell (same process as top welds)
3. Move completed module to submission area
4. Module automatically removed from environment upon submission

**Completion**: Marks end of module construction process

Bottom Weld Pattern
~~~~~~~~~~~~~~~~~~

Bottom shell requires only two welds:

.. code-block:: text

   Bottom Shell Weld Locations:
   +---+---+---+---+
   | W |   |   | W |  <- W = Required welds
   | 1 |   |   | 2 |
   +---+---+---+---+
   |   |   |   |   |
   |   |   |   |   |
   +---+---+---+---+

Module Construction Workflow
----------------------------

Complete Task 2 Sequence
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: text

   1. Cell Install
      ├── Insert bottom shell (service call)
      ├── Pick cells from AGV tray
      ├── Install cells in correct orientation  
      └── Verify cell locking

   2. Tool Change (VG-2)
      ├── Move robot to tool stand
      ├── Align coupler arrows
      ├── Attach tool (service call)
      └── Remove from stand

   3. Top Shell Install
      ├── Move assembly to section 3
      ├── Insert top shell (service call)
      ├── Pick shell from table
      └── Install on module

   4. Top Welds (4 welds)
      ├── Position under gantry
      ├── Align electrodes with plate
      ├── Execute weld (service call)
      └── Repeat for all 4 locations

   5. Module Flip
      ├── Tool change (VG-2 → VG-4)  
      ├── Grasp module with VG-4
      ├── Perform 180° flip
      └── Place back on conveyor

   6. Bottom Welds (2 welds)
      ├── Position under gantry
      ├── Complete 2 bottom welds
      ├── Move to submission area
      └── Module removed automatically

Task 2 Success Criteria
-----------------------

Module Acceptance Requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Four good cells with voltage in specification
* Bottom and top shells properly installed
* All six welds completed successfully (4 top + 2 bottom)
* Module delivered to submission area

Quality Checkpoints
~~~~~~~~~~~~~~~~~

* **Cell Installation**: Proper orientation and secure locking
* **Shell Placement**: Correct alignment and fit
* **Weld Quality**: Proper electrode contact and electrical connection
* **Tool Management**: Successful tool changes and vacuum control
* **Conveyor Coordination**: Smooth movement between sections

Performance Metrics
~~~~~~~~~~~~~~~~~

* **Assembly Time**: Total time from kit arrival to module submission
* **Weld Success Rate**: Percentage of successful weld operations
* **Tool Change Efficiency**: Time for tool attachment/detachment
* **Error Recovery**: Handling failed grasps or misaligned components
* **System Coordination**: Multi-robot and conveyor synchronization