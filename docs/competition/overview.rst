.. _COMPETITION_OVERVIEW:

====================
Competition Overview
====================

For this year's ARIAC competition, competitors are tasked with managing a manufacturing environment for building battery modules for electric vehicles. 

A battery module is comprised of four battery cells that are assembled with a bottom and top shell. The cells are electrically connected through the use of metal
plates embedded in the shells. The competitor must use the gantry mounted welding tool to weld the cells to the metal plates.

-----------
Environment
-----------

The environment for this years competition models a factory setting, with multiple conveyors, AGVs, and robots to aid in the task of creating these modules. A snapshot of the
environment is included here:

.. <insert imnage overview of entire competition>

**Robots**

    .. container::

        * Inspection Robot 1 (UR3e with robotiq 2-finger gripper)
        * Inspection Robot 1 (UR5e with robotiq 2-finger gripper)
        * Assembly Robot 1 (UR5e with robotiq 2-finger gripper)
        * Assembly Robot 2 (UR5e with interchangeable vacuum tools)
        * Three Automated Guided Vehicles (AGV) with part trays capable of holding four battery cells
        * 2-DOF Gantry Mounted Welding Tool

**Conveyors**

    .. container::

        * Inspection Conveyor
            * Moves at speed set by competitor prior to trial run
            * Cells are fed onto conveyor at a rate set by competitor prior to trial run
            * Has an inspection door that must be opened by the competitor to allow good cells to pass
        * Assembly Conveyor
            *  Section 1: Module bottom shell is fixed to conveyor, allowing competitors to assemble four cells from an AGV. After all cells are affixed, an advance service is called send the partially assembled module to the next section
            *  Section 2: This section of the conveyor is fully user controllable allowing speed and direction to be set by the competitor at any time.

**Other**

    .. container::

        * Voltage Testers
        * Inspection sensors
        * Bins
        * Top Shell Table
        * Vacuum tool changer

------
Stages
------

**Physical Inspection**

Before placing the cells into a kit cells must first undergo a physical inspection. Cells are feed onto the inspection conveyor at a rate specified by the conveyor
and move at a set conveyor speed also specified by the competitor. Competitors must place sensors in the inspection regions to analyze the cells as they travel
along the conveyor. Cells can have a variety of defects such as dents. bulges, and deep scratches. An example of a defective cell can be seen below. Please note that
this is not the only type of defect that will be present.

.. container:: image-row

    .. image:: ../images/good_cell.png
        :class: image-item
        :width: 100
        :alt: An example of a non-defective cell

    .. image:: ../images/bad_cell.png
        :class: image-item
        :width: 105
        :alt: An example of a defective cell

After a cell has been analyzed by the competitor, they must submit an inspection report. The minimum inspection report must include whether the cell has passed 
inspection or not. A cell passes inspection only if it has no defects. Competitors can also earn bonus points by properly identifying the type and location of the
defect. When the inspection report is submitted it will open a door on the conveyor allowing it to pass into the next region where it can be picked by the inspection 
robot 1. If the cell does not pass inspection the door will remain shut and the cell will fall into the defective parts bin. If a non-faulty cell falls into this bin
a penalty will be assessed.

**Voltage Testing**

As the non-defective cell progresses along the inspection conveyor, it must be picked up by a ur5e robot and placed within one of two voltage testers. These voltage testers will 
output the voltage of the cells with a small amount of sensor noise present. Competitors should note the voltage for a cell placed in the sensor If the cell is out of spec (provided in 
the documentation) then the cell should be recycled. 

.. <insert image of a cell in voltage tester -> post insertion of new parts>

**Kitting on AGVs**

After a cell is inspected and the voltage is tested it can be picked from the voltage tester and placed onto one of the three available AGV trays using inspection robot 2. For a module kit 
the total combined voltage of the cells must be within the provided specification (provided in the documentation). Once a cell is placed onto a tray it cannot be moved or removed while at the 
inspection station. Competitors must build a kit that meets the specified combined module voltage. 

.. <insert image of robot placing cell into AGV -> post insertion of new parts>

**Moving the AGV**

The AGVs can be moved along pre-determined paths to three locations: 

    * Shipping - Used to turn in kit orders, both regular and high-priority. This location can be accessed from the inspection station only.
    * Assembly - Used as the next step in the assembly of modules. This location can be accessed from the inspection station only.
    * Recycling - Used to recycle any cells that have been placed into the AGV for any reason. This location can be accessed from the inspection station, as well as the shipping location

.. <insert overview image showing all AGV locations labeled>

**Kit Submission**

After placing all four cells on the tray, competitors should move the AGV to the shipping station and submit the order with the order ID. If the module voltage is correct, 
there are no defective cells, and no out of spec cells the submission will be accepted and the tray will be cleared. If the submission is not accepted, competitors must move 
the AGV to the recycling station and call the recycling service to clear the tray for future use.

**Cell Assembly**

The first step of assembly is placing each cell into the module bottom shell. A bottom shell will be automatically fed onto the first section of the assembly conveyor 
at the start of the competition and a new shell will be fed when a partial module is advanced to the next section of the conveyor.

Competitors must use assembly robot 1 to pick each cell from the AGV tray and assemble it to a slot on the bottom shell. To allow proper final voltage the orientation 
of the cell must be alternated. Slot 1 should have a cell installed up, slot 2 should be inverted, slot three should be up, and slot four should be inverted. 

After all four cells are inserted into the bottom shell the competitor can call the advance service to move the partial module to the next section of the conveyor. 

.. <insert image showing proper orientation of assembly -> post insertion of new parts>

**Top Shell Assembly**

The next step in assembly is to place the top shell onto partially assembled module. Competitors must use a camera system to locate a top shell from the table next to 
assembly robot 2. The top shells will be placed in a random orientation and competitors should use + and - symbols located on the part to determine the correct orientation. 

Assembly robot 2 will be equipped with a vacuum end effector that can attach interchangeable vacuum tools. The competitors must use the tool changer station to attach tool_1 
to the robot in order to properly pick a top shell. 

After the correct tool is attached and the pose of the top shell is determined competitors should pick the shell and assemble it to the partial module. Competitors can move 
the module using the conveyor as needed. 

.. <insert image showing proper orientation of top shell -> post insertion of new parts>

**Welding**

The last step in assembly is spot welding the cells to the embedded metal plates in the top and bottom shells. Competitors should use the conveyor to move the module within range 
of the gantry mounted welding tool. Next, the tool should be moved so that it is in contact with one of the weld spots on the metal plate. At that point the weld tool should be activated 
for a specified time to complete the spot weld. Then the tool should be moved to the next weld location. The top case requires a total of four spot welds to complete the circuit. 

After the top shell is properly welded to the cells the competitors should move the module back into reach of assembly robot 2 in order to flip the module. Assembly robot 2 must attach 
tool_2 which is capable of lifting an entire module from the side. After grasping the module, it should be flipped and placed back onto the conveyor. From there the bottom shell must be 
welded in the same way as the top shell. There are two spot welds needed for the bottom shell.

.. <insert image showing the weld locations and a weld in progress, maybe also a weld bead appeared -> post insertion of new parts>

**Module Submission**

Once both sets of welds have been completed, the module can be submitted by moving it forward to the submission box at the end of the conveyor. Competitors should then call the submission 
service to submit the module. This submission box will check that all of the following criteria have been met, and if all criteria has been met, will record a successful comletion and award 
appropriate points:

    * No defective cells are included in the module
    * Each individual cell meet minimum voltage specifications
    * All cells together meet the module volage specification
    * All cells are properly oriented in the bottom shell
    * The top shell is properly oriented
    * All welds are present on both the top and bottom of the module

If the submission is not accepted, the module will be discarded and a new module must be built from scratch.
