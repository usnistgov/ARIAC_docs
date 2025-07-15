.. _COMPETITION_OVERVIEW:

====================
Competition Overview
====================

This year's ARIAC competition will challenge competitors to fully assembly mock battery modules and kits. Competitors will need to go through the full process of creating battery 
modules, from identifying any defective battery cells and disposing of them, to using UR robots to assembly kits on AGV's, to creating the modules through an assembly task, to 
finally welding the modules once fully assembled.

-----------
Environment
-----------

The environment for this years competition models a factory setting, with multiple conveyors, AGVs, and robots to aid in the task of creating these modules. A snapshot of the
environment is included here:

.. <insert imnage overview of entire competition>

------
Stages
------

**Physical Inspection**

The first stage that competitors will be tasked with handling is the physical inspection of cells. Competitors will need to set an inspection conveyor speed which will be static
through the run. Cells will be fed at regular intervals based on this conveyor speed. Competitors will also need to place LiDAR sensors in specified areas around the inspection 
conveyor to allow them to identify defective cells. An example of a defective cell can be seen below. Please note that this is not the only type of defect that will be present.

.. container:: image-row

    .. image:: ../_static/good_cell.png
        :class: image-item
        :width: 100
        :alt: An example of a non-defective cell

    .. image:: ../_static/bad_cell.png
        :class: image-item
        :width: 105
        :alt: An example of a defective cell

After identifying whether the cell is defective or not, competitors will need to handle disposing of defective cells and allowing non-defective cells progress to the next stage.

**Voltage Inspection**

As the non-defective cell progresses along the inspection conveyor, it must be picked up by a ur5e robot and placed within one of two voltage testers. These voltage testers will 
output the voltage of the cells with a small amount of noise present. This noise should be filtered to determine the voltage of the cell. After the voltage of the cell has been
determined, it will be handled in the next stage.

.. <insert image of a cell in voltage tester>

**Kitting on AGVs**

In this stage, four cells will be assembled into kits on AGVs. It is important that these kits stay within specification as a whole, as well as ensuring that each cell is also
within specification individually. We have provided a recyling bin which can be used with no penalty to competitors to recycle cells that are either out of specification, or 
cannot be utilized in the current AGV layout. Once an AGV has been assembled with four cells, it can be used to transport the cells to one of three locations.

.. <insert image of robot placing cell into AGV>

**Moving the AGV**

The AGVs can be moved along pre-determined paths to three locations: 

    * Shipping - Used to turn in kit orders, both regular and high-priority. This location can be accessed from the inspection station only.
    * Assembly - Used as the next step in the assembly of modules. This location can be accessed from the inspection station only.
    * Recycling - Used to recycle any cells that have been placed into the AGV for any reason. This location can be accessed from the inspection station, as well as the shipping location

.. <insert overview image showing all AGV locations labeled>

**Cell Assembly**

Once an AGV with four cells arrives at the assembly location, these cells should be assembled into the bottom modules present on the assembly conveyor. Special attention should be 
made to properly orient the cells into the bottom module. An example of the bottom moduele assembled with the correct orientation is shown below:

.. <insert image showing proper orientation of assembly>

**Top Shell Assembly**

After assembly all four cells into the bottom module, the conveyor should progress, moving the assembled bottom module forward to the final assembly robot. This robot handles assembling
the top shell using one of two vacuum grippers. This assembly gripper allows the top shell to be grabbed from the top and assembled onto the the bottom module assembly. Special 
attention should be made to properly orient the top shell. An example of a properly assembled top shell is shown below:

.. <insert image showing proper orientation of top shell>

**Welding**

After assembling the top shell, the module assembly should be sent to the welding station for the first set of welds. The welding robot will need to be controlled to touch each 
weld location on the shell. These can be seen as holes giving access to the batteries. Once the weld has been completed, a weld bead will appear signaling that this has been 
completed. After completing the first set of welds, the module will need to be sent back to the previous assembly robot to flip the module.

.. <insert image showing the weld locations and a weld in progress, maybe also a weld bead appeared>

**Tool Changing**

In order to flip the module, the assembly robot much undergo a tool change. The robot will need to disengage the tool used to assemble the top shell and engage the tool needed to 
flip the module. The vacuum gripper should then be placed on the side of the module allowing for it to be flipped and giving access to the other necessary welds. The module should 
then be sent back to the welding robot for the final set of welds.

.. <insert image of module flip in progress>

**Module Submission**

Once both sets of welds have been completed, the module can be submitted by moving it forward to the submission box at the end of the conveyor. This submission box will check that 
all of the following criteria have been met, and if all criteria has been met, will record a successful comletion and award appropriate points:

    * No defective cells are included in the module
    * Each individual cell meet minimum voltage specifications
    * All cells together meet the module volage specification
    * All cells are properly oriented in the bottom shell
    * The top shell is properly oriented
    * All welds are present on both the top and bottom of the module
