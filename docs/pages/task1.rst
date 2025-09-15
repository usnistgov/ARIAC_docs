Task 1: Inspection and Kit Building
====================================

Task 1 involves a complete quality control and kit assembly process consisting of five subtasks. This task focuses on ensuring battery cell quality through inspection, voltage testing, and proper kit assembly.

Environment Components
----------------------

* **Inspection Robot 1 & 2**: UR5e robots with Robotiq 2f-85 grippers
* **Inspection Conveyor**: Moving belt delivering battery cells
* **Voltage Testers**: Two stations for electrical testing
* **Inspection Bins**: Collection points for defective cells
* **Recycling Bin**: Disposal for out-of-spec cells
* **AGVs 1-3**: Automated guided vehicles for kit transport
* **Kit Trays**: Containers for assembled four-cell kits

Task 1a: Physical Inspection
---------------------------

**Objective**: Inspect each cell on the conveyor for physical defects using sensors

Process
~~~~~~~

* LiDAR sensors positioned in defined zones around inspection conveyor
* Teams reconstruct cell geometry from sensor data to identify defects
* Submit inspection report with pass/fail determination
* **Bonus points** awarded for reporting defect count, type, and location

**Outcome**: Passed reports open the inspection door; failed cells drop into bins

Task 1b: Conveyor Pickup
------------------------

**Objective**: Grasp cells passing inspection from the moving conveyor

Process
~~~~~~~

* Use two-finger gripper to pick cells from moving conveyor
* Place cells into one of two voltage testers

**Challenge**: Requires precise timing and motion planning for moving target acquisition

Task 1c: Voltage Inspection
---------------------------

**Objective**: Verify electrical specifications of cells

Process
~~~~~~~

* Each cell's voltage sampled from normal distribution around nominal value
* Two testers measure voltage and publish readings (with noise) to ROS topics
* **Tolerance**: ±0.2V from nominal voltage
* **Nominal voltages**:
  
  * Li-Ion = 3.6V
  * NiMH = 1.2V

**Outcome**: Out-of-spec cells must be dropped in recycling bin

Voltage Reading Example
~~~~~~~~~~~~~~~~~~~~~~

The voltage testers provide noisy readings that teams must filter and interpret:

.. code-block:: text

   Time (s)    Reading (V)    Actual (V)
   0.0         3.45           3.6
   0.5         3.62           3.6  
   1.0         3.58           3.6
   1.5         3.61           3.6
   2.0         3.59           3.6

Task 1d: AGV Placement
----------------------

**Objective**: Assemble four-cell kits meeting voltage specifications

Process
~~~~~~~

* Place four cells into AGV tray slots
* **Kit voltage requirement**: V_total ∈ 4 × V_cell ± 0.1V
* Ensure combined voltage is within specification

**Strategy**: Requires voltage matching across multiple cells

Voltage Calculation
~~~~~~~~~~~~~~~~~~

For a valid kit:

* Li-Ion kit: Total voltage must be within 14.4V ± 0.36V (14.04V to 14.76V)
* NiMH kit: Total voltage must be within 4.8V ± 0.12V (4.68V to 4.92V)

Task 1e: Move AGV
-----------------

**Objective**: Transport completed kits to appropriate destinations

AGV Stations
~~~~~~~~~~~~

* **Inspection**: For loading new kits
* **Assembly**: For module construction (Task 2)
* **Shipping**: For kit completion and submission
* **Recycling**: For clearing rejected trays

Process
~~~~~~~

* Send AGV to shipping station and call submit service for kit completion
* If rejected, AGV goes to recycling for tray clearing
* For module construction, send kit to assembly station

AGV Movement Flow
~~~~~~~~~~~~~~~~

.. code-block:: text

   1. Load kit at Inspection station
   2. Move to Shipping station for kit orders
      OR
      Move to Assembly station for module construction
   3. Submit order via appropriate service
   4. If accepted: AGV returns to Inspection
   5. If rejected: AGV goes to Recycling for cleanup

Task 1 Success Criteria
----------------------

Kit Acceptance Requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~

* Four good cells (not defective, within voltage specification)
* Total voltage within specification range
* Proper cell placement in AGV tray slots
* Successful submission at shipping station

Performance Metrics
~~~~~~~~~~~~~~~~~~

* **Inspection Accuracy**: Correctly identifying defective vs. good cells
* **Pickup Success Rate**: Successfully grasping cells from moving conveyor
* **Voltage Matching**: Creating kits within voltage tolerance
* **Cycle Time**: Time from cell arrival to kit completion
* **Error Recovery**: Handling sensor noise and equipment malfunctions