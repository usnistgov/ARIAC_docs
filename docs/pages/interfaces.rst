ROS Interfaces
==============

This section provides a comprehensive reference for all ROS interfaces available in ARIAC 2025. These interfaces enable control of robots, sensors, conveyors, and other competition components.

Competition Control Interfaces
------------------------------

Core competition management services and topics for starting, stopping, and monitoring the competition state.

.. list-table:: Competition Control
   :header-rows: 1
   :widths: 30 25 45

   * - Service/Topic
     - Type
     - Description
   * - ``/start_competition``
     - ``ariac_interfaces/srv/Trigger``
     - Starts the competition
   * - ``/end_competition``
     - ``ariac_interfaces/srv/EndCompetition``
     - Ends competition, optionally shutdown Gazebo
   * - ``/competition_status``
     - ``ariac_interfaces/msg/CompetitionStatus``
     - Query competition state, time, order counts
   * - ``/high_priority_orders``
     - ``ariac_interfaces/msg/HighPriorityOrder``
     - High priority kit requests with order ID
   * - ``/submit_kitting_order``
     - ``ariac_interfaces/srv/Trigger``
     - Submit kit (AGV at shipping required)
   * - ``/submit_module_order``
     - ``ariac_interfaces/srv/Trigger``
     - Submit module (in submission zone required)
   * - ``/submit_high_priority_order``
     - ``ariac_interfaces/srv/SubmitHighPriorityOrder``
     - Submit high priority kit with ID

Competition Status Message
~~~~~~~~~~~~~~~~~~~~~~~~~

The ``CompetitionStatus`` message provides:

* Current competition state (Preparing, Ready, Started, Orders Complete, Ended)
* Elapsed competition time
* Number of kitting orders remaining
* Number of module orders remaining

Task 1 Interfaces
-----------------

Interfaces specific to inspection and kit building operations.

.. list-table:: Task 1 Services and Topics
   :header-rows: 1
   :widths: 35 25 40

   * - Service/Topic
     - Type
     - Description
   * - ``/inspection_conveyor/status``
     - ``ariac_interfaces/msg/ConveyorStatus``
     - Conveyor direction, speed, operating status
   * - ``/inspection_conveyor/cell_feed/status``
     - ``ariac_interfaces/msg/CellFeederStatus``
     - Cell type being fed and feed rate
   * - ``/inspection_conveyor/cell_feed/control``
     - ``ariac_interfaces/srv/ControlCellFeeder``
     - Change cell type being fed
   * - ``/inspection_conveyor/inspection/submit``
     - ``ariac_interfaces/srv/SubmitInspectionReport``
     - Submit inspection with pass/fail and defects
   * - ``/{voltage_tester_number}/voltage``
     - ``ariac_interfaces/msg/VoltageReading``
     - Voltage reading with noise

Conveyor Status
~~~~~~~~~~~~~~

The ``ConveyorStatus`` message includes:

* Direction: Forward (1), Backward (-1), or Stopped (0)
* Current speed in m/s
* Operating status: Operational, Malfunction, or Maintenance

Cell Feeder Control
~~~~~~~~~~~~~~~~~

Use the ``ControlCellFeeder`` service to switch between:

* Li-Ion cell feeding (normal operations)
* NiMH cell feeding (high priority orders)

Inspection Report Submission
~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``SubmitInspectionReport`` service requires:

* Pass/fail determination (boolean)
* Optional defect list with:
  
  * Defect type
  * Location coordinates
  * Confidence score

Voltage Reading
~~~~~~~~~~~~~

The ``VoltageReading`` message provides:

* Measured voltage value (with noise)
* Timestamp of measurement
* Tester status (operational/malfunction)

Task 2 Interfaces
-----------------

Interfaces for module assembly operations including conveyor control and welding.

.. list-table:: Task 2 Services and Topics
   :header-rows: 1
   :widths: 40 25 35

   * - Service/Topic
     - Type
     - Description
   * - ``/assembly_conveyor/{section_number}/control``
     - ``ariac_interfaces/srv/ConveyorControl``
     - Control individual conveyor sections
   * - ``/assembly_conveyor/{section_number}/status``
     - ``ariac_interfaces/msg/ConveyorStatus``
     - Section direction, speed, status
   * - ``/gantry_welder/weld``
     - ``ariac_interfaces/srv/Trigger``
     - Create weld if electrodes contact plate
   * - ``/insert_bottom_shell``
     - ``ariac_interfaces/srv/Trigger``
     - Spawn bottom shell on section 1
   * - ``/insert_top_shell``
     - ``ariac_interfaces/srv/Trigger``
     - Spawn top shell on assembly table

Assembly Conveyor Control
~~~~~~~~~~~~~~~~~~~~~~~

Each conveyor section (1, 2, 3) can be controlled independently:

* **Section 1**: Cell installation area
* **Section 2**: Transfer section
* **Section 3**: Shell installation and welding area

The ``ConveyorControl`` service accepts:

* Speed: Desired conveyor speed (m/s)
* Direction: Forward (1), Backward (-1), Stop (0)

Gantry Welder Operation
~~~~~~~~~~~~~~~~~~~~~

The gantry welder service returns:

* Success/failure status
* Weld quality indicator
* Error messages if applicable

**Prerequisites for welding:**

* Electrodes must be in contact with weld plate
* Module must be properly positioned
* Welder must be operational

Robot Control Interfaces
------------------------

Standard robot control interfaces for UR5e robots and their associated grippers and tools.

UR5e Robot Control
~~~~~~~~~~~~~~~~~

.. list-table:: UR5e Robot Interfaces
   :header-rows: 1
   :widths: 50 25 25

   * - Service/Topic
     - Type
     - Description
   * - ``/{robot_name}/joint_states``
     - ``sensor_msgs/msg/JointState``
     - Current joint states
   * - ``/{robot_name}/joint_trajectory_controller/follow_joint_trajectory``
     - ``control_msgs/FollowJointTrajectory``
     - Joint trajectory commands

**Robot Names:**

* ``inspection_robot_1``
* ``inspection_robot_2``  
* ``assembly_robot_1``
* ``assembly_robot_2``

Joint Trajectory Control
^^^^^^^^^^^^^^^^^^^^^^

The trajectory controller accepts:

* Joint names and target positions
* Velocity and acceleration limits
* Time constraints for execution
* Goal tolerance specifications

Robotiq 2f-85 Gripper Control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: Gripper Control Interface
   :header-rows: 1
   :widths: 50 30 20

   * - Service/Topic
     - Type
     - Description
   * - ``/{robot_name}/gripper_controller/gripper_command``
     - ``ariac_interfaces/action/GripperCommand``
     - Control gripper width

**Gripper Command Parameters:**

* Target width: 0.0 (closed) to 0.085 (fully open) meters
* Force limit: Maximum gripping force
* Speed: Opening/closing velocity

Vacuum Tool Control
~~~~~~~~~~~~~~~~~~

Assembly Robot 2 features a tool changer system for vacuum grippers:

.. list-table:: Vacuum Tool Interfaces
   :header-rows: 1
   :widths: 45 30 25

   * - Service/Topic
     - Type
     - Description
   * - ``/assembly_robot_2/tool_changer/attach_tool``
     - ``ariac_interfaces/srv/AttachTool``
     - Connect vacuum gripper to coupler
   * - ``/assembly_robot_2/tool_changer/detach_tool``
     - ``ariac_interfaces/srv/DetachTool``
     - Disconnect vacuum gripper
   * - ``/vacuum_tool/{vacuum_gripper}/grasp``
     - ``ariac_interfaces/srv/Trigger``
     - Attach gripper to contact object
   * - ``/vacuum_tool/{vacuum_gripper}/release``
     - ``ariac_interfaces/srv/Trigger``
     - Detach gripper from object

**Available Vacuum Grippers:**

* ``VG-2``: Two suction cups for shell handling
* ``VG-4``: Four suction cups for module manipulation

Tool Change Procedure
^^^^^^^^^^^^^^^^^^^

1. Move robot to tool stand position
2. Align tool coupler arrows
3. Call ``attach_tool`` service
4. Verify successful attachment
5. Remove tool from stand

AGV Control Interfaces
----------------------

Automated Guided Vehicle control for kit transport between stations.

.. list-table:: AGV Control
   :header-rows: 1
   :widths: 30 30 40

   * - Service/Topic
     - Type  
     - Description
   * - ``/{agv_name}/move``
     - ``ariac_interfaces/action/MoveAgv``
     - Move AGV to specified station
   * - ``/{agv_name}/info``
     - ``ariac_interfaces/msg/AgvStatus``
     - Current location and pose
   * - ``/{agv_name}/recycle_cells``
     - ``ariac_interfaces/srv/Trigger``
     - Recycle tray at recycling station

**Available AGVs:**

* ``agv_1``
* ``agv_2`` 
* ``agv_3``

**AGV Stations:**

* **Inspection**: Kit loading and cell placement
* **Assembly**: Module construction handoff
* **Shipping**: Kit order submission
* **Recycling**: Tray cleanup and cell disposal

AGV Movement Action
~~~~~~~~~~~~~~~~~

The ``MoveAgv`` action provides:

* Goal: Target station identifier
* Feedback: Current position and movement progress
* Result: Final position and success status

AGV Status Information
~~~~~~~~~~~~~~~~~~~~

The ``AgvStatus`` message includes:

* Current station location
* 3D pose (position and orientation)
* Movement status (moving, stopped, loading)
* Tray contents information

Interface Usage Examples
-----------------------

Starting Competition
~~~~~~~~~~~~~~~~~~

.. code-block:: python

   # Start the competition
   start_client = self.create_client(Trigger, '/start_competition')
   request = Trigger.Request()
   future = start_client.call_async(request)

Submitting Inspection Report
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   # Submit inspection with defects found
   inspection_client = self.create_client(
       SubmitInspectionReport, 
       '/inspection_conveyor/inspection/submit'
   )
   
   request = SubmitInspectionReport.Request()
   request.pass_inspection = False
   request.defects = [
       # Add detected defects with type and location
   ]
   future = inspection_client.call_async(request)

Moving AGV
~~~~~~~~~

.. code-block:: python

   # Move AGV to shipping station
   move_client = self.create_client(MoveAgv, '/agv_1/move')
   
   goal = MoveAgv.Goal()
   goal.station = 'shipping'
   future = move_client.call_async(goal)

Tool Change Operation
~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   # Attach VG-2 tool
   attach_client = self.create_client(
       AttachTool, 
       '/assembly_robot_2/tool_changer/attach_tool'
   )
   
   request = AttachTool.Request()
   request.tool_name = 'VG-2'
   future = attach_client.call_async(request)

Error Handling
-------------

Interface Error Codes
~~~~~~~~~~~~~~~~~~~

Common error responses from services:

* **SUCCESS**: Operation completed successfully
* **INVALID_REQUEST**: Malformed request parameters
* **PRECONDITION_FAILED**: Required conditions not met
* **HARDWARE_MALFUNCTION**: Equipment failure
* **TIMEOUT**: Operation exceeded time limit

Malfunction Detection
~~~~~~~~~~~~~~~~~~~

Monitor these topics for equipment status:

* Conveyor status messages for malfunction flags
* Voltage tester status for operational state
* Robot joint states for collision detection
* AGV status for movement failures

Recovery Strategies
~~~~~~~~~~~~~~~~~

* **Retry Logic**: Implement exponential backoff for transient failures
* **Alternative Paths**: Use backup equipment when primary fails
* **State Verification**: Confirm successful completion before proceeding
* **Graceful Degradation**: Adapt strategy when capabilities are reduced