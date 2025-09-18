.. _API:

=======
ROS API
=======

**Interface Types:**

* :topic-name:`Topics` - Data publication/subscription interfaces
* :service-name:`Services` - Request/response interfaces
* :action-name:`Actions` - Long-running goal-based interfaces

Enumeration Messages
--------------------

.. list-table:: Message Enumerations
   :header-rows: 1
   :widths: 40 60
   :class: api-table

   * - Message
     - Description
   * - :ref:`ariac_interfaces/msg/AgvStations <agvstations_msg>`
     - AGV station location constants
   * - :ref:`ariac_interfaces/msg/CellTypes <cell_types_msg>`
     - Cell type definitions and voltage constants
   * - :ref:`ariac_interfaces/msg/CompetitionStates <competitionstates_msg>`
     - Competition state enumeration values
   * - :ref:`ariac_interfaces/msg/OperationStates <operationstates_msg>`
     - Operation state definitions
   * - :ref:`ariac_interfaces/msg/VacuumTools <vacuumtools_msg>`
     - Vacuum gripper tool type definitions

Data Structure Messages
-----------------------

.. list-table:: Complex Data Structures
   :header-rows: 1
   :widths: 40 60
   :class: api-table

   * - Message
     - Description
   * - :ref:`ariac_interfaces/msg/CellDefect <celldefect_msg>`
     - Cell defect information structure
   * - :ref:`ariac_interfaces/msg/CompetitionTime <competitiontime_msg>`
     - Competition timing data structure
   * - :ref:`ariac_interfaces/msg/InspectionReport <inspectionreport_msg>`
     - Inspection result data structure

Competition Control Interfaces
------------------------------

.. list-table:: Competition Control
   :header-rows: 1
   :widths: 35 65
   :class: api-table

   * - Interface
     - Description
   * - :topic-name:`/competition_status`

       :ref:`ariac_interfaces/msg/CompetitionStatus <competitionstatus_msg>`
     - Query competition state, time, order counts
   * - :topic-name:`/high_priority_orders`

       :ref:`ariac_interfaces/msg/HighPriorityOrder <highpriorityorder_msg>`
     - High priority kit requests with order ID
   * - :service-name:`/start_competition`

       :ref:`ariac_interfaces/srv/Trigger <trigger_srv>`
     - Starts the competition
   * - :service-name:`/end_competition`

       :ref:`ariac_interfaces/srv/EndCompetition <endcompetition_srv>`
     - Ends competition, optionally shutdown Gazebo
   * - :service-name:`/submit_kitting_order`

       :ref:`ariac_interfaces/srv/Trigger <trigger_srv>`
     - Submit kit (AGV at shipping required)
   * - :service-name:`/submit_module_order`

       :ref:`ariac_interfaces/srv/Trigger <trigger_srv>`
     - Submit module (in submission zone required)
   * - :service-name:`/submit_high_priority_order`

       :ref:`ariac_interfaces/srv/SubmitHighPriorityOrder <submithighpriorityorder_srv>`
     - Submit high priority kit with ID

Task 1 Interfaces
-----------------

.. list-table:: Task 1 Services and Topics
   :header-rows: 1
   :widths: 35 65
   :class: api-table

   * - Interface
     - Description
   * - :topic-name:`/inspection_conveyor/status`

       :ref:`ariac_interfaces/msg/ConveyorStatus <conveyorstatus_msg>`
     - Conveyor direction, speed, operating status
   * - :topic-name:`/inspection_conveyor/cell_feed/status`

       :ref:`ariac_interfaces/msg/CellFeederStatus <cellfeederstatus_msg>`
     - Current cell type being fed and feed rate
   * - :topic-name:`/{voltage_tester_number}/voltage`

       :ref:`ariac_interfaces/msg/VoltageReading <voltagereading_msg>`
     - Voltage reading with noise
   * - :service-name:`/inspection_conveyor/cell_feed/control`

       :ref:`ariac_interfaces/srv/ControlCellFeeder <controlcellfeeder_srv>`
     - Change cell type being fed
   * - :service-name:`/inspection_conveyor/inspection/submit`

       :ref:`ariac_interfaces/srv/SubmitInspectionReport <submitinspectionreport_srv>`
     - Submit inspection with pass/fail and defects

Task 2 Interfaces
-----------------

.. list-table:: Task 2 Services and Topics
   :header-rows: 1
   :widths: 35 65
   :class: api-table

   * - Interface
     - Description
   * - :topic-name:`/assembly_conveyor/{section_number}/status`

       :ref:`ariac_interfaces/msg/ConveyorStatus <conveyorstatus_msg>`
     - Section direction, speed, status
   * - :service-name:`/assembly_conveyor/section_1/control`

       :ref:`ariac_interfaces/srv/ConveyorControl <conveyorcontrol_srv>`
     - Control conveyor section 1
   * - :service-name:`/assembly_conveyor/section_2/control`

       :ref:`ariac_interfaces/srv/ConveyorControl <conveyorcontrol_srv>`
     - Control conveyor section 2
   * - :service-name:`/assembly_conveyor/section_3/control`

       :ref:`ariac_interfaces/srv/BidirectionalConveyorControl <bidirectionalconveyorcontrol_srv>`
     - Control bidirectional conveyor section 3
   * - :service-name:`/gantry_welder/weld`

       :ref:`ariac_interfaces/srv/Trigger <trigger_srv>`
     - Create weld if electrodes contact plate
   * - :service-name:`/insert_bottom_shell`

       :ref:`ariac_interfaces/srv/Trigger <trigger_srv>`
     - Spawn bottom shell on section 1
   * - :service-name:`/insert_top_shell`

       :ref:`ariac_interfaces/srv/Trigger <trigger_srv>`
     - Spawn top shell on assembly table

Robot Control Interfaces
------------------------

.. list-table:: Robot Interfaces
   :header-rows: 1
   :widths: 35 65
   :class: api-table

   * - Interface
     - Description
   * - :topic-name:`/{robot_name}/joint_states`

       `sensor_msgs/msg/JointState <https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/JointState.html>`_
     - Current joint states
   * - :action-name:`/{robot_name}/joint_trajectory_controller/follow_joint_trajectory`

       `control_msgs/action/FollowJointTrajectory <https://docs.ros.org/en/jazzy/p/control_msgs/action/FollowJointTrajectory.html>`_
     - Joint trajectory commands

.. list-table:: Gripper Interface
   :header-rows: 1
   :widths: 35 65
   :class: api-table

   * - Interface
     - Description
   * - :action-name:`/{robot_name}/gripper_controller/gripper_command`

       :ref:`ariac_interfaces/action/GripperCommand <grippercommand_action>`
     - Control gripper width

.. list-table:: Vacuum Tool Interfaces
   :header-rows: 1
   :widths: 35 65
   :class: api-table

   * - Interface
     - Description
   * - :topic-name:`/assembly_robot_2/tool_changer/status`

       :ref:`ariac_interfaces/msg/ToolChangerStatus <toolchangerstatus_msg>`
     - Tool changer current state and status
   * - :service-name:`/assembly_robot_2/tool_changer/attach_tool`

       :ref:`ariac_interfaces/srv/AttachTool <attachtool_srv>`
     - Connect vacuum gripper to coupler
   * - :service-name:`/assembly_robot_2/tool_changer/detach_tool`

       :ref:`ariac_interfaces/srv/Trigger <trigger_srv>`
     - Disconnect vacuum gripper
   * - :service-name:`/vacuum_tool/{vacuum_gripper}/grasp`

       :ref:`ariac_interfaces/srv/Trigger <trigger_srv>`
     - Attach gripper to contact object
   * - :service-name:`/vacuum_tool/{vacuum_gripper}/release`

       :ref:`ariac_interfaces/srv/Trigger <trigger_srv>`
     - Detach gripper from object

AGV Control Interfaces
----------------------

.. list-table:: AGV Control
   :header-rows: 1
   :widths: 35 65
   :class: api-table

   * - Interface
     - Description
   * - :topic-name:`/{agv_name}/info`

       :ref:`ariac_interfaces/msg/AgvStatus <agvstatus_msg>`
     - Current location and pose
   * - :topic-name:`/{agv_name}/tray_status`

       :ref:`ariac_interfaces/msg/AgvTrayStatus <agvtraystatus_msg>`
     - Tray occupancy and part information
   * - :service-name:`/{agv_name}/recycle_cells`

       :ref:`ariac_interfaces/srv/Trigger <trigger_srv>`
     - Recycle tray at recycling station
   * - :action-name:`/{agv_name}/move`

       :ref:`ariac_interfaces/action/MoveAgv <moveagv_action>`
     - Move AGV to specified station

Sensor Interfaces
-----------------

.. list-table:: Sensor Topics
   :header-rows: 1
   :widths: 40 60
   :class: api-table

   * - Interface
     - Description
   * - :topic-name:`/{break_beam_name}/status`

       :ref:`ariac_interfaces/msg/BreakBeamStatus <break-beam-anchor>`
     - Reports if an object is detected with a timestamp
   * - :topic-name:`/{break_beam_name}/change`

       :ref:`ariac_interfaces/msg/BreakBeamStatus <break-beam-anchor>`
     - Publishes when the breakbeam status changes
   * - :topic-name:`/{distance_sensor_name}/distance`

       :ref:`ariac_interfaces/msg/DistanceSensor <distance-sensor-anchor>`
     - Reports a distance to the first object in view with a timestamp
   * - :topic-name:`/{camera_name}/image`

       `sensor_msgs/msg/Image <https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html>`_
     - Displays current image seen through camera
   * - :topic-name:`/{camera_name}/info`

       `sensor_msgs/msg/CameraInfo <https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html>`_
     - Info about camera sensor
   * - :topic-name:`/{lidar_name}/scan`

       `sensor_msgs/msg/PointCloud2 <https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html>`_
     - Reports the point cloud detected from the lidar scan

.. note::
   
  The **sensor name** is defined by the team in their configuration file