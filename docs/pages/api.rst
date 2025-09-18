.. _API:

=======
ROS API
=======

**Interface Types:**

* :msg-name:`Messages` - Message definition structures
* :topic-name:`Topics` - Data publication/subscription interfaces
* :service-name:`Services` - Request/response interfaces
* :action-name:`Actions` - Long-running goal-based interfaces

----

Enumeration Messages
--------------------

.. list-table::
   :header-rows: 1
   :widths: 30 70
   :class: api-table msg-def-table

   * - Message
     - Description
   * - :ref:`AgvStations <agvstations_msg>`
     - AGV station location constants
   * - :ref:`CellTypes <cell_types_msg>`
     - Cell type definitions and voltage constants
   * - :ref:`CompetitionStates <competitionstates_msg>`
     - Competition state enumeration values
   * - :ref:`OperationStates <operationstates_msg>`
     - Operation state definitions
   * - :ref:`VacuumTools <vacuumtools_msg>`
     - Vacuum gripper tool type definitions

Data Structure Messages
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 30 70
   :class: api-table msg-def-table

   * - Message
     - Description
   * - :ref:`CellDefect <celldefect_msg>`
     - Cell defect information structure
   * - :ref:`CompetitionTime <competitiontime_msg>`
     - Competition timing data structure
   * - :ref:`InspectionReport <inspectionreport_msg>`
     - Inspection result data structure

Competition Control Interfaces
------------------------------

.. list-table::
   :header-rows: 1
   :widths: 45 55
   :class: api-table

   * - Interface
     - Description
   * - :ref:`/competition_status <competitionstatus_msg>`
     - Query competition state, time, order counts
   * - :ref:`/high_priority_orders <highpriorityorder_msg>`
     - High priority kit requests with order ID
   * - :ref:`/start_competition <trigger_srv>`
     - Starts the competition
   * - :ref:`/end_competition <endcompetition_srv>`
     - Ends competition, optionally shutdown Gazebo
   * - :ref:`/submit_kitting_order <trigger_srv>`
     - Submit kit (AGV at shipping required)
   * - :ref:`/submit_module_order <trigger_srv>`
     - Submit module (in submission zone required)
   * - :ref:`/submit_high_priority_order <submithighpriorityorder_srv>`
     - Submit high priority kit with ID

Task 1 Interfaces
-----------------

.. list-table::
   :header-rows: 1
   :widths: 45 55
   :class: api-table

   * - Interface
     - Description
   * - :ref:`/inspection_conveyor/status <conveyorstatus_msg>`
     - Conveyor direction, speed, operating status
   * - :ref:`/inspection_conveyor/cell_feed/status <cellfeederstatus_msg>`
     - Current cell type being fed and feed rate
   * - :ref:`/voltage_tester_1/voltage <voltagereading_msg>`
     - Voltage reading for tester 1
   * - :ref:`/voltage_tester_2/voltage <voltagereading_msg>`
     - Voltage reading for tester 2
   * - :ref:`/inspection_conveyor/cell_feed/control <controlcellfeeder_srv>`
     - Change cell type being fed
   * - :ref:`/inspection_conveyor/inspection/submit <submitinspectionreport_srv>`
     - Submit inspection with pass/fail and defects

Task 2 Interfaces
-----------------

.. list-table::
   :header-rows: 1
   :widths: 45 55
   :class: api-table

   * - Interface
     - Description
   * - :ref:`/assembly_conveyor/section_{number}/status <conveyorstatus_msg>`
     - Section direction, speed, status
   * - :ref:`/assembly_conveyor/section_1/control <conveyorcontrol_srv>`
     - Control conveyor section 1
   * - :ref:`/assembly_conveyor/section_2/control <conveyorcontrol_srv>`
     - Control conveyor section 2
   * - :ref:`/assembly_conveyor/section_3/control <bidirectionalconveyorcontrol_srv>`
     - Control bidirectional conveyor section 3
   * - :ref:`/gantry_welder/weld <trigger_srv>`
     - Create weld if electrodes contact plate
   * - :ref:`/insert_bottom_shell <trigger_srv>`
     - Spawn bottom shell on section 1
   * - :ref:`/insert_top_shell <trigger_srv>`
     - Spawn top shell on assembly table

Robot Control Interfaces
------------------------

Joint Control
^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 45 55
   :class: api-table

   * - Interface
     - Description
   * - `/{robot_name}/joint_states <https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/JointState.html>`_
     - Current joint states
   * - `/{robot_name}/joint_trajectory_controller/follow_joint_trajectory <https://docs.ros.org/en/jazzy/p/control_msgs/action/FollowJointTrajectory.html>`_
     - Joint trajectory commands

Gripper Control
^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 45 55
   :class: api-table

   * - Interface
     - Description
   * - :ref:`/{robot_name}/gripper_controller/gripper_command <grippercommand_action>`
     - Control gripper width

Vacuum Tool Control
^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 45 55
   :class: api-table

   * - Interface
     - Description
   * - :ref:`/assembly_robot_2/tool_changer/status <toolchangerstatus_msg>`
     - Tool changer current state and status
   * - :ref:`/assembly_robot_2/tool_changer/attach_tool <attachtool_srv>`
     - Attach specified tool to robot, must be in correct position
   * - :ref:`/assembly_robot_2/tool_changer/detach_tool <trigger_srv>`
     - Detach tool from robot
   * - :ref:`/vacuum_tool/vg_2/grasp <trigger_srv>`
     - Grasp object with vg_2, requires contact with object
   * - :ref:`/vacuum_tool/vg_2/release <trigger_srv>`
     - Detach object from vg_2
   * - :ref:`/vacuum_tool/vg_4/grasp <trigger_srv>`
     - Grasp object with vg_4, requires contact with object
   * - :ref:`/vacuum_tool/vg_4/release <trigger_srv>`
     - Detach object from vg_4


AGV Control Interfaces
----------------------

.. list-table::
   :header-rows: 1
   :widths: 45 55
   :class: api-table

   * - Interface
     - Description
   * - :ref:`/{agv_name}/info <agvstatus_msg>`
     - Current location and pose
   * - :ref:`/{agv_name}/tray_status <agvtraystatus_msg>`
     - Tray occupancy and part information
   * - :ref:`/{agv_name}/recycle_cells <trigger_srv>`
     - Recycle tray at recycling station
   * - :ref:`/{agv_name}/move <moveagv_action>`
     - Move AGV to specified station

Sensor Interfaces
-----------------

.. note::
   
  The **sensor name** is defined by the team in their configuration file

Break Beam Sensors
^^^^^^^^^^^^^^^^^^

.. _break_beam_anchor:

.. list-table::
   :header-rows: 1
   :widths: 50 50
   :class: api-table

   * - Interface
     - Description
   * - :ref:`/{break_beam_name}/status <break-beam-anchor>`
     - Reports if an object is detected with a timestamp
   * - :ref:`/{break_beam_name}/change <break-beam-anchor>`
     - Publishes when the breakbeam status changes

Distance Sensors
^^^^^^^^^^^^^^^^

.. _distance_anchor:

.. list-table::
   :header-rows: 1
   :widths: 50 50
   :class: api-table

   * - Interface
     - Description
   * - :ref:`/{distance_sensor_name}/distance <distance-sensor-anchor>`
     - Reports a distance to the first object in view with a timestamp

Camera Sensors
^^^^^^^^^^^^^^

.. _camera_anchor:

.. list-table::
   :header-rows: 1
   :widths: 50 50
   :class: api-table

   * - Interface
     - Description
   * - `/{camera_name}/image <https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html>`_
     - Displays current image seen through camera
   * - `/{camera_name}/info <https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html>`_
     - Info about camera sensor

Lidar Sensors
^^^^^^^^^^^^^

.. _lidar_anchor:

.. list-table::
   :header-rows: 1
   :widths: 50 50
   :class: api-table

   * - Interface
     - Description
   * - `/{lidar_name}/scan <https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html>`_
     - Reports the point cloud detected from the lidar scan

