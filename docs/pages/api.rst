.. _API:

=======
ROS API
=======

.. role:: topic-name
   :class: topic-name

.. role:: service-name
   :class: service-name

.. role:: action-name
   :class: action-name

.. raw:: html

   <style>
   .topic-name { color: #28a745; font-weight: bold; font-family: monospace; }
   .service-name { color: #6f42c1; font-weight: bold; font-family: monospace; }
   .action-name { color: #007bff; font-weight: bold; font-family: monospace; }
   </style>

**Interface Types:**

* :topic-name:`Topics` - Data publication/subscription interfaces
* :service-name:`Services` - Request/response interfaces
* :action-name:`Actions` - Long-running goal-based interfaces

Competition Control Interfaces
------------------------------

.. list-table:: Competition Control
   :header-rows: 1
   :widths: 40 60

   * - Service/Topic
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
   :widths: 50 50

   * - Service/Topic
     - Description
   * - :topic-name:`/inspection_conveyor/status`

       :ref:`ariac_interfaces/msg/ConveyorStatus <conveyorstatus_msg>`
     - Conveyor direction, speed, operating status
   * - :topic-name:`/inspection_conveyor/cell_feed/status`

       :ref:`ariac_interfaces/msg/CellFeederStatus <cellfeederstatus_msg>`
     - Cell type being fed and feed rate
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
   :widths: 50 50

   * - Service/Topic
     - Description
   * - :topic-name:`/assembly_conveyor/{section_number}/status`

       :ref:`ariac_interfaces/msg/ConveyorStatus <conveyorstatus_msg>`
     - Section direction, speed, status
   * - :service-name:`/assembly_conveyor/{section_number}/control`

       :ref:`ariac_interfaces/srv/ConveyorControl <conveyorcontrol_srv>`
     - Control individual conveyor sections
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
   :widths: 60 40

   * - Service/Topic
     - Description
   * - :topic-name:`/{robot_name}/joint_states`

       `sensor_msgs/msg/JointState <https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/JointState.html>`_
     - Current joint states
   * - :action-name:`/{robot_name}/joint_trajectory_controller/follow_joint_trajectory`

       `control_msgs/action/FollowJointTrajectory <https://docs.ros.org/en/jazzy/p/control_msgs/action/FollowJointTrajectory.html>`_
     - Joint trajectory commands

.. list-table:: Gripper Interface
   :header-rows: 1
   :widths: 60 40

   * - Service/Topic
     - Description
   * - :action-name:`/{robot_name}/gripper_controller/gripper_command`

       :ref:`ariac_interfaces/action/GripperCommand <grippercommand_action>`
     - Control gripper width

.. list-table:: Vacuum Tool Interfaces
   :header-rows: 1
   :widths: 50 50

   * - Service/Topic
     - Description
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
   :widths: 50 50

   * - Service/Topic
     - Description
   * - :topic-name:`/{agv_name}/info`

       :ref:`ariac_interfaces/msg/AgvStatus <agvstatus_msg>`
     - Current location and pose
   * - :service-name:`/{agv_name}/recycle_cells`

       :ref:`ariac_interfaces/srv/Trigger <trigger_srv>`
     - Recycle tray at recycling station
   * - :action-name:`/{agv_name}/move`

       :ref:`ariac_interfaces/action/MoveAgv <moveagv_action>`
     - Move AGV to specified station