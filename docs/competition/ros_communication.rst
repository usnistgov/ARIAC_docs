.. _ROS_COMMUNICATION:

=================
ROS Communication
=================

This section shows the ROS topics and services that are used to communicate between the :term:`Competitor Control System (CCS)` and the ARIAC system. The definition of each message and service type is also provided.

------------------
Competition Topics
------------------

.. list-table:: List of topics.
   :widths: auto
   :header-rows: 1
   :name: communications-topics

   * - Topic Name
     - Message Definition
     - Description 
   * - :topic:`/ariac/orders` 
     - :term:`ariac_msgs/msg/Order`
     - Orders that the CCS should submit
   * - :topic:`/ariac/competition_state`
     - :term:`ariac_msgs/msg/CompetitionState`
     - Current state of the competition 
   * - :topic:`/ariac/bin_parts`
     - :term:`ariac_msgs/msg/BinParts`
     - Part information in each bin at program start-up 
   * - :topic:`/ariac/conveyor_parts`
     - :term:`ariac_msgs/msg/ConveyorParts`
     - Parts that will come on the conveyor belt 
   * - :topic:`/ariac/agv{n}_status`
     - :term:`ariac_msgs/msg/AGVStatus`
     - State of the AGV :msg:`{n}` (location, position, velocity)
   * - :topic:`/ariac/{robot}_gripper_state`
     - :term:`ariac_msgs/msg/VacuumGripperState`
     - State of :msg:`{robot}`'s gripper (enabled, attached, type)
   * - :topic:`/ariac/conveyor_state`
     - :term:`ariac_msgs/msg/ConveyorBeltState`
     - State of the conveyor (enabled, power)
   * - :topic:`/ariac/robot_health`
     - :term:`ariac_msgs/msg/Robots`
     - Health of the robots (enabled or disabled)
   * - :topic:`/ariac/sensor_health`
     - :term:`ariac_msgs/msg/Sensors`
     - Health of the sensors (enabled or disabled)
   * - :topic:`/ariac_human/state`
     - :term:`ariac_msgs/msg/HumanState`
     - Position and velocity of the human and the ceiling robot
   * - :topic:`/ariac/assembly_insert_{n}_assembly_state`
     - :term:`ariac_msgs/msg/AssemblyState`
     - State of the assembly station :msg:`as{n}` (battery_attached, pump_attached, sensor_attached, regulator_attached)

.. _SENSOR_TOPICS:

-------------
Sensor Topics
-------------

.. list-table:: List of sensor topics.
   :widths: auto
   :header-rows: 1
   :name: communications-sensor-topics

   * - Sensor Type
     - Topic Name
     - Message Definition 
   * - break_beam
     - :topic:`/ariac/sensors/{sensor_name}/change` |br| :topic:`/ariac/sensors/{sensor_name}/status`
     - :term:`ariac_msgs/msg/BreakBeamStatus` |br| :term:`ariac_msgs/msg/BreakBeamStatus`
   * - proximity
     - :topic:`/ariac/sensors/{sensor_name}/scan`
     - :term:`sensor_msgs/msg/Range`
   * - laser_profiler
     - :topic:`/ariac/sensors/{sensor_name}/scan`
     - :term:`sensor_msgs/msg/LaserScan`
   * - lidar
     - :topic:`/ariac/sensors/{sensor_name}/scan`	
     - :term:`sensor_msgs/msg/PointCloud`
   * - rgb_camera
     - :topic:`/ariac/sensors/{sensor_name}/rgb_image`
     - :term:`sensor_msgs/msg/Image`
   * - rgbd_camera
     - :topic:`/ariac/sensors/{sensor_name}/rgb_image` |br| :topic:`/ariac/sensors/{sensor_name}/depth_image`
     - :term:`sensor_msgs/msg/Image` |br| :term:`sensor_msgs/msg/Image`
   * - basic_logical_camera
     - :topic:`/ariac/sensors/{sensor_name}/image`
     - :term:`ariac_msgs/msg/BasicLogicalCameraImage`
   * - advanced_logical_camera
     - :topic:`/ariac/sensors/{sensor_name}/image`
     - :term:`ariac_msgs/msg/AdvancedLogicalCameraImage`

--------
Services
--------

.. list-table:: List of services.
   :widths: 20 35 45
   :header-rows: 1
   :name: communications-services

   * - Service Name
     - Service Definition
     - Description  
   * - :rosservice:`/ariac/start_competition`
     - :term:`std_srvs/srv/Trigger`
     - Start the competition   
   * - :rosservice:`/ariac/end_competition`
     - :term:`std_srvs/srv/Trigger`
     - End the competition
   * - :rosservice:`/ariac/submit_order`
     - :term:`ariac_msgs/srv/SubmitOrder`
     - Submit an order with the requested **order_id**
   * - :rosservice:`/ariac/perform_quality_check`
     - :term:`ariac_msgs/srv/PerformQualityCheck`
     - Check the quality of a kitting order with the requested **order_id**
   * - :rosservice:`/ariac/get_pre_assembly_poses`
     - :term:`ariac_msgs/srv/GetPreAssemblyPoses`
     - Get the pose of parts on the AGVs prior to assembly for an assembly or combined order with **order_id**
   * - :rosservice:`/ariac/move_agv{n}` 
     - :term:`ariac_msgs/srv/MoveAGV`
     - Move the AGV :msg:`{n}` to the requested location  
   * - :rosservice:`/ariac/agv{n}_lock_tray` 
     - :term:`std_srvs/srv/Trigger`
     - Lock a kit tray to AGV :msg:`{n}` 
   * - :rosservice:`/ariac/agv{n}_unlock_tray`
     - :term:`std_srvs/srv/Trigger`
     - Unlock a kit tray to AGV :msg:`{n}` 
   * - :rosservice:`/ariac/{robot}_enable_gripper`
     - :term:`ariac_msgs/srv/VacuumGripperControl`
     - Set the state of :msg:`{robot}`'s gripper to the request state
   * - :rosservice:`/ariac/{robot}_change_gripper`
     - :term:`ariac_msgs/srv/ChangeGripper`
     - Change the type of :msg:`{robot}`'s gripper to the request type


-------------------
Message Definitions
-------------------

.. glossary::
    :sorted:

    ariac_msgs/msg/Order
      .. code-block:: text

        uint8 KITTING=0
        uint8 ASSEMBLY=1
        uint8 COMBINED=2

        string id
        uint8 type # KITTING, ASSEMBLY, or COMBINED
        bool priority
        ariac_msgs/KittingTask kitting_task 
        ariac_msgs/AssemblyTask assembly_task
        ariac_msgs/CombinedTask combined_task

      - :msg:`id`: The unique identifier for the order
      - :msg:`type`: The type of order. One of the following:

        - :msg:`KITTING`: A kitting order
        - :msg:`ASSEMBLY`: An assembly order
        - :msg:`COMBINED`: A combined order
      - :msg:`priority`: Whether the order is a priority order
      - :msg:`kitting_task`: The kitting task for the order
      - :msg:`assembly_task`: The assembly task for the order
      - :msg:`combined_task`: The combined task for the order

      .. seealso:: 
        
        - :term:`ariac_msgs/msg/KittingTask`
        - :term:`ariac_msgs/msg/AssemblyTask`
        - :term:`ariac_msgs/msg/CombinedTask`

    ariac_msgs/msg/KittingTask
      .. code-block:: text

        uint8 KITTING=0
        uint8 ASSEMBLY_FRONT=1
        uint8 ASSEMBLY_BACK=2
        uint8 WAREHOUSE=3

        uint8 agv_number
        int8 tray_id
        uint8 destination
        ariac_msgs/KittingPart[] parts

      - :msg:`agv_number`: The AGV number to deliver the kit to (1, 2, 3, or 4)
      - :msg:`tray_id`: The tray number to deliver the kit to (1, 2, 3, 4, 5, or 6)
      - :msg:`destination`: The destination of the kit.  One of the following values:

        - :msg:`KITTING`: The kit is to be delivered to the kitting station
        - :msg:`ASSEMBLY_FRONT`: The kit is to be delivered to the front assembly station (:msg:`as1` or :msg:`as3` depending on the AGV number)
        - :msg:`ASSEMBLY_BACK`: The kit is to be delivered to the back assembly station (:msg:`as2` or :msg:`as4` depending on the AGV number)
        - :msg:`WAREHOUSE`: The kit is to be delivered to the warehouse

      - :msg:`parts`: The parts to be placed in the kit

      .. seealso:: :term:`ariac_msgs/msg/KittingPart`


    ariac_msgs/msg/AssemblyTask
      .. code-block:: text

        uint8 AS1=1
        uint8 AS2=2
        uint8 AS3=3
        uint8 AS4=4

        uint8[] agv_numbers
        uint8 station
        ariac_msgs/AssemblyPart[] parts

      - :msg:`agv_numbers`: The AGVs which contain parts for assembly
      - :msg:`station`: The assembly station to assemble the parts at.  One of the following values:

        - :msg:`AS1`: The front assembly station for AGV 1 and 2
        - :msg:`AS2`: The back assembly station for AGV 1 and 2
        - :msg:`AS3`: The front assembly station for AGV 3 and 4
        - :msg:`AS4`: The back assembly station for AGV 3 and 4
      - :msg:`parts`: The parts to be assembled

      .. seealso:: :term:`ariac_msgs/msg/AssemblyPart`

    ariac_msgs/msg/CombinedTask
      .. code-block:: text

        uint8 AS1=1
        uint8 AS2=2
        uint8 AS3=3
        uint8 AS4=4

        uint8 station
        ariac_msgs/AssemblyPart[] parts

      - :msg:`station`: The assembly station to assemble the parts at.  One of the following values:

        - :msg:`AS1`: The front assembly station for AGV 1 and 2
        - :msg:`AS2`: The back assembly station for AGV 1 and 2
        - :msg:`AS3`: The front assembly station for AGV 3 and 4
        - :msg:`AS4`: The back assembly station for AGV 3 and 4
      - :msg:`parts`: The parts to be assembled

      .. seealso:: :term:`ariac_msgs/msg/AssemblyPart`

    ariac_msgs/msg/AssemblyPart
      .. code-block:: text

        ariac_msgs/Part part
        geometry_msgs/PoseStamped assembled_pose
        geometry_msgs/Vector3 install_direction

      - :msg:`part`: The part to be assembled
      - :msg:`assembled_pose`: The pose of the part in the assembly station
      - :msg:`install_direction`: The direction the part should be installed in the assembly station

      .. seealso:: 
        
        - :term:`ariac_msgs/msg/Part`
        - `geometry_msgs/msg/PoseStamped <https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html>`_
        - `geometry_msgs/msg/Vector3 <https://docs.ros2.org/latest/api/geometry_msgs/msg/Vector3.html>`_

    ariac_msgs/msg/KittingPart
      .. code-block:: text

        uint8 QUADRANT1=1
        uint8 QUADRANT2=2
        uint8 QUADRANT3=3
        uint8 QUADRANT4=4

        ariac_msgs/Part part
        uint8 quadrant

      - :msg:`part`: The part to be placed in the kit
      - :msg:`quadrant`: The quadrant of the kit to place the part in.  One of the following values:

        - :msg:`QUADRANT1`: The first quadrant of the kit
        - :msg:`QUADRANT2`: The second quadrant of the kit
        - :msg:`QUADRANT3`: The third quadrant of the kit
        - :msg:`QUADRANT4`: The fourth quadrant of the kit


    ariac_msgs/msg/CompetitionState
      .. code-block:: text
        
        uint8 IDLE=0   
        uint8 READY=1  
        uint8 STARTED=2 
        uint8 ORDER_ANNOUNCEMENTS_DONE=3 
        uint8 ENDED=4 

        uint8 competition_state

      - :msg:`competition_state`: The current state of the competition.  One of the following values:

        - :msg:`IDLE`: The competition is idle
        - :msg:`READY`: The competition is ready to start
        - :msg:`STARTED`: The competition has started
        - :msg:`ORDER_ANNOUNCEMENTS_DONE`: The competition has started and all orders have been announced
        - :msg:`ENDED`: The competition has ended

    ariac_msgs/msg/BinParts
      .. code-block:: text
        
        ariac_msgs/BinInfo[] bins

      - :msg:`bins`: List of bins and their contents

      .. seealso:: :term:`ariac_msgs/msg/BinInfo`

    ariac_msgs/msg/BinInfo
      .. code-block:: text

        uint8 BIN1=1
        uint8 BIN2=2
        uint8 BIN3=3
        uint8 BIN4=4
        uint8 BIN5=5
        uint8 BIN6=6
        uint8 BIN7=7
        uint8 BIN8=8

        uint8 bin_number
        ariac_msgs/PartLot[] parts

      - :msg:`bin_number`: The bin number.  One of the following values:
        
          - :msg:`BIN1`: The first bin
          - :msg:`BIN2`: The second bin
          - :msg:`BIN3`: The third bin
          - :msg:`BIN4`: The fourth bin
          - :msg:`BIN5`: The fifth bin
          - :msg:`BIN6`: The sixth bin
          - :msg:`BIN7`: The seventh bin
          - :msg:`BIN8`: The eighth bin
      - :msg:`parts`: The parts in the bin

      .. seealso:: :term:`ariac_msgs/msg/PartLot`

    ariac_msgs/msg/PartLot
      .. code-block:: text

        ariac_msgs/Part part
        uint8 quantity

      - :msg:`part`: The part
      - :msg:`quantity`: The quantity of the part

      .. seealso:: :term:`ariac_msgs/msg/Part`

    ariac_msgs/msg/ConveyorParts
      .. code-block:: text
        
        ariac_msgs/PartLot[] parts

      - :msg:`parts`: The parts on the conveyor

      .. seealso:: :term:`ariac_msgs/msg/PartLot`

    ariac_msgs/msg/AGVStatus
      .. code-block:: text

        uint8 KITTING=0
        uint8 ASSEMBLY_FRONT=1
        uint8 ASSEMBLY_BACK=2
        uint8 WAREHOUSE=3
        uint8 UNKNOWN=99

        int8 location
        float64 position
        float64 velocity

      - :msg:`location`: The location of the AGV.  One of the following values:
        
          - :msg:`KITTING`: The AGV is at the kitting station
          - :msg:`ASSEMBLY_FRONT`: The AGV is at the front assembly station (:msg:`AS1` or :msg:`AS3` )
          - :msg:`ASSEMBLY_BACK`: The AGV is at the back assembly station (:msg:`AS2` or :msg:`AS4` )
          - :msg:`WAREHOUSE`: The AGV is at the warehouse
          - :msg:`UNKNOWN`: The AGV is at an unknown location

      - :msg:`position`: The current position of the AGV in the workcell
      - :msg:`velocity`: The current velocity of the AGV

    ariac_msgs/msg/VacuumGripperState
      .. code-block:: text

        bool enabled 
        bool attached 
        string type 

      - :msg:`enabled`: Is the suction enabled?
      - :msg:`attached`: Is an object attached to the gripper?
      - :msg:`type`: The type of the gripper

    ariac_msgs/msg/ConveyorBeltState
      .. code-block:: text

        float64 power
        bool enabled  

      - :msg:`power`: The power of the conveyor belt
      - :msg:`enabled`: Is the conveyor belt enabled?

    ariac_msgs/msg/Robots
      .. code-block:: text

        bool floor_robot
        bool ceiling_robot

      - :msg:`floor_robot`: Is the floor robot enabled?
      - :msg:`ceiling_robot`: Is the ceiling robot enabled?

    ariac_msgs/msg/Sensors
      .. code-block:: text

        bool break_beam
        bool proximity
        bool laser_profiler
        bool lidar
        bool camera
        bool logical_camera

      - :msg:`break_beam`: Is the break beam sensor type enabled?
      - :msg:`proximity`: Is the proximity sensor type enabled?
      - :msg:`laser_profiler`: Is the laser profiler type enabled?
      - :msg:`lidar`: Is the lidar type enabled?
      - :msg:`camera`: Is the camera type enabled?
      - :msg:`logical_camera`: Is the logical camera type enabled?

    ariac_msgs/msg/HumanState
      .. code-block:: text

        geometry_msgs/Point human_position
        geometry_msgs/Point robot_position
        geometry_msgs/Vector3 human_velocity
        geometry_msgs/Vector3 robot_velocity

      - :msg:`human_position`: The position of the human in the workcell
      - :msg:`robot_position`: The position of the ceiling robot in the workcell
      - :msg:`human_velocity`: The velocity of the human in the workcell
      - :msg:`robot_velocity`: The velocity of the ceiling robot in the workcell

      .. seealso:: 
        
        - `geometry_msgs/msg/Point <https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html>`_
        - `geometry_msgs/msg/Vector3 <https://docs.ros2.org/latest/api/geometry_msgs/msg/Vector3.html>`_

    ariac_msgs/msg/Part
      .. code-block:: text
        
        uint8 RED=0
        uint8 GREEN=1
        uint8 BLUE=2
        uint8 ORANGE=3
        uint8 PURPLE=4

        uint8 BATTERY=10
        uint8 PUMP=11
        uint8 SENSOR=12
        uint8 REGULATOR=13

        uint8 color
        uint8 type

      - :msg:`color`: The color of the part.  One of the following values:
        
          - :msg:`RED`: The part is red
          - :msg:`GREEN`: The part is green
          - :msg:`BLUE`: The part is blue
          - :msg:`ORANGE`: The part is orange
          - :msg:`PURPLE`: The part is purple
      - :msg:`type`: The type of the part.  One of the following values:
        
          - :msg:`BATTERY`: The part is a battery
          - :msg:`PUMP`: The part is a pump
          - :msg:`SENSOR`: The part is a sensor
          - :msg:`REGULATOR`: The part is a regulator


    ariac_msgs/msg/PartPose
      .. code-block:: text
        
        ariac_msgs/Part part
        geometry_msgs/Pose pose

      - :msg:`part`: The part
      - :msg:`pose`: The pose of the part

      .. seealso:: 
        
        - :term:`ariac_msgs/msg/Part`
        - `geometry_msgs/Pose <https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html>`_

    ariac_msgs/msg/AdvancedLogicalCameraImage
      .. code-block:: text
        
        std_msgs/Header header
        ariac_msgs/PartPose[] part_poses
        ariac_msgs/KitTrayPose[] tray_poses
        geometry_msgs/Pose sensor_pose

      - :msg:`part_poses`: The parts in the camera's field of view
      - :msg:`tray_poses`: The kit trays in the camera's field of view
      - :msg:`sensor_pose`: The pose of the camera in the world frame

      .. seealso:: 
        
        - :term:`ariac_msgs/msg/PartPose`
        - :term:`ariac_msgs/msg/KitTrayPose`
        - `geometry_msgs/Pose <https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html>`_

    ariac_msgs/msg/KitTrayPose
      .. code-block:: text
        
        int8 id
        geometry_msgs/Pose pose

      - :msg:`id`: The ID of the kit tray
      - :msg:`pose`: The pose of the kit tray

      .. seealso:: `geometry_msgs/Pose <https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html>`_

    ariac_msgs/msg/BreakBeamStatus
      .. code-block:: text
        
        std_msgs/Header header
        bool object_detected

      - :msg:`header`: The header of the message
      - :msg:`object_detected`: Is an object detected?

    sensor_msgs/msg/Range
      .. code-block:: text
        
        uint8 ULTRASOUND=0
        uint8 INFRARED=1
        std_msgs/msg/Header header
        uint8 radiation_type
        float field_of_view
        float min_range
        float max_range
        float range

      .. seealso:: `sensor_msgs/Range <https://docs.ros2.org/latest/api/sensor_msgs/msg/Range.html>`_

    sensor_msgs/msg/LaserScan
      .. code-block:: text
        
        std_msgs/msg/Header header
        float angle_min
        float angle_max
        float angle_increment
        float time_increment
        float scan_time
        float range_min
        float range_max
        float[] ranges
        float[] intensities

      .. seealso:: `sensor_msgs/LaserScan <https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html>`_

    sensor_msgs/msg/PointCloud
      .. code-block:: text
        
        std_msgs/msg/Header header
        geometry_msgs/msg/Point32[] points
        sensor_msgs/msg/ChannelFloat32[] channels

      .. seealso:: `sensor_msgs/PointCloud <https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html>`_

    sensor_msgs/msg/Image
      .. code-block:: text
        
        std_msgs/msg/Header header
        uint32 height
        uint32 width
        string encoding
        uint8 is_bigendian
        uint32 step
        uint8[] data

      .. seealso:: `sensor_msgs/Image <https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html>`_

    ariac_msgs/msg/BasicLogicalCameraImage
      .. code-block:: text
        
        std_msgs/Header header
        geometry_msgs/Pose[] part_poses
        geometry_msgs/Pose[] tray_poses
        geometry_msgs/Pose sensor_pose

      - :msg:`part_poses`: The poses of the parts in the camera's field of view
      - :msg:`tray_poses`: The poses of the kit trays in the camera's field of view
      - :msg:`sensor_pose`: The pose of the camera in the world frame

      .. seealso:: `geometry_msgs/Pose <https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html>`_

    ariac_msgs/msg/QualityIssue
      .. code-block:: text
        
        bool all_passed
        bool missing_part
        bool flipped_part
        bool faulty_part
        bool incorrect_part_type
        bool incorrect_part_color

      - :msg:`all_passed`: True if all parts passed the quality check, False otherwise
      - :msg:`missing_part`: True if a part is missing, False otherwise
      - :msg:`flipped_part`: True if a part is flipped, False otherwise
      - :msg:`faulty_part`: True if a part is faulty, False otherwise
      - :msg:`incorrect_part_type`: True if a part has the wrong type, False otherwise
      - :msg:`incorrect_part_color`: True if a part has the wrong color, False otherwise
    
    ariac_msgs/msg/AssemblyState
      .. code-block:: text
        
        bool battery_attached
        bool pump_attached
        bool sensor_attached
        bool regulator_attached

      - :msg:`battery_attached`: True if battery is attached to the assembly insert, False otherwise
      - :msg:`pump_attached`: True if pump is attached to the assembly insert, False otherwise
      - :msg:`sensor_attached`: True if sensor is attached to the assembly insert, False otherwise
      - :msg:`regulator_attached`: True if regulator is attached to the assembly insert, False otherwise


-------------------
Service Definitions
-------------------

.. glossary::
    :sorted:

    std_srvs/srv/Trigger
      .. code-block:: text

        ---
        boolean success
        string message

      - :msg:`success`: True if the service call was successful, False otherwise
      - :msg:`message`: A message describing the result of the service call

    ariac_msgs/srv/SubmitOrder
      .. code-block:: text

        string order_id
        ---
        bool success
        string message

      - :msg:`order_id`: The ID of the order to be submitted
      - :msg:`success`: True if the order was submitted successfully, False otherwise
      - :msg:`message`: A message describing the result of the service call

    ariac_msgs/srv/PerformQualityCheck
      .. code-block:: text

        string order_id
        ---
        bool valid_id
        bool all_passed
        bool incorrect_tray
        ariac_msgs/QualityIssue quadrant1
        ariac_msgs/QualityIssue quadrant2
        ariac_msgs/QualityIssue quadrant3
        ariac_msgs/QualityIssue quadrant4

      - :msg:`order_id`: The ID of the order to be submitted
      - :msg:`valid_id`: True if the order ID is valid, False otherwise
      - :msg:`all_passed`: True if all parts in the order passed the quality check, False otherwise
      - :msg:`incorrect_tray`: True if the detected tray does not have the correct ID for the order, False otherwise
      - :msg:`quadrant1`: The quality issue for the first quadrant
      - :msg:`quadrant2`: The quality issue for the second quadrant
      - :msg:`quadrant3`: The quality issue for the third quadrant
      - :msg:`quadrant4`: The quality issue for the fourth quadrant

      .. seealso:: :term:`ariac_msgs/msg/QualityIssue`

    ariac_msgs/srv/GetPreAssemblyPoses
      .. code-block:: text

        string order_id
        ---
        bool valid_id
        bool agv_at_station
        ariac_msgs/PartPose[] parts

      - :msg:`order_id`: The ID of the order to be submitted
      - :msg:`valid_id`: True if the order ID is valid, False otherwise
      - :msg:`agv_at_station`: True if the AGV is at the station, False otherwise
      - :msg:`parts`: The list of parts to be assembled

      .. seealso:: :term:`ariac_msgs/msg/PartPose`

    ariac_msgs/srv/MoveAGV
      .. code-block:: text

        int8 KITTING=0
        int8 ASSEMBLY_FRONT=1
        int8 ASSEMBLY_BACK=2 
        int8 WAREHOUSE=3 

        int8 location
        ---
        bool success
        string message

      - :msg:`location`: The location to move the AGV to. One of the following values:

        - :msg:`KITTING`: Kitting station
        - :msg:`ASSEMBLY_FRONT`: Assembly station front (:msg:`AS1` or :msg:`AS3` depending on the AGV ID)
        - :msg:`ASSEMBLY_BACK`: Assembly station back (:msg:`AS2` or :msg:`AS4` depending on the AGV ID)
        - :msg:`WAREHOUSE`: Warehouse
      - :msg:`success`: True if the AGV was moved successfully, False otherwise
      - :msg:`message`: A message describing the result of the service call

    ariac_msgs/srv/VacuumGripperControl
      .. code-block:: text

        bool enable
        ---
        bool success

      - :msg:`enable`: True to enable the vacuum gripper, False to disable it
      - :msg:`success`: True if the vacuum gripper was enabled/disabled successfully, False otherwise

    ariac_msgs/srv/ChangeGripper
      .. code-block:: text

        uint8 PART_GRIPPER=1
        uint8 TRAY_GRIPPER=2

        uint8 gripper_type

        ---
        bool success
        string message

      - :msg:`gripper_type`: The type of gripper to change to. One of the following values:

        - :msg:`PART_GRIPPER`: Part gripper
        - :msg:`TRAY_GRIPPER`: Tray gripper
      - :msg:`success`: True if the gripper was changed successfully, False otherwise
      - :msg:`message`: A message describing the result of the service call

.. |br| raw:: html

      <br>