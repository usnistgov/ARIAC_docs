.. _TRIALS:

===================
Configuration Files
===================

Configuration files are described in the YAML format. YAML is a human-readable data serialization format. It is commonly used for configuration files and in applications where data is being stored or transmitted. For more information on YAML, see the `YAML website <https://yaml.org/>`_.

--------------------
Sensor Configuration
--------------------

The sensor configuration file is defined by the competitor and describes the location of sensors added to the competition environment. One example of a sensor configuration file :file:`sensors.yaml` is provided in the test competitor package in the config directory. 

:numref:`sensor-config` shows an example of a sensor configuration file with one of each of the different possible sensors. 

The first field in the yaml file is :yamlname:`robot_cameras`. This is an optional field in the sensor configuration file. If this field is not present in the file cameras will not be activated. Alternatively the :yamlname:`active` field can be used to enable or disable a camera for either robot. Last the :yamlname:`type` field should be either :yaml:`rgb` or :yaml:`rgbd`.

The second field in the configuration file is :yamlname:`static_sensors`. This field should contain all the static sensors that the competitor wants to add. Static sensors each have four fields:

  1. A name (e.g. :yamlname:`right_bins_camera`). This name has to be unique among all sensors in the same configuration file.

  2. A type (e.g. :yamlname:`rgb_camera`). This type has to be one of the following types described in the section :ref:`STATIC_SENSORS`.

  3. A pose defined in the world frame:

    * A position :yamlname:`xyz`. This should be a list with three elements representing the x, y, and z coordinates for the sensor origin.

    * An orientation :yamlname:`rpy`. This should be a three element list containing the roll, pitch and yaw of the sensor using `this convention <https://en.wikipedia.org/wiki/Euler_angles>`_ . The orientation should be defined in radians using either floating-point values or with the :yaml:`pi` constant (:yaml:`pi`, :yaml:`pi/2`, :yaml:`pi/4`, etc). 

  4. A toggle for visualizing the sensor field of view :yamlname:`visualize_fov`. This field is optional and if not present the default is :yaml:`false`.

.. code-block:: yaml
  :caption: Example of a sensor configuration file
  :name: sensor-config

  robot_cameras:
    floor_robot_camera: 
      active: true
      type: rgb
    
    ceiling_robot_camera: 
      active: true
      type: rgbd

  static_sensors:
    breakbeam_0:
      type: break_beam
      visualize_fov: true
      pose:
        xyz: [-0.35, 3, 0.95]
        rpy: [0, 0, pi]

    proximity_sensor_0:
      type: proximity
      visualize_fov: true
      pose:
        xyz: [-0.573, 2.84, 1]
        rpy: [pi/2, pi/6, pi/2]

    laser_profiler_0:
      type: laser_profiler
      visualize_fov: true
      pose:
        xyz: [-0.573, 1.486, 1.526]
        rpy: [pi/2, pi/2, 0]

    lidar_0:
      type: lidar
      visualize_fov: false
      pose:
        xyz: [-2.286, -2.96, 1.8]
        rpy: [pi, pi/2, 0]

    rgb_camera_0:
      type: rgb_camera
      visualize_fov: false
      pose:
        xyz: [-2.286, 2.96, 1.8]
        rpy: [pi, pi/2, 0]

    rgbd_camera_0:
      type: rgbd_camera
      visualize_fov: false
      pose:
        xyz: [-2.286, 4.96, 1.8]
        rpy: [pi, pi/2, 0]

    basic_logical_camera_0:
      visualize_fov: false
      type: basic_logical_camera
      pose:
        xyz: [-2.286, 2.96, 1.8]
        rpy: [pi, pi/2, 0]
    

-------------------
Trial Configuration
-------------------

The trial configuration file contains all the information that the :term:`AM <ARIAC Manager (AM)>` uses to run a given trial. This includes part and kit tray information, orders, and challenges. :numref:`trial-config` shows an example of a trial configuration file with all of the possible fields.  

Fields
======

:yamlname:`time_limit`, :yaml:`Float`
-------------------------------------

The time limit is a positive floating point number that represents the number of seconds that the competitor has to complete the trial. At the end of the time limit the competition will end automatically. 

:yamlname:`kitting_trays`, :yaml:`Dictionary`
---------------------------------------------

This section defines the kit trays that will be spawned onto the kit tray stations at the start of simulation.

  * :yamlname:`tray_ids`, :yaml:`List`: The IDs of the all the trays to be spawned (0-9). Several trays can have the same ID.

  * :yamlname:`slots`, :yaml:`List`: The slots on the kitting tray station where the trays will be spawned (kts1: 0-3, kts2: 4-6). Slots must be unique.

  .. note::
    The :yamlname:`tray_ids` and :yamlname:`slots` lists should be the same length.

:yamlname:`assembly_inserts`, :yaml:`Dictionary`
------------------------------------------------

This section defines the rotation for each of the assembly station inserts. 

  * :yamlname:`as{n}`, :yaml:`Float` or :yaml:`String`: Rotation in radians of assembly insert n (1-4). Pi strings can be used.

:yamlname:`parts`, :yaml:`Dictionary`
-------------------------------------

This section defines all the parts that will be spawned into the environment.

  * :yamlname:`agvs`, :yaml:`Dictionary`: Parts that will placed directly onto the AGV at the start of the simulation. These parts are directly related to the assembly order and will be filled out automatically based on the assembly order if using the trial_generator. 

    * :yamlname:`agv{n}`, :yaml:`Dictionary`: The AGV that will have parts (n: 1-4). Multiple agvs may have parts. 

      * :yamlname:`tray_id`, :yaml:`Integer`: The ID of the kitting tray (0-9) that will be placed on the AGV. Parts on AGVs should always be placed onto a kit tray. 

      * :yamlname:`parts`, :yaml:`List`: Parts that will be spawned on the tray.

        * :yamlname:`type`, :yaml:`String`: The type of the part (:yaml:`'sensor'`, :yaml:`'regulator'`, :yaml:`'pump'`, or :yaml:`'battery'`).

        * :yamlname:`color`, :yaml:`String`: The color of the part (:yaml:`'red'`, :yaml:`'blue'`, :yaml:`'green'`, :yaml:`'orange'`, or :yaml:`'purple'`).

        * :yamlname:`quadrant`, :yaml:`Integer`: The quadrant of the kit tray the part will be spawned onto (1-4).

        * :yamlname:`rotation`, :yaml:`Float` or :yaml:`String`: The rotation of the part in radians. Pi strings can be used.

  * :yamlname:`bins`, :yaml:`Dictionary`: Parts that will be placed into the bins at the start of the simulation.

    * :yamlname:`bin{n}`, :yaml:`Dictionary`: The bin that will have parts (n: 1-8). Multiple bins may have parts.

      * :yamlname:`type`, :yaml:`String`: The type of the part (:yaml:`'sensor'`, :yaml:`'regulator'`, :yaml:`'pump'`, or :yaml:`'battery'`).

      * :yamlname:`color`, :yaml:`String`: The color of the part (:yaml:`'red'`, :yaml:`'blue'`, :yaml:`'green'`, :yaml:`'orange'`, or :yaml:`'purple'`).

      * :yamlname:`rotation`, :yaml:`Float` or :yaml:`String`: The rotation of the part in radians. Pi strings can be used.

      * :yamlname:`flipped`, :yaml:`Boolean`: Whether the part will be flipped. If :yaml:`true` the z-axis of the part will face down instead of up. 

      * :yamlname:`slots`, :yaml:`List`: The slots of the bin that this part can be found (1-9). Each slot should be unique for a given bin. 

  * :yamlname:`conveyor_belt`: Parts that will be spawned onto the conveyor when the competition is started. The cycle of parts will be repeated after all parts are spawned, up until the competition ends. 

    * :yamlname:`active`, :yaml:`Boolean`: Whether the conveyor is active. This toggle is mostly used for testing to enable or disable the conveyor without removing all the parts. 

    * :yamlname:`spawn_rate`, :yaml:`Integer`: The time in seconds between parts spawned on the conveyor.

    * :yamlname:`order`, :yaml:`String`: Either :yaml:`'sequential'` where all parts of the same type will appear before the next type, or  :yaml:`'random'` where the parts will come out in a random order. 

    * :yamlname:`parts_to_spawn`, :yaml:`Dictionary`: This is a list of part lots that will be found on the conveyor.

      * :yamlname:`type`, :yaml:`String`: The type of the part (:yaml:`'sensor'`, :yaml:`'regulator'`, :yaml:`'pump'`, or :yaml:`'battery'`).

      * :yamlname:`color`, :yaml:`String`: The color of the part (:yaml:`'red'`, :yaml:`'blue'`, :yaml:`'green'`, :yaml:`'orange'`, or :yaml:`'purple'`).

      * :yamlname:`number`

      * :yamlname:`offset`

      * :yamlname:`flipped`

      * :yamlname:`rotation`

:yamlname:`orders`
------------------

  * :yamlname:`id`

  * :yamlname:`type`

  * :yamlname:`announcement`

  * :yamlname:`priority`

  * :yaml:`task_info`

:yamlname:`challenges`
----------------------

  * :yamlname:`dropped_part`

    * :yamlname:`robot`

    * :yamlname:`type`

    * :yamlname:`color`

    * :yamlname:`drop_after`

    * :yamlname:`delay`
  
  * :yamlname:`robot_malfunction`

    * :yamlname:`duration`

    * :yamlname:`robots_to_disable`

    * :yaml:`announcement`: 

  * :yamlname:`sensor_blackout`

    * :yamlname:`duration`

    * :yamlname:`sensors_to_disable`

    * :yaml:`announcement`: 

  * :yamlname:`faulty_part`

    * :yamlname:`order_id`
    * :yamlname:`quadrant{n}`


.. code-block:: yaml
  :caption: Example of a trial configuration file
  :name: trial-config
  :linenos:

  # Trial name: example.yaml
  # ARIAC2024
  # Author: Justin Albrecht
  # 2024-01-03 14:23:37

  # ENVIRONMENT SETUP

  time_limit: 500

  # KITTING TRAYS

  kitting_trays:
    tray_ids:
    - 2
    - 2
    - 2
    - 4
    - 4
    - 4
    slots:
    - 1
    - 2
    - 3
    - 4
    - 5
    - 6

  # INSERT ROTATION

  assembly_inserts:
    as1: pi/3
    as2: -pi/4
    as3: 0.0
    as4: pi/2

  # PARTS INFORMATION

  parts:
    agvs:
      agv2:
        tray_id: 0
        parts:
        - type: sensor
          color: blue
          quadrant: 1
          rotation: 0.0
        - type: pump
          color: blue
          quadrant: 2
          rotation: 0.0
        - type: regulator
          color: blue
          quadrant: 3
          rotation: 0.0
        - type: battery
          color: blue
          quadrant: 4
          rotation: 0.0
    bins:
      bin1:
      - type: sensor
        color: green
        rotation: pi/4
        flipped: false
        slots:
        - 1
        - 3
        - 7
        - 9
      bin2:
      - type: pump
        color: purple
        rotation: 0.0
        flipped: false
        slots:
        - 1
        - 3
        - 7
        - 9
      bin5:
      - type: battery
        color: orange
        rotation: 0.0
        flipped: true
        slots:
        - 2
        - 4
        - 6
        - 8
      bin6:
      - type: regulator
        color: blue
        rotation: -pi
        flipped: false
        slots:
        - 2
        - 4
        - 6
        - 8
    conveyor_belt:
      active: true
      spawn_rate: 3.0
      order: random
      parts_to_spawn:
      - type: sensor
        color: orange
        number: 3
        offset: 0.2
        flipped: false
        rotation: pi/3
      - type: battery
        color: red
        number: 5
        offset: 0.0
        flipped: true
        rotation: 0.0

  # ORDERS INFORMATION

  orders:
  - id: E414303S
    type: kitting
    announcement:
      time_condition: 0.0
    priority: false
    kitting_task:
      agv_number: 1
      tray_id: 2
      destination: warehouse
      products:
      - type: sensor
        color: green
        quadrant: 1
      - type: battery
        color: red
        quadrant: 2
      - type: pump
        color: purple
        quadrant: 4
  - id: YTDIGV7W
    type: assembly
    announcement:
      submission_condition:
        order_id: E414303S
    priority: false
    assembly_task:
      agv_number:
      - 2
      station: as1
      products:
      - type: sensor
        color: blue
        assembled_pose:
          xyz:
          - -0.1
          - 0.395
          - 0.045
          rpy:
          - 0
          - 0
          - -pi/2
        assembly_direction:
        - 0
        - -1
        - 0
      - type: pump
        color: blue
        assembled_pose:
          xyz:
          - 0.14
          - 0.0
          - 0.02
          rpy:
          - 0
          - 0
          - -pi/2
        assembly_direction:
        - 0
        - 0
        - -1
      - type: regulator
        color: blue
        assembled_pose:
          xyz:
          - 0.175
          - -0.223
          - 0.215
          rpy:
          - pi/2
          - 0
          - -pi/2
        assembly_direction:
        - 0
        - 0
        - -1
      - type: battery
        color: blue
        assembled_pose:
          xyz:
          - -0.15
          - 0.035
          - 0.043
          rpy:
          - 0
          - 0
          - pi/2
        assembly_direction:
        - 0
        - 1
        - 0
  - id: 7JADKH4U
    type: combined
    announcement:
      part_place_condition:
        color: purple
        type: pump
        agv: 1
    priority: true
    combined_task:
      station: as3
      products:
      - type: regulator
        color: blue
        assembled_pose:
          xyz:
          - 0.175
          - -0.223
          - 0.215
          rpy:
          - pi/2
          - 0
          - -pi/2
        assembly_direction:
        - 0
        - 0
        - -1
      - type: sensor
        color: orange
        assembled_pose:
          xyz:
          - -0.1
          - 0.395
          - 0.045
          rpy:
          - 0
          - 0
          - -pi/2
        assembly_direction:
        - 0
        - -1
        - 0

  # CHALLENGES INFORMATION

  challenges:
  - dropped_part:
      robot: floor_robot
      type: regulator
      color: blue
      drop_after: 0
      delay: 0.5
  - robot_malfunction:
      duration: 20.0
      robots_to_disable:
      - floor_robot
      submission_condition:
        order_id: 7JADKH4U
  - sensor_blackout:
      duration: 20.0
      sensors_to_disable:
      - logical_camera
      - camera
      time_condition: 100.0
  - faulty_part:
      order_id: E414303S
      quadrant1: true






