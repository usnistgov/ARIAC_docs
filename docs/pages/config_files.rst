.. _CONFIG_FILES:

===================
Configuration Files
===================

Configuration files are essential for customizing competition trials and team setups in ARIAC. There are two main configuration file types: trial configuration files that define the competition scenario parameters, and team configuration files that specify team-specific settings like sensors and conveyor parameters.

The configuration system uses YAML files with strict validation to ensure all parameters meet competition requirements. Both configuration files are necessary to start runs.

-------------------
Trial Configuration
-------------------

The trial configuration file defines the competition scenario, including timing, defect rates, required deliverables, and challenges that will occur during the run.

Core Parameters
===============

.. list-table:: Trial Configuration Parameters
   :header-rows: 1
   :widths: 20 15 65
   :class: centered-table

   * - Parameter
     - Type
     - Description
   * - ``ID``
     - string
     - Unique 8-character alphanumeric identifier for the trial
   * - ``SEED``
     - int
     - Random seed for reproducible trial generation (minimum: 0)
   * - ``DEFECT_RATE``
     - number
     - Rate of defective cells in the trial (0.0 to 1.0)
   * - ``TIME_LIMIT``
     - int
     - Maximum trial duration in seconds (minimum: 100)
   * - ``NUM_KITS``
     - int
     - Number of kits to be completed (minimum: 0)
   * - ``NUM_MODULES``
     - int
     - Number of modules to be completed (minimum: 0)

Optional Parameters
===================

.. list-table:: Optional Trial Configuration Parameters
   :header-rows: 1
   :widths: 20 15 65
   :class: centered-table

   * - Parameter
     - Type
     - Description
   * - ``POSSIBLE_DEFECTS``
     - array
     - List of defect types that can occur (integers ≥ 1, see `defect definitions <https://github.com/usnistgov/ARIAC/blob/ariac2025/ariac_setup/config/defects.yaml>`_)
   * - ``CHALLENGES``
     - array
     - Challenge configuration section (see :ref:`challenges page <CHALLENGES>`)

Example
=======

.. _challenges_config_example:

Here is an example trial configuration file with all main parameters:

.. code-block:: yaml

   # Trial Configuration Example
   ID: "LHAF9835"
   SEED: 12345
   DEFECT_RATE: 0.15
   TIME_LIMIT: 900
   NUM_KITS: 3
   NUM_MODULES: 2

   POSSIBLE_DEFECTS: [1, 2, 3, 4, 5]

   CHALLENGES:
     # Conveyor malfunction example
     CONVEYOR_MALFUNCTIONS:
       - START_TIME: 120
         DURATION: 30

     # Voltage tester malfunction example
     VOLTAGE_TESTER_MALFUNCTIONS:
       - START_TIME: 180
         DURATION: 45
         TESTER: 1
       - START_TIME: 300
         DURATION: 25
         TESTER: 2

     # Vacuum tool malfunction example
     VACUUM_TOOL_MALFUNCTIONS:
       - TOOL: 1
         GRASP_OCCURRENCE: 3
       - TOOL: 2
         GRASP_OCCURRENCE: 5

     # High priority order example
     HIGH_PRIORITY_ORDERS:
       - START_TIME: 240
         ID: "K7R3M8"
       - START_TIME: 420
         ID: "N2F9X5"


------------------
Team Configuration
------------------

The team configuration file specifies team-specific settings including competitor information, conveyor parameters, and sensor configurations.

Core Parameters
===============

.. list-table:: Team Configuration Parameters
   :header-rows: 1
   :widths: 20 15 65
   :class: centered-table

   * - Parameter
     - Type
     - Description
   * - ``COMPETITOR_NAME``
     - string
     - Name of the competing team
   * - ``CONVEYOR_SPEED``
     - float
     - Speed of the inspection conveyor (:math:`0 < \text{speed} \leq 0.1`)
   * - ``CELL_FEED_RATE``
     - float
     - Rate at which cells are fed onto the conveyor (:math:`\text{rate} \leq 2 \cdot \text{speed}`)
   * - ``SENSORS``
     - array
     - List of sensor configurations (see :ref:`sensors page <SENSORS>`)

Sensor Configuration
====================

Each sensor in the SENSORS array requires specific parameters depending on its type. For complete sensor specifications and parameter details, see the :ref:`sensors page <SENSORS>`.


Example
=======

Here is an example team configuration file:

.. _sensors_config_example:

.. code-block:: yaml

   # Team Configuration Example
   COMPETITOR_NAME: "example_team"
   CONVEYOR_SPEED: 0.1 # m/s
   CELL_FEED_RATE: 0.1 # One cell every ten seconds

   SENSORS:
     # Break beam sensor example
     - NAME: "conveyor_break_beam"
       TYPE: "break_beam"
       POSE:
         XYZ: [1.0, 2.0, 0.5]
         RPY: [0, 0, 1.57]
       UPDATE_RATE: 30

     # Distance sensor example
     - NAME: "proximity_sensor"
       TYPE: "distance"
       POSE:
         XYZ: [0.5, 1.5, 0.8]
         RPY: [0, 0, 0]
       UPDATE_RATE: 10

     # RGB camera example
     - NAME: "inspection_camera"
       TYPE: "camera"
       POSE:
         XYZ: [2.0, 1.0, 1.2]
         RPY: [0, 0, 0]
       UPDATE_RATE: 30
       RESOLUTION: "1080p"
       FOV: 1.57

     # Lidar sensor example (must be within bounding boxes)
     - NAME: "inspection_lidar"
       TYPE: "lidar"
       POSE:
         XYZ: [0.8, 0.825, 0.52]  # Within Box A limits
         RPY: [0, 0, 0]
       UPDATE_RATE: 20
       HORIZONTAL:
         SAMPLES: 50
         MIN_ANGLE: -1.57
         MAX_ANGLE: 1.57
       VERTICAL:
         SAMPLES: 8
         MIN_ANGLE: -0.5
         MAX_ANGLE: 0.5
   