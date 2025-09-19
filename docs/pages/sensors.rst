.. _SENSORS:

=======
Sensors
=======

Teams are allowed to use sensors to gather information about the environment during competition trials. Sensors can be strategically placed throughout the workspace to inspect cells for defects, monitor conveyor operations, detect object presence, and guide robotic manipulation. The sensor data helps teams make informed decisions about task execution, object handling, and navigation.

Teams are given a budget for their total sensor cost. Each sensor is assigned a grade based on its parameters, and the grade determines its cost. Going over budget will incur a penalty, but staying under will reward bonus points. More information about the associated bonus and penalty can be found on the :ref:`evaluation page <EVALUATION>`.

.. _COMMON_PARAMETERS:

Common Parameters
=================

All sensors share the following common configuration parameters:

.. list-table:: Common Sensor Parameters
   :header-rows: 1
   :widths: 20 20 60
   :class: centered-table

   * - Parameter
     - Type
     - Description
   * - ``NAME``
     - string
     - Unique identifier for the sensor
   * - ``TYPE``
     - string
     - Sensor type ("break_beam", "distance", "camera", or "lidar")
   * - ``POSE.XYZ``
     - [float, float, float]
     - Position coordinates [x, y, z]
   * - ``POSE.RPY``
     - [float, float, float]
     - Orientation angles [roll, pitch, yaw]
   * - ``UPDATE_RATE``
     - int
     - Frequency of sensor updates (1-30 Hz)



----------
Break Beam
----------

The break beam sensor detects when an object interrupts an infrared beam between a transmitter and receiver. It reports binary state information (beam broken/unbroken) and does not provide distance measurements. This sensor is useful for detecting object presence at specific points, such as monitoring conveyor belts. It publishes to these :ref:`topics <break_beam_anchor>`.

.. figure:: /_static/images/break_beam_sensor.png
  :width: 30%

  Break beam sensor

Break beam sensors use only the :ref:`common parameters <COMMON_PARAMETERS>` with ``TYPE`` set to "break_beam".

.. list-table:: Cost and Grades
   :header-rows: 1
   :widths: 20 25 25 30
   :class: centered-table

   * - Grade
     - Update Rate (Hz)
     - Cost ($)
     - Notes
   * - A
     - 30
     - 400
     - High-frequency detection
   * - B
     - 10
     - 200
     - Standard detection

---------------
Distance Sensor
---------------

The distance sensor measures the distance to the nearest object in its detection range using ultrasonic or infrared technology. It continuously scans forward and reports distance measurements in meters. This sensor is useful for collision avoidance, proximity detection, and measuring clearances between objects. It publishes to these :ref:`topics <distance_anchor>`.

.. figure:: /_static/images/distance_sensor.png
  :width: 30%

  Distance sensor

Distance sensors use only the :ref:`common parameters <COMMON_PARAMETERS>` with ``TYPE`` set to "distance".

.. list-table:: Cost and Grades
   :header-rows: 1
   :widths: 20 25 25 30
   :class: centered-table

   * - Grade
     - Update Rate (Hz)
     - Cost ($)
     - Notes
   * - A
     - 30
     - 600
     - High-frequency ranging
   * - B
     - 10
     - 300
     - Standard ranging

----------
RGB Camera
----------

The RGB camera captures color images of the environment for object detection, identification, and scene analysis. It supports 720p and 1080p resolutions with configurable field of view settings. This sensor can be used for visual recognition, quality inspection, and monitoring tasks. It publishes to these :ref:`topics <camera_anchor>`.

.. figure:: /_static/images/camera.png
  :width: 30%

  RGB camera

RGB cameras use the :ref:`common parameters <COMMON_PARAMETERS>` with ``TYPE`` set to "camera", plus the following additional parameters:


.. list-table:: RGB Camera Additional Parameters
   :header-rows: 1
   :widths: 20 20 60
   :class: centered-table

   * - Parameter
     - Type
     - Description
   * - ``RESOLUTION``
     - string
     - Image resolution ("720p" or "1080p")
   * - ``FOV``
     - float
     - Field of view (0 to :math:`\pi` radians)

.. list-table:: Cost and Grades
   :header-rows: 1
   :widths: 20 25 20 25 10
   :class: centered-table

   * - Grade
     - Update Rate (Hz)
     - Resolution
     - Cost ($)
     - Notes
   * - A
     - 30
     - 1080p
     - 800
     - High resolution
   * - B
     - 30
     - 720p
     - 500
     - Standard resolution

-----
Lidar
-----

The lidar sensor uses laser pulses to create 3D point clouds of the surrounding environment. It performs horizontal and vertical scans with configurable sample rates and angular ranges. This sensor provides precise distance measurements, 3D mapping, and obstacle detection for navigation and spatial analysis. It publishes to these :ref:`topics <lidar_anchor>`.

.. figure:: /_static/images/lidar_sensor.png
  :width: 30%

  Lidar

Lidar sensors use the :ref:`common parameters <COMMON_PARAMETERS>` with ``TYPE`` set to "lidar", plus the following additional parameters:


.. list-table:: Lidar Sensor Additional Parameters
   :header-rows: 1
   :widths: 20 20 60
   :class: centered-table

   * - Parameter
     - Type
     - Description

   * - ``HORIZONTAL.SAMPLES``
     - int
     - Number of samples in the horizontal scan
   * - ``HORIZONTAL.MIN_ANGLE``
     - float
     - Minimum angle for horizontal scan
   * - ``HORIZONTAL.MAX_ANGLE``
     - float
     - Maximum angle for horizontal scan
   * - ``VERTICAL.SAMPLES``
     - int
     - Number of samples in the vertical scan
   * - ``VERTICAL.MIN_ANGLE``
     - float
     - Minimum angle for vertical scan
   * - ``VERTICAL.MAX_ANGLE``
     - float
     - Maximum angle for vertical scan

.. list-table:: Cost and Grades
   :header-rows: 1
   :widths: 20 25 35 20
   :class: centered-table

   * - Grade
     - Update Rate (Hz)
     - Sample Limit
     - Cost ($)
   * - A
     - 20
     - :math:`H \cdot V \leq 400`
     - 1500
   * - B
     - 10
     - :math:`200 < H \cdot V \leq 400`
     - 1250
   * - C
     - 10
     - :math:`H \cdot V \leq 200`
     - 1000

.. note::

  For lidar sensors: :math:`H` refers to horizontal samples and :math:`V` refers to vertical samples


Bounding Boxes
==============

Lidar sensors are restricted to physical inspection of battery cells and must be placed within designated bounding boxes around the inspection conveyor. The following table shows the coordinate limits for each available placement area. 

.. list-table::
   :header-rows: 1
   :widths: 25 25 25 25
   :class: centered-table

   * -
     - Box A
     - Box B
     - Box C
   * - X (min, max)
     - (0.5, 1.1)
     - (0.5, 1.1)
     - (0.5, 1.1)
   * - Y (min, max)
     - (0.7, 0.95)
     - (1.05, 1.3)
     - (0.95, 1.05)
   * - Z (min, max)
     - (0.42, 0.62)
     - (0.42, 0.62)
     - (0.52, 0.62)

.. figure:: /_static/images/bounding_box.png
  :width: 50%

  Visualization of the bounding boxes

Configuration Example
=====================

Here is an example YAML configuration showing all sensor types with valid parameters:

.. code-block:: yaml

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
        RPY: [0, 0.5, 0]
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