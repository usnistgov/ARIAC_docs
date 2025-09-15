Sensors
=======

Teams can deploy sensors throughout the environment to gather perception data for inspection, navigation, and task monitoring. Sensors have different grades that affect cost and performance characteristics.

Sensor Types and Specifications
-------------------------------

ARIAC 2025 provides four types of sensors with multiple grade options:

.. list-table:: Sensor Specifications
   :header-rows: 1
   :widths: 15 10 15 15 20 15 10

   * - Sensor Type
     - Grade
     - Update Rate (Hz)
     - Resolution
     - Samples
     - Cost ($)
     - Notes
   * - **Break Beam**
     - A
     - 30
     - --
     - --
     - 400
     - High precision
   * - 
     - B
     - 10
     - --
     - --
     - 200
     - Standard precision
   * - **Distance**
     - A
     - 30
     - --
     - --
     - 600
     - High update rate
   * - 
     - B
     - 10
     - --
     - --
     - 300
     - Standard rate
   * - **Camera**
     - A
     - 30
     - 1080p
     - --
     - 800
     - High resolution
   * - 
     - B
     - 30
     - 720p
     - --
     - 500
     - Standard resolution
   * - **LiDAR**
     - A
     - 20
     - --
     - H × V ≤ 400
     - 1500
     - Highest quality
   * - 
     - B
     - 10
     - --
     - 200 < H × V ≤ 400
     - 1250
     - Medium quality
   * - 
     - C
     - 10
     - --
     - H × V ≤ 200
     - 1000
     - Basic quality

*H refers to horizontal scan samples and V refers to vertical scan samples*

Break Beam Sensors
------------------

Break beam sensors detect when objects interrupt a light beam between transmitter and receiver.

Applications
~~~~~~~~~~~~

* **Conveyor Monitoring**: Detect cell arrival at inspection stations
* **Bin Detection**: Monitor when cells are deposited in collection bins
* **AGV Loading**: Confirm cell placement on AGV trays
* **Safety Systems**: Emergency stops for robot collision avoidance

Specifications
~~~~~~~~~~~~~

* **Detection Method**: Infrared light beam interruption
* **Range**: Typically 0.1m to 2.0m beam length
* **Response Time**: Near-instantaneous detection
* **Environmental Tolerance**: Robust to lighting conditions

Distance Sensors
----------------

Distance sensors measure the range to the nearest object in their field of view.

Applications
~~~~~~~~~~~~

* **Proximity Detection**: Measure distance to cells on conveyor
* **Positioning Feedback**: Verify robot end-effector positioning
* **Clearance Monitoring**: Ensure safe robot movement paths
* **Quality Control**: Detect dimensional variations in components

Specifications
~~~~~~~~~~~~~

* **Measurement Method**: Time-of-flight or ultrasonic
* **Range**: Typically 0.05m to 5.0m
* **Accuracy**: ±1-2mm depending on grade
* **Field of View**: Narrow beam for precise measurements

Camera Sensors
--------------

Camera sensors provide visual data for inspection and recognition tasks.

Applications
~~~~~~~~~~~~

* **Defect Detection**: Visual inspection of battery cell surfaces
* **Orientation Recognition**: Determine cell positive/negative orientation
* **Quality Assessment**: Identify scratches, dents, or other defects
* **Robot Guidance**: Visual servoing for precise manipulation

Specifications
~~~~~~~~~~~~~

* **Image Format**: Standard ROS sensor_msgs/Image
* **Color Space**: RGB or grayscale options
* **Lens Options**: Fixed focus or adjustable
* **Lighting**: Requires adequate illumination for quality images

Grade Differences
~~~~~~~~~~~~~~~~

**Grade A (1080p)**:
* Resolution: 1920 × 1080 pixels
* Better detail for defect detection
* Higher processing requirements

**Grade B (720p)**:
* Resolution: 1280 × 720 pixels
* Sufficient for basic inspection
* Lower computational overhead

LiDAR Sensors
-------------

LiDAR sensors generate 3D point clouds for detailed geometric analysis.

Applications
~~~~~~~~~~~~

* **3D Reconstruction**: Build detailed models of battery cells
* **Defect Localization**: Precisely locate and characterize defects
* **Collision Avoidance**: Create 3D maps for safe robot navigation
* **Quality Metrics**: Measure dimensional accuracy and surface quality

Specifications
~~~~~~~~~~~~~

* **Output Format**: ROS sensor_msgs/PointCloud2
* **Measurement Principle**: Laser time-of-flight
* **Accuracy**: Sub-millimeter precision (grade dependent)
* **Coverage**: Configurable horizontal and vertical scan patterns

Grade Comparison
~~~~~~~~~~~~~~~

**Grade A**: 
* Up to 400 total scan points (H × V ≤ 400)
* 20 Hz update rate
* Highest point density and accuracy
* Best for detailed defect analysis

**Grade B**:
* 201-400 total scan points (200 < H × V ≤ 400)
* 10 Hz update rate
* Good balance of quality and cost
* Suitable for most inspection tasks

**Grade C**:
* Up to 200 total scan points (H × V ≤ 200)
* 10 Hz update rate
* Basic 3D sensing capability
* Cost-effective for simple applications

Sensor Placement Strategy
------------------------

Inspection Conveyor Zone
~~~~~~~~~~~~~~~~~~~~~~~

Recommended sensor placement around the inspection conveyor:

* **Multiple LiDAR sensors**: Position at different angles for complete cell coverage
* **Break beam sensors**: Detect cell arrival and departure from inspection zone
* **Distance sensors**: Monitor conveyor surface and cell positioning

Coverage Zones
^^^^^^^^^^^^^

.. code-block:: text

   Conveyor Direction →
   
   Zone 1: Entry detection (break beam)
   Zone 2: Primary inspection (LiDAR + camera)
   Zone 3: Secondary inspection (LiDAR)
   Zone 4: Exit detection (break beam)

AGV Stations
~~~~~~~~~~~~

Monitor AGV operations at each station:

* **Camera sensors**: Verify cell placement in tray slots
* **Break beam arrays**: Detect tray loading/unloading
* **Distance sensors**: Measure AGV positioning accuracy

Assembly Area
~~~~~~~~~~~~

Track module construction progress:

* **Cameras**: Monitor cell installation and shell placement
* **Distance sensors**: Verify component positioning
* **Break beams**: Detect module movement on conveyor sections

Sensor Budget Management
-----------------------

Teams must balance sensor capability with cost constraints. The total sensor budget affects scoring through the Sensor Cost Bonus.

Cost Optimization Strategies
~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. **Prioritize Critical Areas**: Focus high-grade sensors on inspection zones
2. **Grade Selection**: Choose appropriate sensor grades for each application
3. **Sensor Fusion**: Combine multiple lower-cost sensors instead of fewer expensive ones
4. **Strategic Placement**: Position sensors for maximum coverage overlap

Budget Planning
~~~~~~~~~~~~~~

Example sensor allocation for $5000 budget:

.. code-block:: text

   Inspection Zone:
   - 2 × LiDAR Grade B = $2500
   - 4 × Break Beam Grade A = $1600
   - 2 × Camera Grade B = $1000
   Total: $5100 (over budget by $100)
   
   Alternative:
   - 1 × LiDAR Grade A = $1500
   - 1 × LiDAR Grade C = $1000  
   - 4 × Break Beam Grade B = $800
   - 2 × Camera Grade B = $1000
   - 2 × Distance Grade B = $600
   Total: $4900 (under budget by $100)

Sensor Interfaces
----------------

All sensors publish data to ROS topics with standardized message formats.

Break Beam Interfaces
~~~~~~~~~~~~~~~~~~~~

.. list-table:: Break Beam Topics
   :header-rows: 1
   :widths: 40 30 30

   * - Topic
     - Type
     - Description
   * - ``/{break_beam_name}/status``
     - ``ariac_interfaces/msg/BreakBeamStatus``
     - Current detection state with timestamp
   * - ``/{break_beam_name}/change``
     - ``ariac_interfaces/msg/BreakBeamStatus``
     - State change notifications only

**BreakBeamStatus Message:**

* ``detected``: Boolean indicating object presence
* ``timestamp``: Time of measurement
* ``sensor_id``: Unique sensor identifier

Distance Sensor Interfaces
~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: Distance Sensor Topics
   :header-rows: 1
   :widths: 40 30 30

   * - Topic
     - Type
     - Description
   * - ``/{distance_sensor_name}/distance``
     - ``ariac_interfaces/msg/DistanceSensor``
     - Range measurement with timestamp

**DistanceSensor Message:**

* ``distance``: Range to nearest object (meters)
* ``timestamp``: Time of measurement
* ``min_range``: Minimum detectable distance
* ``max_range``: Maximum detectable distance

Camera Interfaces
~~~~~~~~~~~~~~~~

.. list-table:: Camera Topics
   :header-rows: 1
   :widths: 35 30 35

   * - Topic
     - Type
     - Description
   * - ``/{camera_name}/image``
     - ``sensor_msgs/msg/Image``
     - Raw image data
   * - ``/{camera_name}/info``
     - ``sensor_msgs/msg/CameraInfo``
     - Camera calibration parameters

**Image Processing:**

* Standard ROS image formats (RGB8, MONO8, etc.)
* Use cv_bridge for OpenCV integration
* Camera info provides intrinsic parameters for 3D reconstruction

LiDAR Interfaces
~~~~~~~~~~~~~~~

.. list-table:: LiDAR Topics
   :header-rows: 1
   :widths: 35 30 35

   * - Topic
     - Type
     - Description
   * - ``/{lidar_name}/scan``
     - ``sensor_msgs/msg/PointCloud2``
     - 3D point cloud data

**Point Cloud Processing:**

* Points in sensor coordinate frame
* Includes intensity information
* Use PCL (Point Cloud Library) for processing
* Transform to world coordinates as needed

Sensor Data Processing
---------------------

Defect Detection Pipeline
~~~~~~~~~~~~~~~~~~~~~~~

1. **Data Acquisition**: Collect sensor readings during cell inspection
2. **Preprocessing**: Filter noise and align coordinate frames
3. **Feature Extraction**: Identify potential defect regions
4. **Classification**: Determine defect type and severity
5. **Localization**: Calculate precise defect coordinates
6. **Reporting**: Submit inspection results via ROS service

Example Processing Steps
~~~~~~~~~~~~~~~~~~~~~~

**Point Cloud Analysis:**

.. code-block:: python

   # Pseudocode for LiDAR-based defect detection
   def process_point_cloud(cloud):
       # Remove outliers and noise
       filtered_cloud = statistical_outlier_removal(cloud)
       
       # Segment cell surface
       cell_surface = plane_segmentation(filtered_cloud)
       
       # Detect surface anomalies
       defects = surface_analysis(cell_surface)
       
       # Classify defect types
       classified_defects = defect_classification(defects)
       
       return classified_defects

**Image Processing:**

.. code-block:: python

   # Pseudocode for camera-based inspection
   def process_image(image):
       # Enhance contrast and reduce noise
       processed_image = image_enhancement(image)
       
       # Detect edges and features
       features = edge_detection(processed_image)
       
       # Identify defect patterns
       defects = pattern_matching(features)
       
       return defects

Sensor Fusion
~~~~~~~~~~~~

Combine multiple sensor modalities for improved accuracy:

* **LiDAR + Camera**: 3D geometry with visual texture information
* **Multiple LiDAR**: Different viewing angles for complete coverage
* **Break Beam + Distance**: Presence detection with precise positioning
* **Temporal Fusion**: Combine measurements over time for noise reduction

Performance Optimization
-----------------------

Real-time Processing
~~~~~~~~~~~~~~~~~~

* **Parallel Processing**: Use multiple CPU cores for sensor data processing
* **GPU Acceleration**: Leverage CUDA for intensive image/point cloud operations  
* **Efficient Algorithms**: Choose algorithms suitable for real-time constraints
* **Data Reduction**: Process only relevant sensor data regions

Memory Management
~~~~~~~~~~~~~~~~

* **Streaming Processing**: Process data as it arrives rather than buffering
* **Circular Buffers**: Manage memory for continuous sensor streams
* **Selective Storage**: Keep only essential data for decision making
* **Garbage Collection**: Properly dispose of processed sensor data

Quality Assurance
-----------------

Sensor Calibration
~~~~~~~~~~~~~~~~~

* **Geometric Calibration**: Ensure accurate spatial measurements
* **Temporal Synchronization**: Align timestamps across sensors
* **Cross-Sensor Validation**: Verify consistency between sensor modalities
* **Regular Recalibration**: Account for sensor drift during competition

Validation Metrics
~~~~~~~~~~~~~~~~~

* **Detection Rate**: Percentage of defects correctly identified
* **False Positive Rate**: Incorrect defect classifications
* **Localization Accuracy**: Precision of defect position measurements
* **Processing Latency**: Time from sensor acquisition to decision output