.. _SENSORS:

=======
Sensors
=======

In order to complete the ARIAC 2025 competition, competitors will be allowed to use various sensors to gather information about the environment. The sensors being made available
to competitiors includes:

* Break Beam
* Distance Sensor
* RGB Camera
* Lidar Sensor

Competitors are given a budget of $7000. Going over this budget will incur a penalty, but staying under will reward bonus points. More information about the associated bonus and 
penalty can be found on the :ref:`evaluation page <EVALUATION>`.

Break beams, distance sensors, and RGB camera's can be placed wherever the competitor decides to have them in the environment, but we have provided bounding boxes for the lidar 
sensors. The lidar sensors which are used must remain insdie of this bounding box for the competition to proceed.

We will provide multiple tiers for each sensor, which includes an associated cost. The various sensors with their variants are explained further below.

**Break Beam**

The break beam sensor reports when a beam is broken by an object. It does not provide distance data. The break beam will be offered with two variations.

:ref:`The ROS topics used for the break beam can be found in the ROS API <break_beam_anchor>`.

**Distance Sensor**

The distance sensor will report how far an object is from the sensor when an object is detected. The distance sensor will be offered with two variations.

:ref:`The ROS topics used for the distance sensor can be found in the ROS API <distance_anchor>`.

**RGB Camera**

The RGB camera provides an RGB image. The camera will be offered with two variations.

:ref:`The ROS topics used for the RGB camera can be found in the ROS API <camera_anchor>`.

**Lidar Sensor**

The lidar sensor returns a point cloud of detected objects. The lidar will be offered with three variations. 

:ref:`The ROS topics used for the lidar sensor can be found in the ROS API <lidar_anchor>`.


**Sensor Summary**

.. table:: Sensor Specifications
   :class: centered-table

   +-------------------+-----------------+----------------+------------+--------------------+-----------------+
   |   Sensor Type     |  Grade          |Update Rate (Hz)| Resolution | Samples            |  Cost ($)       |
   +-------------------+-----------------+----------------+------------+--------------------+-----------------+
   | Break Beam        | A               |  30            | ---        |  ---               |   400           |
   +                   +-----------------+----------------+------------+--------------------+-----------------+
   |                   | B               |  10            | ---        |  ---               |   200           |
   +-------------------+-----------------+----------------+------------+--------------------+-----------------+
   | Distance Sensor   | A               |  30            | ---        |  ---               |   600           |
   +                   +-----------------+----------------+------------+--------------------+-----------------+
   |                   | B               |  10            | ---        |  ---               |   300           |
   +-------------------+-----------------+----------------+------------+--------------------+-----------------+
   | RGB Camera        | A               |  30            | 1080 p     |  ---               |   800           |
   +                   +-----------------+----------------+------------+--------------------+-----------------+
   |                   | B               |  30            | 720 p      |  ---               |   500           |
   +-------------------+-----------------+----------------+------------+--------------------+-----------------+
   | Lidar Sensor      | A               |  20            | ---        |  :math:`H*V<=400`  |   1500          |
   +                   +-----------------+----------------+------------+--------------------+-----------------+
   |                   | B               |  10            | ---        |:math:`200<H*V<=400`|   1250          |
   +                   +-----------------+----------------+------------+--------------------+-----------------+
   |                   | C               |  10            | ---        |  :math:`H*V<=200`  |   1000          |
   +-------------------+-----------------+----------------+------------+--------------------+-----------------+

** H refers to horizontal samples and V refers to vertical samples

