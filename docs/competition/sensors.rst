.. _SENSOR_OVERVIEW:

===============
Sensor Overview
===============

In order to complete this year's ARIAC competition, we will be allowing competitors to use various sensors to gather information about the environment. The sensors being made available
to competitiors includes:

* Break Beam
* Distance Sensor
* RGB Camera
* Lidar Sensor

Competitors are given a budget of $7000. Going over this budget will incur a penalty, but staying under will reward bonus points. More information about the associated bonus and 
penalty can be found on the :ref:`scoring page <scoring-anchor>`.

Break beams, distance sensors, and RGB camera's can be placed wherever the competitor decides to have them in the environment, but we have provided bounding boxes for the lidar 
sensors. The lidar sensors which are used must remain insdie of this bounding box for the competition to proceed.

We will provide multiple tiers for each sensor, which includes an associated cost. The various sensors with their variants are explained further below.

**Break Beam**

The break beam sensor reports when a beam is broken by an object. It does not provide distance data. The break beam will be offered with two variations, one which has an update 
rate of 10 Hz and one which has an update rate of 30 Hz.

:ref:`The message definition for the break beam can be found here <break-beam-anchor>`.

**Distance Sensor**

The distance sensor will report how far an object is from the sensor when an object is detected. The distance sensor will be offered with two variations, one which has an update 
rate of 10 Hz and one which has an update rate of 30 Hz.

:ref:`The message definition for the distance sensor can be found here <distance-sensor-anchor>`.

**RGB Camera**

The RGB camera provides an RGB image. The camera will be offered with two variations, one which has a resolution of 720p and one which has a resolution of 1080p.

**Lidar Sensor**

The lidar sensor returns a point cloud of detected objects. The lidar will be offered with three variations, explained in the table below:

.. table:: Lidar Specifications
   :class: centered-table

   +----------+-----------------+----------------+
   |          |  Samples**      |  Update Rate   |
   +----------+-----------------+----------------+
   | Low      | :math:`H*V<200` |  10 Hz         |
   +----------+-----------------+----------------+
   | Medium   | :math:`H*V>=200`|  10 Hz         |
   +----------+-----------------+----------------+
   | High     | :math:`H*V<400` |  20 Hz         |
   +----------+-----------------+----------------+

** H refers to horizontal samples and V refers to vertical samples

**Sensor Cost**

.. table:: Sensor Cost
   :class: centered-table

   +-------------------+-----------------+----------------+------------+
   |                   |  Low            |  Medium        | High       |
   +-------------------+-----------------+----------------+------------+
   | Break Beam        | $200            |  ---           | $400       |
   +-------------------+-----------------+----------------+------------+
   | Distance Sensor   | $300            |  ---           | $600       |
   +-------------------+-----------------+----------------+------------+
   | RGB Camera        | $500            |  ---           | $800       |
   +-------------------+-----------------+----------------+------------+
   | Lidar             | $1,000          | $1,250         | $1,500     |
   +-------------------+-----------------+----------------+------------+
