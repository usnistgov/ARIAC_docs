.. _AGILITY_CHALLENGES:

==================
Agility Challenges
==================

There are 8 possible :term:`agility challenges<Agility Challenge>`. A description of each challenge is provided below. Challenges can occur multiple times in a given trial. 

.. note::
  A trial may consist of some of the challenges described in this page, may consist of no challenge at all, or may consist of all the challenges.


.. _FAULTY_PARTS:

----------------------
Faulty Parts Challenge
----------------------

Faulty parts are parts that are not suitable for use in the competition. If an order is submitted with faulty parts, the score for that order will be zero. Faulty parts are identified by quality control sensors, which are attached to AGVs. This challenge will only occur for kitting orders. 

The goal of this challenge is to test the ability of the CCS to:

#. Correctly use the quality check sensor to detect a faulty part.

#. Replace the faulty part with a new part.

Faulty Part Detection
=====================

The quality control sensor located above an AGV is capable of detecting faulty parts. A quality check can be performed by calling the service :rosservice:`/ariac/perform_quality_check` (:term:`ariac_msgs/srv/PerformQualityCheck`). The argument passed to this service call is an order ID. When a faulty part is detected, the CCS has to discard the part and replace it with a new part. The new part will automatically be set to non-faulty by the AM.

The service will return a response with the following fields:

  * :yamlname:`valid_id` indicating whether or not the order ID is valid. An order ID is not valid if the order ID does not exist or if the order does not contain a kitting task.

  * :yamlname:`all_passed` is set to :yaml:`true` only if:

    * All parts in the kitting tray are NOT faulty.
    * All parts are present in the kitting tray (no parts from the order are missing).
    * All parts have the correct orientation (no flipped part).
    * All parts are of the correct type.
    * All parts are of the correct color.

  * :yamlname:`incorrect_tray` informs on whether or not the kitting task was performed in the correct kitting tray.

  * Information for each quadrant is reported as a :term:`ariac_msgs/msg/QualityIssue`.


.. _FLIPPED_PARTS:

-----------------------
Flipped Parts Challenge
-----------------------

The environment can be started with parts that are flipped. Flipped parts are parts that are upside down (the z-axis of the part faces down instead of up). When a part is spawned as flipped, the CCS is required to flip this part again so it ends up with the correct orientation. If an order is submitted with flipped parts, the score for that order will be zero. 

The goal of this challenge is to evaluate the approach used by the CCS to flip a part. 

.. note::
  Competitors should keep in mind that one of the two robots can malfunction at any point during the trial. This means that the CCS should be able to handle the case where 
  one of the robots is not available to flip a part.


Flipped Parts Setup
===================

Flipped parts apply to a specific part type and color in a specific bin or on the conveyor belt. To set parts as flipped, the :yamlname:`flipped` field in the trial configuration file must be set as :yaml:`true` for the corresponding parts. :numref:`flipped-parts-in-bin` describes all purple regulators as flipped in :yamlname:`bin3`. :numref:`flipped-parts-on-conveyor-belt` describes all orange batteries as flipped on the conveyor belt.

.. code-block:: yaml
  :caption: Setting flipped parts in a bin.
  :name: flipped-parts-in-bin

  bin3:
    - type: 'regulator'
      color: 'purple'
      slots: [2, 3]
      rotation: 'pi/6'
      flipped: true

.. code-block:: yaml
  :caption: Setting flipped parts on the conveyor belt.
  :name: flipped-parts-on-conveyor-belt
  
  conveyor_belt: 
    active: true
    spawn_rate: 3.0 
    order: 'sequential' 
    parts_to_spawn:
      - type: 'battery'
        color: 'orange'
        number: 5
        offset: 0.5 # between -1 and 1
        flipped: true
        rotation: 'pi/6'


Flipped Part Detection
======================

Flipped parts can be detected in one of two ways.

  * The first way is to use a basic logical camera. The logical camera will report the pose of all parts that are visible by the camera. The pose for flipped parts will indicate that the z-axis is facing down.

  * The second way is to use the quality check service. A quality check informs whether or not a part is flipped. See the :ref:`FAULTY_PARTS` section for more information on how to perform a quality check.


.. _DROPPED_PART_CHALLENGE:
.. _FAULTY_GRIPPER_CHALLENGE:

----------------------
Faulty Gripper Challenge (formerly Dropped Part Challenge)
----------------------

The faulty gripper challenge simulates a faulty gripper which can drop a part after the part has been picked up. The gripper can drop a part at any time during the trial. The gripper can drop a part that is in the gripper's grasp even if the gripper or robot is not moving. 

The goal of this challenge is to test the ability of the CCS to: 
  
  #. Recognize that the part has dropped from the gripper. 
  #. Pick a part of the same type and color.

Faulty Gripper Detection
======================

To detect a faulty gripper the CCS needs a subscriber to the topic :topic:`/ariac/{robot}_gripper_state` (:term:`ariac_msgs/msg/VacuumGripperState`). Checking the :yamlname:`attached` field of the message will inform whether or not the gripper is holding a part. If the gripper is not holding a part, the CCS can assume that the gripper has dropped the part.


.. _ROBOT_MALFUNCTION_CHALLENGE:

---------------------------
Robot Malfunction Challenge
---------------------------

The robot malfunction challenge simulates a robot malfunction. The robot can malfunction under some :ref:`conditions <CONDITIONS>` during the trial. The robot can malfunction even if it is not moving. When a robot malfunctions, it stops moving and cannot be controlled by the CCS. The robot will remain in the same position until the malfunction is resolved. To specify how long a robot malfunctions, a time duration of the malfunction is specified in the trial configuration file.

The goal of this challenge is to test the ability of the CCS to use the other robot to complete the tasks that was being performed by the robot which is malfunctioning. 

.. note::
  It can happen that both robots malfunction at the same time. 
  In this case, the CCS must wait until the malfunction is resolved before continuing with the trial

Robot Malfunction Detection
===========================

To detect a robot malfunction, the CCS needs a subscriber to the topic :topic:`/ariac/robot_health` (:term:`ariac_msgs/msg/Robots`). The message contains Boolean-type fields which provide information on the health of the robots. A value of :yaml:`true` indicates that the robot is healthy and can be controlled by the CCS. A value of :yaml:`false` indicates that the robot is malfunctioning and cannot be controlled by the CCS.


.. _SENSOR_BLACKOUT_CHALLENGE:

-------------------------
Sensor Blackout Challenge
-------------------------

The sensor blackout challenge simulates a situation where some sensors stop reporting data for some duration. 

The goal of this challenge is to test the ability of the CCS to use an internal world model to continue the tasks that were being performed before the blackout.

The sensor blackout challenge is triggered based on :ref:`conditions <CONDITIONS>`. When a *sensor type* is disabled, all sensors of this type stop publishing data on their respective topics. Once the challenge is resolved (after a duration), these sensors will start publishing again. 


Sensor Blackout Detection
=========================

To detect a sensor blackout the CCS needs a subscriber to the topic :topic:`/ariac/sensor_health` (:term:`ariac_msgs/msg/Sensors`). The message contains Boolean-type fields which provide information on the health of each sensor type. A :yaml:`true` value indicates that all sensors of a type are healthy (they are publishing) and a :yaml:`false` value indicates that all sensors of a type are malfunctioning (they are not publishing).


.. _HIGH_PRIORITY_ORDER_CHALLENGE:

-----------------------------
High Priority Order Challenge
-----------------------------

The high-priority orders challenge simulates an order that must be completed before a regular-priority order.

The goal of this challenge is to test the ability of the CCS to prioritize high-priority orders over regular-priority orders. This requires the CCS to  be able to detect when a high-priority order is announced and to switch task. 

.. warning::
  A high-priority order can only be announced using the time or part place :ref:`condition <CONDITIONS>`. The submission condition is not used to announce a high-priority order.

.. note::
  A high-priority order will only be announced when only regular-priority orders have been announced. A high-priority order will not be announced if there is already a high-priority order in the queue.


High Priority Order Detection
=============================

To find out out the priority of an order, the CCS is required to parse messages published to the topic :topic:`/ariac/orders` (:term:`ariac_msgs/msg/Order`). For a high-priority order, the value for the field :yamlname:`priority` is set to :yaml:`true`. For a regular-priority order, the value for the field :yamlname:`priority` is set to :yaml:`false`.


.. _INSUFFICIENT_PARTS_CHALLENGE:

----------------------------
Insufficient Parts Challenge
----------------------------

The insufficient parts challenge simulates a situation where the workcell does not contain enough parts to complete one or multiple orders. 

The goal of this challenge is to test whether or not the CCS is capable of identifying insufficient parts to complete one or multiple orders. When an insufficient parts challenge takes place, the CCS must submit incomplete orders.

Insufficient Parts Detection
============================

To figure out if the insufficient parts challenge is part of a trial, the CCS can rely on two important topics to retrieve part type, color, and quantity from bins and the conveyor belt. If the parts on the bins and expected on the conveyor do not meet the requirements of the order, the CCS can assume that an insufficient part challenge is ongoing. 

* The topic :topic:`/ariac/bin_parts` (:term:`ariac_msgs/msg/BinParts`) publishes the type, color, and the quantity of parts for each of the bins. An output from :console:`ros2 topic echo /ariac/bin_parts` is provided in :numref:`bin-parts-outputs`. The output shows that bin1 contains 3 red pumps and 2 blue batteries.

  .. code-block:: console
    :class: no-copybutton
    :caption: Message published on the topic :topic:`/ariac/bin_parts`.
    :name: bin-parts-outputs

    ---
    bins:
    - bin_number: 1
      parts:
      - part:
          color: 0
          type: 11
        quantity: 3
      - part:
          color: 2
          type: 10
        quantity: 2
    ---

  .. note::
    Bins that do not contain parts are not included in the message.

* The topic :topic:`/ariac/conveyor_parts` (:term:`ariac_msgs/msg/ConveyorParts`) outputs information on parts that are expected to spawn on the conveyor belt. An output from :console:`ros2 topic echo /ariac/conveyor_parts` is provided in  :numref:`conveyor-parts-outputs`. The message shows that 2 red batteries, 2 green sensors, 3 blue regulators, and 1 orange pump will spawn on the conveyor belt.

  .. code-block:: console
    :class: no-copybutton
    :caption: Message published on the topic :topic:`/ariac/conveyor_parts`.
    :name: conveyor-parts-outputs

    ---
    parts:
    - part:
        color: 0
        type: 10
      quantity: 2
    - part:
        color: 1
        type: 12
      quantity: 2
    - part:
        color: 2
        type: 13
      quantity: 3
    - part:
        color: 3
        type: 11
      quantity: 1
    ---