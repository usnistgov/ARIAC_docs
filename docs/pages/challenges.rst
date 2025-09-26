.. _CHALLENGES:

==========
Challenges
==========

The agility challenges test teams' ability to adapt to unexpected situations and system failures during competition runs. Successfully handling these challenges is crucial for maintaining high performance and achieving competitive scores.

The competition incorporates challenges to test the robustness of team systems. Teams should be able to recognize when a challenge is occurring and properly handle the situation. There are four possible challenges that can occur during a run:

* **Conveyor Malfunction** - Inspection conveyor stops operating
* **Voltage Tester Malfunction** - One or both voltage testers stop providing data
* **Vacuum Tool Malfunction** - Vacuum gripper fails to grasp objects
* **High Priority Order** - Urgent kit request with time constraints

Conveyor Malfunction
====================

When the conveyor malfunction challenge occurs, the inspection conveyor will pause its motion. In addition, the cell feed will pause, so no new cells can be added during the challenge.

.. list-table:: Conveyor Malfunction Parameters
   :header-rows: 1
   :widths: 20 20 60
   :class: centered-table

   * - Parameter
     - Type
     - Description
   * - ``START_TIME``
     - int
     - Time in seconds when the malfunction begins (minimum: 0)
   * - ``DURATION``
     - int
     - Duration in seconds the malfunction lasts (minimum: 1)

Teams are expected to handle this challenge by:

1. **Detecting the malfunction** by monitoring the conveyor status topic (:ref:`reference <inspection_challenge_anchor>`)
2. **Pausing the inspection system** to avoid conflicts
3. **Waiting for recovery** by monitoring the status topic until it reports operational status
4. **Resuming operations** once the conveyor and cell feed return to normal

Voltage Tester Malfunction
==========================

When the voltage tester malfunction occurs, one or both voltage testers will stop publishing data.

.. list-table:: Voltage Tester Malfunction Parameters
   :header-rows: 1
   :widths: 20 20 60
   :class: centered-table

   * - Parameter
     - Type
     - Description
   * - ``START_TIME``
     - int
     - Time in seconds when the malfunction begins
   * - ``DURATION``
     - int
     - Duration in seconds the malfunction lasts
   * - ``TESTER``
     - int
     - Which voltage tester is affected (1 or 2)

Teams are expected to handle this challenge by:

1. **Detecting the malfunction** by monitoring voltage tester topics for data availability (:ref:`reference <inspection_challenge_anchor>`)
2. **Adapting operations** to avoid using the affected voltage tester(s)
3. **Monitoring for recovery** by watching the status topic until operational status is restored
4. **Continuing progress** using any operational voltage testers to maintain productivity
5. **Resuming full operations** once all voltage testers return to operational status

Vacuum Tool Malfunction
=======================

When the vacuum tool malfunction occurs, a specified vacuum gripper will fail during grasp attempts.

.. list-table:: Vacuum Tool Malfunction Parameters
   :header-rows: 1
   :widths: 20 20 60
   :class: centered-table

   * - Parameter
     - Type
     - Description
   * - ``TOOL``
     - int
     - Which vacuum tool is affected (1 or 2)
   * - ``GRASP_OCCURRENCE``
     - int
     - Which grasp attempt will fail (minimum: 1)

.. note::

  **TOOL**: Refers to the :ref:`vacuum gripper tools <vacuumtools_msg>` VG_2 and VG_4 available to assembly robot 2.

.. note::

  **GRASP_OCCURRENCE**: Specifies which attempt at grasping will fail. For example, if set to 3, the first two grasp attempts will succeed normally, but the third attempt will fail and require retry.

Teams are expected to handle this challenge by:

1. **Detecting the failure** by monitoring the response from the grasp service (:ref:`reference <vacuum_tool_challenge_anchor>`)
2. **Repositioning the gripper** by moving it away from the target object
3. **Retrying the grasp** with proper positioning and approach

High Priority Order
===================

When a high priority order is requested, an internal timer starts tracking completion time. Teams should minimize the time taken to submit this urgent kit request.

.. list-table:: High Priority Order Parameters
   :header-rows: 1
   :widths: 20 20 60
   :class: centered-table

   * - Parameter
     - Type
     - Description
   * - ``START_TIME``
     - int
     - Time in seconds when the high priority order is announced
   * - ``ID``
     - string
     - Unique identifier for the high priority order

Teams are expected to handle this challenge by:

1. **Detecting the request** by monitoring the high priority order topic (:ref:`reference <high-priority-anchor>`)
2. **Switching cell feed** to begin feeding NiMH cells
3. **Building the kit** using four NiMH cells with proper voltage specifications
4. **Delivering the kit** by moving the AGV to the shipping location
5. **Submitting the order** using the high priority submission service with the correct order ID (:ref:`reference <high-priority-anchor>`)
6. **Restoring normal operations** by switching cell feed back to Li-ion batteries
7. **Resuming standard tasks** to continue regular production

Configuration Example
======================

For complete challenge configuration examples showing all challenge types with valid parameters, see the :ref:`Challenges Configuration Reference <challenges_config_example>`.