.. _ORDERS:

======
Orders
======

The main task of the competition is to complete orders. After the competition is started, orders will be published on the topic :topic:`/ariac/orders`. Each order will request competitors to complete one of three possible tasks (:ref:`KITTING_TASK`, :ref:`ASSEMBLY_TASK`, or :ref:`COMBINED_TASK`). An order has the following specifications:

  * :yamlname:`id` Each order in a trial has a unique id. The id is an eight character alphanumeric string. 
  
  * :yamlname:`type` The type of task for the order. Three possible values: :yaml:`kitting`, :yaml:`assembly`, or :yaml:`combined`.
  
  * :yamlname:`priority` If the order is a priority order. When set to :yaml:`false` the order is a regular order and when set to :yaml:`true`, the order is of high priority.
  
  * :yamlname:`announcment` Describes when the task manager will announce the order. There are three possible :ref:`CONDITIONS`.


.. _TASKS:

-------------------
Manufacturing Tasks
-------------------


.. _KITTING_TASK:

Kitting Task
============

Kitting is the process of gathering parts into a 'kit'. For a kitting task, the :abbr:`CCS (Competitor Control System)` is expected to:

  1. Locate a tray with the proper tray id, on one of the two kit tray stations. 

  2. Pick, place, and lock the tray onto the specified :abbr:`AGVs (Automated Guided Vehicles)`.

  .. note::
    After placement, the tray should be locked to the AGV using the service :rosservice:`/ariac/agv{n}_lock_tray` to prevent the tray from shifting during transport. 

  3. Place the requested parts onto that kit tray in the specified quadrant.

  4. Perform a quality check using the ROS service to check for faulty parts and fix any issues with the shipment.

  5. Direct the :abbr:`AGV (Automated Guided Vehicle)` to the warehouse.

  6. Submit the order. 

  .. note::
    The AGV must be at the warehouse location before the order can be submitted. This can be checked using the topic :topic:`/ariac/agv{n}_status`.

Kitting Order YAML configuration
--------------------------------

Orders and tasks are described in the trial configuration yaml file. An example of a kitting task can be seen in :numref:`kitting-order`. The kitting task in this example is described as follows:

  - The kit must be built on AGV 2.
  - The kitting tray with id 3 must be used to build the kit.
  - A blue battery must be place in quadrant 1 in the kitting tray.
  - Once the kit is built, the :abbr:`AGV (Automated Guided Vehicle)` must be directed to the warehouse.

.. code-block:: yaml
  :caption: Example of a kitting task description.
  :name: kitting-order

  orders:
    - id: 'MMB30H2'
      type: 'kitting'
      announcement:
        time_condition: 0
      priority: false
      kitting_task:
        agv_number: 2
        tray_id: 3
        destination: 'warehouse'
        products:
          - type: 'battery'
            color: 'blue'
            quadrant: 1


.. _ASSEMBLY_TASK:


Assembly Task
=============

Assembly is the process of installing parts to a fixed :term:`insert<Insert>`. The :abbr:`CCS (Competitor Control System)` can assemble the parts in any order. For a trial where assembly tasks are required, the ARIAC environment starts with parts already located on :abbr:`AGVs (Automated Guided Vehicles)`. The :abbr:`CCS (Competitor Control System)` is expected to:

  #. Lock the :abbr:`AGV (Automated Guided Vehicle)` trays.

  #. Move the :abbr:`AGVs (Automated Guided Vehicles)` to the correct assembly station.

  #. Call the pre-assembly poses service to get exact locations for the parts. 

  #. Use the ceiling robot to pick parts from the AGV and install each part into the insert.

  #. Submit the assembly order.

.. caution::
  This pre-assembly poses service can be called only once for each order ID. The :abbr:`AGVs (Automated Guided Vehicles)` need to be at the correct assembly station for the service to work.


Assembly Order YAML configuration
---------------------------------

An example of an assembly order in a trial configuration file is presented in :numref:`assembly-order` with the following description:

- The assembly should be performed at assembly station 4.

- The parts needed to complete the task are located on AGV 3.

- A purple regulator should be installed at the specified pose (relative to the insert's coordinate frame) in the direction of the specfied unit vector (also relative to the insert frame).

.. code-block:: yaml
  :caption: Example of an assembly task description.
  :name: assembly-order

  - id: 'MMB30H57'
    type: 'assembly'
    announcement:
      time_condition: 0
    priority: false
    assembly_task:
      agv_number: [3]
      station: 'as4'
      products:
        - type: 'regulator'
          color: 'purple'
          assembled_pose: 
            xyz: [0.175, -0.223, 0.215]
            rpy: ['pi/2', 0, '-pi/2']
          assembly_direction: [0, 0, -1]


.. _COMBINED_TASK:

Combined Task
=============

A combined task is a task which requires both kitting and assembly. For a combined task, the :abbr:`CCS (Competitor Control System)` is expected to first perform a kitting task followed with an assembly task. 

.. note::
  The kitting task information is left to the competitors to figure out based on the assembly task information. 
  The CCS can place parts anywhere on :abbr:`AGVs (Automated Guided Vehicles)` and then move those :abbr:`AGVs (Automated Guided Vehicles)` to the station where assembly is to be performed. 
  Once the assembly is complete, the :abbr:`CCS (Competitor Control System)` can submit the assembly via a ROS service call (see :numref:`communications-topics`). 
  The :abbr:`AM (ARIAC Manager)` will then evaluate the submitted assembly for scoring (kitting task is not scored). 


Combined Order YAML configuration
---------------------------------

An example of a combined task in a trial configuration file is presented in :numref:`combined-order`. with the following description:

- The assembly should be performed at assembly station 2.

- A red sensor should be located in the environment and should be installed at the specified pose.

.. code-block:: yaml
  :caption: Example of a combined task description.
  :name: combined-order

  - id: 'MMB30H58'
    type: 'combined'
    announcement:
      time_condition: 0
    priority: false
    combined_task:
      station: 'as2'
      products:
        - type: 'sensor'
          color: 'red'
          assembled_pose: 
            xyz: [-0.1, 0.395, 0.045]
            rpy: [0, 0, '-pi/2']
          assembly_direction: [0, -1, 0]


.. _CONDITIONS:

----------
Conditions
----------

Orders and challenges are announced under three possible conditions:

  * **Time-based condition**: This condition is used to announce an order when the competition time has reached the time provided in the condition. 

    .. note:: 
      The competition time is set when the competitor starts the competition with the service call :rosservice:`/ariac/start_competition`. This is different from the simulation time which is set when Gazebo starts.

   
    For each trial, the first order always uses a time-based condition with the value 0. This ensures the first order is announced as soon as the competitor starts the competition. :numref:`time-based-condition` shows an example of a time-based condition where an ordered is announced at :yaml:`10.5` seconds into the competition.

    .. code-block:: yaml
      :caption: Time-based condition.
      :name: time-based-condition

      announcement:
        time_condition: 10.5

  * **Part placement condition**: When this condition is used, an order or a challenge is announced as soon as a specific part is placed on a specific :abbr:`AGV (Automated Guided Vehicle)`. :numref:`part-placement-condition` shows an example of a part placement condition which announces an order or a challenge when a red pump is placed on AGV 2.

    .. code-block:: yaml
      :caption: Part placement condition.
      :name: part-placement-condition

      announcement:
        part_place_condition:
          agv: 2
          type: 'pump'
          color: 'red'

  * **Submission condition**: When this condition is used, an order or a challenge is announced as soon as another order is submitted. :numref:`submission-condition` shows an example of a submission condition which announces an order or a challenge when the order :yaml:`'MMB30H56'` is submitted.

    .. code-block:: yaml
      :caption: Submission condition.
      :name: submission-condition

      announcement:
        submission_condition:
          order_id: 'MMB30H56'
