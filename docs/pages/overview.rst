Competition Overview
-------------------

Software Environment
~~~~~~~~~~~~~~~~~~~~

* **Operating System**: Ubuntu 24.04 Noble
* **Framework**: ROS Jazzy Jalisco  
* **Simulator**: Gazebo Sim (Harmonic)

Scenario
~~~~~~~~

ARIAC 2025 simulates an **EV battery production factory** with two primary manufacturing tasks:

1. **Inspection and Kit Building** (Task 1)
2. **Module Construction** (Task 2)

The environment includes natural variability and unexpected disruptions to test system agility. Success is measured by speed, accuracy, and flexibility in handling these challenges.

EV Battery Components
~~~~~~~~~~~~~~~~~~~~

Battery Cells
^^^^^^^^^^^^^

Two types of cells are used in production:

* **Lithium Ion (Li-Ion)**: Used for regular kits and modules
* **Nickel Metal Hydride (NiMH)**: Used for high priority orders

Battery Modules
^^^^^^^^^^^^^^^

Battery modules are constructed from four Li-Ion cells combined with:

* **Bottom shell and top shell**: Form the protective casing
* **Metal weld plates**: Enable electrical connections between cells
* **Directional markings**: Indicate positive (+) and negative (-) orientations

Cells alternate direction within modules, and weld beads are located at specific points for electrical connection.

Terminology
-----------

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Term
     - Definition
   * - **Trial**
     - Set of startup conditions including time limit, required kits/modules, defect rate, and challenges
   * - **Run**
     - Single attempt at completing a trial
   * - **Kit**
     - Tray containing four battery cells
   * - **Module**
     - Assembled unit containing four battery cells, bottom/top shells, and electrical welds

Competition States
-----------------

The competition progresses through the following states:

1. **Preparing**: Environment loading, starting controllers, creating sensors
2. **Ready**: Ready for competition to start
3. **Started**: Competition initiated by user
4. **Orders Complete**: All kits, modules, and high priority orders completed
5. **Ended**: Competition ended by user or time limit reached