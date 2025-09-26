.. _DEMOS:

=====
Demos
=====

This page provides instructions for running ARIAC demo scenarios to help you get familiar with the competition environment. The demos can be run either through the ARIAC App or directly from the terminal.

.. note::

   Before running any demos, ensure you have followed the setup instructions on the :ref:`startup page <STARTUP>`.

--------------------
Running with the App
--------------------

The ARIAC App provides an interface for configuring and running trials.

Configuration
=============

The home page of the ARIAC App is where you configure all the settings needed for a competition run. This configuration page allows you to set up trials, team parameters, database storage, and testing options before starting a run:

.. list-table::
   :header-rows: 1
   :widths: 25 75
   :class: centered-table
   :width: 80%

   * - Setting
     - Description
   * - Trial Config
     - Defines the competition scenario and tasks
   * - Team Config
     - Specifies robot configurations and team settings
   * - Database
     - SQLite database for storing results and scoring
   * - Cheats
     - Testing shortcuts to start at different competition stages

Trial Configuration
-------------------

The trial config defines the competition scenario. For demos, use the pregenerated config file:

.. figure:: /_static/images/select_trial_config.gif
  :width: 60%
  :align: center

**Steps:**

1. Click the "Select" button in the Trial section
2. Navigate to the trial config file using the dialog
3. Use the arrow button (top right) to go up directories if needed
4. Select the appropriate file based on your setup (see table below)

.. list-table::
   :header-rows: 1
   :widths: 20 80
   :class: centered-table
   :width: 80%

   * - Setup Type
     - File Path
   * - Docker
     - ``/team_ws/src/example_team/config/trials/LHAF9835.yaml``
   * - Local
     - ``~/ariac_ws/src/ariac_example_team/example_team/config/trials/LHAF9835.yaml``

Team Configuration
------------------

The team config specifies robot settings and team parameters:

.. figure:: /_static/images/select_team_config.gif
  :width: 60%
  :align: center

**Steps:**

1. Click the "Select" button in the Team Config section
2. Navigate to the team config file using the dialog
3. Use the arrow button (top right) to go up directories if needed
4. Select the appropriate file based on your setup (see table below)

.. list-table::
   :header-rows: 1
   :widths: 20 80
   :class: centered-table
   :width: 80%

   * - Setup Type
     - File Path
   * - Docker
     - ``/team_ws/src/example_team/config/example_team_config.yaml``
   * - Local
     - ``~/ariac_ws/src/ariac_example_team/example_team/config/example_team_config.yaml``

Database Setup
--------------

The ARIAC database stores scoring information and results. While optional for running the environment, it's required for viewing demo results.

.. figure:: /_static/images/create_db.gif
  :width: 60%
  :align: center

**Steps to create a database:**

1. Click the "Create" button in the Database section
2. Navigate to your desired parent directory using the file dialog
3. Select the target directory where you want to save the database
4. Click "OK" to create the database

.. note::

   The database location doesn't affect functionality - choose any convenient location.

Cheats
------

For testing purposes, cheats have been included to start the environment at different stages of the competition:

.. list-table::
   :header-rows: 1
   :widths: 10 35 55
   :class: centered-table
   :width: 80%

   * - Number
     - Cheat Option
     - Description
   * - 1
     - Cells in voltage testers
     - Pre-places cells in voltage testing stations
   * - 2
     - Kit on AGV
     - Places a complete kit on AGV1
   * - 3
     - Kits on AGVs
     - Places kits on multiple AGVs
   * - 4
     - High priority kit
     - Creates a high-priority kit scenario
   * - 5
     - Partial module
     - Starts with partially assembled modules
   * - 6
     - Module
     - Provides complete modules
   * - 7
     - Flipped module
     - Creates flipped module scenarios
   * - 8
     - Module with welds
     - Includes welded module scenarios

Running the Demos
=================

Dummy Inspection Demo
---------------------

.. figure:: /_static/images/dummy_inspection.gif
  :width: 60%
  :align: center

**Overview:** Demonstrates inspection conveyor control and sensor-based reporting by controlling the cell feed on the inspection conveyor and submitting dummy inspection reports using break-beam sensor data.

**Steps to run:**

1. Configure trial and team configs (see sections above)
2. No cheat selection required
3. Click "Confirm" to open the run page
4. Press the green "START RUN" button
5. In the "Team Process" section:

   - Package dropdown: Select "example_team"
   - File dropdown: Select "dummy_inspection"

6. Press the arrow button next to the generated command

Move Robots Demo
----------------

.. figure:: /_static/images/move_robots.gif
  :width: 60%
  :align: center

**Overview:** Demonstrates basic robot motion control across multiple robots by creating MoveIt nodes for all five robots and executing simultaneous linear motions.

**Steps to run:**

1. Configure trial and team configs
2. No cheat selection required
3. Click "Confirm" to open the run page
4. Press the green "START RUN" button
5. In the "Team Process" section:

   - Package dropdown: Select "example_team"
   - File dropdown: Select "move_robots"

6. Press the arrow button next to the generated command

Pick from Voltage Tester Demo
-----------------------------

.. figure:: /_static/images/pick_from_tester.gif
  :width: 60%
  :align: center

**Overview:** Demonstrates cell manipulation and recycling operations by using inspection robot 2 to pick a cell from the voltage tester and drop it in the recycling bin.

**Steps to run:**

1. Configure trial and team configs
2. **Cheat Selection:** Select "Cells in voltage testers" from the dropdown
3. Click "Confirm" to open the run page
4. Press the green "START RUN" button
5. In the "Team Process" section:

   - Package dropdown: Select "example_team"
   - File dropdown: Select "pick_from_tester"

6. Press the arrow button next to the generated command

Submit Kit Demo
---------------

.. figure:: /_static/images/submit_kit_db.gif
  :width: 60%
  :align: center

**Overview:** Demonstrates AGV movement and kit submission workflow by moving AGV1 to the shipping station and submitting a complete kit.

**Steps to run:**

1. Configure trial and team configs
2. **Cheat Selection:** Select "Kit on AGV" from the dropdown
3. Click "Confirm" to open the run page
4. Press the green "START RUN" button
5. In the "Team Process" section:

   - Package dropdown: Select "example_team"
   - File dropdown: Select "submit_kit"

6. Press the arrow button next to the generated command

-------------------------
Running with the Terminal
-------------------------

For advanced users or automation purposes, demos can be run directly from the terminal. This approach requires two separate terminal sessions.

.. important::

   **Local Installation Setup**

   If running locally (not Docker), source the setup files in both terminals:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      source ~/ariac_ws/install/setup.bash

Dummy Inspection
================

**Overview:** Demonstrates inspection conveyor control and sensor-based reporting by controlling the cell feed on the inspection conveyor and submitting dummy inspection reports using break-beam sensor data.

**Terminal 1 - Start Environment:**

*Docker:*

.. code-block:: bash

   ros2 launch ariac_gz ariac.launch.py \
     user_config:=/team_ws/src/example_team/config/example_team_config.yaml \
     trial_config:=/team_ws/src/example_team/config/trials/LHAF9835.yaml

*Local:*

.. code-block:: bash

   ros2 launch ariac_gz ariac.launch.py \
     trial_config:=~/ariac_ws/src/ariac_example_team/example_team/config/trials/LHAF9835.yaml \
     user_config:=~/ariac_ws/src/ariac_example_team/example_team/config/example_team_config.yaml

**Terminal 2 - Run Demo:**

.. code-block:: bash

   ros2 run example_team dummy_inspection

Move Robots
===========

**Overview:** Demonstrates basic robot motion control across multiple robots by creating MoveIt nodes for all five robots and executing simultaneous linear motions.

**Terminal 1 - Start Environment:**

*Docker:*

.. code-block:: bash

   ros2 launch ariac_gz ariac.launch.py \
     user_config:=/team_ws/src/example_team/config/example_team_config.yaml \
     trial_config:=/team_ws/src/example_team/config/trials/LHAF9835.yaml

*Local:*

.. code-block:: bash

   ros2 launch ariac_gz ariac.launch.py \
     trial_config:=~/ariac_ws/src/ariac_example_team/example_team/config/trials/LHAF9835.yaml \
     user_config:=~/ariac_ws/src/ariac_example_team/example_team/config/example_team_config.yaml

**Terminal 2 - Run Demo:**

.. code-block:: bash

   ros2 run example_team move_robots

Pick from Voltage Tester
========================

**Overview:** Demonstrates cell manipulation and recycling operations by using inspection robot 2 to pick a cell from the voltage tester and drop it in the recycling bin.

**Terminal 1 - Start Environment:**

*Docker:*

.. code-block:: bash

   ros2 launch ariac_gz ariac.launch.py \
     user_config:=/team_ws/src/example_team/config/example_team_config.yaml \
     trial_config:=/team_ws/src/example_team/config/trials/LHAF9835.yaml \
     cheat_selection:=1

*Local:*

.. code-block:: bash

   ros2 launch ariac_gz ariac.launch.py \
     trial_config:=~/ariac_ws/src/ariac_example_team/example_team/config/trials/LHAF9835.yaml \
     user_config:=~/ariac_ws/src/ariac_example_team/example_team/config/example_team_config.yaml \
     cheat_selection:=1

**Terminal 2 - Run Demo:**

.. code-block:: bash

   ros2 run example_team pick_from_tester

Submit Kit
==========

**Overview:** Demonstrates AGV movement and kit submission workflow by moving AGV1 to the shipping station and submitting a complete kit.

**Terminal 1 - Start Environment:**

*Docker:*

.. code-block:: bash

   ros2 launch ariac_gz ariac.launch.py \
     user_config:=/team_ws/src/example_team/config/example_team_config.yaml \
     trial_config:=/team_ws/src/example_team/config/trials/LHAF9835.yaml \
     cheat_selection:=2

*Local:*

.. code-block:: bash

   ros2 launch ariac_gz ariac.launch.py \
     trial_config:=~/ariac_ws/src/ariac_example_team/example_team/config/trials/LHAF9835.yaml \
     user_config:=~/ariac_ws/src/ariac_example_team/example_team/config/example_team_config.yaml \
     cheat_selection:=2

**Terminal 2 - Run Demo:**

.. code-block:: bash

   ros2 run example_team submit_kit