.. _DEMOS:

=====
Demos
=====

--------------------
Running with the App
--------------------

For instructions on how to start the app either through Docker or locally, please view the :ref:`startup page <STARTUP>`.

Configuration
=============

The home page of the app configures the settings for a competition run. This includes: 

* Trial config
* Team config 
* Database
* Cheats

Trial Config
------------

For the demos we will use a pregenerated trial config located in the example team's ``config/trials`` folder. 

.. figure:: /_static/images/select_trial_config.gif
  :width: 60%

In the Trial section, click the "Select" button. A dialog window will appear where a trial yaml file can be selected. To go up a directory, press the arrow in the top right corner of the dialog window. Below are the file locations depending on your configuration.

Docker: ``/team_ws/src/example_team/config/trials/LHAF9835.yaml``

Local: ``~/ariac_ws/src/ariac_example_team/example_team/config/trials/LHAF9835.yaml``

Team Config
-----------

For the demos we will use a pregenerated team config located in the example team's ``config`` folder. 

.. figure:: /_static/images/select_team_config.gif
  :width: 60%

In the Team Config section, click the "Select" button. A dialog window will appear where a team config yaml file can be selected. To go up a directory, press the arrow in the top right corner of the dialog window. Below are the file locations depending on your configuration.

Docker: ``/team_ws/src/example_team/config/example_team_config.yaml``

Local: ``~/ariac_ws/src/ariac_example_team/example_team/config/example_team_config.yaml``

Database
--------

The ARIAC database is a local SQLite database that stores the information needed for scoring. The database is optional to run the environment, but is necessary to view results. For the demos, create a database.

.. figure:: /_static/images/create_db.gif
  :width: 60%

To create the database, click the "Create" button in the database section and the file selection dialog will open. From there, navigate to the parent directory of the directory you would like to save the database in. Then, click once on the target directory and click "OK". The dialog will close and a database will be created.

.. note::

  The location of the database does not matter for its functionality.

Cheats
------

For testing purposes, cheats have been included to start the environment at different stages of the competition. The available cheat configurations are:

* Cells in voltage testers
* Kit on AGV
* Kits on AGVs
* High priority kit
* Partial module
* Module
* Flipped module
* Module with welds

The dummy inspection and move robots demos do not require cheats. For the pick from voltage tester demo, start the environment with the "Cells in voltage testers" cheat. For the submit kit demo, start the environment with the "Kit on AGV" cheat.

Running the Demos
=================

Dummy Inspection
----------------

.. figure:: /_static/images/dummy_inspection.gif
  :width: 60%

This example controls the cell feed on the inspection conveyor and submits dummy inspection reports using data from the break-beam sensor.

To run this demo, select the example team's trial and team configs using the steps above. Then, press the confirm button, which will open the run page. To start the environment, press the green "START RUN" button. Then, in the "Team Process" section of the run page, select "example_team" in the package dropdown and "dummy_inspection" in the file dropdown. The command will then fill in below the dropdowns. Press the arrow next to the command to run the demo.

Move Robots
-----------

.. figure:: /_static/images/move_robots.gif
  :width: 60%

This example creates MoveIt.py nodes for all five robots in the environment and moves the robots in simple linear motions simultaneously.

To run this demo, select the example team's trial and team configs using the steps above. Then, press the confirm button, which will open the run page. To start the environment, press the green "START RUN" button. Then, in the "Team Process" section of the run page, select "example_team" in the package dropdown and "move_robots" in the file dropdown. The command will then fill in below the dropdowns. Press the arrow next to the command to run the demo.

Pick from Voltage Tester
------------------------

.. figure:: /_static/images/pick_from_tester.gif
  :width: 60%

This example uses inspection robot 2 to pick a cell from the voltage tester and drop it in the recycling bin.

To run this demo, select the example team's trial and team configs using the steps above. Then, in the Cheat Selection section, select "Cells in voltage testers" from the dropdown. This will spawn cells into the voltage testers when the environment is launched. Then, press the confirm button, which will open the run page. To start the environment, press the green "START RUN" button. Then, in the "Team Process" section of the run page, select "example_team" in the package dropdown and "pick_from_tester" in the file dropdown. The command will then fill in below the dropdowns. Press the arrow next to the command to run the demo.

Submit Kit
----------

.. figure:: /_static/images/submit_kit_db.gif
  :width: 60%

This example moves AGV1 to the shipping station and submits a kit.

To run this demo, select the example team's trial and team configs using the steps above. Then, in the Cheat Selection section, select "Kit on AGV" from the dropdown. This will spawn cells on AGV1 when the environment is launched. Then, press the confirm button, which will open the run page. To start the environment, press the green "START RUN" button. Then, in the "Team Process" section of the run page, select "example_team" in the package dropdown and "submit_kit" in the file dropdown. The command will then fill in below the dropdowns. Press the arrow next to the command to run the demo.

-------------------------
Running with the Terminal
-------------------------

For each of these demos, two terminals are required: one for the environment and another for the example team.

.. note::

  If running locally, be sure to source the setup files in both terminals.

  .. code-block:: bash

    source /opt/ros/jazzy/setup.bash
    source ~/ariac_ws/install/setup.bash

Dummy Inspection
================

In terminal one, start the environment.

Docker:

.. code-block:: bash

  ros2 launch ariac_gz ariac.launch.py user_config:=/team_ws/src/example_team/config/example_team_config.yaml trial_config:=/team_ws/src/example_team/config/trials/LHAF9835.yaml

Local:

.. code-block:: bash

  ros2 launch ariac_gz ariac.launch.py trial_config:=~/ariac_ws/src/ariac_example_team/example_team/config/trials/LHAF9835.yaml user_config:=~/ariac_ws/src/ariac_example_team/example_team/config/example_team_config.yaml

In the second terminal, start the example team:

.. code-block:: bash

  ros2 run example_team dummy_inspection

Move Robots
===========

In terminal one, start the environment.

Docker:

.. code-block:: bash

  ros2 launch ariac_gz ariac.launch.py user_config:=/team_ws/src/example_team/config/example_team_config.yaml trial_config:=/team_ws/src/example_team/config/trials/LHAF9835.yaml

Local:

.. code-block:: bash

  ros2 launch ariac_gz ariac.launch.py trial_config:=~/ariac_ws/src/ariac_example_team/example_team/config/trials/LHAF9835.yaml user_config:=~/ariac_ws/src/ariac_example_team/example_team/config/example_team_config.yaml

In the second terminal, start the example team:

.. code-block:: bash

  ros2 run example_team move_robots

Pick from Tester
================

In terminal one, start the environment.

Docker:

.. code-block:: bash

  ros2 launch ariac_gz ariac.launch.py user_config:=/team_ws/src/example_team/config/example_team_config.yaml trial_config:=/team_ws/src/example_team/config/trials/LHAF9835.yaml cheat_selection:=1

Local:

.. code-block:: bash

  ros2 launch ariac_gz ariac.launch.py trial_config:=~/ariac_ws/src/ariac_example_team/example_team/config/trials/LHAF9835.yaml user_config:=~/ariac_ws/src/ariac_example_team/example_team/config/example_team_config.yaml cheat_selection:=1

In the second terminal, start the example team:

.. code-block:: bash

  ros2 run example_team pick_from_tester

Submit kit
==========

In terminal one, start the environment.

Docker:

.. code-block:: bash

  ros2 launch ariac_gz ariac.launch.py user_config:=/team_ws/src/example_team/config/example_team_config.yaml trial_config:=/team_ws/src/example_team/config/trials/LHAF9835.yaml cheat_selection:=2

Local:

.. code-block:: bash

  ros2 launch ariac_gz ariac.launch.py trial_config:=~/ariac_ws/src/ariac_example_team/example_team/config/trials/LHAF9835.yaml user_config:=~/ariac_ws/src/ariac_example_team/example_team/config/example_team_config.yaml cheat_selection:=2

In the second terminal, start the example team:

.. code-block:: bash

  ros2 run example_team submit_kit