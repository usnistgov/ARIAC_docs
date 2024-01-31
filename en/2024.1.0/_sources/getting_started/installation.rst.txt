.. _INSTALLATION:

============
Installation
============

.. admonition:: Requirements
  :class: attention

    ARIAC 2024 is built for ROS2 Iron running on Ubuntu 22.04 (Jammy Jellyfish). 

    * See the `ROS Iron installation instructions <https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html>`_ for more information.

        * Install :code:`ros-iron-desktop` using the Debian binaries.

---------------------------------
Configuring the ARIAC Environment
---------------------------------

1. Add the ROS Iron setup script to your :file:`~/.bashrc`

  .. code-block:: sh

    echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc && source ~/.bashrc

2. Make a ROS workspace for ARIAC

  .. code-block:: sh

    mkdir -p ~/ariac_ws/src && cd ~/ariac_ws

3. Clone the ARIAC repository

  .. code-block:: sh

    git clone https://github.com/usnistgov/ARIAC.git src/ARIAC

  **Note:** Always use the ``ariac2024`` branch.

4. Install rosdep

  .. code-block:: sh

    sudo apt install python3-rosdep

5. Initialize rosdep

  .. code-block:: sh

    sudo rosdep init 

5. Update rosdep

  .. code-block:: sh

    rosdep update 

6. Install ARIAC dependencies:

  .. code-block:: sh

    rosdep install --from-paths src -y --ignore-src

7. Install necessary packages for building with colcon:

  .. code-block:: sh

    sudo apt install python3-colcon-common-extensions python3-pip && pip install setuptools==58.2.0 

8. Build the workspace

  .. code-block:: sh

    colcon build

9. Add the workspace setup script to your :file:`~/.bashrc`

  .. code-block:: sh

    echo "source ~/ariac_ws/install/setup.bash" >> ~/.bashrc && source ~/.bashrc

-----------------------
Starting the Simulation
-----------------------

Default
=======

  The following command starts ARIAC with the default configuration:

  .. code-block:: sh

    ros2 launch ariac_gazebo ariac.launch.py

  The default trial config is :file:`kitting.yaml`, located in :code:`ARIAC/ariac_gazebo/config/trials/`

  .. note::
    All trial files must be placed in this folder.

Launch Options
==============

:code:`trial_name`
------------------

  To start ARIAC with a different trial configuration, use the following command:

  .. code-block:: sh

    ros2 launch ariac_gazebo ariac.launch.py trial_name:=<trial_file>

  Replace :code:`<trial_file>` with the name of a trial file (without the :code:`.yaml` extension).
    
  **Example:** To start ARIAC with :file:`assembly.yaml` trial file, run the following command:

  .. code-block:: sh

    ros2 launch ariac_gazebo ariac.launch.py trial_name:=assembly

:code:`competitor_pkg`
----------------------

  Competitors will need to create their own competitor ROS package. To create a new competitor package, see :ref:`tutorial 1 <TUTORIAL1>`.

  **Example:** To start ARIAC using the :file:`sensors.yaml` sensor file located in the :code:`config` directory of :code:`my_competitor_pkg`, run the following command:

  .. code-block:: sh

    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=my_competitor_pkg

:code:`sensor_config`
---------------------

  Competitors can have multiple sensor configurations. To specify the file use the :code:`sensor_config` argument.

  **Example:** To start ARIAC using the :file:`my_sensors.yaml` file located in the :code:`config` directory of :code:`my_competitor_pkg`, run the following command:

  .. code-block:: sh

    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=my_competitor_pkg sensor_config:=my_sensors

:code:`dev_mode`
----------------

  During testing, competitors can enable additional features that will not be available during qualifiers or finals. To enable these features set :code:`dev_mode` to :code:`true`. 
    
    * Currently the only additional feature is the ability to spawn advanced logical cameras. More may be added in the future. 

  **Example:**

  .. code-block:: sh

    ros2 launch ariac_gazebo ariac.launch.py dev_mode:=true

:code:`record_state`
--------------------

  This boolean flag controls whether or not gazebo generates a :file:`state.log` file for the given trial. This is default to :code:`false` as enabling will cause gazebo to output urdf parsing errors. The errors can be ignored. 

  **Example:**

  .. code-block:: sh

    ros2 launch ariac_gazebo ariac.launch.py record_state:=true

Running the NIST Competitor
===========================

  To help lower the barrier to entry a test competitor package was created by the NIST team. This competitor package is able to perform most of the necessary functions for the competition but is not programmed to handle any of the agility challenges. 

  1. Clone the NIST competitor repository

  .. code-block:: sh

    cd ~/ariac_ws && git clone https://github.com/usnistgov/nist_competitor.git src/nist_competitor
  
  .. note::
    The NIST competitor is programmed to only work with advanced logical cameras and is unable to handle any of the agility challenges.

  2. Install any dependencies

  .. code-block:: sh

    rosdep install --from-paths src -y --ignore-src

  3. Build and source the package

  .. code-block:: sh

    colcon build && source install/setup.bash

  **Option 1:** Use the :file:`competition.launch.py` launch file. This runs the environment, MoveIt, and the competitor node all in the same terminal.

    .. code-block:: sh

      ros2 launch nist_competitor competition.launch.py trial_name:=kitting record_state:=false

    The NIST competitor should start the competition and start completing the default kitting order specified in :file:`kitting.yaml`. After the kitting order is completed and submitted, the NIST competitor will end the competition and a score will be output in terminal. A score log will also be created in the folder :file:`ariac_log`

  **Option 2:** Use the :file:`competitor.launch.py` launch file. This runs the environment, MoveIt, and the competitor node in separate terminals. This can be useful for testing to cut down on the number of log messages in each terminal.  
  
    Open three terminal windows. 

    **Terminal 1:** Launch the environment

    .. code-block:: sh

      ros2 launch ariac_gazebo ariac.launch.py dev_mode:=true trial_name:=kitting

    **Terminal 2:** Launch the move_group node

    .. code-block:: sh

      ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py

    **Terminal 3:** Launch the nist competitor node

    .. code-block:: sh

      ros2 launch nist_competitor competitor.launch.py


  .. note::
    The NIST competitor has been tested with :file:`kitting.yaml`, :file:`assembly.yaml`, and :file:`combined.yaml`. There is no guarantee that the NIST competitor will work with other trials. 