.. _STARTUP:

=======
Startup
=======

IN ARIAC 2025, there are two methods for running the competition: Docker and locally. In Docker, all of the dependencies for ARIAC are preloaded into the image, making setup easier and fewer potential issues. Local installation is also available to run the competition the same way it has been run in previous years.

--------------
Docker Startup
--------------

Installation
^^^^^^^^^^^^

If docker is not installed on your device, please follow these instructions: `https://docs.docker.com/engine/install/ubuntu/ <https://docs.docker.com/engine/install/ubuntu/>`_.

If you are hoping to use an NVIDIA graphics card and do not have the NVIDIA container toolkit, please follow these instructions: `https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html>`_.

After installing the toolkit, please run this command:

.. code-block:: bash

   sudo nvidia-ctk runtime configure --runtime=docker

After Docker is successfully installed and the runtime is set to NVIDIA if you would like to use a graphics card, the image can be pulled using this command:

.. code-block:: bash

   docker pull nistariac/ariac2025:latest

Starting the App
^^^^^^^^^^^^^^^^
To start the container quickly, the example team has been configured to use Docker compose. So, the example team can be cloned to the host machine using this command:

.. code-block:: bash

   git clone https://github.com/usnistgov/ariac_example_team.git ~/ariac_example_team

   cd ~/ariac_example_team

(RECCOMMENDED) If you have an NVIDIA graphics card that you would like to use, run this command:

.. code-block:: bash

   docker compose up ariac_nvidia

If you do not have an NVIDIA graphics card, run this command:

.. code-block:: bash

   docker compose up ariac

After these commands have been run, the container with all of the correct settings with ARIAC and the example team will be started.

Then, open a new terminal. If you ran the NVIDIA service, run this command:

.. code-block:: bash

   cd ~/ariac_example_team
   docker compose exec ariac_nvidia bash

If you ran the non-NVIDIA service, run this command:

.. code-block:: bash

   cd ~/ariac_example_team
   docker compose exec ariac bash
   
At this point, you should see a terminal inside the docker container.

After you are in the terminal of the docker container, run this command to start the ARIAC app:

.. code-block:: bash

   ros2 run ariac_app app

This will print a message that looks like this:

`NiceGUI ready to go on http://localhost:8080, and http://x.x.x.x:8080`

To open the app, ctrl+click on either of the links in the terminal or click `here <http://localhost:8080>`_.

From the app, you are able to launch the competition, select existing trial and user config files, create trial and user config files, and more.

Launching from the Launch File
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To start the container quickly, the example team has been configured to use Docker compose. So, the example team can be cloned to the host machine using this command:

.. code-block:: bash

   git clone https://github.com/usnistgov/ariac_example_team.git ~/ariac_example_team
   cd ~/ariac_example_team

(RECCOMMENDED) If you have an NVIDIA graphics card that you would like to use, run this command:

.. code-block:: bash

   docker compose up ariac_nvidia

If you do not have an NVIDIA graphics card, run this command:

.. code-block:: bash

   docker compose up ariac

After these commands have been run, the container with all of the correct settings with ARIAC and the example team will be started.

Then, open a new terminal. If you ran the NVIDIA service, run this command:

.. code-block:: bash

   cd ~/ariac_example_team
   docker compose exec ariac_nvidia bash

If you ran the non-NVIDIA service, run this command:

.. code-block:: bash

   cd ~/ariac_example_team
   docker compose exec ariac bash
   
At this point, you should see a terminal inside the docker container and you should be in a directory called `team_ws`.

To launch the environment using the example team's user config and trial, use this command:

.. code-block:: bash

   ros2 launch ariac_gz ariac.launch.py user_config:=src/example_team/config/example_team_config.yaml trial_config:=src/example_team/config/trials/LHAF9835.yaml

-------------
Local Startup
-------------

Install
^^^^^^^

1. Install prerequisites

   ARIAC 2025 requires Ubuntu 24 and ROS2 Jazzy to run locally. To install ROS2 Jazzy, please follow the official documentation instructions, which can be found here: `https://docs.ros.org/en/jazzy/Installation.html <https://docs.ros.org/en/jazzy/Installation.html>`_.

   ARIAC 2025 also requires GZ Harmonic. This can be installed using these instructions: `https://gazebosim.org/docs/harmonic/install_ubuntu/ <https://gazebosim.org/docs/harmonic/install_ubuntu/>`_.

2. Create a workspace

   .. code-block:: bash

      mkdir -p ~/ariac_ws/src && cd ~/ariac_ws

3. Clone ARIAC

   .. code-block:: bash

      git clone https://github.com/usnistgov/ARIAC.git src/ARIAC

4. Rosdep

   Install rosdep

   .. code-block:: bash

      sudo apt install python3-rosdep
   
   Initialize rosdep

   .. code-block:: bash

      sudo rosdep init
   
   Update rosdep
   
   .. code-block:: bash

      rosdep update

   Install ARIAC dependencies

   .. code-block:: bash

      rosdep install --from-paths src --ignore-src -r -y
   
5. More dependencies

   Install neccessary packages for building with colcon:

   .. code-block:: bash

      sudo apt install python3-colcon-common-extensions python3-pip

   Install neccessary python packages for the app:

   .. code-block:: bash

      pip install -r src/ARIAC/ariac_app/requirements.txt --break-system-packages
   
6. Build the workspace

   First, ROS must be sourced using this command:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
   
   After sourcing ROS, the workspace can be built using this command:

   .. code-block:: bash

      colcon build

Starting the App
^^^^^^^^^^^^^^^^
First, ensure that you are in the correct workspace:

.. code-block:: bash
   
   cd ~/ariac_ws

Then, source Jazzy and the workspace:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   source install/setup.bash

Then, to launch the app, run this command:

.. code-block:: bash

   ros2 run ariac_app app

This will print a message that looks like this:

`NiceGUI ready to go on http://localhost:8080, and http://x.x.x.x:8080`

To open the app, ctrl+click on either of the links in the terminal or click `here <http://localhost:8080>`_.

Launching from the Launch File
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First, ensure that you are in the correct workspace:

.. code-block:: bash
   
   cd ~/ariac_ws

Then, source Jazzy:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash

To test the installation of ARIAC, please clone the example team, rebuild the workspace, and source the workspace:

.. code-block:: bash

   git clone https://github.com/usnistgov/ariac_example_team.git src/ariac_example_team
   colcon build
   source install/setup.bash

To launch ariac, this command can be used:

.. code-block:: bash

   ros2 launch ariac_gz ariac.launch.py trial_config:=src/ariac_example_team/example_team/config/trials/LHAF9835.yaml user_config:=src/ariac_example_team/example_team/config/example_team_config.yaml

