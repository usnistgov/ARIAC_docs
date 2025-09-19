.. _STARTUP:

=======
Startup
=======

There are two ways for teams to develop a solution for this year's ARIAC competition: 

* Docker 
* Local

------
Docker
------

Running the competition through Docker ensures all ARIAC dependencies are preloaded into the image. This makes setup easier and can lead to fewer potential issues. However, there can be a steep learning curve if your team is not familiar with Docker. The final evaluation will require teams to submit a Dockerfile for their solution, so this is the recommended approach. 

Installation
============

.. attention::

   Running ARIAC through Docker requires a Linux operating system

1. Install prerequisites

   * `Docker Engine <https://docs.docker.com/engine/install/ubuntu/>`_.

   .. note::
   
      After installing Docker Engine, follow the `post-installation instructions <https://docs.docker.com/engine/install/linux-postinstall/>`_.

   * `Nvidia Container Toolkit <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html>`_.

   .. note::
      
      The toolkit is only necessary if you have an NVIDIA graphics card.

      After installation, run the following command:

      .. code-block:: bash

         sudo nvidia-ctk runtime configure --runtime=docker

   

2. Pull the ARIAC image from Docker Hub

   .. code-block:: bash

      docker pull nistariac/ariac2025:latest


3. Clone the `example team repository <https://github.com/usnistgov/ariac_example_team.git>`_

   .. code-block:: bash

      git clone https://github.com/usnistgov/ariac_example_team.git ~/ariac_example_team

      cd ~/ariac_example_team

4. Start the container with Docker Compose

   If you have an NVIDIA graphics card:

   .. code-block:: bash

      docker compose up ariac_nvidia

   Otherwise:

   .. code-block:: bash

      docker compose up ariac

   .. warning::

      Running the competition without a dedicated GPU will lead to worse performance.
   
.. note::

   This terminal must be kept open while ARIAC is running or use ``-d`` to run the container in detached mode. 

Launching the Environment
=========================

1. Open a new terminal in the container

   .. code-block:: bash

      docker exec -it example_team bash

2. Launch the environment

   .. code-block:: bash

      ros2 launch ariac_gz ariac.launch.py user_config:=/team_ws/src/example_team/config/example_team_config.yaml trial_config:=/team_ws/src/example_team/config/trials/LHAF9835.yaml

Starting the App
================

1. Open a new terminal in the container

   .. code-block:: bash

      docker exec -it example_team bash

2. Start the app 

   .. code-block:: bash

      ros2 run ariac_app app

   .. note:: 

      You should see a message that looks like this:

      `NiceGUI ready to go on http://localhost:8080, and http://x.x.x.x:8080`

3. Open the app in your browser

   To open the app, ctrl+click on either of the links in the terminal or click `here <http://localhost:8080>`_.

4. Run the demos

   See the :ref:`demos page <DEMOS>` for instructions on how to run the demos.

-----
Local
-----

The competition can still be run without using Docker by directly installing and building the ARIAC source code on your machine. This may lead to more issues depending on how your computer is configured. 

.. attention::

   Running ARIAC locally requires Ubuntu 24.04.

.. note::

  These instructions are written for Bash. If you are using a shell other than Bash, the source commands will be different. For example, the command for Zsh is:

  .. code-block:: bash

      source /opt/ros/jazzy/setup.zsh


Installation
============



1. Install prerequisites

   * `ROS2 Jazzy <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>`_.
   
   .. important::

      The step for "Install development tools (optional)" is required.

   * `Gazebo Sim (Harmonic) <https://gazebosim.org/docs/harmonic/install_ubuntu/>`_.

2. Create a workspace

   .. code-block:: bash

      mkdir -p ~/ariac_ws/src && cd ~/ariac_ws

3. Clone ARIAC and the example team

   .. code-block:: bash

      git clone https://github.com/usnistgov/ARIAC.git src/ARIAC
      git clone https://github.com/usnistgov/ariac_example_team.git src/ariac_example_team

4. Rosdep
   
   Initialize rosdep

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      sudo rosdep init
   
   Update rosdep
   
   .. code-block:: bash

      rosdep update

   Install ARIAC dependencies

   .. code-block:: bash

      rosdep install --from-paths src --ignore-src -r -y
   
5. Pip dependencies

   Install necessary Python packages for the app:

   .. code-block:: bash

      sudo apt install python3-pip
      pip install -r src/ARIAC/ariac_app/requirements.txt --break-system-packages
   
6. Build the workspace

   First, ROS must be sourced using this command:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
   
   After sourcing ROS, the workspace can be built using this command:

   .. code-block:: bash

      colcon build

Launching the Environment
=========================

1. Navigate to the workspace and source packages

   .. code-block:: bash
      
      cd ~/ariac_ws
      source /opt/ros/jazzy/setup.bash
      source install/setup.bash
   
2. Launch the environment

   .. code-block:: bash

      ros2 launch ariac_gz ariac.launch.py trial_config:=src/ariac_example_team/example_team/config/trials/LHAF9835.yaml user_config:=src/ariac_example_team/example_team/config/example_team_config.yaml

Starting the App
================

1. Navigate to the workspace and source packages

   .. code-block:: bash
      
      cd ~/ariac_ws
      source /opt/ros/jazzy/setup.bash
      source install/setup.bash

2. Start the app 

   .. code-block:: bash

      ros2 run ariac_app app

   .. note:: 

      You should see a message that looks like this:

      `NiceGUI ready to go on http://localhost:8080, and http://x.x.x.x:8080`

3. Open the app in your browser

   To open the app, ctrl+click on either of the links in the terminal or click `here <http://localhost:8080>`_.

4. Run the demos

   See the :ref:`demos page <DEMOS>` for instructions on how to run the demos.

