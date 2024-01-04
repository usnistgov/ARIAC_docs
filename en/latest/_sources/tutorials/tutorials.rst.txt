.. _TUTORIALS:

=========================
Introduction to Tutorials
=========================

This section contains multiple tutorials on how to achieve basic tasks in ARIAC. Some competitors may find these tutorials useful as a starting point for their own code.

The tutorials are written using python and rclpy. MoveItPy is used for interacting with the robots. If your team would like to use C++ and MoveGroupInterface refer to the test competitor. 

The code for tutorials can be found in a `GitHub repository <https://github.com/usnistgov/ARIAC_tutorials>`_ which is separate from the main ARIAC repository. The code for each tutorial is located in a separate branch of the repository. The branch name is the same as the tutorial name.

.. admonition:: Prerequisites
  :class: attention

    * Ensure your environment is already configured using the :ref:`installation instructions <INSTALLATION>`.

    * Clone the tutorials repository into your workspace.

      .. code-block:: sh

        cd ~/ariac_ws && git clone https://github.com/usnistgov/ARIAC_tutorials.git src/ARIAC_tutorials -b tutorial1

    * Install necessary dependencies and build/source the workspace.

      .. code-block:: sh

        rosdep install --from-paths src -y --ignore-src && colcon build && . install/setup.bash


A brief description of each tutorial is provided below.

:tutorial:`Tutorial 1:` :ref:`Creating a competition package <TUTORIAL1>`
    
  - Demonstrates how to create a package and start the competition.

:tutorial:`Tutorial 2:` :ref:`Reading data from a break beam sensor <TUTORIAL2>`
    
  - Demonstrates how to add a sensor to the sensor configuration file. The sensor is then used to keep track of the number of parts that are spawned on the conveyor belt.

:tutorial:`Tutorial 3:` :ref:`Reading data from a RGB camera <TUTORIAL3>`

  - Demonstrates how to read an image from an RGB to classify objects in a parts bin.

:tutorial:`Tutorial 4:` :ref:`Reading an order <TUTORIAL4>`

  - Demonstrates how to retrieve published orders.

:tutorial:`Tutorial 5:` :ref:`Moving AGVs between stations <TUTORIAL5>`

  - Demonstrates how to use service calls to move AGVs between stations.

:tutorial:`Tutorial 6:` :ref:`Picking a part <TUTORIAL6>`

  - Demonstrates how to use MoveItPy to direct the floor robot to pick a part from the bins.

:tutorial:`Tutorial 7:` :ref:`Picking and placing a kit tray <TUTORIAL7>`

  - Demonstrates how to use MoveItPy to direct the floor robot to change the robot gripper type, then pick and place a kit tray onto an AGV. 

:tutorial:`Tutorial 8:` :ref:`Completing a Kitting Task <TUTORIAL8>`

  - Demonstrates how to complete and submit a kitting task