.. _TUTORIALS:

=========================
Introduction to Tutorials
=========================

This section contains multiple tutorials on how to achieve basic tasks in ARIAC. Some competitors may find these tutorials useful as a starting point for their own code.

The tutorials are written using python and rclpy. MoveItPy is used for interacting with the robots. If your team would like to use C++ and MoveGroupInterface refer to the :file:`test_competitor` package. 

The source code for tutorials can be found in a `GitHub repository <https://github.com/usnistgov/ARIAC_tutorials>`_ which is separate from the main ARIAC repository. 

.. admonition:: Prerequisites
  :class: attention

    * Ensure your environment is already configured using the :ref:`installation instructions <INSTALLATION>`.

    * Clone the tutorials repository into the same ROS workspace used for ARIAC.

      .. code-block:: sh

        cd ~/ariac_ws && git clone https://github.com/usnistgov/ARIAC_tutorials.git src/ARIAC_tutorials

    * Install necessary ROS dependencies

      .. code-block:: sh

        rosdep install --from-paths src -y --ignore-src

    * Install necessary pip dependencies

      .. code-block:: sh

        pip install -r src/ARIAC_tutorials/requirements.txt

    * Build/source the workspace.

      .. code-block:: sh

        colcon build && . install/setup.bash

-----------------
List of Tutorials
-----------------

A brief description of each tutorial is provided below.

:tutorial:`Tutorial 1:` :ref:`Starting the competition <TUTORIAL1>`
    
  - Demonstrates how to call a ROS service to start the competition.

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

  - Demonstrates how to complete and submit a kitting task.

------------------------------
Creating a Competition Package
------------------------------

The following example demonstrates how to create a minimal competition ROS package using python. 

Package Structure
=================

Start by creating a folder named :file:`example_package` inside the :file:`src` directory of the ROS workspace. Create the following files and folders shown in :numref:`example-package-tree`.

.. code-block:: text
  :caption: Example Package Tree
  :name: example-package-tree
  :class: no-copybutton
    
  ariac_ws/
  ├── src/
      ├── example_package/
          ├── config/
          │   └── my_sensors.yaml
          ├── example_package/
          │   ├── __init__.py
          │   └── example.py
          └── scripts/
              └── example_node.py
          ├── CMakeLists.txt
          ├── package.xml

List of Files
=============

Copy the contents below into each corresponding file. 

:file:`my_sensors.yaml`
-----------------------

  .. code-block:: yaml

    robot_cameras:
      floor_robot_camera: 
        active: false
        type: rgb
      
      ceiling_robot_camera: 
        active: false
        type: rgbd

    static_sensors:
      conveyor_breakbeam:
        type: break_beam
        visualize_fov: true
        pose:
          xyz: [-0.36, 3.5, 0.88]
          rpy: [0, 0, pi]

:file:`example.py`
------------------

  .. code-block:: python

    import rclpy
    from rclpy.node import Node

    class ExampleNode(Node):

        def __init__(self):
            super().__init__('example_node')
            self.timer = self.create_timer(1, self.timer_callback)

        def timer_callback(self):
            self.get_logger().info("Example node running")
  

:file:`example_node.py`
-----------------------

  .. code-block:: python

    #!/usr/bin/env python3

    import rclpy

    from example_package.example import ExampleNode

    def main(args=None):
        rclpy.init(args=args)
        
        example_node = ExampleNode()

        rclpy.spin(example_node)

        example_node.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

:file:`CMakeLists.txt`
----------------------

  .. code-block:: cmake

    cmake_minimum_required(VERSION 3.8)
    project(example_package)

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(ament_cmake_python REQUIRED)
    find_package(rclpy REQUIRED)

    # Install the config directory to the package share directory
    install(DIRECTORY 
      config
      DESTINATION share/${PROJECT_NAME}
    )

    # Install Python modules
    ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

    # Install Python executables
    install(PROGRAMS
      scripts/example_node.py
      DESTINATION lib/${PROJECT_NAME}
    )

    ament_package()


:file:`package.xml`
-------------------

  .. code-block:: xml

    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
      <name>example_package</name>
      <version>1.0.0</version>
      <description>Example competition package for ARIAC</description>
      <maintainer email="you@email.com">Your Name</maintainer>
      <license>Apache License 2.0</license>

      <buildtool_depend>ament_cmake</buildtool_depend>

      <depend>rclpy</depend>

      <export>
        <build_type>ament_cmake</build_type>
      </export>
    </package>


Testing the Package
===================

* First build the package

  .. code-block:: sh

    cd ~/ariac_ws && colcon build --packages-select example_package && . install/setup.bash

* Start the competition with the sensor configuration 

  .. code-block:: sh

    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=example_package sensor_config:=my_sensors

  This should launch the environment with the sensors specified in :file:`my_sensors.yaml`

* Run the example node in another terminal

  .. code-block:: sh

    ros2 run example_package example_node.py

  You should see this output:

    .. code-block:: console

      [INFO] [1705512918.432221530] [example_node]: Example node running
      [INFO] [1705512919.420483814] [example_node]: Example node running
      [INFO] [1705512920.420387437] [example_node]: Example node running
    
    The node will continue to log once a second until the process is stopped. 










