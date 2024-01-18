.. _TUTORIAL7:

==============================================
Tutorial 7: Picking and placing a kitting tray
==============================================

.. admonition:: Tutorial 7
  :class: attention
  :name: tutorial_7

  - **Prerequisites:** :ref:`Introduction to Tutorials <TUTORIALS>`
  - **Source Code**: `https://github.com/usnistgov/ARIAC_tutorials <https://github.com/usnistgov/ARIAC_tutorials>`_

This tutorial details the steps necessary to start the competition from a competitor package.

------------------------
Starting the enviornment
------------------------

To start the enviornment, use this command:

.. code-block:: bash
        
            ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorial dev_mode:=True

------------------
Running tutorial 7
------------------

To start tutorial 7, open a new terminal and use this command:

.. code-block:: bash
        
            ros2 launch ariac_tutorials tutorial.launch.py tutorial:=7

-----------------
Code explaination
-----------------

This is the node used for tutorial 7. The functions from competition_interface.py which are used are highlighted.

.. code-block:: python
    :caption: :file:`tutorial_7.py`
    :name: tutorial_7
    :emphasize-lines: 21,23-26

    #!/usr/bin/env python3

    import rclpy
    import threading
    from rclpy.executors import MultiThreadedExecutor
    from ariac_msgs.msg import Order as OrderMsg, KittingTask as KT
    from ariac_tutorials.competition_interface import CompetitionInterface


    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface()
        executor = MultiThreadedExecutor()
        executor.add_node(interface)

        spin_thread = threading.Thread(target=executor.spin)
        spin_thread.start()
        
        interface.start_competition()
        
        interface.add_objects_to_planning_scene()
        
        for order in interface.orders:
            if order.order_type == OrderMsg.KITTING:
                interface.floor_robot_pick_and_place_tray(order.order_task.tray_id, order.order_task.agv_number)
                break

        interface.end_competition()
        interface.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

The purpose of this tutorial is to pick and place a tray onto an agv. The node starts by looping through recieved orders until a kitting order is found.
Then, :python:`interface.floor_robot_pick_and_place_tray` is called with the parameters being the tray id and the AGV number from the order.
Inside of :python:`interface.floor_robot_pick_and_place_tray`, the trays found on kitting stations 1 and 2 are looped through until the correct one is found.
When it is found, the station and pose of the tray is saved.
The robot then moves to the station where the tray was found. If the robot is not already using the tray gripper, the gripper is changed.
The gripper orientation is then set to match the tray rotation and the robot moves directly above the tray.
The robot then slowly moves down until the tray is attached and moves up.
After the tray is attached, the pose of the AGV is obtained and the robot moves above the AGV, matching theAGV tray orientation.
The robot then moves down to the AGV tray, releases the tray, and locks the tray onto the AGV.
Finally, the robot moves up.
