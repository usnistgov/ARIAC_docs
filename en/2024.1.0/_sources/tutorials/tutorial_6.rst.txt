.. _TUTORIAL6:

==========================
Tutorial 6: Picking a part
==========================

This tutorial details the steps to direct the floor robot to pick a part from the bins using MoveItPy. 

---------------------------------------
Starting the environment for Tutorial 6
---------------------------------------

To start the environment, use this command:

.. code-block:: bash
        
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorial dev_mode:=True

------------------
Running tutorial 6
------------------

To start tutorial 6, open a new terminal and use this command:

.. code-block:: bash
        
    ros2 launch ariac_tutorials tutorial.launch.py tutorial:=6

----------------------
Moveit_py requirements
----------------------

For Moveit_py to launch properly, a config file is needed. This file can be found in :file:`/ariac_tutorials/config/moveit_config.yaml`.

-------------------------------
Code explanation for Tutorial 6
-------------------------------

This is the node used for tutorial 6. The functions from competition_interface.py which are used are highlighted.

.. code-block:: python
    :caption: :file:`tutorial_6.py`
    :name: tutorial_6
    :emphasize-lines: 21, 23-29

    #!/usr/bin/env python3

    import rclpy
    import threading
    from rclpy.executors import MultiThreadedExecutor
    from ariac_msgs.msg import Order as OrderMsg
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
                for part in order.order_task.parts:
                    interface.get_logger().info(f"Picking up {interface._part_colors[part.part.color]} {interface._part_types[part.part.type]}")
                    interface.floor_robot_pick_bin_part(part.part)
                    break
                break

        interface.end_competition()
        interface.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

The purpose of this tutorial is to pick up a part from a bin. The node loops through the received orders and when a kitting order is found, it loops through the parts of that kitting order. It then picks up the first part from that kitting order. To do this, :python:`interface.floor_robot_pick_bin_part` is called with the part as a parameter. In this function, the parts on each of the bins are looped through and if the part is found, the pose of the part and which set of bins is saved. The robot then changes gripper type if needed and the robot moves above the bins. The gripper orientation is then set to match the part rotation and the robot moves directly above the part. The gripper is then turned on and the robot slowly moves down to the part until the part is attached to the gripper. Finally, the robot moves up.
