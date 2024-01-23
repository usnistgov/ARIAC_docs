.. _TUTORIAL8:

=====================================
Tutorial 8: Completing a kitting task
=====================================

This tutorial details the steps necessary to complete a kitting task. 

---------------------------------------
Starting the environment for Tutorial 8
---------------------------------------

To start the environment, use this command:

.. code-block:: bash
        
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorial dev_mode:=True

------------------
Running tutorial 8
------------------

To start tutorial 8, open a new terminal and use this command:

.. code-block:: bash
        
    ros2 launch ariac_tutorials tutorial.launch.py tutorial:=8

-------------------------------
Code explanation for Tutorial 8
-------------------------------

This is the node used for tutorial 8. The functions from competition_interface.py which are used are highlighted.

.. code-block:: python
    :caption: :file:`tutorial_8.py`
    :name: tutorial_8
    :emphasize-lines: 21,23-25

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
        
        interface.complete_orders()

        interface.end_competition()
        interface.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()


The purpose of this tutorial is to complete a kitting order, which is done using :python:`interface.complete_orders`.
Inside of this function, the objects like bins, assembly stations, kitting tray stations, and the conveyor belt are added to the planning scene using :python:`interface.add_objects_to_planning_scene`. Then, while the competition is active, the orders are looped through until a kitting order is found, where it is passed into :python:`interface._complete_kitting_order`. The first step of completing a kitting order is picking and placing the correct tray onto the correct agv. Then, the parts in the kitting order are looped through and the robot picks each part and places it on the kitting tray. After all of the parts are on the kitting tray, the AGV moves to the warehouse and the order is submitted.