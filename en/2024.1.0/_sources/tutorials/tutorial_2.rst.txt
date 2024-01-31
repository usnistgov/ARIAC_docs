.. _TUTORIAL2:

=================================================
Tutorial 2: Reading data from a break beam sensor
=================================================

This tutorial shows how to subscribe to a break beam sensor topic to count parts on the conveyor belt. 

---------------------------------------
Starting the environment for Tutorial 2
---------------------------------------

To start the environment, use this command:

.. code-block:: bash
        
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorial dev_mode:=True

------------------
Running tutorial 2
------------------

To start tutorial 2, open a new terminal and use this command:

.. code-block:: bash
        
    ros2 launch ariac_tutorials tutorial.launch.py tutorial:=2

-----------------------------
Expected output of tutorial 2
-----------------------------

.. code-block:: console
    :caption: Tutorial 2 output
    :class: no-copybutton

    [tutorial_2.py-1] [INFO] [1705518089.680128815] [competition_interface]: Waiting for competition to be ready
    [tutorial_2.py-1] [INFO] [1705518104.127781648] [competition_interface]: Competition state is: idle
    [tutorial_2.py-1] [INFO] [1705518218.248132032] [competition_interface]: Competition state is: ready
    [tutorial_2.py-1] [INFO] [1705518218.265702743] [competition_interface]: Competition is ready. Starting...
    [tutorial_2.py-1] [INFO] [1705518218.305600507] [competition_interface]: Started competition.
    [tutorial_2.py-1] [INFO] [1705518220.098969628] [competition_interface]: Competition state is: started
    [tutorial_2.py-1] [INFO] [1705518296.002932807] [competition_interface]: Part detected on conveyor. Count: 1
    [tutorial_2.py-1] [INFO] [1705518347.055104728] [competition_interface]: Part detected on conveyor. Count: 2
    [tutorial_2.py-1] [INFO] [1705518395.987836035] [competition_interface]: Competition state is: order_announcements_done
    [tutorial_2.py-1] [INFO] [1705518397.544910616] [competition_interface]: Part detected on conveyor. Count: 3
    [tutorial_2.py-1] [INFO] [1705518449.918159654] [competition_interface]: Part detected on conveyor. Count: 4
    [tutorial_2.py-1] [INFO] [1705518506.999705068] [competition_interface]: Part detected on conveyor. Count: 5

-------------------------------
Code explanation for Tutorial 2
-------------------------------

This is the node used for tutorial 2. The functions from competition_interface.py which are used are highlighted.

.. code-block:: python
    :caption: :file:`tutorial_2.py`
    :name: tutorial_2
    :emphasize-lines: 18-27

    #!/usr/bin/env python3

    import rclpy
    import threading
    from rclpy.executors import MultiThreadedExecutor
    from ariac_tutorials.competition_interface import CompetitionInterface

    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface(enable_moveit=False)
        executor = MultiThreadedExecutor()
        executor.add_node(interface)

        spin_thread = threading.Thread(target=executor.spin)
        spin_thread.start()
        interface.start_competition()

        count = 0

        while rclpy.ok():
            try:
                if interface.conveyor_part_count > count:
                    interface.get_logger().info(f"Part detected on conveyor. Count: {interface.conveyor_part_count}")
                    count+= 1
            except KeyboardInterrupt:
                
                break
        
        interface.end_competition()
        spin_thread.join()

    if __name__ == '__main__':
        main()

The purpose of this tutorial is to log the number of parts that have been counted by the break beam sensor over the course of the competition. Whenever the break beam sensor senses a part and that part has not already been counted, :python:`interface._breakbeam_cb` is run and :python:`interface.conveyor_part_count` is incremented by one. Inside the node, whenever :python:`interface.conveyor_part_count` is increased, the count is logged.
