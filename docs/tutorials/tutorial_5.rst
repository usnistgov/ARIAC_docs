.. _TUTORIAL5:

=================================
Tutorial 5: Move AGVs to stations
=================================

This tutorial details the steps to move the AGVs for an assembly order.

---------------------------------------
Starting the environment for Tutorial 5
---------------------------------------

To start the environment, use this command:

.. code-block:: bash
        
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorial dev_mode:=True

------------------
Running tutorial 5
------------------

To start tutorial 5, open a new terminal and use this command:

.. code-block:: bash
        
    ros2 launch ariac_tutorials tutorial.launch.py tutorial:=5

-----------------------------
Expected output of tutorial 5
-----------------------------

.. code-block:: console
    :caption: Tutorial 5 output
    :class: no-copybutton

    [tutorial_5.py-1] [INFO] [1705525912.232828149] [competition_interface]: Waiting for competition to be ready
    [tutorial_5.py-1] [INFO] [1705525934.039300460] [competition_interface]: Competition state is: idle
    [tutorial_5.py-1] [INFO] [1705526044.188032907] [competition_interface]: Competition state is: ready
    [tutorial_5.py-1] [INFO] [1705526044.209055609] [competition_interface]: Competition is ready. Starting...
    [tutorial_5.py-1] [INFO] [1705526044.247372429] [competition_interface]: Started competition.
    [tutorial_5.py-1] [INFO] [1705526044.250320126] [competition_interface]: Waiting for assembly order...
    [tutorial_5.py-1] [INFO] [1705526046.405434793] [competition_interface]: Competition state is: started
    [tutorial_5.py-1] [INFO] [1705526134.017383922] [competition_interface]: Assembly order received...
    [tutorial_5.py-1] [INFO] [1705526134.113937118] [competition_interface]: Locked AGV1's tray
    [tutorial_5.py-1] [INFO] [1705526225.522884837] [competition_interface]: Moved AGV1 to assembly station 1
    [tutorial_5.py-1] [INFO] [1705526225.538922602] [competition_interface]: Locked AGV2's tray
    [tutorial_5.py-1] [INFO] [1705526233.423827014] [competition_interface]: Competition state is: order_announcements_done
    [tutorial_5.py-1] [INFO] [1705526314.687703276] [competition_interface]: Moved AGV2 to assembly station 1
    [tutorial_5.py-1] [INFO] [1705526314.691610644] [competition_interface]: Ending competition

-------------------------------
Code explanation for Tutorial 5
-------------------------------

This is the node used for tutorial 5. The functions from competition_interface.py which are used are highlighted.

.. code-block:: python
    :caption: :file:`tutorial_4.py`
    :name: tutorial_4
    :emphasize-lines: 29-30

    #!/usr/bin/env python3

    import rclpy
    import threading
    from rclpy.executors import MultiThreadedExecutor
    from ariac_msgs.msg import Order as OrderMsg
    from ariac_tutorials.competition_interface import CompetitionInterface

    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface(enable_moveit=False)
        executor = MultiThreadedExecutor()
        executor.add_node(interface)

        spin_thread = threading.Thread(target=executor.spin)
        spin_thread.start()

        interface.start_competition()

        # Wait for assembly order to be recieved
        while True:
            interface.get_logger().info("Waiting for assembly order...", once=True)
            if OrderMsg.ASSEMBLY in [order.order_type for order in interface.orders]:
                interface.get_logger().info("Assembly order recieved...", once=True)
                break

        assembly_order = interface.orders[-1]
        for agv in assembly_order.order_task.agv_numbers:
            interface.lock_agv_tray(agv)
            interface.move_agv_to_station(agv, assembly_order.order_task.station)
        
        interface.end_competition()
        spin_thread.join()

    if __name__ == '__main__':
        main()

The purpose of tutorial 5 to move the AGVs to the stations as needed for the first assembly order. The node starts by waiting for an assembly order to be published. Once an assembly order is found in :python:`interface.orders`, the AGV's in the order are looped through. For each AGV, the tray is locked and then moved to the task station. To lock the tray on the AGV, a service call of the :topic:`/ariac/agv{num}_lock_tray` service is called. Then, to move the AGV to the station, :topic:`/ariac/move_agv{num}` is called and the destination in the request is set depending on the station in the assembly task.