.. _TUTORIAL4:

=========================
Tutorial 4: Read an order
=========================

.. admonition:: Tutorial 4
  :class: attention
  :name: tutorial_4

  - **Prerequisites:** :ref:`Introduction to Tutorials <TUTORIALS>`
  - **Source Code**: `https://github.com/usnistgov/ARIAC_tutorials <https://github.com/usnistgov/ARIAC_tutorials>`_

This tutorial details the steps necessary to start the competition from a competitor package.

---------------------------------------
Starting the environment for Tutorial 4
---------------------------------------

To start the environment, use this command:

.. code-block:: bash
        
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorial dev_mode:=True

------------------
Running tutorial 4
------------------

To start tutorial 4, open a new terminal and use this command:

.. code-block:: bash
        
    ros2 launch ariac_tutorials tutorial.launch.py tutorial:=4

-----------------------------
Expected output of tutorial 4
-----------------------------

.. code-block:: console
    :caption: Tutorial 4 output
    :class: no-copybutton

    [tutorial_4.py-1] [INFO] [1705524488.855170123] [competition_interface]: Waiting for competition to be ready
    [tutorial_4.py-1] [INFO] [1705524511.376178625] [competition_interface]: Competition state is: idle
    [tutorial_4.py-1] [INFO] [1705524624.315864392] [competition_interface]: Competition state is: ready
    [tutorial_4.py-1] [INFO] [1705524624.342930104] [competition_interface]: Competition is ready. Starting...
    [tutorial_4.py-1] [INFO] [1705524624.380538068] [competition_interface]: Started competition.
    [tutorial_4.py-1] [INFO] [1705524624.537476295] [competition_interface]: 
    [tutorial_4.py-1] 
    [tutorial_4.py-1] ==========================
    [tutorial_4.py-1] Received Order: MMB30H56
    [tutorial_4.py-1] Priority: False
    [tutorial_4.py-1] Type: Kitting
    [tutorial_4.py-1] ==========================
    [tutorial_4.py-1] AGV: 3
    [tutorial_4.py-1] Destination: warehouse
    [tutorial_4.py-1] Tray ID: 3
    [tutorial_4.py-1] Products:
    [tutorial_4.py-1] ==========================
    [tutorial_4.py-1] 	Quadrant 1: 🟪 Purple Pump
    [tutorial_4.py-1] 	Quadrant 2: -
    [tutorial_4.py-1] 	Quadrant 3: 🟦 Blue Battery
    [tutorial_4.py-1] 	Quadrant 4: -
    [tutorial_4.py-1] 
    [tutorial_4.py-1] [INFO] [1705524626.044062501] [competition_interface]: Competition state is: started
    [tutorial_4.py-1] [INFO] [1705524710.359818890] [competition_interface]: 
    [tutorial_4.py-1] 
    [tutorial_4.py-1] ==========================
    [tutorial_4.py-1] Received Order: 2IZJP127
    [tutorial_4.py-1] Priority: False
    [tutorial_4.py-1] Type: Assembly
    [tutorial_4.py-1] ==========================
    [tutorial_4.py-1] AGV(s): [1, 2]
    [tutorial_4.py-1] Station: Assembly Station 1
    [tutorial_4.py-1] Products:
    [tutorial_4.py-1] ==========================
    [tutorial_4.py-1] Part: 🟥 Red Regulator
    [tutorial_4.py-1]   Position:
    [tutorial_4.py-1]     x: 0.175 (m)
    [tutorial_4.py-1]     y: -0.223 (m)
    [tutorial_4.py-1]     z: 0.215 (m)
    [tutorial_4.py-1]   Orientation:
    [tutorial_4.py-1]     roll: 90°
    [tutorial_4.py-1]     pitch: 0°
    [tutorial_4.py-1]     yaw: -90°
    [tutorial_4.py-1]   Install direction:
    [tutorial_4.py-1]     x: 0.0
    [tutorial_4.py-1]     y: 0.0
    [tutorial_4.py-1]     z: -1.0
    [tutorial_4.py-1] Part: 🟥 Red Battery
    [tutorial_4.py-1]   Position:
    [tutorial_4.py-1]     x: -0.150 (m)
    [tutorial_4.py-1]     y: 0.035 (m)
    [tutorial_4.py-1]     z: 0.043 (m)
    [tutorial_4.py-1]   Orientation:
    [tutorial_4.py-1]     roll: 0°
    [tutorial_4.py-1]     pitch: 0°
    [tutorial_4.py-1]     yaw: 90°
    [tutorial_4.py-1]   Install direction:
    [tutorial_4.py-1]     x: 0.0
    [tutorial_4.py-1]     y: 1.0
    [tutorial_4.py-1]     z: 0.0
    [tutorial_4.py-1] Part: 🟥 Red Pump
    [tutorial_4.py-1]   Position:
    [tutorial_4.py-1]     x: 0.140 (m)
    [tutorial_4.py-1]     y: 0.000 (m)
    [tutorial_4.py-1]     z: 0.020 (m)
    [tutorial_4.py-1]   Orientation:
    [tutorial_4.py-1]     roll: 0°
    [tutorial_4.py-1]     pitch: 0°
    [tutorial_4.py-1]     yaw: -90°
    [tutorial_4.py-1]   Install direction:
    [tutorial_4.py-1]     x: 0.0
    [tutorial_4.py-1]     y: 0.0
    [tutorial_4.py-1]     z: -1.0
    [tutorial_4.py-1] Part: 🟥 Red Sensor
    [tutorial_4.py-1]   Position:
    [tutorial_4.py-1]     x: -0.100 (m)
    [tutorial_4.py-1]     y: 0.395 (m)
    [tutorial_4.py-1]     z: 0.045 (m)
    [tutorial_4.py-1]   Orientation:
    [tutorial_4.py-1]     roll: 0°
    [tutorial_4.py-1]     pitch: 0°
    [tutorial_4.py-1]     yaw: -90°
    [tutorial_4.py-1]   Install direction:
    [tutorial_4.py-1]     x: 0.0
    [tutorial_4.py-1]     y: -1.0
    [tutorial_4.py-1]     z: 0.0
    [tutorial_4.py-1] 
    [tutorial_4.py-1] [INFO] [1705524799.525134193] [competition_interface]: 
    [tutorial_4.py-1] 
    [tutorial_4.py-1] ==========================
    [tutorial_4.py-1] Received Order: 2IZJP320
    [tutorial_4.py-1] Priority: False
    [tutorial_4.py-1] Type: Combined
    [tutorial_4.py-1] ==========================
    [tutorial_4.py-1] Station: Assembly Station 3
    [tutorial_4.py-1] Products:
    [tutorial_4.py-1] ==========================
    [tutorial_4.py-1] Part: 🟧 Orange Pump
    [tutorial_4.py-1]   Position:
    [tutorial_4.py-1]     x: 0.140 (m)
    [tutorial_4.py-1]     y: 0.000 (m)
    [tutorial_4.py-1]     z: 0.020 (m)
    [tutorial_4.py-1]   Orientation:
    [tutorial_4.py-1]     roll: 0°
    [tutorial_4.py-1]     pitch: 0°
    [tutorial_4.py-1]     yaw: -90°
    [tutorial_4.py-1]   Install direction:
    [tutorial_4.py-1]     x: 0.0
    [tutorial_4.py-1]     y: 0.0
    [tutorial_4.py-1]     z: -1.0
    [tutorial_4.py-1] Part: 🟧 Orange Sensor
    [tutorial_4.py-1]   Position:
    [tutorial_4.py-1]     x: -0.100 (m)
    [tutorial_4.py-1]     y: 0.395 (m)
    [tutorial_4.py-1]     z: 0.045 (m)
    [tutorial_4.py-1]   Orientation:
    [tutorial_4.py-1]     roll: 0°
    [tutorial_4.py-1]     pitch: 0°
    [tutorial_4.py-1]     yaw: -90°
    [tutorial_4.py-1]   Install direction:
    [tutorial_4.py-1]     x: 0.0
    [tutorial_4.py-1]     y: -1.0
    [tutorial_4.py-1]     z: 0.0
    [tutorial_4.py-1] 
    [tutorial_4.py-1] [INFO] [1705524800.744276295] [competition_interface]: Competition state is: order_announcements_done
    [tutorial_4.py-1] [INFO] [1705524800.759254660] [competition_interface]: Ending competition
    [tutorial_4.py-1] [INFO] [1705524800.815803037] [competition_interface]: Ended competition.
    [tutorial_4.py-1] [INFO] [1705524802.753138266] [competition_interface]: Competition state is: ended


-------------------------------
Code explanation for Tutorial 4
-------------------------------

This is the node used for tutorial 4. The functions from competition_interface.py which are used are highlighted.

.. code-block:: python
    :caption: :file:`tutorial_4.py`
    :name: tutorial_4
    :emphasize-lines: 20

    #!/usr/bin/env python3

    import rclpy
    import threading
    from rclpy.executors import MultiThreadedExecutor
    from ariac_tutorials.competition_interface import CompetitionInterface
    from ariac_msgs.msg import CompetitionState

    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface(enable_moveit=False)
        executor = MultiThreadedExecutor()
        executor.add_node(interface)

        spin_thread = threading.Thread(target=executor.spin)
        spin_thread.start()

        # The following line enables order displays in the terminal.
        # Set to False to disable.
        interface.parse_incoming_order = True

        interface.start_competition()

        while rclpy.ok():
            try:
                if interface.get_competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
                    break
            except KeyboardInterrupt:
                break
        
        interface.end_competition()
        spin_thread.join()

    if __name__ == '__main__':
        main()

The purpose of this tutorial is to log orders received by :python:`CompetitionInterface`. When an order is published by the :topic:`/ariac/orders`, the :python:`interface._orders_cb` runs. The order msg which was published is then appended to :python:`interface._orders`. Then, since in the node :python:`interface.parse_incoming_order` was set to :python:`True`, the orders received are logged.
This is done using :python:`interface._parce_order`, where the order is parsed and returned as a :python:`string`.
