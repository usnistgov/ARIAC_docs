.. _TUTORIAL1:

====================================
Tutorial 1: Starting the competition
====================================

.. admonition:: Tutorial 1
  :class: attention
  :name: tutorial_1

  - **Prerequisites:** :ref:`Introduction to Tutorials <TUTORIALS>`
  - **Source Code**: `https://github.com/usnistgov/ARIAC_tutorials <https://github.com/usnistgov/ARIAC_tutorials>`_

This tutorial details the steps necessary to start the competition from a competitor package.

---------------------------------------
Starting the environment for Tutorial 1
---------------------------------------

To start the environment, use this command:

.. code-block:: bash
        
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorial dev_mode:=True

------------------
Running tutorial 1
------------------

To start tutorial 1, open a new terminal and use this command:

.. code-block:: bash
        
    ros2 launch ariac_tutorials tutorial.launch.py tutorial:=1

-----------------------------
Expected output of tutorial 1
-----------------------------

.. code-block:: console
    :caption: Tutorial 1 output
    :class: no-copybutton

    [tutorial_1.py-1] [INFO] [1705514939.446586830] [competition_interface]: Waiting for competition to be ready
    [tutorial_1.py-1] [INFO] [1705514964.593999751] [competition_interface]: Competition state is: idle
    [tutorial_1.py-1] [INFO] [1705515096.252907877] [competition_interface]: Competition state is: ready
    [tutorial_1.py-1] [INFO] [1705515096.257258480] [competition_interface]: Competition is ready. Starting...
    [tutorial_1.py-1] [INFO] [1705515096.322821850] [competition_interface]: Started competition.
    [tutorial_1.py-1] [INFO] [1705515099.154943257] [competition_interface]: Competition state is: started
    [tutorial_1.py-1] [INFO] [1705515326.346296824] [competition_interface]: Competition state is: order_announcements_done

-------------------------------
Code explanation for Tutorial 1
-------------------------------

This is the node used for tutorial 1. The functions from competition_interface.py which are used are highlighted.

.. code-block:: python
    :caption: :file:`tutorial_1.py`
    :name: tutorial_1
    :emphasize-lines: 19, 24

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

        interface.start_competition()

        while not interface.get_competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
            pass
        
        interface.end_competition()
        spin_thread.join()


    if __name__ == '__main__':
        main()

First, an instance of the :python:`CompetitionInterface` is created with :python:`enable_moveit` set to :python:`False`, as moveit_py is not needed for this tutorial.
Then, an executor is created containing an instance of :python:`CompetitionInterface`. After this, a thread is created to spin the executor.
The competition is then started using the :python:`start_competition` method in :python:`CompetitionInterface`. This uses the `/ariac/start_competition` service to start the competition.
The node then waits until the competition state is `ORDER_ANNOUNCEMENTS_DONE`. Finally, the competition is ended using the `/ariac/end_competition` service and the thread is joined with the main thread.