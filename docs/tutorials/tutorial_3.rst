.. _TUTORIAL3:

=====================================================
Tutorial 3: Read Data from an Advanced Logical Camera
=====================================================

.. admonition:: Tutorial 3
  :class: attention
  :name: tutorial_3

  - **Prerequisites:** :ref:`Introduction to Tutorials <TUTORIALS>`
  - **Source Code**: `https://github.com/usnistgov/ARIAC_tutorials <https://github.com/usnistgov/ARIAC_tutorials>`_

This tutorial details the steps necessary to start the competition from a competitor package.

---------------------------------------
Starting the environment for Tutorial 3
---------------------------------------

To start the enviornment, use this command:

.. code-block:: bash
        
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorial dev_mode:=True

------------------
Running tutorial 3
------------------

To start tutorial 3, open a new terminal and use this command:

.. code-block:: bash
        
    ros2 launch ariac_tutorials tutorial.launch.py tutorial:=3

-----------------------------
Expected output of tutorial 3
-----------------------------

.. code-block:: console
    :caption: Tutorial 3 output
    :class: no-copybutton

    [tutorial_3.py-1] [INFO] [1705523791.505330909] [competition_interface]: Waiting for competition to be ready
    [tutorial_3.py-1] [INFO] [1705523812.096887359] [competition_interface]: Competition state is: idle
    [tutorial_3.py-1] [INFO] [1705523926.539425501] [competition_interface]: Competition state is: ready
    [tutorial_3.py-1] [INFO] [1705523926.561503733] [competition_interface]: Competition is ready. Starting...
    [tutorial_3.py-1] [INFO] [1705523926.610670594] [competition_interface]: Started competition.
    [tutorial_3.py-1] [INFO] [1705523926.613540083] [competition_interface]: Getting parts from bin 2
    [tutorial_3.py-1] [INFO] [1705523926.616115248] [competition_interface]: No image received yet
    [tutorial_3.py-1] [INFO] [1705523927.618836767] [competition_interface]: No image received yet
    [tutorial_3.py-1] [INFO] [1705523928.144099448] [competition_interface]: Competition state is: started
    [tutorial_3.py-1] [INFO] [1705523928.632360416] [competition_interface]: No image received yet
    [tutorial_3.py-1] [INFO] [1705523929.636587739] [competition_interface]: No image received yet
    [tutorial_3.py-1] [INFO] [1705523931.862293284] [competition_interface]: Slot 1: purple pump
    [tutorial_3.py-1] [INFO] [1705523931.864624017] [competition_interface]: Slot 2: Empty
    [tutorial_3.py-1] [INFO] [1705523931.866752218] [competition_interface]: Slot 3: purple pump
    [tutorial_3.py-1] [INFO] [1705523931.869826096] [competition_interface]: Slot 4: Empty
    [tutorial_3.py-1] [INFO] [1705523931.872064874] [competition_interface]: Slot 5: Empty
    [tutorial_3.py-1] [INFO] [1705523931.874144258] [competition_interface]: Slot 6: Empty
    [tutorial_3.py-1] [INFO] [1705523931.877048187] [competition_interface]: Slot 7: purple pump
    [tutorial_3.py-1] [INFO] [1705523931.880235581] [competition_interface]: Slot 8: Empty
    [tutorial_3.py-1] [INFO] [1705523931.883262169] [competition_interface]: Slot 9: Empty
    [tutorial_3.py-1] [INFO] [1705523931.886214625] [competition_interface]: Ending competition
    [tutorial_3.py-1] [INFO] [1705523931.923620397] [competition_interface]: Ended competition.

-------------------------------
Code explanation for Tutorial 3
-------------------------------

This is the node used for tutorial 3. The functions from competition_interface.py which are used are highlighted.

.. code-block:: python
    :caption: :file:`tutorial_3.py`
    :name: tutorial_3
    :emphasize-lines: 21,23,28

    #!/usr/bin/env python3

    import rclpy
    import threading
    from rclpy.executors import MultiThreadedExecutor
    from ariac_tutorials.competition_interface import CompetitionInterface
    from time import sleep

    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface(enable_moveit=False)
        executor = MultiThreadedExecutor()
        executor.add_node(interface)

        spin_thread = threading.Thread(target=executor.spin)
        spin_thread.start()
        interface.start_competition()

        # Turns on a debug topic to visualize bounding boxes and slots
        # /ariac/sensors/display_bounding_boxes
        interface.display_bounding_boxes = True
        
        bin_number = 2

        interface.get_logger().info(f"Getting parts from bin {bin_number}")
        bin_parts = None
        while bin_parts is None:
            bin_parts = interface.get_bin_parts(bin_number)
            sleep(1)
        if bin_parts:
            for _slot_number, _part in bin_parts.items():
                if _part.type is None:
                    interface.get_logger().info(f"Slot {_slot_number}: Empty")
                else:
                    interface.get_logger().info(f"Slot {_slot_number}: {_part.color} {_part.type}")
        
        interface.end_competition()
        interface.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

The purpose of this tutorial is to use RGB cameras to detect the parts on a chosen bin. The two cameras, :python:`left_bins_rgb_camera` and :python:`right_bins_rgb_camera`, are subscribed to and the image msg is saved in :python:`_left_bins_camera_image` and :python:`_right_bins_camera_image`. The first step of the process of finding parts in the image is to read in an image frame using :python:`imgmsg_to_csv2`. This function will convert the image msg from the camera and return it as an OpenCV image. The second step is finding parts within the image. This is done using the :python:`interface.find_parts` function, where the possible colors and types are looped through. An image mask is created using the color and using the type, a template is used to find the location of any parts of that type and color. The coordinates of where the parts are found are then saved in :python:`interface.part_poses` and :python:`interface.centered_part_poses`. The third step is to organize the parts by slots in the bin. This is done using the :python:`interface.output_by_slot`, where the coordinates saved in :python:`interface.centered_part_poses` are used to find which slots in the bin that the parts are in. After this step, the dictionary containing the slot numbers as the keys and part in the slot as the value is returned and the node logs them.
