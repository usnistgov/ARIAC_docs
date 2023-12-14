
.. _TUTORIAL9:

*******************************************
Tutorial 9: Basic Interface With MoveIt_Py
*******************************************

.. admonition:: Tutorial 9
  :class: attention
  :name: tutorial_9

  - **Prerequisites:** :ref:`Introduction to Tutorials <TUTORIALS>` and :ref:`Tutorial 8 <TUTORIAL8>`
  - **Source Code**: `https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_9 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_9>`_ 
  - **Switch Branch**:

    .. code-block:: bash
        
            cd ~/ariac_ws/src/ariac_tutorials
            git switch tutorial_9


The code used in this tutorial is provided as a reference for how to use moveit_py to command the robots.
The main goal of this tutorial is to show how to use moveit_py to pick a part using the floor robot.
Competitors interested in using moveit_py to command robots are encouraged to use this code as a starting point.
Competitors are encouraged to learn more about moveit_py by reading the `MoveIt_Py documentation <https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html>`_.



Package Structure
=================

Updates and additions that are specific to :tuto:`Tutorial 9`  are highlighted in the tree below.

.. code-block:: text
    :emphasize-lines: 2, 5-7, 9, 12, 14, 24
    :class: no-copybutton
    
    ariac_tutorials
    ├── CMakeLists.txt
    ├── package.xml
    ├── config
    |   ├── collision_object_info.yaml
    |   ├── tutorial9_config.yaml
    │   └── sensors.yaml
    ├── launch
    │   └── tutorial9.launch.py
    ├── ariac_tutorials
    │   ├── __init__.py
    │   ├── utils.py
    |   ├── robot_commander.py
    │   └── competition_interface.py
    └── scripts
        ├── tutorial_1.py
        ├── tutorial_2.py
        ├── tutorial_3.py
        ├── tutorial_4.py
        ├── tutorial_5.py
        ├── tutorial_6.py
        ├── tutorial_7.py
        ├── tutorial_8.py
        └── tutorial_9.py

Updated/Created Files
=====================

Competition Interface
---------------------

.. code-block:: python
    :caption: :file:`competition_interface.py`
    :name: competitioninterface-tutorial10
    :linenos:
    :emphasize-lines: 134-138, 215-245, 665-930

    from time import sleep
    from math import pi
    from copy import copy
    import time
    from sympy import Quaternion
    from ament_index_python import get_package_share_directory
    from moveit import MoveItPy, PlanningSceneMonitor
    import rclpy
    import pyassimp
    import yaml
    from rclpy.time import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter import Parameter

    from geometry_msgs.msg import PoseStamped, Pose, Point
    from shape_msgs.msg import Mesh, MeshTriangle
    from moveit_msgs.msg import CollisionObject
    from std_msgs.msg import Header

    from moveit.core.robot_trajectory import RobotTrajectory
    from moveit.core.robot_state import RobotState, robotStateToRobotStateMsg
    from moveit_msgs.srv import GetCartesianPath, GetPositionFK

    from ariac_msgs.msg import (
        CompetitionState as CompetitionStateMsg,
        BreakBeamStatus as BreakBeamStatusMsg,
        AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
        Part as PartMsg,
        PartPose as PartPoseMsg,
        Order as OrderMsg,
        AssemblyPart as AssemblyPartMsg,
        AGVStatus as AGVStatusMsg,
        AssemblyTask as AssemblyTaskMsg,
        VacuumGripperState,
    )

    from ariac_msgs.srv import (
        MoveAGV,
        VacuumGripperControl
    )

    from std_srvs.srv import Trigger

    from ariac_tutorials.utils import (
        multiply_pose,
        rpy_from_quaternion,
        rad_to_deg_str,
        quaternion_from_euler,
        build_pose,
        AdvancedLogicalCameraImage,
        Order,
        KittingTask,
        CombinedTask,
        AssemblyTask,
        KittingPart
    )


    class Error(Exception):
    def __init__(self, value: str):
        self.value = value

    def __str__(self):
        return repr(self.value)
    
    class CompetitionInterface(Node):
        '''
        Class for a competition interface node.

        Args:
            Node (rclpy.node.Node): Parent class for ROS nodes

        Raises:
            KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
        '''
        _competition_states = {
            CompetitionStateMsg.IDLE: 'idle',
            CompetitionStateMsg.READY: 'ready',
            CompetitionStateMsg.STARTED: 'started',
            CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
            CompetitionStateMsg.ENDED: 'ended',
        }
        '''Dictionary for converting CompetitionState constants to strings'''

        _part_colors = {
            PartMsg.RED: 'red',
            PartMsg.BLUE: 'blue',
            PartMsg.GREEN: 'green',
            PartMsg.ORANGE: 'orange',
            PartMsg.PURPLE: 'purple',
        }
        '''Dictionary for converting Part color constants to strings'''

        _part_colors_emoji = {
            PartMsg.RED: '🟥',
            PartMsg.BLUE: '🟦',
            PartMsg.GREEN: '🟩',
            PartMsg.ORANGE: '🟧',
            PartMsg.PURPLE: '🟪',
        }
        '''Dictionary for converting Part color constants to emojis'''

        _part_types = {
            PartMsg.BATTERY: 'battery',
            PartMsg.PUMP: 'pump',
            PartMsg.REGULATOR: 'regulator',
            PartMsg.SENSOR: 'sensor',
        }
        '''Dictionary for converting Part type constants to strings'''

        _destinations = {
            AGVStatusMsg.KITTING: 'kitting station',
            AGVStatusMsg.ASSEMBLY_FRONT: 'front assembly station',
            AGVStatusMsg.ASSEMBLY_BACK: 'back assembly station',
            AGVStatusMsg.WAREHOUSE: 'warehouse',
        }
        '''Dictionary for converting AGVDestination constants to strings'''

        _stations = {
            AssemblyTaskMsg.AS1: 'assembly station 1',
            AssemblyTaskMsg.AS2: 'assembly station 2',
            AssemblyTaskMsg.AS3: 'assembly station 3',
            AssemblyTaskMsg.AS4: 'assembly station 4',
        }
        '''Dictionary for converting AssemblyTask constants to strings'''
        
        _gripper_states = {
            True: 'enabled',
            False: 'disabled'
        }
        '''Dictionary for converting VacuumGripperState constants to strings'''

        _part_heights = {PartMsg.BATTERY : 0.04,
                        PartMsg.PUMP : 0.12,
                        PartMsg.REGULATOR : 0.07,
                        PartMsg.SENSOR : 0.07}
        '''Dictionary for the heights of each part'''

        def __init__(self):
            super().__init__('competition_interface')

            sim_time = Parameter(
                "use_sim_time",
                rclpy.Parameter.Type.BOOL,
                True
            )

            self.set_parameters([sim_time])

            # Service client for starting the competition
            self._start_competition_client = self.create_client(Trigger, '/ariac/start_competition')

            # Subscriber to the competition state topic
            self._competition_state_sub = self.create_subscription(
                CompetitionStateMsg,
                '/ariac/competition_state',
                self._competition_state_cb,
                10)
            
            # Store the state of the competition
            self._competition_state: CompetitionStateMsg = None

            # Subscriber to the break beam status topic
            self._break_beam0_sub = self.create_subscription(
                BreakBeamStatusMsg,
                '/ariac/sensors/breakbeam_0/status',
                self._breakbeam0_cb,
                qos_profile_sensor_data)
            
            # Store the number of parts that crossed the beam
            self._conveyor_part_count = 0
            
            # Store whether the beam is broken
            self._object_detected = False

            # Subscriber to the logical camera topic
            self._advanced_camera0_sub = self.create_subscription(
                AdvancedLogicalCameraImageMsg,
                '/ariac/sensors/advanced_camera_0/image',
                self._advanced_camera0_cb,
                qos_profile_sensor_data)
            
            # Store each camera image as an AdvancedLogicalCameraImage object
            self._camera_image: AdvancedLogicalCameraImage = None

            # Subscriber to the order topic
            self.orders_sub = self.create_subscription(
                OrderMsg,
                '/ariac/orders',
                self._orders_cb,
                10)
            
            # Flag for parsing incoming orders
            self._parse_incoming_order = False
            
            # List of orders
            self._orders = []
            
            # Subscriber to the floor gripper state topic
            self._floor_robot_gripper_state_sub = self.create_subscription(
                VacuumGripperState,
                '/ariac/floor_robot_gripper_state',
                self._floor_robot_gripper_state_cb,
                qos_profile_sensor_data)

            # Service client for turning on/off the vacuum gripper on the floor robot
            self._floor_gripper_enable = self.create_client(
                VacuumGripperControl,
                "/ariac/floor_robot_enable_gripper")

            # Attribute to store the current state of the floor robot gripper
            self._floor_robot_gripper_state = VacuumGripperState()

            # Moveit_py variables
            self._ariac_robots = MoveItPy(node_name="ariac_robots_moveit_py")
            self._ariac_robots_state = RobotState(self._ariac_robots.get_robot_model())

            self._floor_robot = self._ariac_robots.get_planning_component("floor_robot")
            self._ceiling_robot = self._ariac_robots.get_planning_component("ceiling_robot")

            self._floor_robot_home_quaternion = Quaternion()
            self._ceiling_robot_home_quaternion = Quaternion()

            self._planning_scene_monitor = self._ariac_robots.get_planning_scene_monitor()

            # Parts found in the bins
            self._left_bins_parts = []
            self._right_bins_parts = []
            self._left_bins_camera_pose = Pose()
            self._right_bins_camera_pose = Pose()

            # service clients
            self.get_cartesian_path_client = self.create_client(GetCartesianPath, "compute_cartesian_path")
            self.get_position_fk_client = self.create_client(GetPositionFK, "compute_fk")

            # Camera subs
            self.left_bins_camera_sub = self.create_subscription(AdvancedLogicalCameraImageMsg,
                                                                "/ariac/sensors/left_bins_camera/image",
                                                                self._left_bins_camera_cb,
                                                                qos_profile_sensor_data)
            self.right_bins_camera_sub = self.create_subscription(AdvancedLogicalCameraImageMsg,
                                                                "/ariac/sensors/right_bins_camera/image",
                                                                self._right_bins_camera_cb,
                                                                qos_profile_sensor_data)


        @property
        def orders(self):
            return self._orders

        @property
        def camera_image(self):
            return self._camera_image

        @property
        def conveyor_part_count(self):
            return self._conveyor_part_count

        @property
        def parse_incoming_order(self):
            return self._parse_incoming_order

        @parse_incoming_order.setter
        def parse_incoming_order(self, value):
            self._parse_incoming_order = value

        def _orders_cb(self, msg: Order):
            '''Callback for the topic /ariac/orders
            Arguments:
                msg -- Order message
            '''
            order = Order(msg)
            self._orders.append(order)
            if self._parse_incoming_order:
                self.get_logger().info(self._parse_order(order))

        def _advanced_camera0_cb(self, msg: AdvancedLogicalCameraImageMsg):
            '''Callback for the topic /ariac/sensors/advanced_camera_0/image

            Arguments:
                msg -- AdvancedLogicalCameraImage message
            '''
            self._camera_image = AdvancedLogicalCameraImage(msg.part_poses,
                                                            msg.tray_poses,
                                                            msg.sensor_pose)

        def _breakbeam0_cb(self, msg: BreakBeamStatusMsg):
            '''Callback for the topic /ariac/sensors/breakbeam_0/status

            Arguments:
                msg -- BreakBeamStatusMsg message
            '''
            if not self._object_detected and msg.object_detected:
                self._conveyor_part_count += 1

            self._object_detected = msg.object_detected

        def _competition_state_cb(self, msg: CompetitionStateMsg):
            '''Callback for the topic /ariac/competition_state
            Arguments:
                msg -- CompetitionState message
            '''
            # Log if competition state has changed
            if self._competition_state != msg.competition_state:
                state = CompetitionInterface._competition_states[msg.competition_state]
                self.get_logger().info(f'Competition state is: {state}', throttle_duration_sec=1.0)
            
            self._competition_state = msg.competition_state
            
        def _floor_robot_gripper_state_cb(self, msg: VacuumGripperState):
            '''Callback for the topic /ariac/floor_robot_gripper_state

            Arguments:
                msg -- VacuumGripperState message
            '''
            self._floor_robot_gripper_state = msg

        def start_competition(self):
            '''Function to start the competition.
            '''
            self.get_logger().info('Waiting for competition to be ready')

            if self._competition_state == CompetitionStateMsg.STARTED:
                return
            # Wait for competition to be ready
            while self._competition_state != CompetitionStateMsg.READY:
                try:
                    rclpy.spin_once(self)
                except KeyboardInterrupt:
                    return

            self.get_logger().info('Competition is ready. Starting...')

            # Check if service is available
            if not self._start_competition_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error('Service \'/ariac/start_competition\' is not available.')
                return

            # Create trigger request and call starter service
            request = Trigger.Request()
            future = self._start_competition_client.call_async(request)

            # Wait until the service call is completed
            rclpy.spin_until_future_complete(self, future)

            if future.result().success:
                self.get_logger().info('Started competition.')
            else:
                self.get_logger().warn('Unable to start competition')

        def parse_advanced_camera_image(self, image: AdvancedLogicalCameraImage) -> str:
            '''
            Parse an AdvancedLogicalCameraImage message and return a string representation.
            '''
            
            if len(image._part_poses) == 0:
                return 'No parts detected'

            output = '\n\n'
            for i, part_pose in enumerate(image._part_poses):
                part_pose: PartPoseMsg
                output += '==========================\n'
                part_color = CompetitionInterface._part_colors[part_pose.part.color].capitalize()
                part_color_emoji = CompetitionInterface._part_colors_emoji[part_pose.part.color]
                part_type = CompetitionInterface._part_types[part_pose.part.type].capitalize()
                output += f'Part {i+1}: {part_color_emoji} {part_color} {part_type}\n'
                output += '--------------------------\n'
                output += 'Camera Frame\n'
                output += '--------------------------\n'
                
                output += '  Position:\n'
                output += f'    x: {part_pose.pose.position.x:.3f} (m)\n'
                output += f'    y: {part_pose.pose.position.y:.3f} (m)\n'
                output += f'    z: {part_pose.pose.position.z:.3f} (m)\n'

                roll, pitch, yaw = rpy_from_quaternion(part_pose.pose.orientation)
                output += '  Orientation:\n'
                output += f'    roll: {rad_to_deg_str(roll)}\n'
                output += f'    pitch: {rad_to_deg_str(pitch)}\n'
                output += f'    yaw: {rad_to_deg_str(yaw)}\n'
                
                part_world_pose = multiply_pose(image._sensor_pose, part_pose.pose)
                output += '--------------------------\n'
                output += 'World Frame\n'
                output += '--------------------------\n'

                output += '  Position:\n'
                output += f'    x: {part_world_pose.position.x:.3f} (m)\n'
                output += f'    y: {part_world_pose.position.y:.3f} (m)\n'
                output += f'    z: {part_world_pose.position.z:.3f} (m)\n'

                roll, pitch, yaw = rpy_from_quaternion(part_world_pose.orientation)
                output += '  Orientation:\n'
                output += f'    roll: {rad_to_deg_str(roll)}\n'
                output += f'    pitch: {rad_to_deg_str(pitch)}\n'
                output += f'    yaw: {rad_to_deg_str(yaw)}\n'

                output += '==========================\n\n'

            return output
        
        def _parse_kitting_task(self, kitting_task: KittingTask):
            '''
            Parses a KittingTask object and returns a string representation.
            Args:
                kitting_task (KittingTask): KittingTask object to parse
            Returns:
                str: String representation of the KittingTask object
            '''
            output = 'Type: Kitting\n'
            output += '==========================\n'
            output += f'AGV: {kitting_task.agv_number}\n'
            output += f'Destination: {CompetitionInterface._destinations[kitting_task.destination]}\n'
            output += f'Tray ID: {kitting_task.tray_id}\n'
            output += 'Products:\n'
            output += '==========================\n'

            quadrants = {1: "Quadrant 1: -",
                        2: "Quadrant 2: -",
                        3: "Quadrant 3: -",
                        4: "Quadrant 4: -"}

            for i in range(1, 5):
                product: KittingPart
                for product in kitting_task.parts:
                    if i == product.quadrant:
                        part_color = CompetitionInterface._part_colors[product.part.color].capitalize()
                        part_color_emoji = CompetitionInterface._part_colors_emoji[product.part.color]
                        part_type = CompetitionInterface._part_types[product.part.type].capitalize()
                        quadrants[i] = f'Quadrant {i}: {part_color_emoji} {part_color} {part_type}'
            output += f'\t{quadrants[1]}\n'
            output += f'\t{quadrants[2]}\n'
            output += f'\t{quadrants[3]}\n'
            output += f'\t{quadrants[4]}\n'

            return output

        def _parse_assembly_task(self, assembly_task: AssemblyTask):
            '''
            Parses an AssemblyTask object and returns a string representation.

            Args:
                assembly_task (AssemblyTask): AssemblyTask object to parse

            Returns:
                str: String representation of the AssemblyTask object
            '''
            output = 'Type: Assembly\n'
            output += '==========================\n'
            if len(assembly_task.agv_numbers) == 1:
                output += f'AGV: {assembly_task.agv_number[0]}\n'
            elif len(assembly_task.agv_numbers) == 2:
                output += f'AGV(s): [{assembly_task.agv_numbers[0]}, {assembly_task.agv_numbers[1]}]\n'
            output += f'Station: {self._stations[assembly_task.station].title()}\n'
            output += 'Products:\n'
            output += '==========================\n'

            product: AssemblyPartMsg
            for product in assembly_task.parts:
                part_color = CompetitionInterface._part_colors[product.part.color].capitalize()
                part_color_emoji = CompetitionInterface._part_colors_emoji[product.part.color]
                part_type = CompetitionInterface._part_types[product.part.type].capitalize()

                output += f'Part: {part_color_emoji} {part_color} {part_type}\n'

                output += '  Position:\n'
                output += f'    x: {product.assembled_pose.pose.position.x:.3f} (m)\n'
                output += f'    y: {product.assembled_pose.pose.position.y:.3f} (m)\n'
                output += f'    z: {product.assembled_pose.pose.position.z:.3f} (m)\n'

                roll, pitch, yaw = rpy_from_quaternion(product.assembled_pose.pose.orientation)
                output += '  Orientation:\n'
                output += f'    roll: {rad_to_deg_str(roll)}\n'
                output += f'    pitch: {rad_to_deg_str(pitch)}\n'
                output += f'    yaw: {rad_to_deg_str(yaw)}\n'

                output += f'  Install direction:\n'
                output += f'    x: {product.install_direction.x:.1f}\n'
                output += f'    y: {product.install_direction.y:.1f}\n'
                output += f'    z: {product.install_direction.z:.1f}\n'

            return output

        def _parse_combined_task(self, combined_task: CombinedTask):
            '''
            Parses a CombinedTask object and returns a string representation.

            Args:
                combined_task (CombinedTask): CombinedTask object to parse

            Returns:
                str: String representation of the CombinedTask object
            '''

            output = 'Type: Combined\n'
            output += '==========================\n'
            output += f'Station: {self._stations[combined_task.station].title()}\n'
            output += 'Products:\n'
            output += '==========================\n'

            product: AssemblyPartMsg
            for product in combined_task.parts:
                part_color = CompetitionInterface._part_colors[product.part.color].capitalize()
                part_color_emoji = CompetitionInterface._part_colors_emoji[product.part.color]
                part_type = CompetitionInterface._part_types[product.part.type].capitalize()

                output += f'Part: {part_color_emoji} {part_color} {part_type}\n'

                output += '  Position:\n'
                output += f'    x: {product.assembled_pose.pose.position.x:.3f} (m)\n'
                output += f'    y: {product.assembled_pose.pose.position.y:.3f} (m)\n'
                output += f'    z: {product.assembled_pose.pose.position.z:.3f} (m)\n'

                roll, pitch, yaw = rpy_from_quaternion(product.assembled_pose.pose.orientation)
                output += '  Orientation:\n'
                output += f'    roll: {rad_to_deg_str(roll)}\n'
                output += f'    pitch: {rad_to_deg_str(pitch)}\n'
                output += f'    yaw: {rad_to_deg_str(yaw)}\n'

                output += f'  Install direction:\n'
                output += f'    x: {product.install_direction.x:.1f}\n'
                output += f'    y: {product.install_direction.y:.1f}\n'
                output += f'    z: {product.install_direction.z:.1f}\n'

            return output

        def _parse_order(self, order: Order):
            '''Parse an order message and return a string representation.
            Args:
                order (Order) -- Order message
            Returns:
                String representation of the order message
            '''
            output = '\n\n==========================\n'
            output += f'Received Order: {order.order_id}\n'
            output += f'Priority: {order.order_priority}\n'

            if order.order_type == OrderMsg.KITTING:
                output += self._parse_kitting_task(order.order_task)
            elif order.order_type == OrderMsg.ASSEMBLY:
                output += self._parse_assembly_task(order.order_task)
            elif order.order_type == OrderMsg.COMBINED:
                output += self._parse_combined_task(order.order_task)
            else:
                output += 'Type: Unknown\n'
            return output

        def lock_agv_tray(self, num):
            '''
            Lock the tray of an AGV and parts on the tray. This will prevent tray and parts from moving during transport.
            Args:
                num (int):  AGV number
            Raises:
                KeyboardInterrupt: Exception raised when the user presses Ctrl+C
            '''

            # Create a client to send a request to the `/ariac/agv{num}_lock_tray` service
            tray_locker = self.create_client(
                Trigger,
                f'/ariac/agv{num}_lock_tray'
            )

            # Build the request
            request = Trigger.Request()
            # Send the request
            future = tray_locker.call_async(request)

            # Wait for the response
            try:
                rclpy.spin_until_future_complete(self, future)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

            # Check the response
            if future.result().success:
                self.get_logger().info(f'Locked AGV{num}\'s tray')
            else:
                self.get_logger().warn('Unable to lock tray')

        def move_agv_to_station(self, num, station):
            '''
            Move an AGV to an assembly station.
            Args:
                num (int): AGV number
                station (int): Assembly station number
            Raises:
                KeyboardInterrupt: Exception raised when the user presses Ctrl+C
            '''

            # Create a client to send a request to the `/ariac/move_agv` service.
            mover = self.create_client(
                MoveAGV,
                f'/ariac/move_agv{num}')

            # Create a request object.
            request = MoveAGV.Request()

            # Set the request location.
            if station in [AssemblyTaskMsg.AS1, AssemblyTaskMsg.AS3]:
                request.location = MoveAGV.Request.ASSEMBLY_FRONT
            else:
                request.location = MoveAGV.Request.ASSEMBLY_BACK

            # Send the request.
            future = mover.call_async(request)

            # Wait for the server to respond.
            try:
                rclpy.spin_until_future_complete(self, future)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

            # Check the result of the service call.
            if future.result().success:
                self.get_logger().info(f'Moved AGV{num} to {self._stations[station]}')
            else:
                self.get_logger().warn(future.result().message)  

        def set_floor_robot_gripper_state(self, state):
            '''Set the gripper state of the floor robot.

            Arguments:
                state -- True to enable, False to disable

            Raises:
                KeyboardInterrupt: Exception raised when the user presses Ctrl+C
            '''
            if self._floor_robot_gripper_state.enabled == state:
                self.get_logger().warn(f'Gripper is already {self._gripper_states[state]}')
                return

            request = VacuumGripperControl.Request()
            request.enable = state

            future = self._floor_gripper_enable.call_async(request)

            try:
                rclpy.spin_until_future_complete(self, future)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

            if future.result().success:
                self.get_logger().info(f'Changed gripper state to {self._gripper_states[state]}')
            else:
                self.get_logger().warn('Unable to change gripper state')

        def wait(self, duration):
            '''Wait for a specified duration.

            Arguments:
                duration -- Duration to wait in seconds

            Raises:
                KeyboardInterrupt: Exception raised when the user presses Ctrl+C
            '''
            start = self.get_clock().now()

            while self.get_clock().now() <= start + Duration(seconds=duration):
                try:
                    rclpy.spin_once(self)
                except KeyboardInterrupt as kb_error:
                    raise KeyboardInterrupt from kb_error
        
        def _call_get_cartesian_path (self, 
                                    waypoints : list,
                                    max_step : float):

            self.get_logger().info("Getting cartesian path")
            
            self._ariac_robots_state.update()

            request = GetCartesianPath.Request()

            header = Header()
            header.frame_id = "world"
            header.stamp = self.get_clock().now().to_msg()

            request.header = header
            request.start_state = robotStateToRobotStateMsg(self._ariac_robots_state)
            request.group_name = "floor_robot"
            request.link_name = "floor_gripper"
            request.waypoints = waypoints
            request.max_step = max_step

            future = self.get_cartesian_path_client.call_async(request)


            rclpy.spin_until_future_complete(self, future, timeout_sec=10)

            if not future.done():
                raise Error("Timeout reached when calling move_cartesian service")

            result: GetCartesianPath.Response
            result = future.result()

            return result.solution

        def _call_get_position_fk (self):

            self.get_logger().info("Getting cartesian path")

            request = GetPositionFK.Request()

            header = Header()
            header.frame_id = "world"
            header.stamp = self.get_clock().now().to_msg()
            request.header = header

            request.fk_link_names = ["floor_gripper"]
            self._ariac_robots_state.update()
            request.robot_state = robotStateToRobotStateMsg(self._ariac_robots_state)

            future = self.get_position_fk_client.call_async(request)


            rclpy.spin_until_future_complete(self, future, timeout_sec=10)

            if not future.done():
                raise Error("Timeout reached when calling move_cartesian service")

            result: GetPositionFK.Response
            result = future.result()

            return result.pose_stamped
        
        def _plan_and_execute(
            self,
            robot,
            planning_component,
            logger,
            single_plan_parameters=None,
            multi_plan_parameters=None,
            sleep_time=0.0,
        ):
            """Helper function to plan and execute a motion."""
            # plan to goal
            logger.info("Planning trajectory")
            if multi_plan_parameters is not None:
                plan_result = planning_component.plan(
                    multi_plan_parameters=multi_plan_parameters
                )
            elif single_plan_parameters is not None:
                plan_result = planning_component.plan(
                    single_plan_parameters=single_plan_parameters
                )
            else:
                plan_result = planning_component.plan()
            # execute the plan
            if plan_result:
                logger.info("Executing plan")
                robot_trajectory = plan_result.trajectory
                logger.info(str(robot_trajectory.joint_model_group_name))
                robot.execute(robot_trajectory, controllers=[])
            else:
                logger.error("Planning failed")

            sleep(sleep_time)

        def move_floor_robot_home(self):
            self._floor_robot.set_start_state_to_current_state()
            self._floor_robot.set_goal_state(configuration_name="home")
            self._plan_and_execute(self._ariac_robots,self._floor_robot, self.get_logger(), sleep_time=0.0)
            self._ariac_robots_state.update()
            self._floor_robot_home_quaternion = self._ariac_robots_state.get_pose("floor_gripper").orientation

        def _move_floor_robot_cartesian(self, waypoints):
            with self._planning_scene_monitor.read_write() as scene:
                # instantiate a RobotState instance using the current robot model
                self._ariac_robots_state = scene.current_state
                self._ariac_robots_state.update()

                fk_posestamped = self._call_get_position_fk()

                # Max step
                max_step = 0.1
                self._ariac_robots_state.update()
                trajectory_msg = self._call_get_cartesian_path(
                                            waypoints,
                                            max_step)
                self._ariac_robots_state.update()
                trajectory = RobotTrajectory(self._ariac_robots.get_robot_model())
                trajectory.set_robot_trajectory_msg(self._ariac_robots_state, trajectory_msg)
                trajectory.joint_model_group_name = "floor_robot"
            self._ariac_robots_state.update(True)
            self._ariac_robots.execute(trajectory, controllers=[])

        def _move_floor_robot_to_pose(self,pose : Pose):
            self.get_logger().info(str(pose))
            with self._planning_scene_monitor.read_write() as scene:
                self._floor_robot.set_start_state_to_current_state()

                pose_goal = PoseStamped()
                pose_goal.header.frame_id = "world"
                pose_goal.pose = pose
                self.get_logger().info(str(pose_goal.pose))
                self._floor_robot.set_goal_state(pose_stamped_msg=pose_goal, pose_link="floor_gripper")
            
            self._plan_and_execute(self._ariac_robots, self._floor_robot, self.get_logger())

        def _makeMesh(self, name, pose, filename) -> CollisionObject:
            with pyassimp.load(filename) as scene:
                assert len(scene.meshes)
                
                mesh = Mesh()
                for face in scene.meshes[0].faces:
                    triangle = MeshTriangle()
                    if hasattr(face, 'indices'):
                        if len(face.indices) == 3:
                            triangle.vertex_indices = [face.indices[0],
                                                        face.indices[1],
                                                        face.indices[2]]
                    else:
                        if len(face) == 3:
                            triangle.vertex_indices = [face[0],
                                                        face[1],
                                                        face[2]]
                    mesh.triangles.append(triangle)
                for vertex in scene.meshes[0].vertices:
                    point = Point()
                    point.x = float(vertex[0])
                    point.y = float(vertex[1])
                    point.z = float(vertex[2])
                    mesh.vertices.append(point)
                
            o = CollisionObject()
            o.header.frame_id = "world"
            o.id = name
            o.meshes.append(mesh)
            o.mesh_poses.append(pose)
            o.operation = o.ADD
            return o
        
        def _add_model_to_planning_scene(self,
                                        name : str,
                                        mesh_file : str,
                                        model_pose : Pose
                                        ):
            self.get_logger().info(f"Adding {name} to planning scene")
            package_share_directory = get_package_share_directory("test_competitor")
            model_path = package_share_directory + "/meshes/"+mesh_file
            collision_object = self._makeMesh(name, model_pose,model_path)
            with self._planning_scene_monitor.read_write() as scene:
                scene.apply_collision_object(collision_object)
                scene.current_state.update()
        
        def add_objects_to_planning_scene(self):
            package_share_directory = get_package_share_directory("ariac_tutorials")
            with open(package_share_directory+"/config/collision_object_info.yaml",'r') as object_file:
                objects_dict = yaml.safe_load(object_file)
            
            objects_dict : dict
            for key in objects_dict.keys():

                object_pose = Pose()
                
                object_pose.position.x = float(objects_dict[key]["position"][0])
                object_pose.position.y = float(objects_dict[key]["position"][1])
                object_pose.position.z = float(objects_dict[key]["position"][2])
                
                object_pose.orientation.x = float(objects_dict[key]["orientation"][0])
                object_pose.orientation.y = float(objects_dict[key]["orientation"][1])
                object_pose.orientation.z = float(objects_dict[key]["orientation"][2])
                object_pose.orientation.w = float(objects_dict[key]["orientation"][3])

                self._add_model_to_planning_scene(key, objects_dict[key]["file"], object_pose)
        
        def _left_bins_camera_cb(self,msg : AdvancedLogicalCameraImageMsg):
            self._left_bins_parts = msg.part_poses
            self._left_bins_camera_pose = msg.sensor_pose
        
        def _right_bins_camera_cb(self,msg : AdvancedLogicalCameraImageMsg):
            self._right_bins_parts = msg.part_poses
            self._right_bins_camera_pose = msg.sensor_pose

        def _floor_robot_wait_for_attach(self,timeout : float):
            start_time = time.time()
            while not self._floor_robot_gripper_state.attached:
                self._move_floor_robot_cartesian(0.0, 0.0, -0.001)
                sleep(0.2)
                if time.time()-start_time>=timeout:
                    self.get_logger().error("Unable to pick up part")

        def  floor_robot_pick_bin_part(self,part_to_pick : PartMsg):
            part_pose = Pose()
            found_part = False
            bin_side = ""
            
            print("HOLD")

            for part in self._left_bins_parts:
                part : PartPoseMsg
                if (part.part.type == part_to_pick.type and part.part.color == part_to_pick.color):
                    part_pose = multiply_pose(self._left_bins_camera_pose,part.pose)
                    found_part = True
                    bin_side = "left_bins"
                    break
            
            if not found_part:
                for part in self._right_bins_parts:
                    part : PartPoseMsg
                    if (part.part.type == part_to_pick.type and part.part.color == part_to_pick.color):
                        part_pose = multiply_pose(self._right_bins_camera_pose,part.pose)
                        found_part = True
                        bin_side = "right_bins"
                        break
            
            if not found_part:
                self.get_logger().error("Unable to locate part")
            else:
                self.get_logger().info(f"Part found in {bin_side}")

            part_rotation = rpy_from_quaternion(part_pose.orientation)[2]
            gripper_orientation = quaternion_from_euler(0.0,pi,part_rotation)
            self._move_floor_robot_to_pose(build_pose(part_pose.position.x, part_pose.position.y,
                                                    part_pose.position.z+0.5, gripper_orientation))

            waypoints = []
            waypoints.append(build_pose(part_pose.position.x, part_pose.position.y,
                                        part_pose.position.z+CompetitionInterface._part_heights[part_to_pick.type]+0.005,
                                        gripper_orientation))
            self._move_floor_robot_cartesian(waypoints)
            self.set_floor_robot_gripper_state(True)
            self._floor_robot_wait_for_attach(5.0)

            waypoints = []
            waypoints.append(build_pose(part_pose.position.x, part_pose.position.y,
                                        part_pose.position.z+0.5,
                                        gripper_orientation))
            self._move_floor_robot_cartesian(waypoints)


Code Explanation
^^^^^^^^^^^^^^^^
The competition interface from :ref:`Tutorial 8 <TUTORIAL8>` was augmented with the components described below.

- Instance Variables

    - :python:`_ariac_robots`: Instance of the MoveItPy class which enables the competition interface to interact with the robots using moveit_py.
    - :python:`_ariac_robots_state`: Holds the state of the robots to get the current pose and other information.
    - :python:`_floor_robot`: Moveit_py planning component fo the floor robot.
    - :python:`_ceiling_robot`: Moveit_py planning component fo the ceiling.
    - :python:`_floor_robot_home_quaternion`: Holds the orientation quaternion of the floor robot in the home position.
    - :python:`_ceiling_robot_home_quaternion`: Holds the orientation quaternion of the ceiling robot in the home position.
    - :python:`_planning_scene_monitor`: Planning scene monitor for the robots.
    - :python:`_left_bins_parts`: Holds information about the parts found in the left bins.
    - :python:`_right_bins_parts`: Holds information about the parts found in the right bins.
    - :python:`_left_bins_camera_pose`: Holds the pose of the camera above the left bins.
    - :python:`_right_bins_camera_pose`: Holds information about the parts found in the left bins.
    - :python:`get_cartesian_path_client`: Service client for creating a cartesian path through a set of waypoints.
    - :python:`get_position_fk_client`: Service client for getting the current pose.
    - :python:`left_bins_camera_sub`: Subscriber to the advanced logical camera image above the left bins.
    - :python:`right_bins_camera_sub`: Subscriber to the advanced logical camera image above the right bins.

- Instance Methods

    - :python:`_call_get_cartesian_path(self, waypoints)`: Private method used to get the cartesian calculate a trajectory with given waypoints.
    - :python:`_call_get_position_fk(self)`: Private method that returns the current pose of the floor gripper as a pose_stamped_msg.
    - :python:`_plan_and_execute(self, robot, planning_component, logger, single_plan_parameters, multi_plan_parameters, sleep_time)`: Private method that plans and executes a movement. This function is from the moveit_py documenation.
    - :python:`move_floor_robot_home(self)`: Public method that moves the floor robot to the home position.
    - :python:`_move_floor_robot_cartesian(self, waypoints)`: Private method that takes in a list of waypoints and uses :python:`_call_get_cartesian_path` method to move along the waypoints.
    - :python:`_move_floor_robot_to_pose(self, pose)`: Private method that takes in a pose, makes a pose_stamped, and uses :python:`_plan_and_execute` to move to the pose.
    - :python:`_makeMesh(self, name, pose, filename)`: Private method that uses the parameters to make a mesh and from that, returns a collision object.
    - :python:`_add_model_to_planning_scene(self, name, mesh_file, mesh_pose)`: Private method that opens the mesh file, gets the collision object from :python:`_makeMesh`, and applys the collision object to the planning scene.
    - :python:`add_objects_to_planning_scene(self)`: Public method that reads in the `collision_object_info.yaml` file as a dictionary. Then, it loops through each object and adds the model to the planning scene.
    - :python:`_left_bins_camera_cb(self,msg)`: Private method that gets the part poses and the sensor pose from the left bins camera.
    - :python:`_right_bins_camera_cb(self,msg)`: Private method that gets the part poses and the sensor pose from the right bins camera.
    - :python:`_floor_robot_wait_for_attach(self, timeout)`: Private method used to slowly move down until the part is picked up.
    - :python:`floor_robot_pick_bin_part(self, part_to_pick)`: Public method that uses moveit_py to pick up the type of part that was passed in.

collision_object_info.yaml
---------------

This file contains all of the information for the collision objects needed in the planning scene.

    .. code-block:: yaml

        bin1:
            position:
            - -1.9
            - 3.375
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: bin.stl
            bin2:
            position:
            - -1.9
            - 2.625
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: bin.stl
            bin3:
            position:
            - -1.9
            - -3.375
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: bin.stl
            bin4:
            position:
            - -1.9
            - -2.625
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: bin.stl
            bin5:
            position:
            - -2.65
            - 3.375
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: bin.stl
            bin6:
            position:
            - -2.65
            - 2.625
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: bin.stl
            bin7:
            position:
            - -2.65
            - -3.375
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: bin.stl
            bin8:
            position:
            - -2.65
            - -2.625
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: bin.stl
            as1:
            position:
            - -7.3
            - 3.0
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: assembly_station.stl
            as2:
            position:
            - -7.3
            - -3.0
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: assembly_station.stl
            as3:
            position:
            - -12.3
            - 3.0
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: assembly_station.stl
            as4:
            position:
            - -12.3
            - -3.0
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: assembly_station.stl
            assembly_insert1:
            position:
            - -7.7
            - 3.0
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: assembly_insert.stl
            assembly_insert2:
            position:
            - -7.7
            - -3.0
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: assembly_insert.stl
            assembly_insert3:
            position:
            - -12.7
            - 3.0
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: assembly_insert.stl
            assembly_insert4:
            position:
            - -12.7
            - -3.0
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: assembly_insert.stl
            conveyor:
            position:
            - -0.6
            - 0
            - 0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: conveyor.stl
            kts1_table:
            position:
            - -1.3
            - 5.84
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: kit_tray_table.stl
            kts2_table:
            position:
            - -1.3
            - -5.84
            - 0.0
            orientation:
            - 0.0
            - 0.0
            - 0.0
            - 1.0
            file: kit_tray_table.stl


tutorial9_config.yaml
---------------

This file launches the node with the correct settings for moveit_py.

    .. code-block:: yaml

        planning_scene_monitor_options:
            name: "planning_scene_monitor"
            robot_description: "robot_description"
            joint_state_topic: "/joint_states"
            attached_collision_object_topic: "/planning_scene_monitor"
            publish_planning_scene_topic: "/publish_planning_scene"
            monitored_planning_scene_topic: "/monitored_planning_scene"
            wait_for_initial_state_timeout: 10.0

            planning_pipelines:
            # !! NOTE: pipeline_names seem to be causing conflicts with package moveit_configs_utils default
            #          config files in the default_config folder, see NOTE in next section below for solution
            pipeline_names: ["ompl"]  #, "ompl_rrt_star"]


            # Default
            plan_request_params:
            planning_attempts: 1
            planning_pipeline: ompl
            planner_id: BiTRRT
            max_velocity_scaling_factor: 1.0
            max_acceleration_scaling_factor: 1.0
            planning_time: 1.0


            # !! NOTE: Make sure these namespaces are not the same names as what are in
            #          package moveit_configs_utils default config files in the default_config folder
            ompl_rrtc:  # Namespace for individual plan request
            plan_request_params:  # PlanRequestParameters similar to the ones that are used by the single pipeline planning of moveit_cpp
                planning_attempts: 1  # Number of attempts the planning pipeline tries to solve a given motion planning problem
                planning_pipeline: ompl  # Name of the pipeline that is being used
                planner_id: RRTConnect  # Name of the specific planner to be used by the pipeline
                max_velocity_scaling_factor: 1.0  # Velocity scaling parameter for the trajectory generation algorithm that is called (if configured) after the path planning
                max_acceleration_scaling_factor: 1.0  # Acceleration scaling parameter for the trajectory generation algorithm that is called (if configured) after the path planning
                planning_time: 1.0  # Time budget for the motion plan request. If the planning problem cannot be solved within this time, an empty solution with error code is returned

            pilz_lin:
            plan_request_params:
                planning_attempts: 1
                planning_pipeline: pilz_industrial_motion_planner
                planner_id: PTP
                max_velocity_scaling_factor: 1.0
                max_acceleration_scaling_factor: 1.0
                planning_time: 0.8

            chomp_b:  # This was changed because it conflicts with the chomp default config in moveit_configs_utils
            plan_request_params:
                planning_attempts: 1
                planning_pipeline: chomp
                planner_id: chomp
                max_velocity_scaling_factor: 1.0
                max_acceleration_scaling_factor: 1.0
                planning_time: 1.5

            # Second OMPL pipeline
            ompl_rrt_star:
            plan_request_params:
                planning_attempts: 1
                # planning_pipeline: ompl_rrt_star # Different OMPL pipeline name!  # Original, but gave errors in runtime
                planning_pipeline: ompl
                planner_id: RRTstar
                max_velocity_scaling_factor: 1.0
                max_acceleration_scaling_factor: 1.0
                planning_time: 1.5

            stomp_b:  # Added this
            plan_request_params:
                planning_attempts: 1
                planning_pipeline: stomp
                planner_id: stomp
                max_velocity_scaling_factor: 1.0
                max_acceleration_scaling_factor: 1.0
                planning_time: 1.5

tutorial9.launch.py
---------------

This file launches the main node for this tutorial.

    .. code-block:: python

        import os
        from pytest import param
        import yaml

        from launch import LaunchDescription
        from launch.actions import (
            DeclareLaunchArgument,
            OpaqueFunction,
        )

        from moveit_configs_utils import MoveItConfigsBuilder

        from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
        from launch.conditions import IfCondition
        from launch_ros.actions import Node

        from launch_ros.substitutions import FindPackageShare

        from ament_index_python.packages import get_package_share_directory


        def load_file(package_name, file_path):
            package_path = get_package_share_directory(package_name)
            absolute_file_path = os.path.join(package_path, file_path)

            try:
                with open(absolute_file_path, "r") as file:
                    return file.read()
            except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
                return None

        def load_yaml(package_name, file_path):
            package_path = get_package_share_directory(package_name)
            absolute_file_path = os.path.join(package_path, file_path)

            try:
                with open(absolute_file_path, "r") as file:
                    return yaml.safe_load(file)
            except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
                return None


        def launch_setup(context, *args, **kwargs):

            urdf = os.path.join(get_package_share_directory("ariac_description"), "urdf/ariac_robots/ariac_robots.urdf.xacro")

            moveit_config = (
                MoveItConfigsBuilder("ariac_robots", package_name="ariac_moveit_config")
                .robot_description(urdf)
                .robot_description_semantic(file_path="config/ariac_robots.srdf")
                .trajectory_execution(file_path="config/moveit_controllers.yaml")
                .planning_pipelines(pipelines=["ompl"])
                .joint_limits(file_path="config/joint_limits.yaml")
                .moveit_cpp(
                    file_path=get_package_share_directory("ariac_tutorials")
                    + "/config/tutorial9_config.yaml"
                )
                .to_moveit_configs()
            )

            trajectory_execution = {
                "moveit_manage_controllers": False,
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "trajectory_execution.allowed_start_tolerance": 0.01,
            }

            planning_scene_monitor_parameters = {
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            }
            
            parameters_dict = moveit_config.to_dict()
            parameters_dict["use_sim_time"] = True
            parameters_dict.update(trajectory_execution)
            parameters_dict.update(planning_scene_monitor_parameters)
            moveit_py_test = Node(
                package="ariac_tutorials",
                executable="tutorial_9.py",
                output="screen",
                parameters=[
                    parameters_dict
                ],
            )
            start_rviz = LaunchConfiguration("rviz")

            rviz_config_file = PathJoinSubstitution(
                [FindPackageShare("test_competitor"), "rviz", "test_competitor.rviz"]
            )

            rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_moveit",
                output="log",
                arguments=["-d", rviz_config_file],
                parameters=[
                    moveit_config.to_dict(),
                    {"use_sim_time": True},
                ],
                condition=IfCondition(start_rviz)
            )

            nodes_to_start = [
                moveit_py_test,
                rviz_node
            ]

            return nodes_to_start

        def generate_launch_description():
            declared_arguments = []

            declared_arguments.append(
                DeclareLaunchArgument("rviz", default_value="false", description="start rviz node?")
            )
            return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

tutorial9.py
---------------

This is the main node for this tutorial. It makes a part object, and then uses the new code in the competition interface to pick up a part that matches.

    .. code-block:: python
        #!/usr/bin/env python3
        '''
        To test this script, run the following commands in separate terminals:
        - ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials
        - ros2 run ariac_tutorials tutorial_9.py
        '''

        import rclpy
        from ariac_tutorials.competition_interface import CompetitionInterface
        from ariac_msgs.msg import Part

        def main(args=None):
            rclpy.init(args=args)
            interface = CompetitionInterface()

            interface.start_competition()
            interface.wait(3)
            interface.get_logger().info("Competition started. Adding collision objects to planning scene")

            part_to_pick = Part()
            part_to_pick.type = Part.PUMP
            part_to_pick.color = Part.PURPLE

            interface.add_objects_to_planning_scene()
            interface.move_floor_robot_home()
            interface.floor_robot_pick_bin_part(part_to_pick)

            interface.destroy_node()
            rclpy.shutdown()


        if __name__ == '__main__':
            main()

Run the Executable
==================

- In *terminal 1*, run the following commands:

    .. code-block:: bash

        cd ~/ariac_ws
        colcon build
        . install/setup.bash
        ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py

    This will allow the robots to be controlled using moveit.

- In *terminal 2*, run the following commands:
    .. code-block:: bash

        cd ~/ariac_ws
        colcon build
        . install/setup.bash
        ros2 launch ariac_tutorials tutorial9.launch.py


    The node will wait until the competition is ready.


- In *terminal 3*, run the following commands:

    .. code-block:: bash

        cd ~/ariac_ws
        . install/setup.bash
        ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials dev_mode:=True


Outputs
=======

The floor robot will move to the home position. Then, it will pick up a purple pump.


