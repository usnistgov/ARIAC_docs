
.. _TUTORIAL10:

*******************************************
Tutorial 10: Completing A Kitting Order Using MoveIt_Py
*******************************************

.. admonition:: Tutorial 10
  :class: attention
  :name: tutorial_10

  - **Prerequisites:** :ref:`Introduction to Tutorials <TUTORIALS>` and :ref:`Tutorial 9 <TUTORIAL9>`
  - **Source Code**: `https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_10 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_10>`_ 
  - **Switch Branch**:

    .. code-block:: bash
        
            cd ~/ariac_ws/src/ariac_tutorials
            git switch tutorial_10


The code used in this tutorial is provided as a reference for how to use moveit_py to command the robots to complete a kitting order.
The main goal of this tutorial is to show how to use moveit_py to complete a kitting order.
Competitors interested in using moveit_py to command robots are encouraged to use this code as a starting point.
Competitors are encouraged to learn more about moveit_py by reading the `MoveIt_Py documentation <https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html>`_.



Package Structure
=================

Updates and additions that are specific to :tuto:`Tutorial 10`  are highlighted in the tree below.

.. code-block:: text
    :emphasize-lines: 2, 6, 9, 12, 14, 25
    :class: no-copybutton
    
    ariac_tutorials
    ├── CMakeLists.txt
    ├── package.xml
    ├── config
    |   ├── collision_object_info.yaml
    |   ├── tutorial10_config.yaml
    │   └── sensors.yaml
    ├── launch
    │   └── tutorial10.launch.py
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
        ├── tutorial_9.py
        └── tutorial_10.py

Updated/Created Files
=====================

Competition Interface
---------------------

.. code-block:: python
    :caption: :file:`competition_interface.py`
    :name: competitioninterface-tutorial10
    :linenos:
    :emphasize-lines: 154-157, 304-331, 1146-1317

    from argparse import _MutuallyExclusiveGroup
    from distutils.command import build
    from time import sleep
    from math import cos, sin, pi
    from copy import copy
    import time
    import PyKDL
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
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

    from geometry_msgs.msg import PoseStamped, Pose, Point, TransformStamped
    from shape_msgs.msg import Mesh, MeshTriangle
    from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
    from std_msgs.msg import Header

    from moveit.core.robot_trajectory import RobotTrajectory
    from moveit.core.robot_state import RobotState, robotStateToRobotStateMsg
    from moveit_msgs.srv import GetCartesianPath, GetPositionFK, ApplyPlanningScene, GetPlanningScene
    from moveit.core.kinematic_constraints import construct_joint_constraint

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
        AssemblyState as AssemblyStateMsg,
        CombinedTask as CombinedTaskMsg,
        VacuumGripperState,
    )

    from ariac_msgs.srv import (
        MoveAGV,
        VacuumGripperControl,
        ChangeGripper,
        SubmitOrder,
        GetPreAssemblyPoses
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

    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


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

        _quad_offsets = {1 : (-0.08, 0.12),
                        2 : (0.08, 0.12),
                        3 : (-0.08, -0.12),
                        4 : (0.08, -0.12)}

        _rail_positions = {"agv1":-4.5,
                        "agv2":-1.2,
                        "agv3":1.2,
                        "agv4":4.5,
                        "left_bins":3,
                        "right_bins":-3}

        def __init__(self):
            super().__init__('competition_interface')

            sim_time = Parameter(
                "use_sim_time",
                rclpy.Parameter.Type.BOOL,
                True
            )

            self.set_parameters([sim_time])
            
            # ROS2 callback groups
            self.ariac_cb_group = MutuallyExclusiveCallbackGroup()
            self.moveit_cb_group = MutuallyExclusiveCallbackGroup()
            self.orders_cb_group = ReentrantCallbackGroup()

            # Service client for starting the competition
            self._start_competition_client = self.create_client(Trigger, '/ariac/start_competition')

            # Subscriber to the competition state topic
            self._competition_state_sub = self.create_subscription(
                CompetitionStateMsg,
                '/ariac/competition_state',
                self._competition_state_cb,
                10,
                callback_group=self.ariac_cb_group)
            
            # Store the state of the competition
            self._competition_state: CompetitionStateMsg = None
            
            # Store the number of parts that crossed the beam
            self._conveyor_part_count = 0
            
            # Store whether the beam is broken
            self._object_detected = False

            # Store each camera image as an AdvancedLogicalCameraImage object
            self._camera_image: AdvancedLogicalCameraImage = None

            # Subscriber to the order topic
            self.orders_sub = self.create_subscription(
                OrderMsg,
                '/ariac/orders',
                self._orders_cb,
                10,
                callback_group=self.orders_cb_group)
            
            # Flag for parsing incoming orders
            self._parse_incoming_order = True
            
            # List of orders
            self._orders = []
            
            # Subscriber to the floor gripper state topic
            self._floor_robot_gripper_state_sub = self.create_subscription(
                VacuumGripperState,
                '/ariac/floor_robot_gripper_state',
                self._floor_robot_gripper_state_cb,
                qos_profile_sensor_data,
                callback_group=self.ariac_cb_group)
            
            # Subscriber to the ceiling gripper state topic
            self._ceiling_robot_gripper_state_sub = self.create_subscription(
                VacuumGripperState,
                '/ariac/ceiling_robot_gripper_state',
                self._ceiling_robot_gripper_state_cb,
                qos_profile_sensor_data,
                callback_group=self.ariac_cb_group)

            # Service client for turning on/off the vacuum gripper on the floor robot
            self._floor_gripper_enable = self.create_client(
                VacuumGripperControl,
                "/ariac/floor_robot_enable_gripper")
            
            # Service client for turning on/off the vacuum gripper on the ceiling robot
            self._ceiling_gripper_enable = self.create_client(
                VacuumGripperControl,
                "/ariac/ceiling_robot_enable_gripper")

            # Attribute to store the current state of the floor robot gripper
            self._floor_robot_gripper_state = VacuumGripperState()

            # Attribute to store the current state of the ceiling robot gripper
            self._ceiling_robot_gripper_state = VacuumGripperState()

            # Moveit_py variables
            self._ariac_robots = MoveItPy(node_name="ariac_robots_moveit_py")
            self._ariac_robots_state = RobotState(self._ariac_robots.get_robot_model())

            self._floor_robot = self._ariac_robots.get_planning_component("floor_robot")
            self._ceiling_robot = self._ariac_robots.get_planning_component("ceiling_robot")

            self._floor_robot_home_quaternion = Quaternion()
            self._ceiling_robot_home_quaternion = Quaternion()

            self._planning_scene_monitor = self._ariac_robots.get_planning_scene_monitor()

            self._world_collision_objects = []

            # Parts found in the bins
            self._left_bins_parts = []
            self._right_bins_parts = []
            self._left_bins_camera_pose = Pose()
            self._right_bins_camera_pose = Pose()

            # Tray information
            self._kts1_trays = []
            self._kts2_trays = []
            self._kts1_camera_pose = Pose()
            self._kts2_camera_pose = Pose()

            # service clients
            self.get_cartesian_path_client = self.create_client(GetCartesianPath, "compute_cartesian_path")
            self.get_position_fk_client = self.create_client(GetPositionFK, "compute_fk")

            # Camera subs
            self.left_bins_camera_sub = self.create_subscription(AdvancedLogicalCameraImageMsg,
                                                                "/ariac/sensors/left_bins_camera/image",
                                                                self._left_bins_camera_cb,
                                                                qos_profile_sensor_data,
                                                                callback_group=self.moveit_cb_group)
            self.right_bins_camera_sub = self.create_subscription(AdvancedLogicalCameraImageMsg,
                                                                "/ariac/sensors/right_bins_camera/image",
                                                                self._right_bins_camera_cb,
                                                                qos_profile_sensor_data,
                                                                callback_group=self.moveit_cb_group)
            self.kts1_camera_sub_ = self.create_subscription(AdvancedLogicalCameraImageMsg,
                                                            "/ariac/sensors/kts1_camera/image",
                                                            self._kts1_camera_cb,
                                                            qos_profile_sensor_data,
                                                                callback_group=self.moveit_cb_group)
            self.kts2_camera_sub_ = self.create_subscription(AdvancedLogicalCameraImageMsg,
                                                            "/ariac/sensors/kts2_camera/image",
                                                            self._kts2_camera_cb,
                                                            qos_profile_sensor_data,
                                                            callback_group=self.moveit_cb_group)
            
            # AGV status subs
            self._agv_locations = {i+1:-1 for i in range(4)}
            
            self.agv1_status_sub = self.create_subscription(AGVStatusMsg,
                                                            "/ariac/agv1_status",
                                                            self._agv1_status_cb,
                                                            10,
                                                            callback_group=self.moveit_cb_group)
            self.agv2_status_sub = self.create_subscription(AGVStatusMsg,
                                                            "/ariac/agv2_status",
                                                            self._agv2_status_cb,
                                                            10,
                                                            callback_group=self.moveit_cb_group)
            self.agv3_status_sub = self.create_subscription(AGVStatusMsg,
                                                            "/ariac/agv3_status",
                                                            self._agv3_status_cb,
                                                            10,
                                                            callback_group=self.moveit_cb_group)
            self.agv4_status_sub = self.create_subscription(AGVStatusMsg,
                                                            "/ariac/agv4_status",
                                                            self._agv4_status_cb,
                                                            10,
                                                            callback_group=self.moveit_cb_group)
            
            # TF
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.tf_broadcaster = StaticTransformBroadcaster(self)
            self.static_transforms = []

            self.floor_robot_attached_part_ = PartMsg()
            self.ceiling_robot_attached_part_ = PartMsg()

            self._change_gripper_client = self.create_client(ChangeGripper, "/ariac/floor_robot_change_gripper")
            
            # Planning Scene Info
            self.planning_scene_sub = self.create_subscription(PlanningScene,
                                                            "/planning_scene",
                                                                self.get_planning_scene_msg,
                                                                10,
                                                                callback_group=self.moveit_cb_group)
            self.planning_scene_msg = PlanningScene()

            # Meshes file path
            self.mesh_file_path = get_package_share_directory("test_competitor") + "/meshes/"

            
            self.floor_joint_positions_arrs = {
                "floor_kts1_js_":[4.0,1.57,-1.57,1.57,-1.57,-1.57,0.0],
                "floor_kts2_js_":[-4.0,-1.57,-1.57,1.57,-1.57,-1.57,0.0]
            }
            self.floor_position_dict = {key:self._create_floor_joint_position_state(self.floor_joint_positions_arrs[key])
                                        for key in self.floor_joint_positions_arrs.keys()}

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

        def _orders_cb(self, msg: OrderMsg):
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
        
        def _ceiling_robot_gripper_state_cb(self, msg: VacuumGripperState):
            '''Callback for the topic /ariac/floor_ceiling_gripper_state

            Arguments:
                msg -- VacuumGripperState message
            '''
            self._ceiling_robot_gripper_state = msg

        def start_competition(self):
            '''Function to start the competition.
            '''
            self.get_logger().info('Waiting for competition to be ready')

            if self._competition_state == CompetitionStateMsg.STARTED:
                return
            # Wait for competition to be ready
            while self._competition_state != CompetitionStateMsg.READY:
                # try:
                #     rclpy.spin_once(self)
                # except KeyboardInterrupt:
                #     return
                pass

            self.get_logger().info('Competition is ready. Starting...')

            # Check if service is available
            if not self._start_competition_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error('Service \'/ariac/start_competition\' is not available.')
                return

            # Create trigger request and call starter service
            request = Trigger.Request()
            future = self._start_competition_client.call_async(request)

            while not future.done():
                pass
            # Wait until the service call is completed
            # rclpy.spin_until_future_complete(self, future)

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

        def set_ceiling_robot_gripper_state(self, enable):
            if self._ceiling_robot_gripper_state.enabled == enable:
                if self._ceiling_robot_gripper_state:
                    self.get_logger().info("Already enabled")
                else:
                    self.get_logger().info("Already disabled")
                return

            request = VacuumGripperControl.Request()
            request.enable = enable

            future = self._ceiling_gripper_enable.call_async(request)

            try:
                rclpy.spin_until_future_complete(self, future)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

            if future.result().success:
                self.get_logger().info(f'Changed gripper state to {self._gripper_states[enable]}')
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
        
        def _call_get_cartesian_path (self, waypoints : list, 
                                    max_velocity_scaling_factor : float, 
                                    max_acceleration_scaling_factor : float,
                                    avoid_collision : bool,
                                    robot : str):

            self.get_logger().info("Getting cartesian path")
            self._ariac_robots_state.update()

            request = GetCartesianPath.Request()

            header = Header()
            header.frame_id = "world"
            header.stamp = self.get_clock().now().to_msg()

            request.header = header
            request.start_state = robotStateToRobotStateMsg(self._ariac_robots_state)
            if robot == "floor_robot":
                request.group_name = "floor_robot"
                request.link_name = "floor_gripper"
            else:
                request.group_name = "ceiling_robot"
                request.link_name = "ceiling_gripper"
            request.waypoints = waypoints
            request.max_step = 0.1
            request.avoid_collisions = avoid_collision
            request.max_velocity_scaling_factor = max_velocity_scaling_factor
            request.max_acceleration_scaling_factor = max_acceleration_scaling_factor

            
            future = self.get_cartesian_path_client.call_async(request)

            rclpy.spin_until_future_complete(self, future, timeout_sec=10)


            if not future.done():
                raise Error("Timeout reached when calling move_cartesian service")

            result: GetCartesianPath.Response
            result = future.result()

            return result.solution

        def _call_get_position_fk (self):

            request = GetPositionFK.Request()


            header = Header()
            header.frame_id = "world"
            header.stamp = self.get_clock().now().to_msg()
            request.header = header


            request.fk_link_names = ["floor_gripper"]
            request.robot_state = robotStateToRobotStateMsg(self._ariac_robots_state)

            future = self.get_position_fk_client.call_async(request)


            rclpy.spin_until_future_complete(self, future, timeout_sec=10)

            if not future.done():
                raise Error("Timeout reached when calling get_position_fk service")

            result: GetPositionFK.Response
            result = future.result()

            return result.pose_stamped[0].pose
        
        def _plan_and_execute(
            self,
            robot,
            planning_component,
            logger,
            robot_type,
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
                with self._planning_scene_monitor.read_write() as scene:
                    scene.current_state.update(True)
                    self._ariac_robots_state = scene.current_state
                    robot_trajectory = plan_result.trajectory
                robot.execute(robot_trajectory, controllers=["floor_robot_controller","linear_rail_controller"] if robot_type=="floor_robot" else ["ceiling_robot_controller","gantry_controller"])
            else:
                logger.error("Planning failed")
                return False
            return True

        def move_floor_robot_home(self):
            with self._planning_scene_monitor.read_write() as scene:
                self._floor_robot.set_start_state(robot_state = scene.current_state)
                self._floor_robot.set_goal_state(configuration_name="home")
            self._plan_and_execute(self._ariac_robots,self._floor_robot, self.get_logger(),"floor_robot", sleep_time=0.0)
            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update()
                self._ariac_robots_state = scene.current_state
                self._floor_robot_home_quaternion = self._ariac_robots_state.get_pose("floor_gripper").orientation
        
        def move_ceiling_robot_home(self):
            with self._planning_scene_monitor.read_write() as scene:
                self._ceiling_robot.set_start_state(robot_state = scene.current_state)
                self._ceiling_robot.set_goal_state(configuration_name="home")
            self._plan_and_execute(self._ariac_robots,self._ceiling_robot, self.get_logger(),"ceiling_robot", sleep_time=0.0)
            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update()
                self._ariac_robots_state = scene.current_state
                self._ceiling_robot_home_quaternion = self._ariac_robots_state.get_pose("ceiling_gripper").orientation

        def _move_floor_robot_cartesian(self, waypoints, velocity, acceleration, avoid_collision = True):
            with self._planning_scene_monitor.read_write() as scene:
                # instantiate a RobotState instance using the current robot model
                self._ariac_robots_state = scene.current_state
                # Max step
                self._ariac_robots_state.update()
                trajectory_msg = self._call_get_cartesian_path(waypoints, velocity, acceleration, avoid_collision, "floor_robot")
                self._ariac_robots_state.update()
                trajectory = RobotTrajectory(self._ariac_robots.get_robot_model())
                trajectory.set_robot_trajectory_msg(self._ariac_robots_state, trajectory_msg)
                trajectory.joint_model_group_name = "floor_robot"
            self._ariac_robots_state.update(True)
            self._ariac_robots.execute(trajectory, controllers=[])

        def _move_floor_robot_to_pose(self,pose : Pose):
            self.get_logger().info(str(pose))
            with self._planning_scene_monitor.read_write() as scene:
                self._floor_robot.set_start_state(robot_state = scene.current_state)

                pose_goal = PoseStamped()
                pose_goal.header.frame_id = "world"
                pose_goal.pose = pose
                self.get_logger().info(str(pose_goal.pose))
                self._floor_robot.set_goal_state(pose_stamped_msg=pose_goal, pose_link="floor_gripper")
            
            while not self._plan_and_execute(self._ariac_robots, self._floor_robot, self.get_logger(), "floor_robot"):
                pass
        
        def _move_ceiling_robot_to_pose(self, pose: Pose):
            self.get_logger().info(str(pose))
            with self._planning_scene_monitor.read_write() as scene:
                self._ceiling_robot.set_start_state(robot_state = scene.current_state)

                pose_goal = PoseStamped()
                pose_goal.header.frame_id = "world"
                pose_goal.pose = pose
                self.get_logger().info(str(pose_goal.pose))
                self._ceiling_robot.set_goal_state(pose_stamped_msg=pose_goal, pose_link="ceiling_gripper")
            
            while not self._plan_and_execute(self._ariac_robots, self._ceiling_robot, self.get_logger(), "ceiling_robot"):
                pass

        def _makeMesh(self, name, pose, filename, frame_id) -> CollisionObject:
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
            o.header.frame_id = frame_id
            o.id = name
            o.meshes.append(mesh)
            o.mesh_poses.append(pose)
            o.operation = o.ADD
            return o
        
        def _add_model_to_planning_scene(self,
                                        name : str,
                                        mesh_file : str,
                                        model_pose : Pose,
                                        frame_id = "world"):
            self.get_logger().info(f"Adding {name} to planning scene")
            model_path = self.mesh_file_path+mesh_file
            collision_object = self._makeMesh(name, model_pose,model_path, frame_id = frame_id)
            with self._planning_scene_monitor.read_write() as scene:
                scene.apply_collision_object(collision_object)
                self._world_collision_objects.append(collision_object)
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
        
        def _kts1_camera_cb(self, msg: AdvancedLogicalCameraImageMsg):
            self._kts1_trays = msg.tray_poses
            self._kts1_camera_pose = msg.sensor_pose

        def _kts2_camera_cb(self, msg: AdvancedLogicalCameraImageMsg):
            self._kts2_trays = msg.tray_poses
            self._kts2_camera_pose = msg.sensor_pose
        
        def _agv1_status_cb(self, msg : AGVStatusMsg):
            self._agv_locations[1] = msg.location
        
        def _agv2_status_cb(self, msg : AGVStatusMsg):
            self._agv_locations[2] = msg.location
        
        def _agv3_status_cb(self, msg : AGVStatusMsg):
            self._agv_locations[3] = msg.location
        
        def _agv4_status_cb(self, msg : AGVStatusMsg):
            self._agv_locations[4] = msg.location

        def _floor_robot_wait_for_attach(self,timeout : float, orientation : Quaternion):
            with self._planning_scene_monitor.read_write() as scene:
                current_pose = scene.current_state.get_pose("floor_gripper")
            self.get_logger().info("Got current pose")
            start_time = time.time()
            while not self._floor_robot_gripper_state.attached:
                current_pose=build_pose(current_pose.position.x, current_pose.position.y,
                                        current_pose.position.z-0.001,
                                        orientation)
                waypoints = [current_pose]
                self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)
                sleep(0.2)
                if time.time()-start_time>=timeout:
                    self.get_logger().error("Unable to pick up part")

        def floor_robot_pick_bin_part(self,part_to_pick : PartMsg):
            part_pose = Pose()
            found_part = False
            bin_side = ""
            
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

            if self._floor_robot_gripper_state.type != "part_gripper":
                if part_pose.position.y<0:
                    station = "kts1"
                else: 
                    station = "kts2"
                self.floor_robot_move_to_joint_position(f"floor_{station}_js_")
                self._floor_robot_change_gripper(station, "parts")
            self.floor_robot_move_joints_dict({"linear_actuator_joint":self._rail_positions[bin_side],
                                        "floor_shoulder_pan_joint":0})
            part_rotation = rpy_from_quaternion(part_pose.orientation)[2]
            
            gripper_orientation = quaternion_from_euler(0.0,pi,part_rotation)
            self._move_floor_robot_to_pose(build_pose(part_pose.position.x, part_pose.position.y,
                                                    part_pose.position.z+0.5, gripper_orientation))

            waypoints = [build_pose(part_pose.position.x, part_pose.position.y,
                                    part_pose.position.z+CompetitionInterface._part_heights[part_to_pick.type]+0.008,
                                    gripper_orientation)]
            self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)
            self.set_floor_robot_gripper_state(True)
            self._floor_robot_wait_for_attach(30.0, gripper_orientation)

            self._attach_model_to_floor_gripper(part_to_pick, part_pose)

            self.floor_robot_attached_part_ = part_to_pick
            self.get_logger().info("Part attached. Attempting to move up")
            waypoints = [build_pose(part_pose.position.x, part_pose.position.y,
                                    part_pose.position.z+0.5,
                                    gripper_orientation)]
            self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)
            self.get_logger().info("After move up")
        
        def complete_orders(self):
            
            while len(self._orders) == 0:
                self.get_logger().info("No orders have been recieved yet", throttle_duration_sec=5.0)

            self.add_objects_to_planning_scene()

            success = True
            while True:
                if (self._competition_state == CompetitionStateMsg.ENDED):
                    success = False
                    break

                if len(self._orders) == 0:
                    if (self._competition_state == CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE):
                        self.get_logger().info("Waiting for orders...")
                        while len(self._orders) == 0:
                            sleep(1)
                    else:
                        self.get_logger().info("Completed all orders")
                        success = True
                        break

                self.current_order = copy(self._orders[0])
                self.current_order : Order
                del self._orders[0]
                kitting_agv_num = -1

                if self.current_order.order_type == OrderMsg.KITTING:
                    self.complete_kitting_order(self.current_order.order_task)
                    kitting_agv_num = self.current_order.order_task.agv_number
                    agv_location = -1
                    # while agv_location !=AGVStatusMsg.WAREHOUSE:
                    #     agv_location = self._agv_locations[kitting_agv_num]
                elif self.current_order.order_type == OrderMsg.ASSEMBLY:
                    self.complete_assembly_order(self.current_order.order_task)
                else:
                    self.complete_combined_order(self.current_order.order_task)
                
                self.submit_order(self.current_order.order_id)
            return success

        def complete_kitting_order(self, kitting_task:KittingTask):
            self._floor_robot_pick_and_place_tray(kitting_task._tray_id, kitting_task._agv_number)

            for kitting_part in kitting_task._parts:
                self.floor_robot_pick_bin_part(kitting_part._part)
                self._floor_robot_place_part_on_kit_tray(kitting_task._agv_number, kitting_part.quadrant)
            
            self.move_agv(kitting_task._agv_number, kitting_task._destination)

        def _floor_robot_pick_and_place_tray(self, tray_id, agv_number):
            tray_pose = Pose
            station = ""
            found_tray = False

            for tray in self._kts1_trays:
                if tray.id == tray_id:
                    station = "kts1"
                    tray_pose = multiply_pose(self._kts1_camera_pose, tray.pose)
                    found_tray = True
                    break
            
            if not found_tray:
                for tray in self._kts2_trays:
                    if tray.id == tray_id:
                        station = "kts2"
                        tray_pose = multiply_pose(self._kts2_camera_pose, tray.pose)
                        found_tray = True
                        break
            
            if not found_tray:
                return False
            
            tray_rotation = rpy_from_quaternion(tray_pose.orientation)[2]

            self.floor_robot_move_to_joint_position(f"floor_{station}_js_")

            if self._floor_robot_gripper_state.type != "tray_gripper":
                self._floor_robot_change_gripper(station, "trays")
            
            gripper_orientation = quaternion_from_euler(0.0,pi,tray_rotation)
            self._move_floor_robot_to_pose(build_pose(tray_pose.position.x, tray_pose.position.y,
                                                    tray_pose.position.z+0.5, gripper_orientation))
            
            waypoints = [build_pose(tray_pose.position.x, tray_pose.position.y,
                                    tray_pose.position.z+0.003,
                                    gripper_orientation)]
            self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)
            self.set_floor_robot_gripper_state(True)
            self._floor_robot_wait_for_attach(30.0, gripper_orientation)
            waypoints = [build_pose(tray_pose.position.x, tray_pose.position.y,
                                    tray_pose.position.z+0.5,
                                    gripper_orientation)]
            self._move_floor_robot_cartesian(waypoints, 0.3, 0.3)

            # self.floor_robot_move_joints_dict({"linear_actuator_joint":self._rail_positions[f"agv{agv_number}"],
            #                                "floor_shoulder_pan_joint":0})

            agv_tray_pose = self._frame_world_pose(f"agv{agv_number}_tray")
            agv_rotation = rpy_from_quaternion(agv_tray_pose.orientation)[2]

            agv_quaternion = quaternion_from_euler(0.0,pi,agv_rotation)

            self._move_floor_robot_to_pose(build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                                    agv_tray_pose.position.z+0.5,agv_quaternion))
            
            waypoints = [build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                    agv_tray_pose.position.z+0.01,agv_quaternion)]
            
            self._move_floor_robot_cartesian(waypoints, 0.3, 0.3)
            self.set_floor_robot_gripper_state(False)
            self.lock_agv_tray(agv_number)

            waypoints = [build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                    agv_tray_pose.position.z+0.3,quaternion_from_euler(0.0,pi,0.0))]
            self._move_floor_robot_cartesian(waypoints,0.3,0.3)

        
        def _frame_world_pose(self,frame_id : str):
            self.get_logger().info(f"Getting transform for frame: {frame_id}")
            # try:
            t = self.tf_buffer.lookup_transform("world",frame_id,rclpy.time.Time())
            # except:
            #     self.get_logger().error("Could not get transform")
            #     quit()
            
            pose = Pose()
            pose.position.x = t.transform.translation.x
            pose.position.y = t.transform.translation.y
            pose.position.z = t.transform.translation.z
            pose.orientation = t.transform.rotation

            return pose
            
        def _floor_robot_place_part_on_kit_tray(self, agv_num : int, quadrant : int):
            
            if not self._floor_robot_gripper_state.attached:
                self.get_logger().error("No part attached")
                return False

            self.floor_robot_move_joints_dict({"linear_actuator_joint":self._rail_positions[f"agv{agv_num}"],
                                        "floor_shoulder_pan_joint":0})
            
            agv_tray_pose = self._frame_world_pose(f"agv{agv_num}_tray")

            part_drop_offset = build_pose(CompetitionInterface._quad_offsets[quadrant][0],
                                        CompetitionInterface._quad_offsets[quadrant][1],
                                        0.0, quaternion_from_euler(0.0,pi,0.0))
            
            part_drop_pose = multiply_pose(agv_tray_pose, part_drop_offset)

            self._move_floor_robot_to_pose(build_pose(part_drop_pose.position.x, part_drop_pose.position.y,
                                                    part_drop_pose.position.z+0.3, quaternion_from_euler(0.0, pi, 0.0)))
            
            waypoints = [build_pose(part_drop_pose.position.x, part_drop_pose.position.y,
                                    part_drop_pose.position.z+CompetitionInterface._part_heights[self.floor_robot_attached_part_.type]+0.01, 
                                    quaternion_from_euler(0.0, pi, 0.0))]
            
            self._move_floor_robot_cartesian(waypoints, 0.3, 0.3,False)

            self.set_floor_robot_gripper_state(False)

            self._remove_model_from_floor_gripper()

            waypoints = [build_pose(part_drop_pose.position.x, part_drop_pose.position.y,
                                    part_drop_pose.position.z+0.3, 
                                    quaternion_from_euler(0.0, pi, 0.0))]
            
            self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)

            return True

        def _floor_robot_change_gripper(self,station : str, gripper_type : str):
            self.get_logger().info(f"Changing gripper to type: {gripper_type}")

            tc_pose = self._frame_world_pose(f"{station}_tool_changer_{gripper_type}_frame")

            self._move_floor_robot_to_pose(build_pose(tc_pose.position.x, tc_pose.position.y,
                                                    tc_pose.position.z+0.4,
                                                    quaternion_from_euler(0.0, pi, 0.0)))
            
            waypoints = [build_pose(tc_pose.position.x, tc_pose.position.y,tc_pose.position.z,
                                    quaternion_from_euler(0.0,pi,0.0))]
            
            self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)

            request = ChangeGripper.Request()

            if gripper_type == "trays":
                request.gripper_type = ChangeGripper.Request.TRAY_GRIPPER
            elif gripper_type == "parts":
                request.gripper_type = ChangeGripper.Request.PART_GRIPPER
            
            future = self._change_gripper_client.call_async(request)


            rclpy.spin_until_future_complete(self, future, timeout_sec=10)

            if not future.done():
                raise Error("Timeout reached when calling change_gripper service")

            result: ChangeGripper.Response
            result = future.result()

            if not result.success:
                self.get_logger().error("Error calling change gripper service")
            
            waypoints = [build_pose(tc_pose.position.x, tc_pose.position.y,tc_pose.position.z + 0.4,
                                    quaternion_from_euler(0.0,pi,0.0))]
            
            self._move_floor_robot_cartesian(waypoints, 0.3, 0.3, False)
        
        def submit_order(self, order_id):
            submit_order_client = self.create_client(
                SubmitOrder,
                '/ariac/submit_order'
            )
            request = SubmitOrder.Request()
            request.order_id = order_id

            # Send the request
            future = submit_order_client.call_async(request)

            # Wait for the response
            try:
                rclpy.spin_until_future_complete(self, future)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

            # Check the response
            if future.result().success:
                self.get_logger().info(f'Submitted order')
            else:
                self.get_logger().warn('Unable to submit order')
        
        def move_agv(self, num, destination):
            '''
            Move an AGV to an assembly station.
            Args:
                num (int): AGV number
                destination(int): Destination
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
            request.location = destination

            # Send the request.
            future = mover.call_async(request)

            # Wait for the server to respond.
            try:
                rclpy.spin_until_future_complete(self, future)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

            # Check the result of the service call.
            if future.result().success:
                self.get_logger().info(f'Moved AGV{num} to {self._destinations[destination]}')
            else:
                self.get_logger().warn(future.result().message)  
                        
        def _makeAttachedMesh(self, name, pose, filename, robot) -> AttachedCollisionObject:
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
                
            o = AttachedCollisionObject()
            if robot == "floor_robot":
                o.link_name = "floor_gripper"
            else:
                o.link_name = "ceiling_gripper"
            o.object.header.frame_id = "world"
            o.object.id = name
            o.object.meshes.append(mesh)
            o.object.mesh_poses.append(pose)
            return o
        
        def apply_planning_scene(self, scene):
            apply_planning_scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

            # Create a request object.
            request = ApplyPlanningScene.Request()

            # Set the request location.
            request.scene = scene

            # Send the request.
            future = apply_planning_scene_client.call_async(request)

            # Wait for the server to respond.
            try:
                rclpy.spin_until_future_complete(self, future)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

            # Check the result of the service call.
            if future.result().success:
                self.get_logger().info(f'Succssefully applied new planning scene')
            else:
                self.get_logger().warn(future.result().message)
        
        def get_planning_scene_msg(self, msg:PlanningScene) -> PlanningScene:
            self.planning_scene_msg = msg
        
        def _attach_model_to_floor_gripper(self, part_to_pick : PartMsg, part_pose : Pose):
            part_name = self._part_colors[part_to_pick.color]+"_"+self._part_types[part_to_pick.type]

            self.get_logger().info(f"Attaching {part_name} to floor gripper")
            model_path = self.mesh_file_path + self._part_types[part_to_pick.type]+".stl"
            attached_collision_object = self._makeAttachedMesh(part_name, part_pose,model_path, "floor_robot")
            temp_scene = copy(self.planning_scene_msg)
            with self._planning_scene_monitor.read_write() as scene:
                temp_scene.world.collision_objects = self._world_collision_objects
                temp_scene.robot_state = robotStateToRobotStateMsg(scene.current_state)
                temp_scene.robot_state.attached_collision_objects.append(attached_collision_object)
                self.apply_planning_scene(temp_scene)
                scene.current_state.update()
                self._ariac_robots_state = scene.current_state
                
        def _remove_model_from_floor_gripper(self):
            self.get_logger().info("Removing attached part from floor gripper")
            temp_scene = copy(self.planning_scene_msg)
            with self._planning_scene_monitor.read_write() as scene:
                temp_scene.world.collision_objects = self._world_collision_objects
                temp_scene.robot_state = robotStateToRobotStateMsg(scene.current_state)
                temp_scene.robot_state.attached_collision_objects.clear()
                self.apply_planning_scene(temp_scene)
                scene.current_state.update()
                self._ariac_robots_state = scene.current_state
        
        def _create_floor_joint_position_state(self, joint_positions : list)-> dict:
            return {"linear_actuator_joint":joint_positions[0],
                    "floor_shoulder_pan_joint":joint_positions[1],
                    "floor_shoulder_lift_joint":joint_positions[2],
                    "floor_elbow_joint":joint_positions[3],
                    "floor_wrist_1_joint":joint_positions[4],
                    "floor_wrist_2_joint":joint_positions[5],
                    "floor_wrist_3_joint":joint_positions[6]}

        def _create_floor_joint_position_dict(self, dict_positions = {}):
            with self._planning_scene_monitor.read_write() as scene:
                current_positions = scene.current_state.get_joint_group_positions("floor_robot")
                current_position_dict = self._create_floor_joint_position_state(current_positions)
                for key in dict_positions.keys():
                    current_position_dict[key] = dict_positions[key]
            return current_position_dict

        def floor_robot_move_joints_dict(self, dict_positions : dict):
            new_joint_position = self._create_floor_joint_position_dict(dict_positions)
            with self._planning_scene_monitor.read_write() as scene:
                self._floor_robot.set_start_state(robot_state = scene.current_state)
                scene.current_state.joint_positions = new_joint_position
                joint_constraint = construct_joint_constraint(
                        robot_state=scene.current_state,
                        joint_model_group=self._ariac_robots.get_robot_model().get_joint_model_group("floor_robot"),
                )
                self._floor_robot.set_goal_state(motion_plan_constraints=[joint_constraint])
            self._plan_and_execute(self._ariac_robots,self._floor_robot, self.get_logger(), "floor_robot")
        
        def floor_robot_move_to_joint_position(self, position_name : str):
            with self._planning_scene_monitor.read_write() as scene:
                self.get_logger().info("Right set start state")
                self._floor_robot.set_start_state(robot_state=scene.current_state)
                self.get_logger().info("Setting joint states")
                scene.current_state.joint_positions = self.floor_position_dict[position_name]
                self.get_logger().info("Right before construct joint state")
                joint_constraint = construct_joint_constraint(
                        robot_state=scene.current_state,
                        joint_model_group=self._ariac_robots.get_robot_model().get_joint_model_group("floor_robot"),
                )
                self.get_logger().info("Right before set goal state")
                self._floor_robot.set_goal_state(motion_plan_constraints=[joint_constraint])
            self.get_logger().info("Out of with statement")
            self._plan_and_execute(self._ariac_robots,self._floor_robot, self.get_logger(), robot_type="floor_robot")


Code Explanation
^^^^^^^^^^^^^^^^
The competition interface from :ref:`Tutorial 9 <TUTORIAL9>` was augmented with the components described below.

- Instance Variables

    - :python:`_agv_locations`: Holds the location ids for the agvs stays at -1 until it is at a named location.
    - :python:`agv1_status_sub`: Subscribes to the agv1_status message to get the location live.
    - :python:`agv2_status_sub`: Subscribes to the agv2_status message to get the location live.
    - :python:`agv3_status_sub`: Subscribes to the agv3_status message to get the location live.
    - :python:`agv4_status_sub`: Subscribes to the agv4_status message to get the location live.
    - :python:`tf_buffer`: Used to get the transforms.
    - :python:`tf_listener`: Updates the tf_buffer with the current transforms.
    - :python:`tf_broadcaster`: Broadcasts the transforms.
    

- Instance Methods

    - :python:`complete_orders(self)`: Public method used to loop through orders in the competition and complete them.
    - :python:`complete_kitting_order(self, kitting_task)`: Private method used to complete a kitting order using the other methods in the class.
    - :python:`_floor_robot_pick_and_place_tray(self, tray_id, agv_number)`: Private method used to pick a tray and place it on an agv.
    - :python:`_frame_world_pose(self, frame_id)`: Gets a pose from the transform buffer in relation to the world frame.
    - :python:`_floor_robot_place_part_on_kit_tray(self, agv_num, quadrant)`: Private method takes in the agv number and quadrant and places the part attached to the floor gripper onto the tray on the given agv.
    - :python:`_floor_robot_change_gripper(self, station, gripper_type)`: Private method that takes in a station and gripper type and changes the gripper on the floor robot.
    - :python:`submit_order(self, name, pose, filename)`: Private method takes in an order id and calls the `submit_order` service.
    - :python:`move_agv(self, name, num, destination)`: Private method that takes in the number of the agv and the destination and calls the move agv service for the correct agv to move to that location.

tutorial10.py
---------------

This is the main node for this tutorial. It creates a thread which spins the competition_interface class to run the callbacks. In the main thread, it starts the competition and runs the `complete_orders` function.

    .. code-block:: python
        #!/usr/bin/env python3
        '''
        To test this script, run the following commands in separate terminals:
        - ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials
        - ros2 run ariac_tutorials tutorial_10.py
        '''
        import threading
        import rclpy
        from ariac_tutorials.competition_interface import CompetitionInterface
        from ariac_msgs.msg import Part
        from rclpy.executors import MultiThreadedExecutor

        def main(args=None):
            rclpy.init(args=args)
            interface = CompetitionInterface()
            executor = MultiThreadedExecutor()
            executor.add_node(interface)

            spin_thread = threading.Thread(target=executor.spin)
            spin_thread.start()
            
            interface.start_competition()
            
            interface.complete_orders()

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
        ros2 launch ariac_tutorials tutorial10.launch.py


    The node will wait until the competition is ready.


- In *terminal 3*, run the following commands:

    .. code-block:: bash

        cd ~/ariac_ws
        . install/setup.bash
        ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials dev_mode:=True


Outputs
=======

The floor robot will move to the home position. Then, it will complete a kitting order.


