.. _CHANGELOG:

=========
Changelog
=========

---------------------
2024.1.0, (2024-1-30)
---------------------

Initial release of the 2024 ARIAC competition.

Updates relevant to competitors:
================================

* Competition environment moved to ROS Iron on Ubuntu 22.04. 

* Added RGB/RGBD cameras to the robot end effectors `#274 <https://github.com/usnistgov/ARIAC/pull/274>`_

* Removed the ability to use advanced_logical_cameras for the competition. `#258 <https://github.com/usnistgov/ARIAC/pull/258>`_

  * Advanced logical cameras can still be used for development using the launch argument dev_mode:=true

* Updated conveyor belt to continually spawn parts throughout the competition `#264 <https://github.com/usnistgov/ARIAC/pull/264>`_

* Added new challenge where assembly inserts can be rotated on assembly tables `#267 <https://github.com/usnistgov/ARIAC/pull/267>`_

* Fixed issue with part inertias that was affecting part pick up (especially pump) `#260 <https://github.com/usnistgov/ARIAC/pull/260>`_

* Changed initial poses of the robots to be their home pose (from ariac_moveit_config srdf) `#271 <https://github.com/usnistgov/ARIAC/pull/271>`_

* Added feature to clear the AGV of tray and parts after kitting orders are submitted `#266 <https://github.com/usnistgov/ARIAC/pull/276>`_

* Updated task manager scoring of tasks and improved logging display of scoring. Added output of the competitors's sensor cost and trial score to the ariac_logs directory  `#296 <https://github.com/usnistgov/ARIAC/pull/296>`_

* Changed RGB/RGBD cameras to publish a blank image during sensor blackout challenge instead of not publishing `#295 <https://github.com/usnistgov/ARIAC/pull/295>`_

* Fixed issues to make part picking and assembly more reliable `#288 <https://github.com/usnistgov/ARIAC/pull/288>`_

* Removed human challenge for 2024 competition `#280 <https://github.com/usnistgov/ARIAC/pull/280>`_

Other Updates
=============

* Added static controllers to fix issue with new version of ros2_control where the robots drift when no controllers are active. `#259 <https://github.com/usnistgov/ARIAC/pull/259>`_

* Updated ariac_moveit_config to use MoveItConfigsBuilder. `#257 <https://github.com/usnistgov/ARIAC/pull/257>`_

* Moved documentation and automated evaluation to separate repositories `#275 <https://github.com/usnistgov/ARIAC/pull/275>`_

* Moved/renamed test_competitor in separate repository `#298 <https://github.com/usnistgov/ARIAC/pull/298>`_

* General cleanup of packages `#261 <https://github.com/usnistgov/ARIAC/pull/261>`_