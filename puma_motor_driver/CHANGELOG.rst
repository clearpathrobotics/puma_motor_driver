^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package puma_motor_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2024-07-25)
------------------
* Removed info logging
* Restored commandSpeed
* Spacing fixes
* Updated topics to match API
* Renamed socketcan_interface to clearpath_socket_interface
* Topic in node namespace
* Renamed robot to multi_puma_node
* Fixed include
* Add launch files and configuration files
* Add new multi puma node
* Modified header to include new functions
* Removed launch files
* Removed split multi control node
* Add retrieval functions with received checks
* properly using Boost shared library now
* added -fPIC build option
* exported header files
* built in ROS Foxy without errors
* Contributors: Joep Tool, Luis Camero, joe28965

0.3.1 (2020-11-23)
------------------
* [puma_motor_driver] Made socketcan_interface a depend rather than build_depend.
* Contributors: Tony Baltovski

0.3.0 (2020-11-12)
------------------
* [puma_motor_driver] Improved logging.
* [puma_motor_driver] Added settings for socketcan interface.
* [puma_motor_driver] Fixes for roslint.
* [puma_motor_driver] Switched to socketcan_interface, updated configuration and added extra field handling.
* [puma_motor_driver] Removed serial gateway.
* [puma_motor_driver] Update for cpp11.
* Contributors: Tony Baltovski

0.2.1 (2019-11-22)
------------------
* [puma_motor_driver] Disabld checking of message if it was a status cause it was causing issues.
* Contributors: Tony Baltovski

0.2.0 (2019-09-03)
------------------
* Renamed verifying 16x16 raw bytes and added verfying 8x8 function. Added parentheses to evaluate AND before comparison and added default case to switch statement in non-void function. Also, minor linter fixes.
* Updated fixed-point conversion.
* Contributors: Tony Baltovski

0.1.1 (2015-11-20)
------------------
* Package format 2, dependency fix.
* Contributors: Mike Purvis

0.1.0 (2015-11-20)
------------------
* First release of CAN/serial ROS driver for Puma.
* Contributors: Mike Purvis, Tony Baltovski
