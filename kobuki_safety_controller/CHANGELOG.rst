^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kobuki_safety_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2022-02-13)
------------------
* Enable the safety controller by default.
* Update all of the package.xml versions.
* Switch to auto-generating the executables for the components.
* Add a TODO to the safety controller.
* Add a kobuki_bumper2pc node. (`#32 <https://github.com/kobuki-base/kobuki_ros/issues/32>`_)
* Remove unnecessary setvbuf from nodes. (`#31 <https://github.com/kobuki-base/kobuki_ros/issues/31>`_)
* Remove parameter update from safety controller.
* Make sure to initialize rclcpp::Time appropriately. (`#8 <https://github.com/kobuki-base/kobuki_ros/issues/8>`_)
* Install the parameter update callback. (`#7 <https://github.com/kobuki-base/kobuki_ros/issues/7>`_)
* Port the safety controller to ROS 2. (`#2 <https://github.com/kobuki-base/kobuki_ros/issues/2>`_)
* Contributors: Chris Lalancette

0.6.6 (2015-05-27)
------------------

0.6.5 (2014-11-21)
------------------

0.6.4 (2014-08-26)
------------------

0.6.3 (2014-08-25)
------------------
* added initialization for msg to kobuki_safety_controller
* added time_to_extend_bump_cliff_events param to kobuki_safety_controller
* Contributors: Kaijen Hsiao

0.6.2 (2014-08-11)
------------------

0.6.1 (2014-08-08)
------------------

0.6.0 (2014-08-08)
------------------
* Add missing run dependency on yocs_cmd_vel_mux
* Contributors: Jorge Santos

0.5.5 (2013-10-11)
------------------

0.5.4 (2013-09-06)
------------------

0.5.3 (2013-08-30)
------------------

0.5.0 (2013-08-29)
------------------
* Added extra url info on all packages.
* Fix URL to the previous changelog wiki.
* Changelogs at package level.

0.4.0 (2013-08-09)
------------------


Previous versions, bugfixing
============================

Available in ROS wiki: http://ros.org/wiki/kobuki/ChangeList
