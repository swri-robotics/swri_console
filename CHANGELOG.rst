^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package swri_console
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.6 (2024-09-27)
------------------
* Fix: "Human readable time" now shows milliseconds correctly. (`#67 <https://github.com/swri-robotics/swri_console/issues/67>`_)
  Co-authored-by: David Anthony <djanthony@gmail.com>
* Update industrial_ci.yml
  Adds CI for Jazzy and Rolling.
* Allow default support for mcap files (`#65 <https://github.com/swri-robotics/swri_console/issues/65>`_)
* Add: Added support for log files of the format "Example: 1724314618.146484723 [INFO] [namespace.Node::Fuction]:  The actual log msg". (`#66 <https://github.com/swri-robotics/swri_console/issues/66>`_)
  Refactor: Removed nested if's
* Update README.md
* Contributors: David Anthony, Rasmus Skovgaard Andersen, Tim Clephas

2.0.5 (2024-06-20)
------------------
* Implement reading from rosbag2 files (`#64 <https://github.com/swri-robotics/swri_console/issues/64>`_)
* Add human readable time (`#63 <https://github.com/swri-robotics/swri_console/issues/63>`_)
* Remove Old Distro Support (`#62 <https://github.com/swri-robotics/swri_console/issues/62>`_)
* Update package.xml
* Adding ROS-Industrial CI Actions for ROS2 Releases
* Contributors: David Anthony, Matthijs van der Burgh, Ramon Wijnands

2.0.4 (2023-04-04)
------------------
* Adding missing dependency needed by build farm (`#58 <https://github.com/swri-robotics/swri_console/issues/58>`_)
* Contributors: David Anthony

2.0.3 (2023-02-23)
------------------
* Fix QoS on Humble (`#55 <https://github.com/swri-robotics/swri_console/issues/55>`_)
  * Use Humble's Default rosout QoS Settings
  Co-authored-by: David Anthony <david.anthony@swri.org>
  Co-authored-by: Tony Najjar <tony.najjar.1997@gmail.com>
* Contributors: David Anthony, Tony Najjar

2.0.2 (2022-11-03)
------------------
* Update Qt Flags (`#52 <https://github.com/swri-robotics/swri_console/issues/52>`_)
  * Update to use new flags
* Contributors: David Anthony

2.0.1 (2022-11-03)
------------------
* Switching to system default QoS (`#50 <https://github.com/swri-robotics/swri_console/issues/50>`_)
* Merge pull request `#45 <https://github.com/swri-robotics/swri_console/issues/45>`_ from rasmusan/bugfix/log_queue_not_emptied
  Fix: Before maximum 1 log entry every 50 ms was processed. Now all en…
* Merge pull request `#44 <https://github.com/swri-robotics/swri_console/issues/44>`_ from rasmusan/bugfix/rosout_qos_fixed
  Fix: Changed rosout subsciption to use rcl_qos_profile_rosout_default…
* Merge pull request `#43 <https://github.com/swri-robotics/swri_console/issues/43>`_ from nobleo/fix/boost-thread-dep
  Fix boost dependency
* Merge pull request `#41 <https://github.com/swri-robotics/swri_console/issues/41>`_ from nobleo/ros-ok
  Replace deprecated is_initialized() with ok()
* Merge pull request `#36 <https://github.com/swri-robotics/swri_console/issues/36>`_ from rasmusan/bugfix/fix_loglevel_masking
  Fixed loglevel masking (for ROS2)
* Merge pull request `#42 <https://github.com/swri-robotics/swri_console/issues/42>`_ from nobleo/libboost-thread
  More specific boost dependency
* Merge pull request `#37 <https://github.com/swri-robotics/swri_console/issues/37>`_ from rasmusan/feature/add-display-options
  Added options to show logger_name and function in each log line
* Fixed loglevel masking (for ROS2)
* Contributors: David Anthony, Matthew, Rasmus Skovgaard Andersen, Tim Clephas, rasmus.andersen

2.0.0 (2020-03-31)
------------------
* ROS 2 Dashing Support (`#31 <https://github.com/swri-robotics/swri_console/issues/31>`_)
* Contributors: Jacob Hassold, P. J. Reed

1.1.0 (2018-05-11)
------------------
* Added simple rosout_agg_recorder (`#15 <https://github.com/pjreed/swri_console/issues/15>`_)
* Change logger levels from within swri_console (`#20 <https://github.com/pjreed/swri_console/issues/20>`_)
* Load ROS logs and directories of ROS logs
* Fix compiler warnings found with Clang
* Add search bar
* Contributors: Edward Venator, P. J. Reed, Phil Westhart, Victor Murray, elliotjo, jgassaway

1.0.0 (2017-04-01)
------------------
* Add kinetic and lunar and simplify CI
  - Remove cruft from CI configuration
  - Add ROS kinetic to CI configuration
  - Add ROS lunar to CI configuration
  - Remove shadow-fixed builds from CI configuration. Since this repository has very few catkin dependencies, there's no reason to build for both shadow-fixed and released.
* Fix compiler warnings
* Contributors: Edward Venator, P. J. Reed, elliotjo

0.2.0 (2016-10-23)
------------------
* Port to Qt5 `#16 <https://github.com/swri-robotics/swri_console/issues/16>`_
* Contributors: Edward Venator, P. J. Reed

0.1.0 (2016-05-28)
------------------
* Remove settings_keys.h from qt4_wrap_cpp to suppress warning
  There are no QT-wrappable classes in this header, so the QT build emits a
  warning that it's doing nothing on the file. Since it's doing nothing,
  we might as well remove the file from the wrapper macro.
* Adding support for Travis CI
  This is based on the script provided by ros-industrial at:
  https://github.com/ros-industrial/industrial_ci
* Fix for issue `#12 <https://github.com/swri-robotics/swri_console/issues/12>`_; "Copy Extended" was only copying blank lines.
* Add option to alternate row colors for stronger line distinction.
  This uses the QAbstractItemView's alternatingRowColors property
  to alternate the background color of each line in the log list.  You
  can disable the alternating colors by right clicking on the log, and
  your preference should be persistent between sessions.
* Fix incorrect target in CMakeLists.txt
* Select all nodes when user hits Ctrl+A in the node list.
  Closes `#6 <https://github.com/swri-robotics/swri_console/issues/6>`_.
* Sync behavior between saving to bags and text files.
  This closes `#10 <https://github.com/swri-robotics/swri_console/issues/10>`_ by using the same behavior when saving to text
  and bag files.  Both now save the same messages that are in the user's
  current view.  Partly because this lets the user save subsets or
  everything by deliberately selecting everything, and partly because it
  was a lot easier to change the save to bag behavior than vice versa.
* Handle log messages with multiple lines correctly.
  This fixes the console to handle log messages with multiple
  lines correctly.  We're going to be using a little bit more memory
  now, but it shouldn't be a major issue and performance still seems
  great.  Closes `#8 <https://github.com/swri-robotics/swri_console/issues/8>`_.
* Closes Issue `#04 <https://github.com/swri-robotics/swri_console/issues/04>`_ - Subscribe to rosout_agg instead of rosout.
  I spent an hour adding an option to subscribe to either /rosout or
  /rosout_agg thinking that we would still need to listen to /rosout
  when playing back a bag file that only had /rosout recorded. But it
  turns out that the republisher catches those and puts them on
  /rosout_agg anyways. I reverted the changes and just changed RosThread
  to subscribe to /rosout_agg.
* Fix issue 05 - Node list not clearing out properly.
  The root cause of this problem was that the message count in the log
  database was not being cleared out.  I've refactored the structure a
  little bit to make the dependencies a little cleaner.  The log
  database doesn't know about the NodeListModel anymore (though the
  model now knows about the database).  This puts things more in line
  with the log database just being the main source of information, and
  the node list model and log database proxy being different views onto
  the database.
* Merge pull request `#7 <https://github.com/swri-robotics/swri_console/issues/7>`_ from pjreed/user_settings
  User settings
* Saving user settings for issue `#2 <https://github.com/swri-robotics/swri_console/issues/2>`_.
* Fixing a typo that prevented the Fatal-level message from printing.
* Adds license headers to all code.
* Adding screenshot for github front page.
* Adding console_generator.py script to generate fake messages.
* Create README.md
* Add url package file.
* Update package metadata.
  * Adds maintainer and author name
  * Sets license to BSD
  * Adds a more verbose description
* Remove unused dependency on qt_build, which is gone in Jade.
* Fix catkin lint
* Add extended copy option.
  This is an extended version of the copy command that also copies extra
  information like the node name, source file, source line, etc to the
  clipboard.
* Change default warning color to orange.
  Yellow-on-white is particularly hard for my eyes to read, especially
  out on a vehicle in daylight.
* Add the ability to write logs to txt or bag files.
* Fixing an issue with the "install" target.
* Fixing issues found by cppcheck.
* Adding the ability to colorize logs.
* Adding a menu open to read bag files directly.
* Setting default appropriate default values for some member variables.
* Adding a visual cue to the "Select All" menu option.
* Adding the ability to multi-select & copy logs.
* Cleaning up some catkin_lint issues and adding comments.
* Fixing issue with master disconnect/reconnect.
  After merging in ros_thread, I noticed that the console no longer
  reconnected properly when if the ros core is shutdown and restarted.
  I think this was due to using ros::Rate to limit the main RosThread
  loop.  I removed this in favor of QThread::msleep so that the loop is
  independent of ROS.  Also removed the added ros::NodeHandle member and
  initial startRos call that I'm guessing was added to allow ros::Rate
  to run without an exception.
  Using QThread::msleep might be be preferrable anyways so that the
  console continues to read messages immediately even if we're running
  simulations with the ROS clock turned waaaay down.
* Minor formatting.
* Fix bug in ordering from processing older messages.
  The old message processing had a small bug where each chunk of
  messages ended up in reversed order.  For example if you had an
  ordering like:
  ABCDEF
  and change the filtering, they might be reordered as
  CBAFED
  This was a small bug that is fixed by correctly building the early
  messages queue in the proper reversed order.
* Adding buttons for clearing the node & log lists.
* Adding comments; cleaning up code; reducing the update rate slightly.
* Moving ROS processing into a separate thread from the GUI.
* Adding regexp support to the include/exclude filters.
* Set window title with node names.
* Support to change fonts.
* Fixing node list selection model.
* Trying to get better layout sizes.
* Make substring filters case insensitive.
* Change include filter behavior.
  This changes the include filter to pass messages that contain at least
  one of the substrings instead of all substrings.
* Adding support for exlude/include filters.
* Adding relative/absolute times to output lines.
* Automatically toggle auto-scrolling based on slider position.
* Adding initial version.
  Basic functionality, but stays snappy with large logs so far.
* Contributors: Ed Venator, Elliot Johnson, P. J. Reed
