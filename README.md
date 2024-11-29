## Overview
This package is to be used hand-in-hand with [monkey_std_msgs](https://github.com/MonKey-Robotics/monkey_std_msgs/tree/main) package, meaning that any custom messages that has been added to that repository will be supported here as well. Any custom messages not supported by monkey_std_msgs package will therefore also not be supported in this package.

## How To Add New bt_plugins
Add the files that you have created for bt_plugins into the correct directory such as `bt_ros2_actions` or `bt_ros2_services`. Make sure to include the header file in the right place as well.
Make sure to `colcon build` in your workspace after you have done to make sure all the bt_plugins are registered.

## Features
There is no need to edit the `CMakeLists.txt` file once you have done inserting the new plugins as it will automatically find and register all of the bt_plugins.
