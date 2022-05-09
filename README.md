[![Build status](https://github.com/ros-planning/warehouse_ros_sqlite/actions/workflows/build_and_test.yaml/badge.svg?branch=ros2)](https://github.com/ros-planning/warehouse_ros_sqlite/actions/workflows/build_and_test.yaml)
[![codecov](https://codecov.io/gh/ros-planning/warehouse_ros_sqlite/branch/ros2/graph/badge.svg?token=QHPGDZM8HX)](https://codecov.io/gh/ros-planning/warehouse_ros_sqlite)

# SQLite backend for warehouse_ros

This is a storage backend for warehouse_ros using SQLite.
The name of the sqlite file will be taken from the ROS parameter `warehouse_host`. The `warehouse_port` will be ignored.
Note that the MD5 sums of the messages have changed from ROS1 to ROS2,
so your ROS1 sqlite database won't work with ROS2.

## Installation

Make sure that you have [installed ROS2](https://docs.ros.org/en/rolling/Installation.html)
and activated it (e.g. with `source /opt/ros/foxy/setup.bash`).
Create a folder which will become your workspace.
Create a file named `warehouse.repos` with the following content:
```yaml
repositories:
  warehouse_ros_sqlite:
    type: git
    url: https://github.com/ros-planning/warehouse_ros_sqlite
    version: ros2
```

Now open a terminal and navigate to this folder.
Initialize your workspace and fetch the source files with `vcs` and `rosdep`:
```bash
$ vcs import src < warehouse.repos
$ rosdep update
$ rosdep install --from-paths src --ignore-src
```

Build your workspace with colcon:
```bash
$ colcon build
```
After that, activate your workspace:
```bash
$ source install/local_setup.bash
```
The plugin is now installed, please refer to the warehouse_ros documentation for the usage of the interface or have a look into the test files in `test/`.

## Adapt the .launch files

You can use this plugin together with the whole MoveIt stack,
but you may need to adapt the .launch files.
Make sure that this plugin is loaded instead of `warehouse_ros_mongo`.
If you're using RViz, you'll have to enter the path to your database file into the `Host` field and click on `connect`.

### run_move_group.launch.py

The demo launch file needs a bit attention, too.

```diff
--- install/run_move_group/share/run_move_group/launch/run_move_group.launch.py 2021-06-20 15:24:38.000000000 +0000
+++ install/run_move_group/share/run_move_group/launch/run_move_group_sqlite.launch.py  2021-06-20 20:46:42.061550552 +0000
@@ -89,6 +89,11 @@
         "publish_transforms_updates": True,
     }

+    warehouse_ros_config = {
+        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
+        "warehouse_host": "/path/to/my/warehouse_db.sqlite",
+    }
+
     # Start the actual move_group node/action server
     run_move_group_node = Node(
         package="moveit_ros_move_group",
@@ -102,6 +107,7 @@
             trajectory_execution,
             moveit_controllers,
             planning_scene_monitor_parameters,
+            warehouse_ros_config,
         ],
     )

@@ -120,6 +126,7 @@
             robot_description_semantic,
             ompl_planning_pipeline_config,
             kinematics_yaml,
+            warehouse_ros_config,
         ],
     )

@@ -172,18 +179,6 @@
             )
         ]

-    # Warehouse mongodb server
-    mongodb_server_node = Node(
-        package="warehouse_ros_mongo",
-        executable="mongo_wrapper_ros.py",
-        parameters=[
-            {"warehouse_port": 33829},
-            {"warehouse_host": "localhost"},
-            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
-        ],
-        output="screen",
-    )
-
     return LaunchDescription(
         [
             rviz_node,
@@ -191,7 +186,6 @@
             robot_state_publisher,
             run_move_group_node,
             ros2_control_node,
-            mongodb_server_node,
         ]
         + load_controllers
     )
```
