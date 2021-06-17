[![Build status](https://github.com/gleichdick/warehouse_ros_sqlite/actions/workflows/build_and_test.yaml/badge.svg?branch=ros2)](https://github.com/gleichdick/warehouse_ros_sqlite/actions/workflows/build_and_test.yaml)
[![codecov](https://codecov.io/gh/gleichdick/warehouse_ros_sqlite/branch/ros2/graph/badge.svg?token=QHPGDZM8HX)](https://codecov.io/gh/gleichdick/warehouse_ros_sqlite)

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
  warehouse_ros:
    type: git
    url: https://github.com/gleichdick/warehouse_ros
    version: cleanup_and_md5
  warehouse_ros_sqlite:
    type: git
    url: https://github.com/gleichdick/warehouse_ros_sqlite
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
Remember to set `db:=True` on your `roslaunch` command.
If you're using RViz, you'll have to enter the path to your database file into the `Host` field and click on `connect`.

### warehouse.launch

```diff
  <!-- Run the DB server -->
-  <node name="$(anon mongo_wrapper_ros)" cwd="ROS_HOME" type="mongo_wrapper_ros.py" pkg="warehouse_ros_mongo">
-    <param name="overwrite" value="false"/>
-    <param name="database_path" value="$(arg moveit_warehouse_database_path)" />
-  </node>
```

### warehouse_settings.launch.xml

```diff
   <!-- The default DB host for moveit -->
-  <arg name="moveit_warehouse_host" default="localhost" />
+  <arg name="moveit_warehouse_host" default="/path/to/your/file.sqlite" />

   <!-- Set parameters for the warehouse -->
   <param name="warehouse_port" value="$(arg moveit_warehouse_port)"/>
   <param name="warehouse_host" value="$(arg moveit_warehouse_host)"/>
-  <param name="warehouse_plugin" value="warehouse_ros_mongo::MongoDatabaseConnection" />
+  <param name="warehouse_plugin" value="warehouse_ros_sqlite::DatabaseConnection" />
```
