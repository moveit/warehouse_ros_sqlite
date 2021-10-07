[![Build and Test](https://github.com/ros-planning/warehouse_ros_sqlite/actions/workflows/build_and_test.yaml/badge.svg?branch=master)](https://github.com/ros-planning/warehouse_ros_sqlite/actions/workflows/build_and_test.yaml)
[![codecov](https://codecov.io/gh/ros-planning/warehouse_ros_sqlite/branch/master/graph/badge.svg?token=QHPGDZM8HX)](https://codecov.io/gh/ros-planning/warehouse_ros_sqlite)

# SQLite backend for warehouse_ros

This is a storage backend for warehouse_ros using SQLite.
The name of the sqlite file will be taken from the ROS parameter `warehouse_host`. The `warehouse_port` will be ignored.


## Installation

Create a folder which will become your catkin workspace.
Create a file named `warehouse.rosinstall` with the following content:
```yaml
- git:
    local-name: warehouse_ros
    uri: https://github.com/ros-planning/warehouse_ros.git
    version: kinetic-devel
- git:
    local-name: warehouse_ros_sqlite
    uri: https://github.com/ros-planning/warehouse_ros_sqlite.git
    version: master
```

Now open a terminal and navigate to this folder.
Make sure to have `wstool` and `catkin_tools` installed.
Initialize your workspace and fetch the source files with `wstool`:
```bash
$ rosdep update
$ wstool init src
$ wstool merge -t src warehouse.rosinstall
$ wstool update -t src
```
Install the missing dependencies:
```bash
$ rosdep install -y --from-paths src --ignore-src
```
Build your workspace with catkin:
```bash
$ catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release
...
$ catkin build
```
After that, activate your workspace:
```bash
$ source install/setup.bash
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
