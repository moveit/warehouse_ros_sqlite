^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package warehouse_ros_sqlite
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2022-05-17)
------------------
* Add humble testing CI (`#37 <https://github.com/ros-planning/warehouse_ros_sqlite/issues/37>`_)
* add missing stdlib include
* disable header include order check
* don't build warehouse_ros from source
* Contributors: Bjar Ne, Vatan Aksoy Tezer

1.0.2 (2021-10-12)
------------------
* Update CMakeLists.txt
* Adapt github URLs in README
* Fix whitespaces in codecov script
* Enable pre-commit in CI
* Enable prerelease tests
* Add pre-commit config
* Switch to upstream warehouse_ros
* Contributors: Bjar Ne, Nisala Kalupahana

1.0.1 (2021-06-21)
---------------------------------
* ROS 2 Port
* Remove duplicate header guard
* Silence CMake warning
* Delete LICENSE.txt
* Add ROS2 launch config to README
* partially update readme
* Enable codecov
* Switch to github workflow
* Switch to sqlite3_vendor
* Contributors: Bjar Ne

1.0.0 (2020-11-14)
------------------
* version bump
* add busy handler for concurrent writes to db
* Test warehouse plugin loading
* fix clang-format
* Export interfaces (dllexport/visibility=hidden)
* fix clang-tidy
* Versioning of the database scheme
* Adapt scheme to be more precise
* Readme and notes on database schema
* fix clang-format
* Fix query with ordering
* Use proper exception types
* implemented dropping databases
* Support multiple databases with name mangling
  The collection name and the database name are mangled and concatenated
  to support multiple databases.
  SQLite only supports one database per file.
* rollback on initialization error
* Fix validation of stored MD5 sum
* SQL String escaping
* query tests and bugfix
* test change of metadata
* test and fix Null value metadata handling
* test and fix MD5 validation
* local warehouse_ros package
* Add first API test
* more fixes
* add clang-tidy config
* clang-tidy fixes
* fixes
* enable travis
* more stuff
* initial commit
* Contributors: Bjar Ne
