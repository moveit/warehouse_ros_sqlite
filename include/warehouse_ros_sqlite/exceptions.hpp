// Copyright 2020 Bjarne von Horn
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// SPDX-License-Identifier: BSD-3-Clause

#ifndef WAREHOUSE_ROS_SQLITE__EXCEPTIONS_HPP_
#define WAREHOUSE_ROS_SQLITE__EXCEPTIONS_HPP_

#include <warehouse_ros/exceptions.h>
#include <warehouse_ros_sqlite/warehouse_ros_sqlite_export.hpp>
#include <boost/format.hpp>

extern "C" {
struct sqlite3;
struct sqlite3_stmt;
}

namespace warehouse_ros_sqlite
{
struct WAREHOUSE_ROS_SQLITE_EXPORT InternalError : public warehouse_ros::WarehouseRosException
{
  using warehouse_ros::WarehouseRosException::WarehouseRosException;
  InternalError(const char * msg, sqlite3 * db);
  InternalError(const char * msg, sqlite3_stmt * stmt);
};

struct WAREHOUSE_ROS_SQLITE_EXPORT DatatypeMismatch : public warehouse_ros::WarehouseRosException
{
  using warehouse_ros::WarehouseRosException::WarehouseRosException;
};

struct WAREHOUSE_ROS_SQLITE_EXPORT SchemaVersionMismatch : public warehouse_ros::
  WarehouseRosException
{
  int version_in_database_, version_compiled_in_;
  using warehouse_ros::WarehouseRosException::WarehouseRosException;
  SchemaVersionMismatch(int version_in_database, int version_compiled_in)
  : warehouse_ros::WarehouseRosException(
      boost::format(
        "Database schema version mismatch, stored in file: %1%, compiled in version: %2%") %
      version_in_database % version_compiled_in),
    version_in_database_(version_in_database),
    version_compiled_in_(version_compiled_in)
  {
  }
};
}  // namespace warehouse_ros_sqlite


#endif  // WAREHOUSE_ROS_SQLITE__EXCEPTIONS_HPP_
