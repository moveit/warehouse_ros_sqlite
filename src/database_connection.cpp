// SPDX-License-Identifier: BSD-3-Clause

/*
 * Copyright (c) 2020, Bjarne von Horn
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL BJARNE VON HORN BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <warehouse_ros_sqlite/database_connection.h>

#include <warehouse_ros_sqlite/message_collection_helper.h>
#include <warehouse_ros_sqlite/utils.h>

#include <boost/make_shared.hpp>
#include <pluginlib/class_list_macros.h>
#include <sqlite3.h>

/// Setup the database connection. This call assumes setParams() has been previously called.
/// Returns true if the connection was succesfully established.
bool warehouse_ros_sqlite::DatabaseConnection::connect()
{
  sqlite3* s = nullptr;
  if (sqlite3_open(uri_.c_str(), &s) != SQLITE_OK)
  {
    return false;
  }
  db_.reset(s, warehouse_ros_sqlite::sqlite3_delete);
  return true;
}

/// Returns whether the database is connected.
bool warehouse_ros_sqlite::DatabaseConnection::isConnected()
{
  return static_cast<bool>(db_);
}

/// \brief Drop a db and all its collections.
/// A DbClientConnection exception will be thrown if the database is not connected.
void warehouse_ros_sqlite::DatabaseConnection::dropDatabase(const std::string& db_name)
{
}

/// \brief Return the ROS Message type of a given collection
std::string warehouse_ros_sqlite::DatabaseConnection::messageType(const std::string& db_name,
                                                                  const std::string& collection_name)
{
  check_dbname(db_name);
}

warehouse_ros::MessageCollectionHelper::Ptr
warehouse_ros_sqlite::DatabaseConnection::openCollectionHelper(const std::string& db_name,
                                                               const std::string& collection_name)
{
  return boost::make_shared<warehouse_ros_sqlite::MessageCollectionHelper>(db_, collection_name);
}

void warehouse_ros_sqlite::sqlite3_stmt_deleter::operator()(sqlite3_stmt* stmt) const
{
  sqlite3_finalize(stmt);
}
void warehouse_ros_sqlite::sqlite3_delete(sqlite3* db)
{
  if (sqlite3_close(db) != SQLITE_OK)
  {
    ROS_ERROR("sqlite connection closed when still in use");
  }
}

PLUGINLIB_EXPORT_CLASS(warehouse_ros_sqlite::DatabaseConnection, warehouse_ros::DatabaseConnection)
