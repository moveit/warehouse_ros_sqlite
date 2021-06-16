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

#include <warehouse_ros_sqlite/database_connection.hpp>

#include <warehouse_ros_sqlite/exceptions.hpp>
#include <warehouse_ros_sqlite/message_collection_helper.hpp>
#include <warehouse_ros_sqlite/utils.hpp>

#include <boost/make_shared.hpp>
#include <boost/format.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sqlite3.h>

#include <chrono>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("warehouse_ros_sqlite.database_connection");

int busy_handler(void * /* user_ptr */, int times_called_before)
{
  constexpr auto wait_interval =
    std::chrono::milliseconds{warehouse_ros_sqlite::DatabaseConnection::BUSY_WAIT_MILLISECS};
  if (times_called_before >= warehouse_ros_sqlite::DatabaseConnection::BUSY_MAX_RETRIES) {
    return 0;
  }

  std::this_thread::sleep_for((times_called_before + 1) * wait_interval);
  return 1;
}
}  // namespace

// instance to avoid linking errors
const int warehouse_ros_sqlite::DatabaseConnection::BUSY_WAIT_MILLISECS;
const int warehouse_ros_sqlite::DatabaseConnection::BUSY_MAX_RETRIES;

/// Setup the database connection. This call assumes setParams() has been previously called.
/// Returns true if the connection was succesfully established.
bool warehouse_ros_sqlite::DatabaseConnection::connect()
{
  if (!db_) {
    sqlite3 * s = nullptr;
    if (sqlite3_open(uri_.c_str(), &s) != SQLITE_OK) {
      return false;
    }
    db_.reset(s, warehouse_ros_sqlite::sqlite3_delete);
  }
  if (sqlite3_busy_handler(db_.get(), busy_handler, nullptr) != SQLITE_OK) {
    throw InternalError("setting busy handler failed", db_.get());
  }
  initDb();
  return true;
}

/// Returns whether the database is connected.
bool warehouse_ros_sqlite::DatabaseConnection::isConnected()
{
  return static_cast<bool>(db_);
}

std::vector<std::string> warehouse_ros_sqlite::DatabaseConnection::getTablesOfDatabase(
  const std::string & db_name)
{
  std::ostringstream query_builder;
  query_builder << "SELECT " << schema::M_D5_TABLE_INDEX_COLUMN << " FROM " <<
    schema::M_D5_TABLE_NAME << " WHERE " <<
    schema::M_D5_TABLE_DATABASE_COLUMN << " == ?;";
  const auto select_query = query_builder.str();
  sqlite3_stmt * raw_stmt = nullptr;
  if (sqlite3_prepare_v2(
      db_.get(), select_query.c_str(), select_query.size() + 1, &raw_stmt,
      nullptr) != SQLITE_OK)
  {
    throw InternalError("Prepare statement for getTablesOfDatabase() failed", db_.get());
  }
  sqlite3_stmt_ptr stmt(std::exchange(raw_stmt, nullptr));
  if (sqlite3_bind_text(
      stmt.get(), 1, db_name.c_str(), db_name.size(),
      SQLITE_STATIC) != SQLITE_OK)
  {
    throw InternalError("Bind parameter for getTablesOfDatabase() failed", db_.get());
  }
  std::vector<std::string> tables;
  for (int res = sqlite3_step(stmt.get()); res != SQLITE_DONE; res = sqlite3_step(stmt.get())) {
    if (res == SQLITE_ROW) {
      tables.emplace_back(
        reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 0)),
        sqlite3_column_bytes(stmt.get(), 0));
    } else {
      throw InternalError("Get results for getTablesOfDatabase() failed", db_.get());
    }
  }
  return tables;
}

/// \brief Drop a db and all its collections.
/// A DbClientConnection exception will be thrown if the database is not connected.
void warehouse_ros_sqlite::DatabaseConnection::dropDatabase(const std::string & db_name)
{
  const auto tables_to_be_dropped = getTablesOfDatabase(db_name);
  std::ostringstream query_builder;
  for (const auto & table : tables_to_be_dropped) {
    const auto escaped_table_string = schema::escape_string_literal_without_quotes(table);
    const auto escaped_table_identifier = schema::escape_identifier(table);
    query_builder << "DELETE FROM " << schema::M_D5_TABLE_NAME << " WHERE " <<
      schema::M_D5_TABLE_INDEX_COLUMN <<
      " == '" << escaped_table_string << "'; ";
    query_builder << "DROP TABLE " << escaped_table_identifier << ";";
  }
  query_builder << "COMMIT;";
  const auto query = query_builder.str();
  if (sqlite3_exec(db_.get(), "BEGIN TRANSACTION;", nullptr, nullptr, nullptr) == SQLITE_OK) {
    if (sqlite3_exec(db_.get(), query.c_str(), nullptr, nullptr, nullptr) == SQLITE_OK) {
      return;
    }
    sqlite3_exec(db_.get(), "ROLLBACK;", nullptr, nullptr, nullptr);
  }
  throw InternalError("Drop tables failed", db_.get());
}

/// \brief Return the ROS Message type of a given collection
std::string warehouse_ros_sqlite::DatabaseConnection::messageType(
  const std::string & db_name,
  const std::string & collection_name)
{
  using warehouse_ros_sqlite::schema::M_D5_TABLE_DATATYPE_COLUMN;
  using warehouse_ros_sqlite::schema::M_D5_TABLE_NAME;
  using warehouse_ros_sqlite::schema::M_D5_TABLE_INDEX_COLUMN;

  std::ostringstream query_builder;
  query_builder << "SELECT " << M_D5_TABLE_DATATYPE_COLUMN << " FROM " << M_D5_TABLE_NAME <<
    " WHERE " <<
    M_D5_TABLE_INDEX_COLUMN << " = ?;";
  const auto query = query_builder.str();
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db_.get(), query.c_str(), query.size() + 1, &stmt, nullptr) != SQLITE_OK) {
    throw InternalError("Prepare statement for messageType() failed", db_.get());
  }
  const sqlite3_stmt_ptr guard(stmt);
  const auto mangled_name = schema::mangle_database_and_collection_name(db_name, collection_name);
  if (sqlite3_bind_text(
      stmt, 1, mangled_name.c_str(), mangled_name.size(),
      SQLITE_STATIC) != SQLITE_OK)
  {
    throw InternalError("Bind parameter for getTablesOfDatabase() failed", db_.get());
  }
  switch (sqlite3_step(stmt)) {
    case SQLITE_ROW:
      break;
    case SQLITE_DONE:
    default:
      throw InternalError("Get result for getTablesOfDatabase() failed", db_.get());
  }
  return std::string(
    reinterpret_cast<const char *>(sqlite3_column_text(
      stmt,
      0)),
    sqlite3_column_bytes(stmt, 0));
}

void warehouse_ros_sqlite::DatabaseConnection::initDb()
{
  if (schemaVersionSet()) {
    return;
  }
  std::ostringstream query_builder;
  query_builder << "PRAGMA user_version = " << schema::VERSION << ";" <<
    "CREATE TABLE " << schema::M_D5_TABLE_NAME << " ( " << schema::M_D5_TABLE_INDEX_COLUMN <<
    " TEXT PRIMARY KEY, " << schema::M_D5_TABLE_M_D5_COLUMN << " BLOB NOT NULL, " <<
    schema::M_D5_TABLE_TABLE_COLUMN << " TEXT NOT NULL, " << schema::M_D5_TABLE_DATABASE_COLUMN <<
    " TEXT NOT NULL, " << schema::M_D5_TABLE_DATATYPE_COLUMN << " TEXT NOT NULL);";
  const auto query = query_builder.str();
  RCLCPP_DEBUG_STREAM(LOGGER, "MD5 table init: " << query);
  if (sqlite3_exec(db_.get(), query.c_str(), nullptr, nullptr, nullptr) != SQLITE_OK) {
    throw InternalError("Could not initialize Database", db_.get());
  }
}

bool warehouse_ros_sqlite::DatabaseConnection::schemaVersionSet()
{
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db_.get(), "PRAGMA user_version;", -1, &stmt, nullptr) != SQLITE_OK) {
    throw InternalError("Could not get schema version", db_.get());
  }
  sqlite3_stmt_ptr stmt_guard(std::exchange(stmt, nullptr));
  if (sqlite3_step(stmt_guard.get()) != SQLITE_ROW) {
    throw InternalError("Could not get schema version", db_.get());
  }
  const int current_schema_version = sqlite3_column_int(stmt_guard.get(), 0);
  if (current_schema_version == 0) {
    return false;
  } else {
    if (current_schema_version == schema::VERSION) {
      return true;
    } else {
      throw SchemaVersionMismatch(current_schema_version, schema::VERSION);
    }
  }
}

warehouse_ros::MessageCollectionHelper::Ptr
warehouse_ros_sqlite::DatabaseConnection::openCollectionHelper(
  const std::string & db_name,
  const std::string & collection_name)
{
  return boost::make_shared<warehouse_ros_sqlite::MessageCollectionHelper>(
    db_, db_name,
    collection_name);
}

void warehouse_ros_sqlite::Sqlite3StmtDeleter::operator()(sqlite3_stmt * stmt) const
{
  sqlite3_finalize(stmt);
}
void warehouse_ros_sqlite::sqlite3_delete(sqlite3 * db)
{
  if (sqlite3_close(db) != SQLITE_OK) {
    RCLCPP_ERROR(LOGGER, "sqlite connection closed when still in use");
  }
}

warehouse_ros_sqlite::InternalError::InternalError(const char * msg, sqlite3 * db)
: warehouse_ros::WarehouseRosException(boost::format("%1% %2%") % msg % sqlite3_errmsg(db))
{
}
warehouse_ros_sqlite::InternalError::InternalError(const char * msg, sqlite3_stmt * stmt)
: InternalError(msg, sqlite3_db_handle(stmt))
{
}

PLUGINLIB_EXPORT_CLASS(warehouse_ros_sqlite::DatabaseConnection, warehouse_ros::DatabaseConnection)
