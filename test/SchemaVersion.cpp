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

#include <gtest/gtest.h>
#include <sqlite3.h>

#include <warehouse_ros_sqlite/database_connection.hpp>
#include <warehouse_ros_sqlite/exceptions.hpp>
#include <warehouse_ros_sqlite/utils.hpp>

#include <string>

class SchemaVersion : public ::testing::Test
{
protected:
  void SetUp() override
  {
    sqlite3 * db = nullptr;
    ASSERT_EQ(sqlite3_open(":memory:", &db), SQLITE_OK);
    db_.reset(db, &warehouse_ros_sqlite::sqlite3_delete);
  }

  static warehouse_ros_sqlite::sqlite3_ptr db_;
};

warehouse_ros_sqlite::sqlite3_ptr SchemaVersion::db_;

TEST_F(SchemaVersion, WrongSchemaVersion)
{
  const int wrong_schema = 2;
  static_assert(
    wrong_schema != warehouse_ros_sqlite::schema::VERSION,
    "schema version is not invalid");

  const std::string query = "PRAGMA user_version = " + std::to_string(wrong_schema) + ";";
  ASSERT_EQ(sqlite3_exec(db_.get(), query.c_str(), nullptr, nullptr, nullptr), SQLITE_OK);

  warehouse_ros_sqlite::DatabaseConnection conn(db_);
  try {
    conn.connect();
    FAIL() << "connect() didn't throw any exception";
  } catch (const warehouse_ros_sqlite::SchemaVersionMismatch & m) {
    EXPECT_EQ(m.version_compiled_in_, warehouse_ros_sqlite::schema::VERSION);
    EXPECT_EQ(m.version_in_database_, wrong_schema);
  } catch (...) {
    FAIL() << "connect() threw the wrong exception type";
  }
}

TEST_F(SchemaVersion, NoSchemaVersionButTable)
{
  const int default_schema = 0;
  sqlite3_stmt * stmt = nullptr;
  ASSERT_EQ(sqlite3_prepare_v2(db_.get(), "PRAGMA user_version;", -1, &stmt, nullptr), SQLITE_OK);
  warehouse_ros_sqlite::sqlite3_stmt_ptr stmt_guard(stmt);
  ASSERT_EQ(sqlite3_step(stmt), SQLITE_ROW);
  ASSERT_EQ(sqlite3_column_int(stmt, 0), default_schema);

  const std::string query =
    std::string("CREATE TABLE ") + warehouse_ros_sqlite::schema::M_D5_TABLE_NAME +
    "(ID INTEGER PRIMARY KEY);";
  ASSERT_EQ(sqlite3_exec(db_.get(), query.c_str(), nullptr, nullptr, nullptr), SQLITE_OK);

  warehouse_ros_sqlite::DatabaseConnection conn(db_);
  EXPECT_THROW(conn.connect(), warehouse_ros_sqlite::InternalError);
}

TEST_F(SchemaVersion, CorrectSet)
{
  {
    warehouse_ros_sqlite::DatabaseConnection conn(db_);
    ASSERT_NO_THROW(conn.connect());
  }
  {
    warehouse_ros_sqlite::DatabaseConnection conn(db_);
    ASSERT_NO_THROW(conn.connect());
  }
  sqlite3_stmt * stmt = nullptr;
  ASSERT_EQ(sqlite3_prepare_v2(db_.get(), "PRAGMA user_version;", -1, &stmt, nullptr), SQLITE_OK);
  warehouse_ros_sqlite::sqlite3_stmt_ptr stmt_guard(stmt);
  ASSERT_EQ(sqlite3_step(stmt), SQLITE_ROW);
  ASSERT_EQ(sqlite3_column_int(stmt, 0), warehouse_ros_sqlite::schema::VERSION);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
