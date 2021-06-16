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

#include <boost/filesystem.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <warehouse_ros_sqlite/database_connection.hpp>
#include <warehouse_ros_sqlite/utils.hpp>
#include <warehouse_ros_sqlite/exceptions.hpp>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace bf = boost::filesystem;
using V = geometry_msgs::msg::Vector3;

struct BusyHandler : ::testing::Test
{
  void SetUp() override
  {
    tmp_dir_ = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path();
    if (!bf::create_directory(tmp_dir_)) {
      // clear tmp_dir so that it's not removed in TearDown()
      tmp_dir_ = "";
      FAIL() << "could not create temporary dir";
    }
    tmp_file_ = tmp_dir_ / "db.sqlite";
    db_conn_.reset(new warehouse_ros_sqlite::DatabaseConnection());
    db_conn_->setParams(tmp_file_.string(), 0);
    ASSERT_TRUE(db_conn_->connect());
    coll_ = db_conn_->openCollectionPtr<V>("main", "coll");
    ASSERT_TRUE(coll_->md5SumMatches());
  }

  void TearDown() override
  {
    bf::remove_all(tmp_dir_);
    coll_.reset();
    db_conn_.reset();
  }

  static bf::path tmp_dir_;
  static bf::path tmp_file_;
  static std::unique_ptr<warehouse_ros_sqlite::DatabaseConnection> db_conn_;
  static warehouse_ros::MessageCollection<V>::Ptr coll_;
  static sqlite3 * startSecondInsert() noexcept;
};

bf::path BusyHandler::tmp_dir_;
bf::path BusyHandler::tmp_file_;
warehouse_ros::MessageCollection<V>::Ptr BusyHandler::coll_;
std::unique_ptr<warehouse_ros_sqlite::DatabaseConnection> BusyHandler::db_conn_;

sqlite3 * BusyHandler::startSecondInsert() noexcept
{
  sqlite3 * raw_db = nullptr;
  if (sqlite3_open(tmp_file_.c_str(), &raw_db) != SQLITE_OK) {
    return nullptr;
  }
  const char * query =
    "BEGIN TRANSACTION; INSERT INTO \"T_main@coll\" ( \"Data\" ) VALUES (x'DEADBEEF');";
  if (sqlite3_exec(raw_db, query, nullptr, nullptr, nullptr) != SQLITE_OK) {
    sqlite3_close(raw_db);
    return nullptr;
  }
  return raw_db;
}

TEST_F(BusyHandler, TimeoutFires)
{
  auto meta = coll_->createMetadata();
  // prepare second database accessor
  warehouse_ros_sqlite::sqlite3_ptr second_db(
    startSecondInsert(), warehouse_ros_sqlite::sqlite3_delete);
  ASSERT_TRUE(static_cast<bool>(second_db));
  EXPECT_THROW(coll_->insert(V(), meta), warehouse_ros_sqlite::InternalError);
}

TEST_F(BusyHandler, HandlerWorks)
{
  std::mutex m;
  std::condition_variable cv;
  std::atomic_bool second_db_setup{false};
  auto meta = coll_->createMetadata();

  std::unique_lock<std::mutex> lk1(m);

  // start second thread, protected by mutex
  std::thread worker([&cv, &m, &second_db_setup]() {
      std::unique_lock<std::mutex> lk2(m);
      // block the database, protected under the mutex
      warehouse_ros_sqlite::sqlite3_ptr second_db(
        startSecondInsert(), warehouse_ros_sqlite::sqlite3_delete);

      // tell main thread if blocking the db was successul and wake it up
      second_db_setup.store(static_cast<bool>(second_db));
      lk2.unlock();
      cv.notify_one();

      if (second_db_setup) {
        // wait for some time and unblock the db afterwards
        constexpr auto wait_interval =
        3 * std::chrono::milliseconds{
          warehouse_ros_sqlite::DatabaseConnection::BUSY_WAIT_MILLISECS};
        std::this_thread::sleep_for(wait_interval);
        second_db.reset();
      }
    });

  // wait for the second thread to block the db
  cv.wait(lk1);
  if (!second_db_setup) {
    ADD_FAILURE() << "Second db setup failed";
  } else {
    // inserting data should work after second thread unblocks in the background
    EXPECT_NO_THROW(coll_->insert(V(), meta));
  }
  // join second thread
  worker.join();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
