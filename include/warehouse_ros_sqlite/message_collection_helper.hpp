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

#ifndef WAREHOUSE_ROS_SQLITE__MESSAGE_COLLECTION_HELPER_HPP_
#define WAREHOUSE_ROS_SQLITE__MESSAGE_COLLECTION_HELPER_HPP_

#include <warehouse_ros/message_collection.h>
#include <warehouse_ros_sqlite/utils.hpp>
#include <warehouse_ros_sqlite/warehouse_ros_sqlite_export.hpp>

#include <string>
#include <utility>

namespace warehouse_ros_sqlite
{
class WAREHOUSE_ROS_SQLITE_EXPORT MessageCollectionHelper : public warehouse_ros::
  MessageCollectionHelper
{
  sqlite3_ptr db_;
  std::string collection_name_;
  std::string db_name_;
  std::string mangled_tablename_;
  schema::escaped_tablename escaped_mangled_name_;

public:
  MessageCollectionHelper() = default;
  MessageCollectionHelper(sqlite3_ptr db, const std::string & db_name, const std::string & name)
  : db_(std::move(db)),
    collection_name_(name),
    db_name_(db_name),
    mangled_tablename_(schema::mangle_database_and_collection_name(db_name, name)),
    escaped_mangled_name_(schema::escape_and_mangle_database_and_collection_name(db_name, name))
  {
  }
  bool initialize(const std::string & datatype, const std::string & md5) override;
  void insert(char * msg, size_t msg_size, warehouse_ros::Metadata::ConstPtr metadata) override;
  warehouse_ros::ResultIteratorHelper::Ptr query(
    warehouse_ros::Query::ConstPtr query, const std::string & sort_by = "",
    bool ascending = true) const override;
  unsigned removeMessages(warehouse_ros::Query::ConstPtr query) override;
  void modifyMetadata(
    warehouse_ros::Query::ConstPtr q,
    warehouse_ros::Metadata::ConstPtr m) override;
  unsigned count() override;
  warehouse_ros::Query::Ptr createQuery() const override;
  warehouse_ros::Metadata::Ptr createMetadata() const override;
  std::string collectionName() const override
  {
    return collection_name_;
  }

private:
  enum class Md5CompareResult
  {
    EMPTY,
    MATCH,
    MISMATCH
  };
  Md5CompareResult findAndMatchMd5Sum(const std::array<unsigned char, 16> & md5_bytes);
};

}  // namespace warehouse_ros_sqlite

#endif  // WAREHOUSE_ROS_SQLITE__MESSAGE_COLLECTION_HELPER_HPP_
