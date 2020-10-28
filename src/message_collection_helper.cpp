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
#include <warehouse_ros_sqlite/message_collection_helper.h>

#include <warehouse_ros_sqlite/query.h>
#include <warehouse_ros_sqlite/metadata.h>
#include <warehouse_ros_sqlite/result_iteration_helper.h>
#include <warehouse_ros_sqlite/impl/variant.h>

#include <boost/make_shared.hpp>
#include <sqlite3.h>
#include <sstream>
#include <cstdlib>
#include <ros/console.h>

namespace
{
std::string getMetadataColumn(const std::string& c)
{
  return warehouse_ros_sqlite::schema::METADATA_COLUMN_PREFIX + c;
}
}  // namespace

std::vector<char> warehouse_ros_sqlite::MessageCollectionHelper::findMd5sum()
{
  sqlite3_stmt* stmt = nullptr;
  std::ostringstream query_builder;
  query_builder << "SELECT " << schema::M_D5_TABLE_M_D5_COLUMN << " FROM " << schema::M_D5_TABLE_NAME << " WHERE "
                << schema::M_D5_TABLE_INDEX_COLUMN << " == ? ;";
  const auto query = query_builder.str();
  if (sqlite3_prepare_v2(db_.get(), query.c_str(), query.size() + 1, &stmt, nullptr) != SQLITE_OK)
  {
    throw std::runtime_error("");
  }
  sqlite3_stmt_ptr stmt_ptr(stmt);
  if (sqlite3_bind_text(stmt, 1, name_.c_str(), name_.size(), SQLITE_STATIC) != SQLITE_OK)
  {
    throw std::runtime_error("");
  }
  std::vector<char> ans;
  if (sqlite3_step(stmt) == SQLITE_ROW)
  {
    const int count = sqlite3_column_bytes(stmt, 0);
    const auto col = static_cast<const char*>(sqlite3_column_blob(stmt, 0));
    ans.reserve(count);
    std::copy(col, col + count, std::back_inserter(ans));
  }
  return ans;
}

bool warehouse_ros_sqlite::MessageCollectionHelper::initialize(const std::string& datatype, const std::string& md5)
{
  using namespace warehouse_ros_sqlite::schema;
  const auto current_md5 = findMd5sum();
  if (!current_md5.empty())
  {
    if (md5.size() != 32)
    {
      throw std::invalid_argument("md5.size() must equal 32");
    }
    std::vector<char> binary_md5(16);
    std::istringstream s(md5);
    s >> std::hex;
    size_t md5_idx = 0;
    for (auto& c : binary_md5)
    {
      const auto t = std::strtoul(md5.substr(md5_idx, 2).c_str(), nullptr, 16);
      if (t == ULONG_MAX)
        throw std::invalid_argument("md5 is not hex string");
      c = static_cast<unsigned char>(t);
      md5_idx += 2;
    }
    return current_md5 == binary_md5;
  }
  std::ostringstream query_builder;
  query_builder << "BEGIN TRANSACTION; CREATE TABLE " << getTableName() << "(" << schema::DATA_COLUMN_NAME
                << " BLOB NOT NULL, " << schema::METADATA_COLUMN_PREFIX << "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                << schema::METADATA_COLUMN_PREFIX << "creation_time INTEGER)"
                << "; INSERT INTO " << M_D5_TABLE_NAME << " ( " << M_D5_TABLE_INDEX_COLUMN << " , "
                << M_D5_TABLE_M_D5_COLUMN << " , " << M_D5_TABLE_DATATYPE_COLUMN << ") VALUES ('" << name_ << "' , x'"
                << md5 << "' , '" << datatype << "'); COMMIT TRANSACTION;";
  const auto query = query_builder.str();
  ROS_DEBUG_NAMED("warehouse_ros_sqlite", "initialize query: %s", query.c_str());
  return sqlite3_exec(db_.get(), query.c_str(), nullptr, nullptr, nullptr) == SQLITE_OK;
}

void warehouse_ros_sqlite::MessageCollectionHelper::insert(char* msg, size_t msg_size,
                                                           warehouse_ros::Metadata::ConstPtr metadata)
{
  auto meta = reinterpret_cast<const warehouse_ros_sqlite::Metadata*>(metadata.get());
  if (!meta || !msg || !msg_size)
    throw std::runtime_error("");
  meta->ensureColumns(db_.get(), getTableName());
  std::ostringstream query;
  query << "INSERT INTO " << getTableName() << "(" << schema::DATA_COLUMN_NAME;

  const auto& data = meta->data();
  for (const auto& kv : data)
  {
    query << ", " << getMetadataColumn(std::get<0>(kv));
  }
  query << ") VALUES ( ? ";
  for (size_t i = 0; i < data.size(); ++i)
    query << ", ? ";
  query << ");";

  sqlite3_stmt* stmt = nullptr;
  const auto query_str = query.str();
  ROS_DEBUG_NAMED("warehouse_ros_sqlite", "insert query: %s", query_str.c_str());
  if (sqlite3_prepare_v2(db_.get(), query_str.c_str(), query_str.size() + 1, &stmt, nullptr) != SQLITE_OK)
    throw std::runtime_error("");
  const sqlite3_stmt_ptr stmt_guard(stmt);

  if (sqlite3_bind_blob(stmt, 1, msg, msg_size, SQLITE_STATIC) != SQLITE_OK)
    throw std::runtime_error("");
  warehouse_ros_sqlite::BindVisitor visitor(stmt, 2);
  for (const auto& kv : data)
  {
    if (boost::apply_visitor(visitor, std::get<1>(kv)) != SQLITE_OK)
      throw std::runtime_error("");
  }

  assert(sqlite3_bind_parameter_count(stmt) == visitor.getTotalBinds());
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("");
}

warehouse_ros::ResultIteratorHelper::Ptr
warehouse_ros_sqlite::MessageCollectionHelper::query(warehouse_ros::Query::ConstPtr query, const std::string& sort_by,
                                                     bool ascending) const
{
  std::string outro;
  if (!sort_by.empty())
  {
    outro += " ORDER BY " + sort_by + (ascending ? " ASC" : " DESC");
  }
  auto query_ptr = dynamic_cast<const warehouse_ros_sqlite::Query*>(query.get());
  assert(query_ptr);
  std::ostringstream intro;
  intro << "SELECT * FROM " << getTableName();
  if (!query_ptr->empty())
  {
    intro << " WHERE ";
  }
  auto stmt = query_ptr->prepare(db_.get(), intro.str(), outro);
  switch (sqlite3_step(stmt.get()))
  {
    case SQLITE_DONE:
    case SQLITE_ROW:
      break;
    default:
      throw std::runtime_error("");
  }
  return boost::make_shared<warehouse_ros_sqlite::ResultIteratorHelper>(std::move(stmt));
}

unsigned warehouse_ros_sqlite::MessageCollectionHelper::removeMessages(warehouse_ros::Query::ConstPtr query)
{
  auto pquery = dynamic_cast<warehouse_ros_sqlite::Query const*>(query.get());
  if (!pquery)
    throw std::runtime_error("Query was not initialized by createQuery()");
  auto stmt = pquery->prepare(db_.get(), "DELETE FROM " + getTableName() + " WHERE ");
  if (sqlite3_step(stmt.get()) != SQLITE_DONE)
  {
    throw std::runtime_error("");
  }
  return sqlite3_changes(db_.get());
}

namespace
{
template <typename It>
void comma_concat_meta_column_names(std::ostringstream& buf, It it, It end)
{
  if (it == end)
    return;
  buf << getMetadataColumn(it->first);
  it++;
  while (it != end)
  {
    buf << " = ?, " << getMetadataColumn(it->first);
    it++;
  }
  buf << " = ?";
}
}  // namespace

void warehouse_ros_sqlite::MessageCollectionHelper::modifyMetadata(warehouse_ros::Query::ConstPtr q,
                                                                   warehouse_ros::Metadata::ConstPtr m)
{
  auto query = dynamic_cast<const warehouse_ros_sqlite::Query*>(q.get());
  auto metadata = dynamic_cast<const warehouse_ros_sqlite::Metadata*>(m.get());
  if (!query || !metadata)
    throw std::runtime_error("");
  metadata->ensureColumns(db_.get(), getTableName());
  const int mt_count = metadata->data().size();
  if (mt_count == 0)
    return;

  std::ostringstream query_builder;
  query_builder << "UPDATE " << getTableName() << " SET ";

  comma_concat_meta_column_names(query_builder, metadata->data().begin(), metadata->data().end());
  query_builder << " WHERE ";
  auto stmt = query->prepare(db_.get(), query_builder.str(), "", mt_count + 1);
  warehouse_ros_sqlite::BindVisitor visitor(stmt.get(), 1);
  for (const auto& kv : metadata->data())
  {
    if (boost::apply_visitor(visitor, std::get<1>(kv)) != SQLITE_OK)
      throw std::runtime_error("");
  }

  if (sqlite3_step(stmt.get()) != SQLITE_DONE)
    throw std::runtime_error("");
}

unsigned warehouse_ros_sqlite::MessageCollectionHelper::count()
{
  const std::string query = "SELECT COUNT(*) FROM " + getTableName() + ";";
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(db_.get(), query.c_str(), query.size() + 1, &stmt, nullptr) != SQLITE_OK)
    throw std::runtime_error("");
  const warehouse_ros_sqlite::sqlite3_stmt_ptr stmt_guard(stmt);

  if (sqlite3_step(stmt) != SQLITE_ROW)
    throw std::runtime_error("");

  assert(sqlite3_column_count(stmt) == 1);

  return sqlite3_column_int(stmt, 0);
}
warehouse_ros::Query::Ptr warehouse_ros_sqlite::MessageCollectionHelper::createQuery() const
{
  return boost::make_shared<warehouse_ros_sqlite::Query>();
}
warehouse_ros::Metadata::Ptr warehouse_ros_sqlite::MessageCollectionHelper::createMetadata() const
{
  return boost::make_shared<warehouse_ros_sqlite::Metadata>();
}
