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
#include <warehouse_ros_sqlite/variant.h>

#include <boost/make_shared.hpp>
#include <sqlite3.h>
#include <sstream>

std::string warehouse_ros_sqlite::MessageCollectionHelper::find_md5sum()
{
  sqlite3_stmt* stmt = nullptr;
  std::ostringstream query_builder;
  query_builder << "SELECT " << schema::MD5TableMD5Column << " FROM " << schema::MD5TableName << " WHERE "
                << schema::MD5TableIndexColumn << " = ?;";
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
  if (sqlite3_step(stmt) == SQLITE_ROW)
  {
    return std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1)), sqlite3_column_bytes(stmt, 1));
  }
  return "";
}

bool warehouse_ros_sqlite::MessageCollectionHelper::initialize(const std::string& datatype, const std::string& md5)
{
  using namespace warehouse_ros_sqlite::schema;
  const auto current_md5 = find_md5sum();
  if (!current_md5.empty())
  {
    return current_md5 == md5;
  }
  std::ostringstream query_builder;
  query_builder << "BEGIN TRANSACTION; CREATE TABLE " << TableNamePrefix << name_ << "; INSERT INTO " << MD5TableName
                << " ( " << MD5TableIndexColumn << " , " << MD5TableMD5Column << ") VALUES (" << name_ << " , " << md5
                << "); COMMIT TRANSACTION;";
  const auto query = query_builder.str();
  return sqlite3_exec(db_.get(), query.c_str(), nullptr, nullptr, nullptr) == SQLITE_OK;
}
void warehouse_ros_sqlite::MessageCollectionHelper::insert(char* msg, size_t msg_size,
                                                           warehouse_ros::Metadata::ConstPtr metadata)
{
  auto meta = reinterpret_cast<const warehouse_ros_sqlite::Metadata*>(metadata.get());
  if (!meta || !msg || !msg_size)
    throw std::runtime_error("");
  std::ostringstream query;
  query << "INSERT INTO " << schema::TableNamePrefix << name_ << "(";
  query << schema::DataColumnName << ",";

  const auto& data = meta->data();
  for (const auto& kv : data)
  {
    query << ", " << std::get<0>(kv);
  }
  query << ") VALUES ( ? ";
  for (size_t i = 0; i < data.size(); ++i)
    query << ", ? ";
  query << ");";

  sqlite3_stmt* stmt = nullptr;
  const auto query_str = query.str();
  if (sqlite3_prepare_v2(db_.get(), query_str.c_str(), query_str.size() + 1, &stmt, nullptr) != SQLITE_OK)
    throw std::runtime_error("");
  const sqlite3_stmt_ptr stmt_guard(stmt);

  if (sqlite3_bind_blob(stmt, 1, msg, msg_size, SQLITE_STATIC) != SQLITE_OK)
    throw std::runtime_error("");
  warehouse_ros_sqlite::BindVisitor visitor(stmt, 2);
  for (const auto& kv: data)
  {
    if (boost::apply_visitor(visitor, std::get<1>(kv)) != SQLITE_OK)
      throw std::runtime_error("");
  }

  assert(sqlite3_bind_parameter_count(stmt) == visitor.get_total_binds());
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
    outro += " ORDER BY " + sort_by;
  }
  if (!ascending)
  {
    outro += " DESC";
  }
  auto query_ptr = dynamic_cast<const warehouse_ros_sqlite::Query*>(query.get());
  assert(query_ptr);
  std::ostringstream intro;
  intro << "SELECT * FROM " << schema::TableNamePrefix << name_ << " WHERE ";
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
  auto stmt = pquery->prepare(db_.get(), "DELETE FROM " + name_ + "WHERE ");
  return sqlite3_changes(db_.get());
}
void warehouse_ros_sqlite::MessageCollectionHelper::modifyMetadata(warehouse_ros::Query::ConstPtr q,
                                                                   warehouse_ros::Metadata::ConstPtr m)
{
}
unsigned warehouse_ros_sqlite::MessageCollectionHelper::count()
{
}
warehouse_ros::Query::Ptr warehouse_ros_sqlite::MessageCollectionHelper::createQuery() const
{
  return boost::make_shared<warehouse_ros_sqlite::Query>();
}
warehouse_ros::Metadata::Ptr warehouse_ros_sqlite::MessageCollectionHelper::createMetadata() const
{
  return boost::make_shared<warehouse_ros_sqlite::Metadata>();
}
