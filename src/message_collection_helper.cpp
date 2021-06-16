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


#include <warehouse_ros_sqlite/message_collection_helper.hpp>

#include <warehouse_ros_sqlite/exceptions.hpp>
#include <warehouse_ros_sqlite/impl/variant.hpp>
#include <warehouse_ros_sqlite/metadata.hpp>
#include <warehouse_ros_sqlite/query.hpp>
#include <warehouse_ros_sqlite/result_iteration_helper.hpp>

#include <boost/make_shared.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sqlite3.h>

#include <cstring>
#include <sstream>
#include <string>
#include <utility>

static const rclcpp::Logger LOGGER = rclcpp::get_logger(
  "warehouse_ros_sqlite.message_collection_helper");


warehouse_ros_sqlite::MessageCollectionHelper::Md5CompareResult
warehouse_ros_sqlite::MessageCollectionHelper::findAndMatchMd5Sum(
  const std::array<unsigned char,
  16> & md5_bytes)
{
  sqlite3_stmt * stmt = nullptr;
  std::ostringstream query_builder;
  query_builder << "SELECT " << schema::M_D5_TABLE_M_D5_COLUMN << " FROM " <<
    schema::M_D5_TABLE_NAME << " WHERE " <<
    schema::M_D5_TABLE_INDEX_COLUMN << " == ? ;";
  const auto query = query_builder.str();
  if (sqlite3_prepare_v2(db_.get(), query.c_str(), query.size() + 1, &stmt, nullptr) != SQLITE_OK) {
    throw InternalError("Prepare statement for findAndMatchMd5Sum() failed", db_.get());
  }
  sqlite3_stmt_ptr stmt_ptr(stmt);
  if (sqlite3_bind_text(
      stmt, 1, mangled_tablename_.c_str(), mangled_tablename_.size(),
      SQLITE_STATIC) != SQLITE_OK)
  {
    throw InternalError("Bind parameter for findAndMatchMd5Sum() failed", db_.get());
  }
  switch (sqlite3_step(stmt)) {
    case SQLITE_DONE:
      return Md5CompareResult::EMPTY;
    case SQLITE_ROW:
      break;
    default:
      throw InternalError("Fetch result for findAndMatchMd5Sum() failed", db_.get());
  }

  if (std::size_t(sqlite3_column_bytes(stmt, 0)) != md5_bytes.size()) {
    throw std::invalid_argument("invalid md5 value");
  }
  if (std::memcmp(&md5_bytes[0], sqlite3_column_blob(stmt, 0), md5_bytes.size()) == 0) {
    return Md5CompareResult::MATCH;
  } else {
    return Md5CompareResult::MISMATCH;
  }
}

bool warehouse_ros_sqlite::MessageCollectionHelper::initialize(
  const std::string & datatype,
  const std::string & md5)
{
  namespace schema = warehouse_ros_sqlite::schema;
  const auto md5_bytes = warehouse_ros_sqlite::parse_md5_hexstring(md5);
  const auto current_md5_state = findAndMatchMd5Sum(md5_bytes);
  if (current_md5_state != Md5CompareResult::EMPTY) {
    return current_md5_state == Md5CompareResult::MATCH;
  }

  std::ostringstream query_builder;
  const auto & esc = schema::escape_string_literal_without_quotes;
  query_builder << "BEGIN TRANSACTION; CREATE TABLE " << escaped_mangled_name_ << "(" <<
    schema::DATA_COLUMN_NAME <<
    " BLOB NOT NULL, " << schema::METADATA_COLUMN_PREFIX <<
    "id INTEGER PRIMARY KEY AUTOINCREMENT, " <<
    schema::METADATA_COLUMN_PREFIX << "creation_time INTEGER)" <<
    "; INSERT INTO " << schema::M_D5_TABLE_NAME <<
    " ( " << schema::M_D5_TABLE_INDEX_COLUMN << " , " <<
    schema::M_D5_TABLE_TABLE_COLUMN << " , " << schema::M_D5_TABLE_DATABASE_COLUMN << " , " <<
    schema::M_D5_TABLE_M_D5_COLUMN <<
    " , " << schema::M_D5_TABLE_DATATYPE_COLUMN << ") VALUES ('" <<
    esc(mangled_tablename_) << "', '" <<
    esc(collection_name_) << "', '" << esc(db_name_) << "' , x'" << verify_md5_string(md5) <<
    "' , '" << esc(datatype) <<
    "'); COMMIT TRANSACTION;";
  const auto query = query_builder.str();
  RCLCPP_DEBUG_STREAM(LOGGER, "initialize query: " << query);
  if (sqlite3_exec(db_.get(), query.c_str(), nullptr, nullptr, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR_STREAM(LOGGER, "Database initialization failed: " << sqlite3_errmsg(db_.get()));
    sqlite3_exec(db_.get(), "ROLLBACK;", nullptr, nullptr, nullptr);
    return false;
  }
  return true;
}

void warehouse_ros_sqlite::MessageCollectionHelper::insert(
  char * msg, size_t msg_size,
  warehouse_ros::Metadata::ConstPtr metadata)
{
  auto meta = reinterpret_cast<const warehouse_ros_sqlite::Metadata *>(metadata.get());
  if (!meta || !msg || !msg_size) {
    throw std::invalid_argument("meta, msg or msg_size is 0");
  }
  meta->ensureColumns(db_.get(), mangled_tablename_);
  std::ostringstream query;
  query << "INSERT INTO " << escaped_mangled_name_ << " (" << schema::DATA_COLUMN_NAME;

  const auto & data = meta->data();
  for (const auto & kv : data) {
    query << ", " << schema::escape_columnname_with_prefix(std::get<0>(kv));
  }
  query << ") VALUES ( ? ";
  for (size_t i = 0; i < data.size(); ++i) {
    query << ", ? ";
  }
  query << ");";

  sqlite3_stmt * stmt = nullptr;
  const auto query_str = query.str();
  RCLCPP_DEBUG_STREAM(LOGGER, "insert query:" << query_str);
  if (sqlite3_prepare_v2(
      db_.get(), query_str.c_str(), query_str.size() + 1, &stmt,
      nullptr) != SQLITE_OK)
  {
    throw InternalError("Prepare statement for insert() failed", db_.get());
  }
  const sqlite3_stmt_ptr stmt_guard(stmt);

  if (sqlite3_bind_blob(stmt, 1, msg, msg_size, SQLITE_STATIC) != SQLITE_OK) {
    throw InternalError("Bind parameter for insert() failed", db_.get());
  }
  warehouse_ros_sqlite::BindVisitor visitor(stmt, 2);
  for (const auto & kv : data) {
    if (boost::apply_visitor(visitor, std::get<1>(kv)) != SQLITE_OK) {
      throw InternalError("Bind parameter for insert() failed", db_.get());
    }
  }

  assert(sqlite3_bind_parameter_count(stmt) == visitor.getTotalBinds());
  if (sqlite3_step(stmt) != SQLITE_DONE) {
    throw InternalError("insert() failed", db_.get());
  }
}

warehouse_ros::ResultIteratorHelper::Ptr
warehouse_ros_sqlite::MessageCollectionHelper::query(
  warehouse_ros::Query::ConstPtr query, const std::string & sort_by,
  bool ascending) const
{
  std::string outro;
  if (!sort_by.empty()) {
    outro += " ORDER BY " + schema::escape_columnname_with_prefix(sort_by) +
      (ascending ? " ASC" : " DESC");
  }
  auto query_ptr = dynamic_cast<const warehouse_ros_sqlite::Query *>(query.get());
  assert(query_ptr);
  std::ostringstream intro;
  intro << "SELECT * FROM " << escaped_mangled_name_;
  if (!query_ptr->empty()) {
    intro << " WHERE ";
  }
  auto stmt = query_ptr->prepare(db_.get(), intro.str(), outro);
  if (stmt) {
    switch (sqlite3_step(stmt.get())) {
      case SQLITE_DONE:
      case SQLITE_ROW:
        break;
      default:
        throw InternalError("query() failed", db_.get());
    }
  }
  return boost::make_shared<warehouse_ros_sqlite::ResultIteratorHelper>(std::move(stmt));
}

unsigned warehouse_ros_sqlite::MessageCollectionHelper::removeMessages(
  warehouse_ros::Query::ConstPtr query)
{
  auto pquery = dynamic_cast<warehouse_ros_sqlite::Query const *>(query.get());
  if (!pquery) {
    throw std::invalid_argument("Query was not initialized by createQuery()");
  }
  auto stmt = pquery->prepare(db_.get(), "DELETE FROM " + escaped_mangled_name_ + " WHERE ");
  if (sqlite3_step(stmt.get()) != SQLITE_DONE) {
    throw InternalError("Prepare statement for removeMessages() failed", db_.get());
  }
  return sqlite3_changes(db_.get());
}

namespace
{
template<typename It>
void comma_concat_meta_column_names(std::ostringstream & buf, It it, It end)
{
  using warehouse_ros_sqlite::schema::escape_columnname_with_prefix;
  if (it == end) {
    return;
  }
  buf << escape_columnname_with_prefix(it->first);
  it++;
  while (it != end) {
    buf << " = ?, " << escape_columnname_with_prefix(it->first);
    it++;
  }
  buf << " = ?";
}
}  // namespace

void warehouse_ros_sqlite::MessageCollectionHelper::modifyMetadata(
  warehouse_ros::Query::ConstPtr q,
  warehouse_ros::Metadata::ConstPtr m)
{
  auto query = dynamic_cast<const warehouse_ros_sqlite::Query *>(q.get());
  auto metadata = dynamic_cast<const warehouse_ros_sqlite::Metadata *>(m.get());
  if (!query || !metadata) {
    throw std::invalid_argument("q or m not created by createQuery() or createMetadata()");
  }
  metadata->ensureColumns(db_.get(), mangled_tablename_);
  const int mt_count = metadata->data().size();
  if (mt_count == 0) {
    return;
  }

  std::ostringstream query_builder;
  query_builder << "UPDATE " << escaped_mangled_name_ << " SET ";

  comma_concat_meta_column_names(query_builder, metadata->data().begin(), metadata->data().end());
  query_builder << " WHERE ";
  auto stmt = query->prepare(db_.get(), query_builder.str(), "", mt_count + 1);
  if (!stmt) {
    throw InternalError("modifyMetadata() failed", db_.get());
  }
  warehouse_ros_sqlite::BindVisitor visitor(stmt.get(), 1);
  for (const auto & kv : metadata->data()) {
    if (boost::apply_visitor(visitor, std::get<1>(kv)) != SQLITE_OK) {
      throw InternalError("Bind parameter failed for modifyMetadata()", db_.get());
    }
  }

  if (sqlite3_step(stmt.get()) != SQLITE_DONE) {
    throw InternalError("modifyMetadata() failed", db_.get());
  }
}

unsigned warehouse_ros_sqlite::MessageCollectionHelper::count()
{
  const std::string query = "SELECT COUNT(*) FROM " + escaped_mangled_name_ + ";";
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db_.get(), query.c_str(), query.size() + 1, &stmt, nullptr) != SQLITE_OK) {
    throw InternalError("Prepare statement for count() failed", db_.get());
  }
  const warehouse_ros_sqlite::sqlite3_stmt_ptr stmt_guard(stmt);

  if (sqlite3_step(stmt) != SQLITE_ROW) {
    throw InternalError("count() failed", db_.get());
  }

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
