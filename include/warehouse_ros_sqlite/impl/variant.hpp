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

#ifndef WAREHOUSE_ROS_SQLITE__IMPL__VARIANT_HPP_
#define WAREHOUSE_ROS_SQLITE__IMPL__VARIANT_HPP_

#include <sqlite3.h>

#include <boost/variant.hpp>

#include <warehouse_ros_sqlite/utils.hpp>
#include <warehouse_ros_sqlite/exceptions.hpp>

#include <string>
#include <sstream>
#include <utility>

namespace warehouse_ros_sqlite
{
class BindVisitor : boost::static_visitor<int>
{
  sqlite3_stmt * stmt_;
  int idx_;

public:
  explicit BindVisitor(sqlite3_stmt * stmt, int start_idx = 1)
  : stmt_(stmt), idx_(start_idx)
  {
  }
  int operator()(int i)
  {
    return sqlite3_bind_int64(stmt_, idx_++, i);
  }
  int operator()(double d)
  {
    return sqlite3_bind_double(stmt_, idx_++, d);
  }
  int operator()(const std::string & s)
  {
    return sqlite3_bind_blob64(stmt_, idx_++, s.data(), s.size(), SQLITE_STATIC);
  }
  int operator()(NullValue /* unused */)
  {
    return sqlite3_bind_null(stmt_, idx_++);
  }
  int getTotalBinds() const
  {
    return idx_ - 1;
  }
};

class EnsureColumnVisitor : boost::static_visitor<>
{
  sqlite3 * db_;
  std::string unescaped_tablename_;
  schema::escaped_tablename escaped_tablename_;
  std::string unescaped_colname_;
  bool columnExists()
  {
    const std::string colname(schema::METADATA_COLUMN_PREFIX + unescaped_colname_);
    return sqlite3_table_column_metadata(
      db_, schema::DB_NAME, unescaped_tablename_.c_str(), colname.c_str(), nullptr,
      nullptr, nullptr, nullptr, nullptr) == SQLITE_OK;
  }
  void addColumn(const char * datatype)
  {
    std::ostringstream query_builder;
    query_builder << "ALTER TABLE " << escaped_tablename_ << " ADD " <<
      schema::escape_columnname_with_prefix(unescaped_colname_) << " " << datatype << ";";
    if (sqlite3_exec(db_, query_builder.str().c_str(), nullptr, nullptr, nullptr) != SQLITE_OK) {
      throw InternalError("could not create column", db_);
    }
  }

public:
  EnsureColumnVisitor(sqlite3 * db, const std::string & unescaped_tablename)
  : db_(db),
    unescaped_tablename_(unescaped_tablename),
    escaped_tablename_(schema::escape_identifier(unescaped_tablename))
  {
  }
  void operator()(int /*unused*/)  // NOLINT
  {
    if (!columnExists()) {
      addColumn("INTEGER");
    }
  }
  void operator()(double /*unused*/)  // NOLINT
  {
    if (!columnExists()) {
      addColumn("FLOAT");
    }
  }
  void operator()(const std::string & /*unused*/)
  {
    if (!columnExists()) {
      addColumn("BLOB");
    }
  }
  void operator()(NullValue /* unused */)
  {
    if (!columnExists()) {
      throw std::runtime_error("not implemented");
    }
  }
  EnsureColumnVisitor & setColumnName(const std::string & unescaped_column)
  {
    unescaped_colname_ = unescaped_column;
    return *this;
  }
};

namespace detail
{
template<typename R, typename T>
struct NullValueGet
{
  static R get(T /* unused */)
  {
    throw boost::bad_get();
  }
};

template<typename R>
struct NullValueGet<R, typename std::enable_if<!std::is_same<R, NullValue>::value, R>::type>
{
  static R get(R r)
  {
    return std::forward<R>(r);
  }
};

template<typename R>
struct NullValueGet<R, NullValue>
{
  static R get(NullValue /* unused */)
  {
    return R();
  }
};

}  // namespace detail

template<typename R>
struct NullValueVisitor : boost::static_visitor<R>
{
  template<typename T>
  R operator()(T t) const
  {
    return detail::NullValueGet<R, T>::get(std::forward<T>(t));
  }
};
}  // namespace warehouse_ros_sqlite

#endif  // WAREHOUSE_ROS_SQLITE__IMPL__VARIANT_HPP_
