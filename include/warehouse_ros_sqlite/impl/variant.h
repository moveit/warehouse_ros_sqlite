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

#pragma once

#include <boost/variant.hpp>
#include <string>
#include <sstream>
#include <sqlite3.h>

namespace warehouse_ros_sqlite
{
class BindVisitor : boost::static_visitor<int>
{
  sqlite3_stmt* stmt_;
  int idx_;

public:
  BindVisitor(sqlite3_stmt* stmt, int start_idx = 1) : stmt_(stmt), idx_(start_idx)
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
  int operator()(const std::string& s)
  {
    return sqlite3_bind_blob64(stmt_, idx_++, s.data(), s.size(), SQLITE_STATIC);
  }
  int operator()(std::nullptr_t)
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
  sqlite3* db_;
  const char* tablename_;
  std::string colname_;
  bool columnExists();
  void addColumn(const char* datatype)
  {
    std::ostringstream query_builder;
    query_builder << "ALTER TABLE " << tablename_ << " ADD " << colname_ << " " << datatype << ";";
    if (sqlite3_exec(db_, query_builder.str().c_str(), nullptr, nullptr, nullptr) != SQLITE_OK)
    {
      throw std::runtime_error("could not create column");
    }
  }

public:
  EnsureColumnVisitor(sqlite3* db, const char* tablename) : db_(db), tablename_(tablename)
  {
  }
  void operator()(int /*unused*/)
  {
    if (!columnExists())
      addColumn("INTEGER");
  }
  void operator()(double /*unused*/)
  {
    if (!columnExists())
      addColumn("FLOAT");
  }
  void operator()(const std::string& /*unused*/)
  {
    if (!columnExists())
      addColumn("BLOB");
  }
  void operator()(std::nullptr_t)
  {
    if (!columnExists())
      throw std::runtime_error("not implemented");
  }
  EnsureColumnVisitor& setColumnName(std::string&& c)
  {
    colname_ = std::move(c);
    return *this;
  }
};

namespace detail
{
template <typename R, typename T>
R NullValueGet(T /*unused*/)
{
  throw boost::bad_get();
}
template <typename R>
typename std::enable_if<!std::is_same<R, std::nullptr_t>::value, R>::type NullValueGet(R r)
{
  return std::forward<R>(r);
}
template <typename R>
R NullValueGet(std::nullptr_t)
{
  return R();
}
}  // namespace detail

template <typename R>
struct NullValueVisitor : boost::static_visitor<R>
{
  template <typename T>
  R operator()(T t) const
  {
    return detail::NullValueGet<R, T>(std::forward<T>(t));
  }
};
}  // namespace warehouse_ros_sqlite
