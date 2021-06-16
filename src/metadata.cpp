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

#include <warehouse_ros_sqlite/metadata.hpp>

#include <warehouse_ros_sqlite/impl/variant.hpp>
#include <warehouse_ros_sqlite/utils.hpp>

#include <sqlite3.h>

#include <set>
#include <string>

template<typename R>
R warehouse_ros_sqlite::Metadata::doLookup(const std::string & name) const
{
  const auto res = data_.find(name);
  if (res == data_.end()) {
    throw std::range_error("");
  }
  return boost::apply_visitor(warehouse_ros_sqlite::NullValueVisitor<R>(), res->second);
}

void warehouse_ros_sqlite::Metadata::append(const std::string & name, const std::string & val)
{
  data_[name] = val;
}
void warehouse_ros_sqlite::Metadata::append(const std::string & name, const double val)
{
  data_[name] = val;
}
void warehouse_ros_sqlite::Metadata::append(const std::string & name, const int val)
{
  data_[name] = val;
}
void warehouse_ros_sqlite::Metadata::append(const std::string & name, const bool val)
{
  data_[name] = static_cast<int>(val);
}
std::string warehouse_ros_sqlite::Metadata::lookupString(const std::string & name) const
{
  return doLookup<std::string>(name);
}
double warehouse_ros_sqlite::Metadata::lookupDouble(const std::string & name) const
{
  return doLookup<double>(name);
}
int warehouse_ros_sqlite::Metadata::lookupInt(const std::string & name) const
{
  return doLookup<int>(name);
}
bool warehouse_ros_sqlite::Metadata::lookupBool(const std::string & name) const
{
  return static_cast<bool>(doLookup<int>(name));
}
bool warehouse_ros_sqlite::Metadata::lookupField(const std::string & name) const
{
  return data_.find(name) != data_.end();
}
std::set<std::string> warehouse_ros_sqlite::Metadata::lookupFieldNames() const
{
  std::set<std::string> ans;
  std::transform(
    data_.begin(), data_.end(), std::inserter(ans, ans.end()), [](const auto & it) {
      return it.first;
    });
  return ans;
}

void warehouse_ros_sqlite::Metadata::append(const std::string & name, sqlite3_stmt * stmt, int col)
{
  switch (sqlite3_column_type(stmt, col)) {
    case SQLITE_NULL:
      data_[name] = NullValue();
      break;
    case SQLITE_BLOB:
      data_[name] =
        std::string(
        reinterpret_cast<const char *>(sqlite3_column_blob(
          stmt,
          col)),
        sqlite3_column_bytes(stmt, col));
      break;
    case SQLITE_INTEGER:
      data_[name] = sqlite3_column_int(stmt, col);
      break;
    case SQLITE_FLOAT:
      data_[name] = sqlite3_column_double(stmt, col);
      break;
    default:
      throw DatatypeMismatch("Unknown Datatype when reading Metadata from DB");
  }
}

void warehouse_ros_sqlite::Metadata::ensureColumns(
  sqlite3 * db,
  const std::string & unescaped_table_name) const
{
  warehouse_ros_sqlite::EnsureColumnVisitor visitor(db, unescaped_table_name);
  for (const auto & kv : data_) {
    boost::apply_visitor(visitor.setColumnName(std::get<0>(kv)), std::get<1>(kv));
  }
}
