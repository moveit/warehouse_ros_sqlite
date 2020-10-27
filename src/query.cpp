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
#include <warehouse_ros_sqlite/query.h>

#include <sqlite3.h>
#include <iomanip>
#include <boost/variant/static_visitor.hpp>
#include <warehouse_ros/exceptions.h>
#include <warehouse_ros_sqlite/variant.h>

int warehouse_ros_sqlite::BindVisitor::operator()(int i)
{
  return sqlite3_bind_int64(stmt_, idx_++, i);
}
int warehouse_ros_sqlite::BindVisitor::operator()(double d)
{
  return sqlite3_bind_double(stmt_, idx_++, d);
}
int warehouse_ros_sqlite::BindVisitor::operator()(const std::string& s)
{
  return sqlite3_bind_blob64(stmt_, idx_++, s.data(), s.size(), SQLITE_STATIC);
}
int warehouse_ros_sqlite::BindVisitor::operator()(std::nullptr_t)
{
  return sqlite3_bind_null(stmt_, idx_++);
}

warehouse_ros_sqlite::Query::Query()
{
}

warehouse_ros_sqlite::sqlite3_stmt_ptr warehouse_ros_sqlite::Query::prepare(sqlite3* db_conn, const std::string& intro, const std::string& outro ) const
{
  sqlite3_stmt* stmt = nullptr;
  const auto query = intro + query_.str() + outro + ";";
  if (sqlite3_prepare_v2(db_conn, query.c_str(), query.size() + 1 /* null terminator*/, &stmt, nullptr) != SQLITE_OK)
  {
    throw warehouse_ros::WarehouseRosException("Prepare query failed");
  }
  warehouse_ros_sqlite::sqlite3_stmt_ptr ans(stmt);

  if (sqlite3_bind_parameter_count(stmt) != values_.size())
  {
    throw warehouse_ros::WarehouseRosException("Prepare query failed, parameter count mismatch");
  }

  warehouse_ros_sqlite::BindVisitor visitor(stmt);
  for (const auto& value : values_)
  {
    if (boost::apply_visitor(visitor, value) != SQLITE_OK)
    {
      throw warehouse_ros::WarehouseRosException("Binding parameter to query failed");
    }
  }

  return ans;
}

void warehouse_ros_sqlite::Query::append(const std::string& name, const std::string& val)
{
  doappend(name, " == ", val);
}
void warehouse_ros_sqlite::Query::append(const std::string& name, const double val)
{
  doappend(name, " == ", val);
}
void warehouse_ros_sqlite::Query::append(const std::string& name, const int val)
{
  doappend(name, " == ", val);
}
void warehouse_ros_sqlite::Query::append(const std::string& name, const bool val)
{
  doappend(name, " == ", static_cast<int>(val));
}
void warehouse_ros_sqlite::Query::appendLT(const std::string& name, const double val)
{
  doappend(name, " < ", val);
}
void warehouse_ros_sqlite::Query::appendLT(const std::string& name, const int val)
{
  doappend(name, " < ", val);
}
void warehouse_ros_sqlite::Query::appendLTE(const std::string& name, const double val)
{
  doappend(name, " <= ", val);
}
void warehouse_ros_sqlite::Query::appendLTE(const std::string& name, const int val)
{
  doappend(name, " <= ", val);
}
void warehouse_ros_sqlite::Query::appendGT(const std::string& name, const double val)
{
  doappend(name, " > ", val);
}
void warehouse_ros_sqlite::Query::appendGT(const std::string& name, const int val)
{
  doappend(name, " > ", val);
}
void warehouse_ros_sqlite::Query::appendGTE(const std::string& name, const double val)
{
  doappend(name, " >= ", val);
}
void warehouse_ros_sqlite::Query::appendGTE(const std::string& name, const int val)
{
  doappend(name, " >= ", val);
}
void warehouse_ros_sqlite::Query::appendRange(const std::string& name, const double lower, const double upper)
{
  doappend(name, " > ", lower);
  doappend(name, " < ", upper);
}
void warehouse_ros_sqlite::Query::appendRange(const std::string& name, const int lower, const int upper)
{
  doappend(name, " > ", lower);
  doappend(name, " < ", upper);
}
void warehouse_ros_sqlite::Query::appendRangeInclusive(const std::string& name, const double lower, const double upper)
{
  doappend(name, " >= ", lower);
  doappend(name, " <= ", upper);
}
void warehouse_ros_sqlite::Query::appendRangeInclusive(const std::string& name, const int lower, const int upper)
{
  doappend(name, " >= ", lower);
  doappend(name, " <= ", upper);
}
