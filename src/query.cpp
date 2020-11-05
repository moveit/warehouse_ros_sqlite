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
#include <warehouse_ros_sqlite/impl/variant.h>
#include <cassert>
#include <ros/console.h>

warehouse_ros_sqlite::sqlite3_stmt_ptr warehouse_ros_sqlite::Query::prepare(sqlite3* db_conn, const std::string& intro,
                                                                            const std::string& outro,
                                                                            int bind_start_col) const
{
  sqlite3_stmt* stmt = nullptr;
  const auto query = intro + query_.str() + outro + ";";
  warehouse_ros_sqlite::sqlite3_stmt_ptr ans;
  ROS_DEBUG_NAMED("warehouse_ros_sqlite", "query query: %s", query.c_str());
  if (sqlite3_prepare_v2(db_conn, query.c_str(), query.size() + 1 /* null terminator*/, &stmt, nullptr) != SQLITE_OK)
  {
    // TODO: check if all column exists and return nullptr if not
    // not throwing an exception, missing column means empty result
    ROS_ERROR_NAMED("warehouse_ros_sqlite", "Preparing Query failed: %s", sqlite3_errmsg(db_conn));
    return ans;
  }
  ans.reset(stmt);
  assert(static_cast<size_t>(sqlite3_bind_parameter_count(stmt)) == (values_.size() + bind_start_col - 1));

  warehouse_ros_sqlite::BindVisitor visitor(stmt, bind_start_col);
  for (const auto& value : values_)
  {
    if (boost::apply_visitor(visitor, value) != SQLITE_OK)
    {
      throw InternalError("Binding parameter to query failed", db_conn);
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
