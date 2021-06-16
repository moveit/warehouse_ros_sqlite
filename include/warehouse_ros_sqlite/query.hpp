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

#ifndef WAREHOUSE_ROS_SQLITE__QUERY_HPP_
#define WAREHOUSE_ROS_SQLITE__QUERY_HPP_

#include <warehouse_ros/metadata.h>

#include <boost/variant.hpp>

#include <warehouse_ros_sqlite/utils.hpp>
#include <warehouse_ros_sqlite/warehouse_ros_sqlite_export.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace warehouse_ros_sqlite
{
class WAREHOUSE_ROS_SQLITE_EXPORT Query : public warehouse_ros::Query
{
public:
  using Variant = boost::variant<std::string, double, int>;
  void append(const std::string & name, const std::string & val) override;
  void append(const std::string & name, const double val) override;
  void append(const std::string & name, const int val) override;
  void append(const std::string & name, const bool val) override;
  void appendLT(const std::string & name, const double val) override;
  void appendLT(const std::string & name, const int val) override;
  void appendLTE(const std::string & name, const double val) override;
  void appendLTE(const std::string & name, const int val) override;
  void appendGT(const std::string & name, const double val) override;
  void appendGT(const std::string & name, const int val) override;
  void appendGTE(const std::string & name, const double val) override;
  void appendGTE(const std::string & name, const int val) override;
  void appendRange(const std::string & name, const double lower, const double upper) override;
  void appendRange(const std::string & name, const int lower, const int upper) override;
  void appendRangeInclusive(
    const std::string & name, const double lower,
    const double upper) override;
  void appendRangeInclusive(const std::string & name, const int lower, const int upper) override;

  sqlite3_stmt_ptr prepare(
    sqlite3 * db_conn, const std::string & intro, const std::string & outro = "",
    int bind_start_col = 1) const;
  bool empty() const
  {
    return values_.empty();
  }

private:
  template<typename T>
  void doappend(const std::string & name, const char * op, T val)
  {
    if (!values_.empty()) {
      query_ << " AND ";
    }
    values_.emplace_back(val);
    query_ << schema::escape_columnname_with_prefix(name) << op << '?';
  }
  std::vector<Variant> values_;
  std::stringstream query_;
};

}  // namespace warehouse_ros_sqlite


#endif  // WAREHOUSE_ROS_SQLITE__QUERY_HPP_
