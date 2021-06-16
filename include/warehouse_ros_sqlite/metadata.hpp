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

#ifndef WAREHOUSE_ROS_SQLITE__METADATA_HPP_
#define WAREHOUSE_ROS_SQLITE__METADATA_HPP_

#include <warehouse_ros_sqlite/utils.hpp>
#include <warehouse_ros_sqlite/warehouse_ros_sqlite_export.hpp>

#include <warehouse_ros/metadata.h>
#include <boost/variant.hpp>

#include <map>
#include <set>
#include <string>

extern "C" {
struct sqlite3_stmt;
struct sqlite3;
}

namespace warehouse_ros_sqlite
{
class WAREHOUSE_ROS_SQLITE_EXPORT Metadata : public warehouse_ros::Metadata
{
public:
  using Variant = boost::variant<NullValue, std::string, double, int>;
  void append(const std::string & name, const std::string & val) override;
  void append(const std::string & name, const double val) override;
  void append(const std::string & name, const int val) override;
  void append(const std::string & name, const bool val) override;
  std::string lookupString(const std::string & name) const override;
  double lookupDouble(const std::string & name) const override;
  int lookupInt(const std::string & name) const override;
  bool lookupBool(const std::string & name) const override;
  bool lookupField(const std::string & name) const override;
  std::set<std::string> lookupFieldNames() const override;
  void append(const std::string & name, sqlite3_stmt * stmt, int col);
  const auto & data() const
  {
    return data_;
  }
  void ensureColumns(sqlite3 * db, const std::string & unescaped_table_name) const;

private:
  // ordered map for reproducible iterating
  std::map<std::string, Variant> data_;

  template<typename R>
  R doLookup(const std::string & name) const;
};

}  // namespace warehouse_ros_sqlite

#endif  // WAREHOUSE_ROS_SQLITE__METADATA_HPP_
