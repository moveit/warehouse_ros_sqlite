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

#include <warehouse_ros_sqlite/result_iteration_helper.hpp>

#include <warehouse_ros_sqlite/metadata.hpp>
#include <warehouse_ros_sqlite/exceptions.hpp>

#include <boost/make_shared.hpp>

#include <sqlite3.h>
#include <cassert>
#include <cstring>
#include <string>

bool warehouse_ros_sqlite::ResultIteratorHelper::next()
{
  if (!stmt_) {
    return false;
  }
  switch (sqlite3_step(stmt_.get())) {
    case SQLITE_ROW:

      return true;
    case SQLITE_DONE:
      stmt_.reset();
      return false;
    default:
      throw InternalError("next() failed", stmt_.get());
  }
}
bool warehouse_ros_sqlite::ResultIteratorHelper::hasData() const
{
  if (!stmt_) {
    return false;
  }
  switch (sqlite3_column_type(stmt_.get(), schema::DATA_COLUMN_INDEX)) {
    case SQLITE_BLOB:
      return sqlite3_column_bytes(stmt_.get(), schema::DATA_COLUMN_INDEX) != 0;
    case SQLITE_NULL:
      return false;
    default:
      throw DatatypeMismatch("Data Column has wrong data type");
  }
}

warehouse_ros::Metadata::ConstPtr warehouse_ros_sqlite::ResultIteratorHelper::metadata() const
{
  assert(static_cast<bool>(stmt_));
  auto ans = boost::make_shared<warehouse_ros_sqlite::Metadata>();
  for (const auto & col_pair : metadata_cols_) {
    ans->append(std::get<0>(col_pair), stmt_.get(), std::get<1>(col_pair));
  }
  return ans;
}

std::string warehouse_ros_sqlite::ResultIteratorHelper::message() const
{
  if (!hasData()) {
    return std::string();
  }
  return std::string(
    reinterpret_cast<const char *>(sqlite3_column_blob(stmt_.get(), schema::DATA_COLUMN_INDEX)),
    sqlite3_column_bytes(stmt_.get(), schema::DATA_COLUMN_INDEX));
}

namespace
{
int constexpr strlength(const char * str)
{
  return *str ? 1 + strlength(str + 1) : 0;
}
}  // namespace

void warehouse_ros_sqlite::ResultIteratorHelper::initMetadataCols()
{
  if (!stmt_) {
    return;
  }
  constexpr int max_length = strlength(schema::METADATA_COLUMN_PREFIX);
  for (int i = 0; i < sqlite3_column_count(stmt_.get()); ++i) {
    const char * col_name = sqlite3_column_name(stmt_.get(), i);
    if (std::strncmp(schema::METADATA_COLUMN_PREFIX, col_name, max_length) == 0) {
      // strip off prefix
      metadata_cols_.emplace_back(col_name + max_length, i);
    }
  }
}
