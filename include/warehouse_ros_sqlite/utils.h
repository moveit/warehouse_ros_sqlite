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

#include <memory>

extern "C" {
struct sqlite3_stmt;
struct sqlite3;
}

namespace warehouse_ros_sqlite
{
struct sqlite3_stmt_deleter
{
  void operator()(sqlite3_stmt* stmt) const;
};
void sqlite3_delete(sqlite3* db);

using sqlite3_stmt_ptr = std::unique_ptr<sqlite3_stmt, sqlite3_stmt_deleter>;
using sqlite3_ptr = std::shared_ptr<sqlite3>;

namespace schema
{
constexpr const char* MetadataColumnPrefix = "M_";
constexpr const char* DataColumnName = "Data";
constexpr const char* TableNamePrefix = "T_";
constexpr const char* MD5TableName = "MessageMD5s";
constexpr const char* MD5TableIndexColumn = "TableName";
constexpr const char* MD5TableMD5Column = "MessageMD5";
const int DataColumnIndex = 0;
}  // namespace schema

}  // namespace warehouse_ros_sqlite
