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
#include <array>
#include <climits>
#include <cstdlib>

extern "C" {
struct sqlite3_stmt;
struct sqlite3;
}

namespace warehouse_ros_sqlite
{
struct Sqlite3StmtDeleter
{
  void operator()(sqlite3_stmt* stmt) const;
};
void sqlite3_delete(sqlite3* db);

using sqlite3_stmt_ptr = std::unique_ptr<sqlite3_stmt, Sqlite3StmtDeleter>;
using sqlite3_ptr = std::shared_ptr<sqlite3>;

namespace schema
{
namespace detail
{
template <typename = void>
void check_do_escape(std::string& /*unused*/, char /*unused*/)
{
}
template <char escaped_char, char... other_chars>
void check_do_escape(std::string& s, char c)
{
  if (c == escaped_char)
    s.push_back(escaped_char);
  check_do_escape<other_chars...>(s, c);
}

template <char... escaped_chars>
std::string escape(const std::string& s)
{
  std::string ans;
  ans.reserve(4 * sizeof...(escaped_chars) + s.size());
  for (const auto c : s)
  {
    ans.push_back(c);
    check_do_escape<escaped_chars...>(ans, c);
  }
  return ans;
}
}  // namespace detail

constexpr const char* DB_NAME = "main";
constexpr const char* METADATA_COLUMN_PREFIX = "M_";
constexpr const char* DATA_COLUMN_NAME = "Data";
constexpr const char* TABLE_NAME_PREFIX = "T_";
constexpr const char* M_D5_TABLE_NAME = "MessageMD5s";
constexpr const char* M_D5_TABLE_INDEX_COLUMN = "TableName";
constexpr const char* M_D5_TABLE_M_D5_COLUMN = "MessageMD5";
constexpr const char* M_D5_TABLE_DATATYPE_COLUMN = "MessageDataType";
const int DATA_COLUMN_INDEX = 0;

using escaped_columnname = std::string;
using escaped_tablename = std::string;
inline escaped_columnname escape_columnname_with_prefix(const std::string& c)
{
  return "\"" + std::string(METADATA_COLUMN_PREFIX) + detail::escape<'"'>(c) + "\"";
}
inline escaped_tablename escape_tablename_with_prefix(const std::string& c)
{
  return "\"" + std::string(TABLE_NAME_PREFIX) + detail::escape<'"'>(c) + "\"";
}
inline std::string escape_string_literal_without_quotes(const std::string& c)
{
  return schema::detail::escape<'\''>(c);
}

}  // namespace schema

struct NullValue
{
};

inline std::array<unsigned char, 16> parse_md5_hexstring(const std::string& md5)
{
  if (md5.size() != 32)
  {
    throw std::invalid_argument("md5.size() must equal 32");
  }
  std::array<unsigned char, 16> binary_md5;
  size_t md5_idx = 0;
  for (auto& c : binary_md5)
  {
    char* end;
    const auto substr = md5.substr(md5_idx, 2);
    const auto t = std::strtoul(substr.c_str(), &end, 16);
    if (substr.c_str() + 2 != end)
      throw std::invalid_argument("md5 is not hex string");
    c = static_cast<unsigned char>(t);
    md5_idx += 2;
  }
  return binary_md5;
}
}  // namespace warehouse_ros_sqlite
