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

extern "C" {
struct sqlite3_stmt;
}

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
  int operator()(int i);
  int operator()(double d);
  int operator()(const std::string& s);
  int operator()(std::nullptr_t);
  int get_total_binds() const { return idx_ - 1; }
};

namespace detail
{
template <typename R, typename T>
R NullValueGet(T)
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
