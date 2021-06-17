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

#include <gtest/gtest.h>
#include <warehouse_ros/database_loader.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

TEST(DatabaseLoader, LoadSQLite)
{
  const auto node = std::make_shared<rclcpp::Node>("tester");
  node->declare_parameter<std::string>(
    "warehouse_plugin",
    "warehouse_ros_sqlite::DatabaseConnection");
  warehouse_ros::DatabaseLoader l(node);

  const auto d = l.loadDatabase();

  ASSERT_TRUE(static_cast<bool>(d));
  d->setParams(":memory:", 0);

  ASSERT_TRUE(d->connect());

  using V = geometry_msgs::msg::Vector3;
  auto coll = d->openCollection<V>("main", "coll");
  auto meta1 = coll.createMetadata();
  meta1->append("x", 3);

  coll.insert(V(), meta1);

  EXPECT_EQ(coll.count(), 1U);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
