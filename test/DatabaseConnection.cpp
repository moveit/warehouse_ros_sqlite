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

#include <gtest/gtest.h>
#include <warehouse_ros_sqlite/database_connection.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

class ConnectionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    conn_.reset(new warehouse_ros_sqlite::DatabaseConnection());
    conn_->setParams(":memory:", 0);
    ASSERT_TRUE(conn_->connect());
  }

  static std::unique_ptr<warehouse_ros_sqlite::DatabaseConnection> conn_;
};
std::unique_ptr<warehouse_ros_sqlite::DatabaseConnection> ConnectionTest::conn_;

TEST_F(ConnectionTest, CreateCollection)
{
  using V = geometry_msgs::Vector3;
  auto coll = conn_->openCollection<V>("main", "coll");

  EXPECT_STREQ(conn_->messageType("main", "coll").c_str(), ros::message_traits::DataType<V>::value());

  EXPECT_EQ(coll.count(), 0U);
}

TEST_F(ConnectionTest, InsertQueryAndDeleteMessage)
{
  using V = geometry_msgs::Vector3;
  auto coll = conn_->openCollection<V>("main", "coll");
  auto meta1 = coll.createMetadata();
  meta1->append("x", 3);

  V v1, v2;
  v1.x = 3.0;
  v1.y = 1.0;

  coll.insert(v1, meta1);

  EXPECT_EQ(coll.count(), 1U);

  v2.x = 5.0;
  v2.y = 7.0;

  auto meta2 = coll.createMetadata();
  meta2->append("x", 5);

  coll.insert(v2, meta2);

  EXPECT_EQ(coll.count(), 2U);

  auto query = coll.createQuery();
  query->append("x", 3);

  const auto list = coll.queryList(query);

  ASSERT_EQ(list.size(), size_t(1));
  ASSERT_EQ(list[0]->y, 1.0);

  EXPECT_EQ(coll.removeMessages(query), 1U);
  EXPECT_EQ(coll.count(), 1U);
}

TEST_F(ConnectionTest, MD5Validation)
{
  auto coll1 = conn_->openCollection<geometry_msgs::Vector3>("main", "coll");
  ASSERT_TRUE(coll1.md5SumMatches());
  auto coll2 = conn_->openCollection<geometry_msgs::Pose>("main", "coll");
  ASSERT_FALSE(coll2.md5SumMatches());
  auto coll3 = conn_->openCollection<geometry_msgs::Vector3>("main", "coll");
  ASSERT_TRUE(coll3.md5SumMatches());
}

int main(int argc, char** argv)
{
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
