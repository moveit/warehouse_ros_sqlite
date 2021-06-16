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

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <warehouse_ros_sqlite/database_connection.hpp>
#include <warehouse_ros_sqlite/utils.hpp>

#include <memory>

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
  using V = geometry_msgs::msg::Vector3;
  auto coll = conn_->openCollection<V>("main", "coll");

  EXPECT_STREQ(conn_->messageType("main", "coll").c_str(), rosidl_generator_traits::data_type<V>());

  EXPECT_EQ(coll.count(), 0U);
}

TEST_F(ConnectionTest, InsertQueryAndDeleteMessage)
{
  using V = geometry_msgs::msg::Vector3;
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
  auto coll1 = conn_->openCollection<geometry_msgs::msg::Vector3>("main", "coll");
  ASSERT_TRUE(coll1.md5SumMatches());
  auto coll2 = conn_->openCollection<geometry_msgs::msg::Pose>("main", "coll");
  ASSERT_FALSE(coll2.md5SumMatches());
  auto coll3 = conn_->openCollection<geometry_msgs::msg::Vector3>("main", "coll");
  ASSERT_TRUE(coll3.md5SumMatches());
}

TEST_F(ConnectionTest, MetadataNullValue)
{
  using V = geometry_msgs::msg::Vector3;
  auto coll = conn_->openCollection<V>("main", "coll");
  auto meta1 = coll.createMetadata();
  meta1->append("x", 3);

  V v1, v2;
  v1.x = 3.0;
  v1.y = 1.0;
  v2.x = 5.0;
  v2.y = 7.0;

  coll.insert(v1, meta1);
  meta1->append("y", 8);
  coll.insert(v2, meta1);

  auto query = coll.createQuery();
  query->append("x", 3);

  const auto list = coll.queryList(query);
  ASSERT_EQ(list.size(), size_t(2));
  EXPECT_EQ(list[0]->lookupInt("x"), 3);
  EXPECT_EQ(list[1]->lookupInt("x"), 3);

  EXPECT_TRUE(
    ((list[0]->lookupInt("y") == 0 && list[1]->lookupInt("y") == 8) ||
    (list[1]->lookupInt("y") == 0 && list[0]->lookupInt("y") == 8)));
}

TEST_F(ConnectionTest, UniqueID)
{
  using V = geometry_msgs::msg::Vector3;
  auto coll = conn_->openCollection<V>("main", "coll");
  auto meta1 = coll.createMetadata();
  meta1->append("x", 3);

  coll.insert(V(), meta1);
  coll.insert(V(), meta1);

  auto query = coll.createQuery();
  query->append("x", 3);

  const auto list = coll.queryList(query);
  ASSERT_EQ(list.size(), size_t(2));
  ASSERT_NE(list[0]->lookupInt("id"), list[1]->lookupInt("id"));
}

TEST_F(ConnectionTest, OverwriteMetadata)
{
  using V = geometry_msgs::msg::Vector3;
  auto coll = conn_->openCollection<V>("main", "coll");
  auto meta1 = coll.createMetadata();
  meta1->append("x", 3);

  const int y1 = 8;
  meta1->append("y", y1);

  V v1, v2;
  v1.x = 3.0;
  v1.y = 9.0;
  v2.x = 5.0;
  v2.y = 7.0;

  coll.insert(v1, meta1);
  meta1->append("y", 6);
  coll.insert(v2, meta1);

  auto query = coll.createQuery();
  query->append("y", y1);

  const auto list = coll.queryList(query);
  ASSERT_EQ(list.size(), size_t(1));
  EXPECT_EQ(list[0]->lookupInt("x"), 3);
  EXPECT_EQ(list[0]->lookupInt("y"), y1);
  EXPECT_EQ(list[0]->y, 9.0);
}

TEST_F(ConnectionTest, ModifyMetadata)
{
  using V = geometry_msgs::msg::Vector3;
  auto coll = conn_->openCollection<V>("main", "coll");
  auto meta1 = coll.createMetadata();

  V v1, v2;
  v1.x = 3.0;
  v2.x = 5.0;
  const int x1 = 8;

  meta1->append("x", x1);
  meta1->append("y", "one");
  coll.insert(v1, meta1);

  auto meta2 = coll.createMetadata();
  meta2->append("x", 2);
  meta2->append("y", "one");
  coll.insert(v2, meta2);

  auto modify_query = coll.createQuery();
  modify_query->append("x", x1);
  auto meta_modify = coll.createMetadata();
  meta_modify->append("y", "two");
  coll.modifyMetadata(modify_query, meta_modify);

  auto query = coll.createQuery();
  query->append("y", "two");
  const auto list = coll.queryList(query);

  ASSERT_EQ(list.size(), size_t(1));
  EXPECT_EQ(list[0]->lookupInt("x"), x1);
  EXPECT_EQ(list[0]->lookupString("y"), "two");
  EXPECT_EQ(list[0]->x, 3.0);
}

TEST_F(ConnectionTest, Metadata)
{
  using V = geometry_msgs::msg::Vector3;
  auto coll = conn_->openCollection<V>("main", "coll");
  auto meta1 = coll.createMetadata();

  auto keys = meta1->lookupFieldNames();
  EXPECT_TRUE(keys.empty());
  meta1->append("x", false);

  keys = meta1->lookupFieldNames();
  EXPECT_NE(keys.find("x"), keys.end());
  EXPECT_EQ(keys.size(), 1U);
  EXPECT_TRUE(meta1->lookupField("x"));
  EXPECT_FALSE(meta1->lookupField("z"));
}

TEST_F(ConnectionTest, ComplexQuery)
{
  using V = geometry_msgs::msg::Vector3;
  auto coll = conn_->openCollection<V>("main", "coll");
  auto meta1 = coll.createMetadata();
  meta1->append("x", 3);

  const int y1 = 8;
  meta1->append("y", y1);

  V v1, v2;
  v1.x = 3.0;
  v1.y = 9.0;
  v2.x = 5.0;
  v2.y = 7.0;

  coll.insert(v1, meta1);
  meta1->append("y", 6);
  coll.insert(v2, meta1);

  auto query = coll.createQuery();
  query->append("x", 3);
  query->append("y", y1);

  const auto list = coll.queryList(query);
  ASSERT_EQ(list.size(), size_t(1));
  EXPECT_EQ(list[0]->lookupInt("x"), 3);
  EXPECT_EQ(list[0]->lookupInt("y"), y1);
  EXPECT_EQ(list[0]->y, 9.0);
}

TEST_F(ConnectionTest, EmptyQuery)
{
  using V = geometry_msgs::msg::Vector3;
  auto coll = conn_->openCollection<V>("main", "coll");
  auto meta1 = coll.createMetadata();
  meta1->append("x", 7);
  coll.insert(V(), meta1);

  // known column, but wrong value
  auto query = coll.createQuery();
  query->append("x", 3);

  auto it_pair = coll.query(query);
  EXPECT_EQ(std::get<0>(it_pair), std::get<1>(it_pair));

  auto list = coll.queryList(query);
  EXPECT_EQ(list.size(), size_t(0));

  // unknown column
  query->append("y", 3);

  it_pair = coll.query(query);
  EXPECT_EQ(std::get<0>(it_pair), std::get<1>(it_pair));

  list = coll.queryList(query);
  EXPECT_EQ(list.size(), size_t(0));
}

TEST_F(ConnectionTest, DifferentDatabases)
{
  using V = geometry_msgs::msg::Vector3;
  auto coll1 = conn_->openCollection<V>("main1", "coll");
  auto meta1 = coll1.createMetadata();
  meta1->append("x", 7);
  coll1.insert(V(), meta1);

  auto coll2 = conn_->openCollection<V>("main2", "coll");
  {
    auto query2 = coll2.createQuery();
    query2->append("x", 7);
    const auto list2 = coll2.queryList(query2);
    EXPECT_EQ(list2.size(), size_t(0));
  }
  {
    auto meta2 = coll2.createMetadata();
    meta2->append("x", 7);
    coll2.insert(V(), meta2);
  }
  {
    auto query1 = coll1.createQuery();
    query1->append("x", 7);
    const auto list1 = coll1.queryList(query1);
    EXPECT_EQ(list1.size(), size_t(1));
  }
}

TEST_F(ConnectionTest, DropDatabase)
{
  using V = geometry_msgs::msg::Vector3;
  // create two independent databases
  {
    auto coll = conn_->openCollection<V>("main", "coll");
    auto meta1 = coll.createMetadata();
    meta1->append("x", 7);
    coll.insert(V(), meta1);
  }
  {
    auto coll = conn_->openCollection<V>("main2", "coll");
    auto meta1 = coll.createMetadata();
    meta1->append("x", 7);
    coll.insert(V(), meta1);
  }
  // delete first one
  conn_->dropDatabase("main");
  // second one still there?
  {
    auto coll1 = conn_->openCollection<V>("main2", "coll");
    auto query1 = coll1.createQuery();
    query1->append("x", 7);
    const auto list1 = coll1.queryList(query1);
    EXPECT_EQ(list1.size(), size_t(1));
  }
  // first one gone?
  {
    auto coll1 = conn_->openCollection<V>("main", "coll");
    auto query1 = coll1.createQuery();
    query1->append("x", 7);
    const auto list1 = coll1.queryList(query1);
    EXPECT_EQ(list1.size(), size_t(0));
  }
}

TEST_F(ConnectionTest, FindOne)
{
  using V = geometry_msgs::msg::Vector3;
  auto coll = conn_->openCollection<V>("main", "coll");
  auto meta1 = coll.createMetadata();
  meta1->append("x", 7);
  coll.insert(V(), meta1);

  auto query = coll.createQuery();
  query->append("x", 9);
  EXPECT_THROW(coll.findOne(query), warehouse_ros::NoMatchingMessageException);
}

TEST_F(ConnectionTest, Sorting)
{
  using V = geometry_msgs::msg::Vector3;
  auto coll = conn_->openCollection<V>("main", "coll");
  auto meta1 = coll.createMetadata();
  meta1->append("x", 71);
  coll.insert(V(), meta1);
  meta1->append("x", 10);
  coll.insert(V(), meta1);

  {
    // ascending
    auto query = coll.createQuery();
    const auto list1 = coll.queryList(query, false, "x", true);
    ASSERT_EQ(list1.size(), size_t(2));
    EXPECT_EQ(list1[0]->lookupInt("x"), 10);
    EXPECT_EQ(list1[1]->lookupInt("x"), 71);
  }
  {
    // descending
    auto query = coll.createQuery();
    const auto list1 = coll.queryList(query, false, "x", false);
    ASSERT_EQ(list1.size(), size_t(2));
    EXPECT_EQ(list1[0]->lookupInt("x"), 71);
    EXPECT_EQ(list1[1]->lookupInt("x"), 10);
  }
}

TEST(Utils, Md5Validation)
{
  const char * a = "4a842b65f413084dc2b10fb484ea7f17";
  const std::array<unsigned char, 16> b{
    0x4a, 0x84, 0x2b, 0x65, 0xf4, 0x13, 0x08, 0x4d, 0xc2, 0xb1, 0x0f, 0xb4, 0x84, 0xea, 0x7f, 0x17,
  };

  EXPECT_EQ(warehouse_ros_sqlite::parse_md5_hexstring(a), b);

  EXPECT_THROW(warehouse_ros_sqlite::parse_md5_hexstring("123abc"), std::invalid_argument);
  const char * c = "Za842b65f413084dc2b10fb484ea7f17";
  const char * d = "aZ842b65f413084dc2b10fb484ea7f17";
  EXPECT_THROW(warehouse_ros_sqlite::parse_md5_hexstring(c), std::invalid_argument);
  EXPECT_THROW(warehouse_ros_sqlite::parse_md5_hexstring(d), std::invalid_argument);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
