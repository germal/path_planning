#include <gtest/gtest.h>
#include "../src/a_star.h"

TEST(EuclideanDistTest, equalPoints){
    A_star pathPlanner{};
    double dist = pathPlanner.calculateEuclideanDistence(Node(0, 0, nullptr), Node(0,0,nullptr));
    ASSERT_FLOAT_EQ(dist, 0);
}

TEST(EuclideanDistTest, nonEqualPoints){
    A_star pathPlanner{};
    double dist = pathPlanner.calculateEuclideanDistence(Node(0,0,nullptr), Node(1,1,nullptr));
    ASSERT_FLOAT_EQ(dist, sqrt(2));
}

TEST(EuclideanDistTest, negativePoints){
    A_Star pathPlanner{};
    double dist = pathPlanner.calculateEuclideanDistence(Node(-1,-1,nullptr), Node(-2,-2,nullptr));
    ASSERT_FLOAT_EQ(dist, sqrt(2));
}

TEST(NodeCtor, withoutParent){
    EXPECT_NO_THROW(Node node(1, 2, nullptr));
}

TEST(NodeCtor, withParent){
    Node node1(1, 2, nullptr);
    EXPECT_NO_THROW(Node node2(3,4, &node1));
}


int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}