#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include "../src/a_star.h"

class NodeTest : public testing::Test{
    protected:
    A_star pathPlanner{};
    static void SetUpTestSuite(){
        nav_msgs::OccupancyGrid cost_map;
    }
    static void TearDownTestSuite(){

    }
    virtual void SetUp(){

    }
    virtual void TearDown(){

    }
};

class OneRoute : public testing::Test{
    protected:
    A_star pathPlanner{};
    static void SetUpTestSuite(){
        nav_msgs::OccupancyGrid cost_map;
        int8_t arr[4][4] = {{0, 0, 0, 0},
                    {9, 9, 9, 0},
                     {0, 0, 9, 0},
                     {0, 0, 9, 0}};
        cost_map.data = arr;
    }
    static void TearDownTestSuite(){

    }
    virtual void SetUp(){

    }
    virtual void TearDown(){

    }
};

TEST(NodeTest, euclideanDistEqualPoints){
    double dist = pathPlanner.calculateEuclideanDistence(Node(0, 0, nullptr), Node(0,0,nullptr));
    ASSERT_FLOAT_EQ(dist, 0);
}

TEST(NodeTest, euclideanDistNonEqualPoints){
    double dist = pathPlanner.calculateEuclideanDistence(Node(0,0,nullptr), Node(1,1,nullptr));
    ASSERT_FLOAT_EQ(dist, sqrt(2));
}

TEST(NodeTest, euclideanDistNegativePoints){
    double dist = pathPlanner.calculateEuclideanDistence(Node(-1,-1,nullptr), Node(-2,-2,nullptr));
    ASSERT_FLOAT_EQ(dist, sqrt(2));
}

TEST(NodeTest, nodeCtorWithoutParent){
    EXPECT_NO_THROW(Node node(1, 2, nullptr));
}

TEST(NodeTest, nodeCtorWithParent){
    Node node1(1, 2, nullptr);
    EXPECT_NO_THROW(Node node2(3,4, &node1));
}

TEST(NodeTest, fCost){
    Node node(1, 2, nullptr);
    //ASSERT_EQ(node.f(), //todo);
}

TEST(IntegerOverflowTest, fCost){
    Node node(1,2, nullptr);
    //ASSERT_EQ(node.f(), //todo);
}

TEST(NoTarget, backtracking){
    //todo
}

TEST(OneRoute, callBacktrackTwice){
 //todo
}

TEST(EmptyCostMap, validNode){
    // todo: check valid node func when costmap is empty
}

TEST(OneRoute, negativeCoordValid){
    //todo
}

TEST(OneRoute, coordOutOfBoundsValid){
    //todo
}

TEST(NotApparentRoute, alreadyExploredNode){
    //todo
}

TEST(NotApparentRoute, alreadyExploredNodeLowerCost){
    //todo
}

TEST(NotApparentRoute, alreadyExploredNodeHigherCost){
    //todo
}

TEST(NoRoute, processValidNode){
    //todo
}

TEST(NoRoute, processInvalidNode){
    //todo
}

TEST(EmptyCostMap, search){

}

TEST(NoRoute, search){

}

TEST(TwoRoutes, search){

}

TEST(DenseCostMap, search){

}

TEST(MalformedGPS, search){

}

TEST(TargetOutOfBounds, search){

}

TEST(MalformedCostMap, search){

}

TEST(StartOutOfBounds, search){

}

TEST(StartOnObject, search){

}

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}