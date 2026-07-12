#include <gtest/gtest.h>
#include "../cost_map.hpp"

namespace mrover {
    class TestCostMap : public ::testing::Test {
    protected:

        static void SetUpTestSuite()
        {
            rclcpp::init(0, nullptr);
        }

        static void TearDownTestSuite()
        {
            rclcpp::shutdown();
        }

        static void testCoordinate() {
            CostMapNode::Coordinate c1{1, 2};
            CostMapNode::Coordinate c2{-1, 1};
            
            CostMapNode::Coordinate c3 = c1 - c2;
            ASSERT_EQ(c3.row, 2);
            ASSERT_EQ(c3.col, 1);

            CostMapNode::Coordinate c4 = c1 + c2;
            ASSERT_EQ(c4.row, 0);
            ASSERT_EQ(c4.col, 3);
        }
    };

    TEST_F(TestCostMap, Coordinate) {
        testCoordinate();
    }
}
