#include "harness.hpp"

namespace mrover {
    TEST_F(ArmControllerHarness, ArmPos) {
        ArmPos pos1;

        pos1 = pos1 + R3d{-1, 2, 3};
        EXPECT_EQ(pos1.x, -1);
        EXPECT_EQ(pos1.y, 2);
        EXPECT_EQ(pos1.z, 3);

        msg::IK ik;
        ik.pos.x = 1;
        ik.pos.y = 2;
        ik.pos.z = 3;
        ik.pitch = 4;
        ik.roll = 5;
        ArmPos pos2;
        pos2 = ik;
        EXPECT_EQ(pos2.x, 1);
        EXPECT_EQ(pos2.y, 2);
        EXPECT_EQ(pos2.z, 3);
        EXPECT_EQ(pos2.pitch, 4);
        EXPECT_EQ(pos2.roll, 5);

        // test conversion to SE3
        ArmPos pos3;
        SE3d se3 = pos3.toSE3();
        EXPECT_EQ(se3, SE3d::Identity());

        se3 = pos2.toSE3();
        EXPECT_EQ(se3.x(), 1);
        EXPECT_EQ(se3.y(), 2);
        EXPECT_EQ(se3.z(), 3);
    }

    TEST_F(ArmControllerHarness, JointWrapper) {
        JointWrapper joint{
            .limits = { .minPos = 0, .maxPos = 1, .minVel = -1, .maxVel = 1},
            .pos = 0
        };

        EXPECT_TRUE(joint.limits.posInBounds(0.5));
        EXPECT_FALSE(joint.limits.posInBounds(-0.5));
        EXPECT_FALSE(joint.limits.posInBounds(1.5));

        EXPECT_TRUE(joint.limits.velInBounds(0.5));
        EXPECT_TRUE(joint.limits.velInBounds(-0.5));
        EXPECT_FALSE(joint.limits.velInBounds(2));
        EXPECT_FALSE(joint.limits.velInBounds(-1.5));
    }
}
