#include "harness.hpp"

namespace mrover {
    void ArmControllerHarness::testArmPos() {
        ArmController::ArmPos pos1;

        pos1 = pos1 + R3d{-1, 2, 3};
        ASSERT_EQ(pos1.x, -1);
        ASSERT_EQ(pos1.y, 2);
        ASSERT_EQ(pos1.z, 3);

        msg::IK ik;
        ik.pos.x = 1;
        ik.pos.y = 2;
        ik.pos.z = 3;
        ik.pitch = 4;
        ik.roll = 5;
        ArmController::ArmPos pos2;
        pos2 = ik;
        ASSERT_EQ(pos2.x, 1);
        ASSERT_EQ(pos2.y, 2);
        ASSERT_EQ(pos2.z, 3);
        ASSERT_EQ(pos2.pitch, 4);
        ASSERT_EQ(pos2.roll, 5);
    }

    void ArmControllerHarness::testJointWrapper() {
        ArmController::JointWrapper joint{
            .limits = { .minPos = 0, .maxPos = 1, .minVel = -1, .maxVel = 1},
            .pos = 0
        };

        ASSERT_TRUE(joint.limits.posInBounds(0.5));
        ASSERT_FALSE(joint.limits.posInBounds(-0.5));
        ASSERT_FALSE(joint.limits.posInBounds(1.5));

        ASSERT_TRUE(joint.limits.velInBounds(0.5));
        ASSERT_TRUE(joint.limits.velInBounds(-0.5));
        ASSERT_FALSE(joint.limits.velInBounds(2));
        ASSERT_FALSE(joint.limits.velInBounds(-1.5));
    }

    TEST_F(ArmControllerHarness, ArmPos) {
        testArmPos();
    }

    TEST_F(ArmControllerHarness, JointWrapper) {
        testJointWrapper();
    }
}
