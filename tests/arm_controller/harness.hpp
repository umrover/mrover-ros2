#pragma once
#include "tests/mtest/mtest.h"
#include "navigation/arm_controller/arm_controller.hpp"

namespace mrover {
    class ArmControllerHarness : public NodeHarness<ArmController> {
    protected:
        // these using statements are needed so these private types are
        // accessible inside TEST_F functions
        using ArmPos = ArmController::ArmPos;
        using JointWrapper = ArmController::JointWrapper;

        static void checkPos(ArmPos pos1, ArmPos pos2) {
            EXPECT_FLOAT_EQ(pos1.x, pos2.x);
            EXPECT_FLOAT_EQ(pos1.y, pos2.y);
            EXPECT_FLOAT_EQ(pos1.z, pos2.z);
            EXPECT_FLOAT_EQ(pos1.pitch, pos2.pitch);
            EXPECT_FLOAT_EQ(pos1.roll, pos2.roll);
            EXPECT_FLOAT_EQ(pos1.gripper, pos2.gripper);
        }

        void testPosCallback();
        void testVelCallback();
        void testHandleGoal();
        void testModeCallback();

        void testIkPosCalc();
        void testIkFk();
        void testIkVelCalc();
    };

    class ArmControllerRosHarness : public NodeRosHarness<ArmControllerHarness> {
    protected:
        void changeIkMode(srv::IkMode_Request::_mode_type req);
    };
}
