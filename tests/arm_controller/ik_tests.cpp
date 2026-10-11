#include "harness.hpp"

namespace mrover {
    void ArmControllerHarness::testIkPosCalc() {
        // something unreachable cause links are too short
        ASSERT_EQ(arm_controller.ikPosCalc(ArmController::ArmPos{10000}), std::nullopt);

        // something unreachable cause insufficient angular ROM
        ASSERT_EQ(arm_controller.ikPosCalc(ArmController::ArmPos{0.5}), std::nullopt);

        // something reachable
        auto res = arm_controller.ikPosCalc(ArmController::ArmPos{0.7});
        ASSERT_TRUE(res.has_value());

        // test naming mismatch
        arm_controller.joints = {};
        res = arm_controller.ikPosCalc(ArmController::ArmPos{0.7});
        ASSERT_FALSE(res.has_value());
    }

    TEST_F(ArmControllerHarness, ikPosCalc) {
        testIkPosCalc();
    }
}
