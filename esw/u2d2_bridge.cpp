#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include <mrover/msg/servo_configure.hpp>
#include <mrover/msg/servo_in.hpp>
#include <mrover/msg/servo_out.hpp>
#include <parameter.hpp>
#include <u2d2.hpp>

namespace mrover {

    class U2D2Bridge : public rclcpp::Node {

        static constexpr uint8_t ADDR_PRESENT_POSITION = 132;
        static constexpr uint8_t ADDR_PRESENT_VELOCITY = 128;
        static constexpr uint8_t ADDR_PRESENT_CURRENT = 126;

    public:
        U2D2Bridge() : Node("u2d2_bridge") {
            std::string u2d2Device;
            std::vector<ParameterWrapper> parameters = {
                    {"u2d2_device", u2d2Device, "/dev/u2d2"}};

            ParameterWrapper::declareParameters(this, parameters);

            mU2D2 = U2D2::getSharedInstance();

            if (mU2D2->init(u2d2Device) != U2D2::Status::Success) {
                RCLCPP_FATAL(this->get_logger(), "failed to initialize U2D2 on %s", u2d2Device.c_str());
                rclcpp::shutdown();
            }

            mConfigSub = this->create_subscription<msg::ServoConfigure>(
                    "/u2d2/configure", rclcpp::QoS(10).transient_local(),
                    [this](msg::ServoConfigure::ConstSharedPtr const& msg) -> void {
                        uint8_t const id = msg->id;
                        std::string const& name = msg->name;

                        // skip registered servos
                        if (mPubs.find(id) != mPubs.end()) return;

                        mU2D2->registerServo(id);
                        mServoIDs.push_back(id);

                        mPubs[id] = this->create_publisher<msg::ServoOut>("/u2d2/" + name + "/out", 10);

                        mSubs.push_back(this->create_subscription<msg::ServoIn>(
                                "/u2d2/" + name + "/in", 10,
                                [this, id](msg::ServoIn::ConstSharedPtr const& inMsg) -> void {
                                    uint8_t hwStatus;
                                    if (inMsg->length == 1)
                                        mU2D2->write1Byte(inMsg->addr, inMsg->value, id, &hwStatus);
                                    else if (inMsg->length == 2)
                                        mU2D2->write2Byte(inMsg->addr, inMsg->value, id, &hwStatus);
                                    else if (inMsg->length == 4)
                                        mU2D2->write4Byte(inMsg->addr, inMsg->value, id, &hwStatus);
                                }));

                        RCLCPP_INFO(get_logger(), "registered servo %s (ID %d)", name.c_str(), id);
                    });

            mPublishTimer = this->create_wall_timer(std::chrono::milliseconds(20), [this]() -> void {
                for (uint8_t const id: mServoIDs) {
                    msg::ServoOut outMsg;
                    uint8_t hwStatus;

                    auto const s1 = mU2D2->read4Byte(ADDR_PRESENT_POSITION, outMsg.position, id, &hwStatus);
                    mU2D2->read4Byte(ADDR_PRESENT_VELOCITY, outMsg.velocity, id, &hwStatus);
                    mU2D2->read2Byte(ADDR_PRESENT_CURRENT, outMsg.current, id, &hwStatus);

                    outMsg.status = static_cast<int32_t>(s1);
                    mPubs[id]->publish(outMsg);
                }
            });
        }

    private:
        std::shared_ptr<U2D2> mU2D2;
        std::vector<uint8_t> mServoIDs;
        std::unordered_map<uint8_t, rclcpp::Publisher<msg::ServoOut>::SharedPtr> mPubs;
        std::vector<rclcpp::Subscription<msg::ServoIn>::SharedPtr> mSubs;
        rclcpp::Subscription<msg::ServoConfigure>::SharedPtr mConfigSub;
        rclcpp::TimerBase::SharedPtr mPublishTimer;
    };

} // namespace mrover

auto main(int const argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::U2D2Bridge>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
