#pragma once
#include <gtest/gtest.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

namespace mrover {
    // so that we can use nice time literals like 50ms
    using namespace std::chrono_literals;

    template<class node_t>
    class NodeHarness : public ::testing::Test {
    protected:
        std::shared_ptr<node_t> node;

        void SetUp() override {
            node = std::make_shared<node_t>();
        }

        static void SetUpTestSuite() {
            rclcpp::init(0, nullptr);
        }

        static void TearDownTestSuite() {
            rclcpp::shutdown();
        }
    };

    template<class harness_t>
    class NodeRosHarness : public harness_t {
    protected:
        using harness_t::node;
        std::shared_ptr<rclcpp::Node> harness_node;
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

        void SetUp() override {
            harness_t::SetUp();
            harness_node = std::make_shared<rclcpp::Node>("harness_node");
            executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

            executor->add_node(harness_node);
            executor->add_node(node);
        }

        template<typename MsgT>
        void publish(const MsgT& msg, const std::string& topic) {
            auto pub = getOrCreate(publisher_map, topic, [&]() -> auto {
                return harness_node->create_publisher<MsgT>(topic, 10);
            });
            pub->publish(msg);
        }

        template<typename MsgT, typename TimeRep, typename TimePeriod>
        auto subOne(const std::string& topic, std::chrono::duration<TimeRep, TimePeriod> timeout) -> std::optional<MsgT> {
            auto sub = getOrCreate(subscription_map, topic, [&]() -> auto {
                auto ptr = std::make_shared<SubWrapper<MsgT>>();
                // need to use a weak pointer to prevent ownership loop that causes memory leak
                std::weak_ptr<SubWrapper<MsgT>> weak_ptr = ptr;
                ptr->sub = harness_node->create_subscription<MsgT>(topic, 1, [weak_ptr](MsgT::ConstSharedPtr const& msg) -> void {
                    if (auto locked = weak_ptr.lock())
                        locked->msg = *msg;
                });
                return ptr;
            });

            sub->msg.reset();
            auto start_time = std::chrono::steady_clock::now();

            // spin until we get a message or time out
            while (rclcpp::ok()) {
                if (start_time + timeout < std::chrono::steady_clock::now())
                    break;

                executor->spin_once();

                if (sub->msg.has_value())
                    return sub->msg;
            }

            return std::nullopt;
        }

    private:
        std::unordered_map<std::string, std::shared_ptr<rclcpp::PublisherBase>> publisher_map;

        struct SubBase {
            virtual ~SubBase() = default;
        };

        template<typename MsgT>
        struct SubWrapper : SubBase {
            std::shared_ptr<rclcpp::Subscription<MsgT>> sub;
            std::optional<MsgT> msg;
        };

        std::unordered_map<std::string, std::shared_ptr<SubBase>> subscription_map;

        template<typename BaseT, typename FactoryFn>
        auto getOrCreate(std::unordered_map<std::string, std::shared_ptr<BaseT>>& map, const std::string& name, FactoryFn fun) {
            using DerivedT = std::invoke_result_t<FactoryFn>::element_type;
            auto it = map.find(name);
            if (it != map.end()) {
                auto result = std::dynamic_pointer_cast<DerivedT>(it->second);
                return result;
            }

            auto created = fun();
            map.emplace(name, created);
            return created;
        }
    };
}
