#pragma once

#include "TargetKey.hpp"

namespace mrover{
TargetKey::TargetKey(const std::shared_ptr<GoalHandleKeyAction> _goal) : goal(_goal)
{

}

auto TargetKey::onLoop() -> State*{


}
}
