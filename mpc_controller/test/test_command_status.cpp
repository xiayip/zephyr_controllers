// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "../plugin/command_status.hpp"

using zephyr::mpc_rviz::CommandStatus;
using Status = CommandStatus::Message;
using namespace std::chrono_literals;

TEST(CommandStatus, RequiresFreshReadyFeedbackAndNoActionOwner) {
  CommandStatus view;
  const auto now = CommandStatus::Clock::now();
  EXPECT_FALSE(view.view(now).ready);
  Status status;
  status.state = Status::IDLE;
  status.feedback_valid = true;
  status.message = "Ready";
  view.update(status, now);
  EXPECT_TRUE(view.view(now).ready);
  EXPECT_FALSE(view.view(now + 1001ms).ready);
  for (auto state : {Status::INACTIVE, Status::INITIALIZING, Status::EXECUTING,
                     Status::SETTLING, Status::STOPPING, Status::FAULT}) {
    status.state = state; view.update(status, now);
    EXPECT_FALSE(view.view(now).ready);
  }
  status.state = Status::IDLE;
  status.goal_id = "owner"; view.update(status, now);
  EXPECT_FALSE(view.view(now).ready);
  status.goal_id.clear(); status.reason = Status::SOLVER_FAILED; view.update(status, now);
  EXPECT_FALSE(view.view(now).ready);
  status.reason = Status::NONE; status.feedback_valid = false; view.update(status, now);
  EXPECT_FALSE(view.view(now).ready);
  status.feedback_valid = true; view.update(status, now + 2s);
  EXPECT_TRUE(view.view(now + 2s).ready);  // No independent resume state.
}
