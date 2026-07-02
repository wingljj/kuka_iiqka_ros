#include <gtest/gtest.h>

#include <hardware_interface/hardware_interface.h>

#include "kuka_rsi_hw_interface/cartesian_command_interface.h"

using kuka_rsi::CartesianCorrectionCommandInterface;
using kuka_rsi::CartesianCorrectionHandle;
using kuka_rsi::CartesianStateHandle;
using kuka_rsi::CartesianStateInterface;

class CartesianInterfacesTest : public ::testing::Test {
 protected:
  double pos_[6] = {100.0, 200.0, 300.0, 10.0, 20.0, 30.0};
  double cmd_[6] = {0, 0, 0, 0, 0, 0};
};

TEST_F(CartesianInterfacesTest, StateHandleExposesPose) {
  CartesianStateHandle h("kuka_tcp", &pos_[0], &pos_[1], &pos_[2], &pos_[3],
                         &pos_[4], &pos_[5]);
  EXPECT_EQ(h.getName(), "kuka_tcp");
  EXPECT_DOUBLE_EQ(h.getX(), 100.0);
  EXPECT_DOUBLE_EQ(h.getC(), 30.0);
  pos_[0] = 101.0;  // handle reads live data
  EXPECT_DOUBLE_EQ(h.getX(), 101.0);
}

TEST_F(CartesianInterfacesTest, StateHandleRejectsNullPointer) {
  EXPECT_THROW(CartesianStateHandle("bad", nullptr, &pos_[1], &pos_[2],
                                    &pos_[3], &pos_[4], &pos_[5]),
               hardware_interface::HardwareInterfaceException);
}

TEST_F(CartesianInterfacesTest, CorrectionHandleWritesCommand) {
  CartesianStateHandle state("kuka_tcp", &pos_[0], &pos_[1], &pos_[2],
                             &pos_[3], &pos_[4], &pos_[5]);
  CartesianCorrectionHandle h(state, &cmd_[0], &cmd_[1], &cmd_[2], &cmd_[3],
                              &cmd_[4], &cmd_[5]);
  h.setCommand(0.1, 0.2, 0.3, 0.01, 0.02, 0.03);
  EXPECT_DOUBLE_EQ(cmd_[0], 0.1);
  EXPECT_DOUBLE_EQ(cmd_[5], 0.03);
  EXPECT_DOUBLE_EQ(h.getCommandX(), 0.1);
  EXPECT_DOUBLE_EQ(h.getCommandC(), 0.03);
  EXPECT_DOUBLE_EQ(h.getX(), 100.0);  // still sees state
}

TEST_F(CartesianInterfacesTest, InterfacesRegisterAndRetrieveByName) {
  CartesianStateHandle state("kuka_tcp", &pos_[0], &pos_[1], &pos_[2],
                             &pos_[3], &pos_[4], &pos_[5]);
  CartesianStateInterface state_if;
  state_if.registerHandle(state);
  EXPECT_DOUBLE_EQ(state_if.getHandle("kuka_tcp").getY(), 200.0);

  CartesianCorrectionHandle corr(state, &cmd_[0], &cmd_[1], &cmd_[2],
                                 &cmd_[3], &cmd_[4], &cmd_[5]);
  CartesianCorrectionCommandInterface cmd_if;
  cmd_if.registerHandle(corr);
  cmd_if.getHandle("kuka_tcp").setCommand(1, 2, 3, 4, 5, 6);
  EXPECT_DOUBLE_EQ(cmd_[2], 3.0);
  EXPECT_THROW(cmd_if.getHandle("missing"),
               hardware_interface::HardwareInterfaceException);
}

TEST_F(CartesianInterfacesTest, CommandInterfaceClaimsResources) {
  CartesianStateHandle state("kuka_tcp", &pos_[0], &pos_[1], &pos_[2],
                             &pos_[3], &pos_[4], &pos_[5]);
  CartesianCorrectionHandle corr(state, &cmd_[0], &cmd_[1], &cmd_[2],
                                 &cmd_[3], &cmd_[4], &cmd_[5]);
  CartesianCorrectionCommandInterface cmd_if;
  cmd_if.registerHandle(corr);
  cmd_if.claim("kuka_tcp");
  EXPECT_EQ(cmd_if.getClaims().size(), 1u);  // exclusive-claim interface
  CartesianStateInterface state_if;
  state_if.registerHandle(state);
  state_if.claim("kuka_tcp");
  EXPECT_TRUE(state_if.getClaims().empty());  // read-only: never claims
}
