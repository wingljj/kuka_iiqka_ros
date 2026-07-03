#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "kuka_rsi_hw_interface/command_limiter.h"

using kuka_rsi::CommandLimiter;
using kuka_rsi::CommandLimits;

TEST(CommandLimiter, PassesSmallCommandUnchanged) {
  CommandLimiter lim;
  double c[6] = {0.1, -0.2, 0.3, 0.01, -0.02, 0.03};
  EXPECT_FALSE(lim.clamp(c, CommandLimits{}));
  EXPECT_DOUBLE_EQ(c[0], 0.1);
  EXPECT_DOUBLE_EQ(c[5], 0.03);
}

TEST(CommandLimiter, ClampsTranslationAxes) {
  CommandLimiter lim;
  double c[6] = {2.0, -2.0, 0.0, 0.0, 0.0, 0.0};
  EXPECT_TRUE(lim.clamp(c, CommandLimits{}));
  EXPECT_DOUBLE_EQ(c[0], 0.5);
  EXPECT_DOUBLE_EQ(c[1], -0.5);
}

TEST(CommandLimiter, ClampsRotationAxes) {
  CommandLimiter lim;
  double c[6] = {0.0, 0.0, 0.0, 1.0, -1.0, 0.04};
  EXPECT_TRUE(lim.clamp(c, CommandLimits{}));
  EXPECT_DOUBLE_EQ(c[3], 0.05);
  EXPECT_DOUBLE_EQ(c[4], -0.05);
  EXPECT_DOUBLE_EQ(c[5], 0.04);  // within limit, untouched
}

TEST(CommandLimiter, NonFiniteBecomesZeroAndFlags) {
  CommandLimiter lim;
  double c[6] = {std::nan(""), 0.1, 0.0, 0.0, 0.0, 0.0};
  EXPECT_TRUE(lim.clamp(c, CommandLimits{}));
  EXPECT_DOUBLE_EQ(c[0], 0.0);
  EXPECT_DOUBLE_EQ(c[1], 0.1);
  double inf[6] = {0.0, 0.0, 0.0,
                   std::numeric_limits<double>::infinity(), 0.0, 0.0};
  EXPECT_TRUE(lim.clamp(inf, CommandLimits{}));
  EXPECT_DOUBLE_EQ(inf[3], 0.0);
}

TEST(CommandLimiter, CustomLimitsRespected) {
  CommandLimiter lim;
  CommandLimits l;
  l.max_trans = 0.1;
  l.max_rot = 0.01;
  double c[6] = {0.2, 0.0, 0.0, 0.02, 0.0, 0.0};
  EXPECT_TRUE(lim.clamp(c, l));
  EXPECT_DOUBLE_EQ(c[0], 0.1);
  EXPECT_DOUBLE_EQ(c[3], 0.01);
}
