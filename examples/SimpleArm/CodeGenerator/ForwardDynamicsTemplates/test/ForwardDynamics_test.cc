#include "drake/examples/multibody/SimpleArm/CodeGenerator/ForwardDynamicsTemplates/ForwardDynamics.h"

#include <gtest/gtest.h>


namespace Drake {
// Fixture required for test.
class FDTest : public ::testing::Test {
  //Set parameters using SimpleArm
  double Mass_BaseLink = 100.0;
  double BodyFrame1_BaseLink = (0.0,0.0,0.25,0.0,0.0,0.0);
  double OutboardMob1_ElbowJoint = (0.0);
  double Hinge_ElbowJoint;
  double Mass_IntermediateLink;
  double Pose_IntermediateLink;
  double Pose_WristJoint;
  double Hinge_WristJoint;
  char Gripper;
};
TEST_F(FDTest, SKO_Test) {
  int N = SKO();
  EXPECT_EQ(N, 1);
}
}
