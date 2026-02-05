// Copyright (c) 2026, b»robotized group
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <kinematics_interface/kinematics_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

class DummyDeepTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    loader_ = std::make_unique<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      "kinematics_interface", "kinematics_interface::KinematicsInterface");

    // Create plugin instance
    instance_ = loader_->createSharedInstance("dummy_ikfast/DummyKinematics");

    auto node = std::make_shared<rclcpp::Node>("test_dummy_node");
    node->declare_parameter("tip", "link_6");
    node->declare_parameter("base", "base_link");
    node->declare_parameter("alpha", 0.0001);

    instance_->initialize("test_desc", node->get_node_parameters_interface(), "");
    num_joints_ = 6;
  }

  std::shared_ptr<kinematics_interface::KinematicsInterface> instance_;
  std::unique_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> loader_;
  int num_joints_;
};

// --- 1. UNIT TEST: FORWARD KINEMATICS ---
TEST_F(DummyDeepTest, TestForwardKinematicsConsistency)
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_joints_);
  Eigen::Isometry3d T1, T2;

  // Test that same joints always produce same Pose
  ASSERT_TRUE(instance_->calculate_link_transform(q, "link_6", T1));
  ASSERT_TRUE(instance_->calculate_link_transform(q, "link_6", T2));

  EXPECT_TRUE(T1.isApprox(T2));
  EXPECT_FALSE(T1.matrix().hasNaN());
}

// --- 2. UNIT TEST: JACOBIAN ACCURACY ---
TEST_F(DummyDeepTest, TestJacobianRank)
{
  Eigen::VectorXd q(num_joints_);
  q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;  // Non-singular position

  Eigen::Matrix<double, 6, Eigen::Dynamic> J;
  J.resize(6, num_joints_);

  ASSERT_TRUE(instance_->calculate_jacobian(q, "link_6", J));

  // A 6-DOF robot in a general position should have a full rank Jacobian (rank 6)
  Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(J);
  EXPECT_EQ(lu_decomp.rank(), 6);
}

// --- 3. UNIT TEST: SINGULARITY HANDLING (Damping Test) ---
TEST_F(DummyDeepTest, TestSingularityDamping)
{
  Eigen::VectorXd q_singular = Eigen::VectorXd::Zero(num_joints_);  // Usually singular

  Eigen::Matrix<double, Eigen::Dynamic, 6> J_inv;
  J_inv.resize(num_joints_, 6);
  // IKFast kinematics .cpp uses Damped Least Squares.
  // This test ensures that even at a singularity, the inverse is NOT infinite.
  ASSERT_TRUE(instance_->calculate_jacobian_inverse(q_singular, "link_6", J_inv));

  for (int i = 0; i < J_inv.size(); ++i)
  {
    EXPECT_TRUE(std::isfinite(J_inv.data()[i]));
  }
}

// --- 4. UNIT TEST: INVERSE KINEMATICS (The "Big One") ---
TEST_F(DummyDeepTest, TestIKPositionSolution)
{
  Eigen::VectorXd q_expected(num_joints_);
  q_expected << 0.5, -0.4, 0.8, 0.0, 0.5, 0.1;

  Eigen::Isometry3d target_pose;
  ASSERT_TRUE(instance_->calculate_link_transform(q_expected, "link_6", target_pose));

  // 1. Use a smaller offset
  Eigen::VectorXd q_current = q_expected;
  for (int i = 0; i < num_joints_; ++i)
  {
    q_current[i] += 0.001;
  }

  Eigen::Isometry3d current_pose;
  instance_->calculate_link_transform(q_current, "link_6", current_pose);

  // 2. Calculate full 6-DOF Delta X (Translation + Rotation)
  Eigen::Matrix<double, 6, 1> dx;

  // Translation delta
  dx.head<3>() = target_pose.translation() - current_pose.translation();

  // Rotation delta (using axis-angle representation of the rotation error)
  Eigen::Matrix3d R_error = target_pose.linear() * current_pose.linear().transpose();
  Eigen::AngleAxisd angle_axis(R_error);
  dx.tail<3>() = angle_axis.axis() * angle_axis.angle();

  Eigen::VectorXd dq;
  ASSERT_TRUE(instance_->convert_cartesian_deltas_to_joint_deltas(q_current, dx, "link_6", dq));

  // 3. Verification: Check if the new joint angles are closer to q_expected
  double error_before = (q_expected - q_current).norm();
  double error_after = (q_expected - (q_current + dq)).norm();

  std::cout << "Error Before: " << error_before << " | Error After: " << error_after << std::endl;
  EXPECT_LT(error_after, error_before);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
