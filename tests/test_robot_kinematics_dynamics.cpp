#include <gtest/gtest.h>

# include "robot_kinematics_dynamics.cpp"


TEST(RobotKinematicsDynamics, RobotKinematicsDynamics)
{

    const std::string urdf_filename = TESTS_DIR + std::string("/go1_description/urdf/go1.urdf");
    RobotKinematicsDynamics robot_kinematics_dynamics(urdf_filename, true, true);
    EXPECT_EQ(0, 0);
}

TEST(RobotKinematicsDynamics, get_joint_id)
{
    const std::string urdf_filename = TESTS_DIR + std::string("/go1_description/urdf/go1.urdf");

    RobotKinematicsDynamics robot_kinematics_dynamics(urdf_filename, true, false);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("universe"), 0);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("root_joint"), 1);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("FL_hip_joint"), 2);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("FL_thigh_joint"), 3);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("FL_calf_joint"), 4);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("FR_hip_joint"), 5);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("FR_thigh_joint"), 6);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("FR_calf_joint"), 7);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("RL_hip_joint"), 8);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("RL_thigh_joint"), 9);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("RL_calf_joint"), 10);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("RR_hip_joint"), 11);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("RR_thigh_joint"), 12);
    EXPECT_EQ(robot_kinematics_dynamics.get_joint_id("RR_calf_joint"), 13);
    EXPECT_THROW(robot_kinematics_dynamics.get_joint_id("x"), std::invalid_argument);
}



TEST(RobotKinematicsDynamics, get_com_position)
{
    const std::string urdf_filename = TESTS_DIR + std::string("/go1_description/urdf/go1.urdf");

    RobotKinematicsDynamics robot_kinematics_dynamics(urdf_filename, true, false);
    robot_kinematics_dynamics.set_robot_states(Eigen::VectorXd::Zero(3),
                                               Eigen::VectorXd::Zero(4),
                                               Eigen::VectorXd::Zero(3),
                                               Eigen::VectorXd::Zero(3),
                                               robot_kinematics_dynamics.get_actuated_joint_names(),
                                               Eigen::VectorXd::Zero(robot_kinematics_dynamics.get_num_of_actuated_joints()),
                                               Eigen::VectorXd::Zero(robot_kinematics_dynamics.get_num_of_actuated_joints()));
    Eigen::Vector3d com_position = robot_kinematics_dynamics.get_com_position();
    EXPECT_NEAR(com_position[0], 0.0081758198032934326, 1e-6);
    EXPECT_NEAR(com_position[1], 0.0008478253536040644, 1e-6);
    EXPECT_NEAR(com_position[2], -0.031095085500370677, 1e-6);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}