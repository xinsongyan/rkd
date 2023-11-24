#include "robot_kinematics_dynamics.h"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>


RobotKinematicsDynamics::RobotKinematicsDynamics(const std::string&urdf_filename,
                                                 const bool floating_base,
                                                 bool verbose) {
    _urdf_filename = urdf_filename;
    _floating_base = floating_base;


    if (_floating_base) {
        pinocchio::urdf::buildModel(_urdf_filename, pinocchio::JointModelFreeFlyer(), _model, false);
    }
    else {
        pinocchio::urdf::buildModel(_urdf_filename, _model, false);
    }

    _data = pinocchio::Data(_model);
    _name = get_name();
    _total_mass = pinocchio::computeTotalMass(_model);
    _nq = get_nq();
    _nv = get_nv();
    _num_of_actuated_joints = get_num_of_actuated_joints();
    _num_of_joints = get_num_of_joints();
    _num_of_frames = get_num_of_frames();
    _joint_names = get_joint_names();
    _actuated_joint_names = get_actuated_joint_names();
    _frame_names = get_frame_names();

    _q = Eigen::VectorXd::Zero(_nq);
    _v = Eigen::VectorXd::Zero(_nv);
    _a = Eigen::VectorXd::Zero(_nv);

    if (verbose)
        print_model_info();
}

void RobotKinematicsDynamics::print_model_info() {
    std::cout << "name: " << _name << std::endl;
    std::cout << "total_mass: " << _total_mass << std::endl;
    std::cout << "nq: " << _nq << std::endl;
    std::cout << "nv: " << _nv << std::endl;
    std::cout << "num_of_actuated_joints: " << _num_of_actuated_joints << std::endl;
    std::cout << "num_of_joints: " << _num_of_joints << std::endl;
    std::cout << "num_of_frames: " << _num_of_frames << std::endl;
    std::cout << "joint_names: " << std::endl;
    for (const auto&str: _joint_names) std::cout << str << " ";
    std::cout << std::endl;
    std::cout << "actuated_joint_names: " << std::endl;
    for (const auto&str: _actuated_joint_names) std::cout << str << " ";
    std::cout << std::endl;
    std::cout << "frame_names: " << std::endl;
    for (const auto&str: _frame_names) std::cout << str << " ";
    std::cout << std::endl;
}

std::string RobotKinematicsDynamics::get_name() {
    return _model.name;
}

int RobotKinematicsDynamics::get_nq() {
    return _model.nq;
}

int RobotKinematicsDynamics::get_nv() {
    return _model.nv;
}

int RobotKinematicsDynamics::get_num_of_actuated_joints() {
    if (_floating_base) {
        return _model.nv - 6;
    }
    else {
        return _model.nv;
    }
}

int RobotKinematicsDynamics::get_num_of_joints() {
    return _model.njoints;
}

int RobotKinematicsDynamics::get_num_of_frames() {
    return _model.nframes;
}

std::vector<std::string> RobotKinematicsDynamics::get_joint_names() {
    return _model.names;
}

std::vector<std::string> RobotKinematicsDynamics::get_actuated_joint_names() {
    std::vector<std::string> actuated_joint_names;
    for (auto joint_name: get_joint_names()) {
        if (joint_name != "universe" && joint_name != "root_joint") {
            actuated_joint_names.push_back(joint_name);
        }
    }
    return actuated_joint_names;
}

std::vector<std::string> RobotKinematicsDynamics::get_frame_names() {
    std::vector<std::string> frame_names;
    for (auto frame: _model.frames) {
        frame_names.push_back(frame.name);
    }
    return frame_names;
}

void RobotKinematicsDynamics::set_robot_states(const Eigen::VectorXd&global_base_position,
                                               const Eigen::VectorXd&global_base_quaternion,
                                               const Eigen::VectorXd&local_base_velocity_linear,
                                               const Eigen::VectorXd&local_base_velocity_angular,
                                               const std::vector<std::string>&joint_names,
                                               const Eigen::VectorXd&joint_positions,
                                               const Eigen::VectorXd&joint_velocities) {
    // create map between joint_names and joint_positions
    std::map<std::string, double> joint_name_position_map;
    std::map<std::string, double> joint_name_velocity_map;
    for (int i = 0; i < joint_names.size(); ++i) {
        joint_name_position_map[joint_names[i]] = joint_positions[i];
        joint_name_velocity_map[joint_names[i]] = joint_velocities[i];
    }

    Eigen::VectorXd actuated_joint_postions = Eigen::VectorXd::Zero(_num_of_actuated_joints);
    Eigen::VectorXd actuated_joint_velocities = Eigen::VectorXd::Zero(_num_of_actuated_joints);
    for (int i = 0; i < _num_of_actuated_joints; ++i) {
        actuated_joint_postions[i] = joint_name_position_map[_actuated_joint_names[i]];
        actuated_joint_velocities[i] = joint_name_velocity_map[_actuated_joint_names[i]];
    }

    if (_floating_base) {
        _q << global_base_position, global_base_quaternion, actuated_joint_postions;
        _v << local_base_velocity_linear, local_base_velocity_angular, actuated_joint_velocities;
    }
    else {
        _q << actuated_joint_postions;
        _v << actuated_joint_velocities;
    }

    // update data
    pinocchio::computeAllTerms(_model, _data, _q, _v);
    pinocchio::updateFramePlacements(_model, _data);
}

int RobotKinematicsDynamics::get_joint_id(std::string joint_name) {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(_model.existJointName(joint_name), joint_name + " does not exist in the model");
    if (_model.existJointName(joint_name)) {
        return _model.getJointId(joint_name);
    }
    else {
        return -1;
    }
}


int RobotKinematicsDynamics::get_frame_id(std::string frame_name) {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(_model.existFrame(frame_name), frame_name + " does not exist in the model");
    if (_model.existFrame(frame_name)) {
        return _model.getFrameId(frame_name);
    }
    else {
        return -1;
    }
}

Eigen::Vector3d RobotKinematicsDynamics::get_com_position() {
    return _data.com[0];
}

Eigen::Vector3d RobotKinematicsDynamics::get_com_velocity() {
    return _data.vcom[0];
}

Eigen::MatrixXd RobotKinematicsDynamics::get_com_jacobian() {
    return _data.Jcom;
}

Eigen::MatrixXd RobotKinematicsDynamics::get_mass_matrix() {
    return _data.M;
}

Eigen::VectorXd RobotKinematicsDynamics::get_nonlinear_effects() {
    return _data.nle;
}

pinocchio::SE3 RobotKinematicsDynamics::get_frame_pose(std::string frame_name) {
    return pinocchio::updateFramePlacement(_model, _data, get_frame_id(frame_name));
}

Eigen::Vector3d RobotKinematicsDynamics::get_frame_position(std::string frame_name) {
    return get_frame_pose(frame_name).translation();
}

Eigen::Matrix3d RobotKinematicsDynamics::get_frame_rotation(std::string frame_name) {
    return get_frame_pose(frame_name).rotation();
}

Eigen::Vector3d RobotKinematicsDynamics::get_frame_velocity(std::string frame_name) {
    return pinocchio::getFrameVelocity(_model, _data, get_frame_id(frame_name), pinocchio::LOCAL_WORLD_ALIGNED).
            linear();
}

Eigen::MatrixXd RobotKinematicsDynamics::get_frame_jacobian(std::string frame_name) {
    Eigen::MatrixXd frame_jacobian = Eigen::MatrixXd::Zero(6, _nv);
    pinocchio::getFrameJacobian(_model, _data, get_frame_id(frame_name), pinocchio::LOCAL_WORLD_ALIGNED,
                                frame_jacobian);
    return frame_jacobian;
}
