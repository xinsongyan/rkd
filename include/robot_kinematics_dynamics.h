#ifndef ROBOT_KINEMATICS_DYNAMICS_H
#define ROBOT_KINEMATICS_DYNAMICS_H

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>


class RobotKinematicsDynamics {
public:
    RobotKinematicsDynamics(const std::string&urdf_filename,
                            const bool floating_base = true,
                            bool verbose = false);

    std::string get_name();

    int get_nq();

    int get_nv();

    int get_num_of_actuated_joints();

    int get_num_of_joints();

    int get_num_of_frames();

    std::vector<std::string> get_joint_names();

    std::vector<std::string> get_actuated_joint_names();

    std::vector<std::string> get_frame_names();

    void print_model_info();


    void set_robot_states(const Eigen::VectorXd&global_base_position,
                          const Eigen::VectorXd&global_base_quaternion,
                          const Eigen::VectorXd&local_base_velocity_linear,
                          const Eigen::VectorXd&local_base_velocity_angular,
                          const std::vector<std::string>&joint_names,
                          const Eigen::VectorXd&joint_positions,
                          const Eigen::VectorXd&joint_velocities);

    int get_joint_id(std::string joint_name);

    int get_frame_id(std::string frame_name);


    Eigen::Vector3d get_com_position();

    Eigen::Vector3d get_com_velocity();

    Eigen::MatrixXd get_com_jacobian();

    Eigen::MatrixXd get_mass_matrix();

    Eigen::VectorXd get_nonlinear_effects();

    pinocchio::SE3 get_frame_pose(std::string frame_name);

    Eigen::Vector3d get_frame_position(std::string frame_name);

    Eigen::Matrix3d get_frame_rotation(std::string frame_name);

    Eigen::Vector3d get_frame_velocity(std::string frame_name);

    Eigen::MatrixXd get_frame_jacobian(std::string frame_name);

protected:
    std::string _urdf_filename;

    bool _floating_base;

    pinocchio::Model _model;

    pinocchio::Data _data;

    std::string _name;

    int _total_mass;

    int _nq;

    int _nv;

    int _num_of_actuated_joints;

    int _num_of_joints;

    int _num_of_frames;

    std::vector<std::string> _joint_names;

    std::vector<std::string> _actuated_joint_names;

    std::vector<std::string> _frame_names;

    Eigen::VectorXd _q;

    Eigen::VectorXd _v;

    Eigen::VectorXd _a;
};


#endif //ROBOT_KINEMATICS_DYNAMICS_H
