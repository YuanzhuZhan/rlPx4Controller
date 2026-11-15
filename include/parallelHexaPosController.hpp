#ifndef __PARALLEL_HEXA_POS_CONTROL_HPP_
#define __PARALLEL_HEXA_POS_CONTROL_HPP_
#include "HexaPositionController.hpp"
#include "Px4AttitudeController.hpp"
#include "Px4RateController.hpp"
#include "Px4HexaMixer.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "droneStatus.hpp"
#include <vector>

// A Fully-Actuated Hexarotor extension of rlPx4Controller
// Input target: position + attitude [x, y, z, qw, qx, qy, qz]
// Output: (N, 6) normalized rotor thrusts
// Call configure() with geometry and total robot masses before update().
// State: Z-up world position, linear/angular velocity; WXYZ body-to-world quaternion.
class pyParallelHexaPositionControl
{
private:
    std::vector<DroneStatus_t> _drons_stats;

    std::vector<HexaPositionController> _pos_control;
    std::vector<Px4AttitudeController> _atti_control;
    std::vector<Px4RateController> _rate_control;
    std::vector<HexaMixer> _mixers;
    int _envs_num = 0;
    Eigen::MatrixXd _commands;

public:

    pyParallelHexaPositionControl(int envs_num);
    void configure(const Eigen::MatrixXd& positions_b, const Eigen::MatrixXd& axes_b,
                   const Eigen::VectorXd& reaction_signed_m, const Eigen::VectorXd& max_thrust_n,
                   const Eigen::VectorXd& masses, double motor_tau = 0.0);
    void set_status(Eigen::MatrixXd pos_matrix, Eigen::MatrixXd q_matrix, Eigen::MatrixXd vel_matrix, Eigen::MatrixXd ang_vel_matrix, double dt);
    Eigen::MatrixXd update(const Eigen::MatrixXd &actions);
    ~pyParallelHexaPositionControl()
    {

    };
};

inline pyParallelHexaPositionControl::pyParallelHexaPositionControl(int envs_num)
{
    if (envs_num <= 0) throw std::invalid_argument("envs_num must be positive");
    _envs_num = envs_num;
    _commands.resize(_envs_num, 6);
    _commands.setZero();

    for (int i = 0; i < _envs_num; i++)
    {
        _drons_stats.push_back(DroneStatus_t());
        // Initialize drone quaternion and dt
        _drons_stats.at(i).quaternion = Eigen::Vector4d(1, 0, 0, 0);
        _drons_stats.at(i).dt = 0.01;

        _pos_control.push_back(HexaPositionController());
        _atti_control.push_back(Px4AttitudeController());
        _rate_control.push_back(Px4RateController());
        _mixers.push_back(HexaMixer());
    }
}

inline void pyParallelHexaPositionControl::configure(
    const Eigen::MatrixXd& positions_b, const Eigen::MatrixXd& axes_b,
    const Eigen::VectorXd& reaction_signed_m, const Eigen::VectorXd& max_thrust_n,
    const Eigen::VectorXd& masses, double motor_tau)
{
    if (masses.size() != _envs_num || !masses.allFinite() || (masses.array() <= 0.0).any())
        throw std::invalid_argument("masses must contain one positive total robot mass per environment");
    HexaMixer mixer;
    mixer.configure(positions_b, axes_b, reaction_signed_m, max_thrust_n, motor_tau);
    for (int i = 0; i < _envs_num; i++)
    {
        _mixers.at(i) = mixer;
        _pos_control.at(i).set_mass(masses(i));
        _pos_control.at(i).set_force_scale(mixer.wrenchScale().head<3>());
    }
}

inline void pyParallelHexaPositionControl::set_status(Eigen::MatrixXd pos_matrix, Eigen::MatrixXd q_matrix, Eigen::MatrixXd vel_matrix, Eigen::MatrixXd ang_vel_matrix, double dt)
{
    if (pos_matrix.rows() != _envs_num || pos_matrix.cols() != 3 ||
        q_matrix.rows() != _envs_num || q_matrix.cols() != 4 ||
        vel_matrix.rows() != _envs_num || vel_matrix.cols() != 3 ||
        ang_vel_matrix.rows() != _envs_num || ang_vel_matrix.cols() != 3 ||
        !std::isfinite(dt) || dt <= 0.0)
        throw std::invalid_argument("invalid state dimensions or dt");

    for (int i = 0; i < _envs_num; i++)
    {
        _drons_stats.at(i).position = pos_matrix.row(i).transpose();
        _drons_stats.at(i).velocity = vel_matrix.row(i).transpose();
        _drons_stats.at(i).quaternion = q_matrix.row(i).transpose();
        _drons_stats.at(i).angle_velocity = ang_vel_matrix.row(i).transpose();
        _drons_stats.at(i).dt = dt;

        _rate_control.at(i).set_q_world(q_matrix.row(i).transpose());
    }
}

inline Eigen::MatrixXd pyParallelHexaPositionControl::update(const Eigen::MatrixXd &actions)
{
    if (actions.rows() != _envs_num || actions.cols() != 7)
        throw std::invalid_argument("actions must have shape (envs_num, 7)");

    for (int i = 0; i < _envs_num; i++)
    {
        Eigen::Vector3d action_pos = actions.block(i, 0, 1, 3).transpose();
        Eigen::Vector4d action_quat = actions.block(i, 3, 1, 4).transpose();

        Eigen::Vector3d thrust_sp = _pos_control.at(i).update(action_pos, _drons_stats.at(i));
        
        Eigen::Vector3d rate_sp = _atti_control.at(i).update(action_quat, _drons_stats.at(i).quaternion);
        // Rate PID output uses the same signed normalized convention as the quad mixer.
        Eigen::Vector3d torque_sp = _rate_control.at(i).update(rate_sp, _drons_stats.at(i).angle_velocity, Eigen::Vector3d(0, 0, 0), _drons_stats.at(i).dt);

        Eigen::Matrix<double, 6, 1> wrench_sp;
        wrench_sp << thrust_sp, torque_sp;

        Eigen::Matrix<double, 6, 1> cmd = _mixers.at(i).update(wrench_sp, _drons_stats.at(i).dt);

        _commands.row(i) << cmd(0), cmd(1), cmd(2), cmd(3), cmd(4), cmd(5);
    }
    return _commands;
}
#endif
