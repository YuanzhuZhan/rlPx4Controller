#ifndef __PX4_HEXA_MIXER_HPP_
#define __PX4_HEXA_MIXER_HPP_

#include <Eigen/Eigen>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <stdexcept>

// Fully actuated hexarotor mixer.
// Input: normalized [Fx, Fy, Fz, Tx, Ty, Tz] in the Z-up body frame.
// Fx/Fy and torques are signed [-1, 1]; Fz is [0, 1].
// Each axis is scaled by its maximum one-sided rotor authority from configure().
// These per-axis limits do not imply that all axes are jointly achievable.
// Output: six normalized rotor thrust commands [0, 1].
// The rotor order must match Isaac Lab's rotor body order.
class HexaMixer
{
private:
    using Vec3 = Eigen::Vector3d;
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Mat6 = Eigen::Matrix<double, 6, 6>;

    uint8_t _rotor_count = 6;
    Mat6 _allocation = Mat6::Zero();
    Mat6 _hessian = Mat6::Zero();
    Mat6 _weighted_allocation = Mat6::Zero();
    Eigen::FullPivLU<Mat6> _allocation_lu;

    Vec6 _wrench_scale = Vec6::Ones(); // N and Nm per unit normalized command
    Vec6 _max_thrust = Vec6::Constant(23.0);
    Vec6 _rotor_outputs = Vec6::Zero(); // physical thrust (N), after motor lag
    Vec6 _wrench_error = Vec6::Zero();
    double _motor_tau = 0.0;
    double _step_size = 0.0;
    bool _configured = false;

    Vec6 allocate(const Vec6& wrench) const;

public:
    HexaMixer() = default;
    ~HexaMixer() = default;

    void configure(const Eigen::MatrixXd& positions_b,
                   const Eigen::MatrixXd& axes_b,
                   const Eigen::VectorXd& reaction_signed_m,
                   const Eigen::VectorXd& max_thrust_n,
                   double motor_tau = 0.0);

    Vec6 update(const Vec6& wrench, double dt);
    void reset();

    const Mat6& allocationMatrix() const { return _allocation; }
    const Vec6& wrenchError() const { return _wrench_error; } // normalized achieved - requested
    const Vec6& wrenchScale() const { return _wrench_scale; }
};

inline void HexaMixer::configure(
    const Eigen::MatrixXd& positions_b,
    const Eigen::MatrixXd& axes_b,
    const Eigen::VectorXd& reaction_signed_m,
    const Eigen::VectorXd& max_thrust_n,
    double motor_tau)
{
    if (positions_b.rows() != _rotor_count || positions_b.cols() != 3 ||
        axes_b.rows() != _rotor_count || axes_b.cols() != 3 ||
        reaction_signed_m.size() != _rotor_count ||
        max_thrust_n.size() != _rotor_count ||
        !std::isfinite(motor_tau) || motor_tau < 0.0)
        throw std::invalid_argument("HexaMixer: invalid geometry or motor time constant");

    for (int i = 0; i < _rotor_count; i++)
    {
        const Vec3 position = positions_b.row(i).transpose();
        Vec3 axis = axes_b.row(i).transpose();

        if (!position.allFinite() || !axis.allFinite() || axis.norm() < 1e-8 ||
            !std::isfinite(reaction_signed_m(i)) ||
            !std::isfinite(max_thrust_n(i)) || max_thrust_n(i) <= 0.0)
            throw std::invalid_argument("HexaMixer: invalid rotor parameter");

        axis.normalize();
        _allocation.block<3, 1>(0, i) = axis;
        _allocation.block<3, 1>(3, i) =
            position.cross(axis) + reaction_signed_m(i) * axis;
    }

    if ((_allocation.row(2).array() <= 0.0).any())
        throw std::invalid_argument("HexaMixer: rotors must thrust along positive body Z");

    _allocation_lu.compute(_allocation);
    if (_allocation_lu.rank() != _rotor_count)
        throw std::invalid_argument("HexaMixer: allocation matrix is singular");

    for (int row = 0; row < 6; row++)
    {
        const auto authority = (_allocation.row(row).array() * max_thrust_n.transpose().array()).eval();
        _wrench_scale(row) = std::max(authority.max(0.0).sum(), -authority.min(0.0).sum());
    }

    // Keep the previous version's bounded weighted-least-squares allocation.
    Mat6 weights = Mat6::Identity();
    weights.diagonal().tail<3>().setConstant(3.0);
    _weighted_allocation = _allocation.transpose() * weights * weights;
    _hessian = _weighted_allocation * _allocation;
    const double max_eigenvalue =
        _hessian.selfadjointView<Eigen::Upper>().eigenvalues().maxCoeff();
    _step_size = 1.0 / std::max(max_eigenvalue, 1e-9);

    _max_thrust = max_thrust_n;
    _motor_tau = motor_tau;
    _configured = true;
    reset();
}

inline HexaMixer::Vec6 HexaMixer::update(const Vec6& wrench, double dt)
{
    if (!_configured)
        throw std::runtime_error("HexaMixer: configure() must be called first");
    if (!wrench.allFinite() || !std::isfinite(dt) || dt <= 0.0)
        throw std::invalid_argument("HexaMixer: invalid wrench or dt");

    Vec6 command = wrench.cwiseMax(Vec6::Constant(-1.0)).cwiseMin(Vec6::Ones());
    command(2) = std::max(0.0, command(2));
    const Vec6 physical_wrench = command.cwiseProduct(_wrench_scale);
    const Vec6 thrust = allocate(physical_wrench);

    // First-order motor lag. With tau=0, thrust follows immediately.
    if (_motor_tau == 0.0)
        _rotor_outputs = thrust;
    else
        _rotor_outputs += -std::expm1(-dt / _motor_tau) * (thrust - _rotor_outputs);

    _wrench_error = (_allocation * _rotor_outputs).cwiseQuotient(_wrench_scale) - command;
    return (_rotor_outputs.array() / _max_thrust.array()).matrix();
}

inline HexaMixer::Vec6 HexaMixer::allocate(const Vec6& wrench) const
{
    Vec6 thrust = _allocation_lu.solve(wrench);

    // Exact allocation when every motor is within limits.
    if ((thrust.array() >= 0.0).all() &&
        (thrust.array() <= _max_thrust.array()).all())
        return thrust;

    // Bounded allocation fallback (same algorithm as previous version).
    thrust = thrust.cwiseMax(Vec6::Zero()).cwiseMin(_max_thrust);
    const Vec6 rhs = _weighted_allocation * wrench;

    for (int i = 0; i < 80; i++)
        thrust = (thrust - _step_size * (_hessian * thrust - rhs))
                     .cwiseMax(Vec6::Zero()).cwiseMin(_max_thrust);

    return thrust;
}

inline void HexaMixer::reset()
{
    _rotor_outputs.setZero();
    _wrench_error.setZero();
}

#endif
