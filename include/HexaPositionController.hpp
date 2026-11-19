#ifndef HEXA_POSITION_CONTROLLER_HPP
#define HEXA_POSITION_CONTROLLER_HPP

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "droneStatus.hpp"

// Independent 3D position controller for a fully actuated hexarotor.
// World frame Z-up. Returns normalized force in the Z-up body frame.
// Signed Fx/Fy [-1, 1], positive Fz [0, 1]; scales must match the mixer.
class HexaPositionController {
public:
    using Vec3 = Eigen::Vector3d;

    void set_gains(const Vec3& kp, const Vec3& kv) {
        if (!kp.allFinite() || !kv.allFinite() ||
            (kp.array() < 0.0).any() || (kv.array() < 0.0).any()) {
            throw std::invalid_argument("position gains must be finite and nonnegative");
        }
        kp_ = kp;
        kv_ = kv;
    }

    void set_mass(double mass) {
        if (!std::isfinite(mass) || mass <= 0.0)
            throw std::invalid_argument("mass must be positive");
        mass_ = mass;
    }

    void set_gravity(double gravity) {
        if (!std::isfinite(gravity) || gravity <= 0.0)
            throw std::invalid_argument("gravity must be positive");
        gravity_ = gravity;
    }

    void set_force_scale(const Vec3& force_scale) {
        if (!force_scale.allFinite() || (force_scale.array() <= 0.0).any())
            throw std::invalid_argument("force scales must be finite and positive");
        force_scale_ = force_scale;
    }

    void reset_integral() {
        pos_integral_.setZero();
    }
    Vec3 update(const Vec3& target_pos, const DroneStatus_t& state) {
        // Each environment keeps its own integral state.
        const Vec3 e_p = target_pos - state.position;
        const Vec3 integral_limit(0.5, 0.5, 5.0);
        pos_integral_ += e_p * state.dt;
        pos_integral_ = pos_integral_.cwiseMax(-integral_limit)
                                    .cwiseMin(integral_limit);


        Vec3 acceleration = kp_.cwiseProduct(e_p)
                        - kv_.cwiseProduct(state.velocity)
                        + ki_.cwiseProduct(pos_integral_);
        acceleration = acceleration.cwiseMax(Vec3::Constant(-4.0))
                                   .cwiseMin(Vec3::Constant(4.0));
        const Vec3 force_w = mass_ * (acceleration + Vec3(0.0, 0.0, gravity_));
        const Eigen::Vector4d& q = state.quaternion; // WXYZ
        const Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
        Vec3 command = (quat.conjugate() * force_w).cwiseQuotient(force_scale_);
        command = command.cwiseMax(Vec3::Constant(-1.0)).cwiseMin(Vec3::Ones());
        command.z() = std::max(0.0, command.z());
        return command;
    }

private:
    // Horizontal response fitted to P2.bag at 0.35/0.70 rad/s; altitude was not excited.
    Vec3 kp_ = Vec3(2.2943, 4.9691, 3.3727);
    Vec3 kv_ = Vec3(2.4175, 4.0609, 3.4577);
    Vec3 ki_ = Vec3(0.0758, 0.1113, 0.4533);
    Vec3 pos_integral_ = Vec3::Zero();
    Vec3 force_scale_ = Vec3::Constant(6.0 * 21.7); // overridden by mixer geometry
    double mass_ = 8.358; // overwritten via configure() for actual robot
    double gravity_ = 9.81;
};

#endif
