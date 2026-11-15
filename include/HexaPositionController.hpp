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

    Vec3 update(const Vec3& target_pos, const DroneStatus_t& state) const {
        Vec3 acceleration = kp_.cwiseProduct(target_pos - state.position)
                          - kv_.cwiseProduct(state.velocity);
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

private:  // TODO: doublechekc the kp_ and kv_gains
    Vec3 kp_ = Vec3(0.7, 0.7, 0.7);
    Vec3 kv_ = Vec3(0.7, 0.7, 0.7);
    Vec3 force_scale_ = Vec3::Constant(6.0 * 23.0); // overridden by mixer geometry
    double mass_ = 8.358; // overwritten via configure() for actual robot
    double gravity_ = 9.81;
};

#endif
