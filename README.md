# rlPx4Controller

rlPx4Controller is a quadcopter control library implemented in C++ and provides a python interface through pybind11. ***The motivation of this work is to build a parallel flight control for large-scale quadrotor reinforcement learning (RL) and Sim-to-Real on the PX4 stack.*** Thus, the implementation of the controller is consistent with the Px4 flight control to ensure the consistency of sim2real.

## Features
rlPx4Controller is developed for DRL simulation flight control. The attributes include:
- CPU-based parallel flight geometric control computation.
- Strict **PX4-like** flight controller with differential flatness.
- Parallel control for different levels that supports: position & yaw control (**PY**), velocity & yaw control (**LV**), collective thrust & attitude angle (**CTA**), collective thrust & body rate (**CTBR**)

4 modules have been implemented so far:

- PolyTrajGen: Fifth degree polynomial trajectory generation
- Lemniscate:  Figure-eight curve generation
- pyControl: Single aircraft controller
- pyParallelControl：Multi-aircraft controller for reinforcement learning in vectorized environments.

For usage and more details, please refer to the [documentation](https://rlpx4controller.readthedocs.io/).
## Fully actuated hexa controller

`ParallelHexaPosControl` and `HexaMixer` are available from
`rlPx4Controller.pyParallelControl` after rebuilding the extensions.

```python
from rlPx4Controller.pyParallelControl import ParallelHexaPosControl

controller = ParallelHexaPosControl(num_envs)
controller.configure(
    positions_b,       # (6, 3), metres, ordered rotor_0 ... rotor_5
    axes_b,            # (6, 3), thrust directions in the Z-up body frame
    reaction_signed_m, # (6,), signed reaction torque / thrust, Nm/N
    max_thrust_n,      # (6,), each motor's maximum thrust in N
    masses,            # (num_envs,), TOTAL robot mass in kg
    motor_tau=0.0,     # seconds; zero disables motor lag
)
controller.set_status(pos, q_wxyz, vel_world, ang_vel_world, dt)
commands = controller.update(target_pos_and_q_wxyz)  # (num_envs, 7) -> (num_envs, 6)
```

Positions are measured from the desired torque reference point (normally the
vehicle COM), expressed in body axes. Quaternions are unit WXYZ body-to-world
rotations. Isaac Lab XYZW quaternions must be reordered before calling this API.
Geometry is shared across environments; total masses may differ.

The controller-to-mixer input is normalized `[Fx, Fy, Fz, Tx, Ty, Tz]`:
Fx/Fy and all torque axes use `[-1, 1]`; Fz uses `[0, 1]` in this project's
Z-up frame. Each physical allocation column is
`[axis; position.cross(axis) + reaction_signed_m * axis]`.
The scale for each wrench row is the larger of its positive and negative
one-sided authority, summed over the six maximum motor thrusts.
`HexaMixer.wrench_scale` exposes these N and Nm per normalized unit.
For example, six 23 N motors tilted 30 degrees give an Fz scale of about
119.51 N; an 8.358 kg robot requests about 0.686 normalized Fz at hover.
These axis scales are geometry-specific; they do not duplicate PX4's allocation
normalization algorithm or guarantee that simultaneous axis limits are feasible.

The position controller converts acceleration and gravity to normalized body
force using these scales. The existing rate PID output is interpreted as signed
normalized torque, as in the quad controller. The mixer clamps normalized inputs,
converts them internally to physical units, and retains bounded allocation and
motor lag. Outputs are normalized **thrust fractions** `[0, 1]`, not motor RPM or
PWM: multiply by each motor's `max_thrust_n` to apply force in simulation.
`HexaMixer.wrench_error` reports normalized achieved minus clamped requested wrench.
No manual N/Nm conversion should be added between these controllers and the mixer.

Rebuild from this directory:

```bash
../am_force/.venv/bin/python setup.py build_ext --inplace --force
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../am_force/.venv/bin/python -m 
```
