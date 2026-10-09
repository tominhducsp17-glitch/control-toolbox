# control-toolbox + OpenArm (branch `openarm-ctc`)

This branch starts from the original [ETH ADRL control-toolbox](https://github.com/ethz-adrl/control-toolbox)
release `v3.0.2` (commit `7d36e42`, full upstream history kept) and adds the OpenArm control packages, so the
OpenArm model and controllers live next to the toolbox's own optimal-control solvers (`ct_optcon`: LQR, iLQR/GNMS
NLOC, MPC, DMS) and can use them directly. The upstream `README.md` is unchanged; this file describes the additions.

## What is in this branch

| Path | From | Content |
| --- | --- | --- |
| `ct_core`, `ct_optcon` | upstream | built by colcon on ROS 2 Humble (plain CMake packages) |
| `ct_optcon/examples` | upstream, **re-enabled** | NLOC (iLQR), NLOC_MPC, Kalman filters, constraint output; LQR with CppADCodeGen; DMS and NLP with IPOPT; constrained NLOC with HPIPM |
| `ct`, `ct_doc`, `ct_rbd`, `ct_models` | upstream | not built yet (`COLCON_IGNORE`): `ct` is a ROS 1 catkin metapackage, `ct_doc` is documentation, `ct_rbd`/`ct_models` need `kindr` and RobCoGen-generated models |
| `openarm/openarm_control` | OpenArm | library, no node: `OpenArmDynamics` (`ct::core::ControlledSystem`, Pinocchio, official URDF), tracking controllers (`ct::core::Controller`: `ctc_feedforward`, `gravity_compensation`) |
| `openarm/openarm_trajectory` | OpenArm | node `trajectory_node`: plays a trajectory, publishes `joint_commands` (q, dq) |
| `openarm/openarm_controller` | OpenArm | node `controller_node`: calls the library, publishes `controller/tau_ff`; launch file |
| `scripts/install_deps_ubuntu22.sh`, `docker/Dockerfile` | OpenArm | Ubuntu 22.04: ROS 2 Humble + Pinocchio + IPOPT + the toolbox dependencies pinned as in `ct/install_cppadcg.sh` and `ct/install_hpipm.sh` (CppAD 20200000.3, CppADCodeGen v2.4.3, BLASFEO 0.1.2, HPIPM 0.1.3) |
| `colcon.meta` | OpenArm | builds `ct_core` and `ct_optcon` with their examples; Python plotting of `ct_core` off (its matplotlib bridge fails with matplotlib ≥ 3.5, and the runtime stays Python-free) |
| `ct_core/CMakeLists.txt` | upstream, **1 fix** | `"${Python_VERSION_MAJOR}"` quoted so CMake configures when Python is disabled |

Why Pinocchio for the model: control-toolbox does not read URDF. Its rigid-body module (`ct_rbd`) needs
RobCoGen code generated from a `.kindsl` description (URDF → urdf2robcogen → RobCoGen → C++). Pinocchio reads the
official OpenArm URDF directly and is wrapped as a `ct::core::ControlledSystem`, so every `ct_optcon` solver accepts
it. The toolbox authors drafted the same idea on the upstream branch `feature/test_pin` (`PinocchioRBD`, 2020).

## Build and test

Ubuntu 22.04 (same as the integration side), no Docker needed:

```bash
./scripts/install_deps_ubuntu22.sh   # ROS 2 Humble + all toolbox/OpenArm dependencies (pinned versions)
source /opt/ros/humble/setup.bash
colcon build                        # from the repository root; colcon.meta sets the toolbox options
colcon test && colcon test-result --verbose
```

Checked on a fresh `ubuntu:22.04`: script, `colcon build` (5 packages), 7/7 tests, all 11 toolbox examples.

Any other host: `docker build -t openarm_toolbox:humble-ct -f docker/Dockerfile .` (runs the same script), then the same commands inside
`docker run --rm -it -v $PWD:/ws -w /ws openarm_toolbox:humble-ct bash`.

## Run

```bash
source install/setup.bash
ros2 launch openarm_controller trajectory_controller.launch.py       # trajectory + CTC feedforward
./build/ct_optcon/examples/ex_NLOC                                   # toolbox example: iLQR on an oscillator
```

Toolbox examples checked on this branch (Humble, GCC 11), all run to the end:

| Example | Method | Result printed |
| --- | --- | --- |
| `ex_LQR` | LQR, linearization by CppADCodeGen | gain matrix (the example ends with `return 1`) |
| `ex_NLOC` | iLQR / GNMS (nonlinear optimal control) | final cost 0.208 |
| `ex_NLOC_MPC` | MPC around NLOC | average solve delay 0.68 ms |
| `ex_NLOC_boxConstrained`, `ex_NLOC_generalConstrained` | constrained NLOC with HPIPM | final cost 947.1, 249.7 |
| `ex_DMS`, `ex_Nlp_2D`, `ex_Nlp_3D` | direct multiple shooting / NLP with IPOPT | objective 3.554, −2, −7.071 |
| `ex_KalmanFiltering`, `ex_KalmanDisturbanceFiltering` | Kalman filters | complete |
| `ex_ConstraintOutput` | constraint sparsity printout | printed (ends with `return 1`) |

Parameters are read once at start from `openarm/openarm_trajectory/config/trajectory.yaml` and
`openarm/openarm_controller/config/controller.yaml` (no defaults in the code; a missing parameter stops the node
with its name).

## Adding an optimal controller for OpenArm

1. Derive from `openarm_control::JointTrackingController<NJ>` and implement `torque(state, t)`, `clone()`, `name()`;
   the model is `dynamics_` (`OpenArmDynamics`, a `ct::core::ControlledSystem<2 NJ, NJ>`).
2. Inside, use a `ct_optcon` solver on that system (for example `ct::optcon::LQR` on a linearization, or
   `ct::optcon::NLOptConSolver` / `ct::optcon::MPC`).
3. Register the name in `makeTrackingController()` and select it with `controller_type` in `controller.yaml`.
