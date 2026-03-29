# cs4mt_arm_dynamic_footprint_nav2

**Real-time arm-aware footprint projection for Nav2** — computes the 2D convex hull of a mounted manipulator arm from live joint states and publishes it to the Nav2 costmap at runtime, enabling collision-safe mobile manipulator navigation without modifying the navigation stack.

Validated on the **Neobotix MPO-700 + UR10** platform under **ROS 2 Humble**.

---

## Overview

Standard Nav2 treats the robot footprint as a static parameter — typically the base platform outline. This is unsafe for mobile manipulators: when the arm extends beyond the base, the navigation stack has no awareness of the additional collision volume.

`cs4mt_arm_dynamic_footprint_nav2` solves this by:
- Querying TF for each arm link pose in the base frame at runtime
- Approximating each link's ground-plane cross-section as a disc of calibrated radius
- Computing the convex hull of all disc samples unioned with the base rectangle
- Publishing the resulting polygon to `/local_costmap/footprint` and `/global_costmap/footprint` at **20 Hz**

The footprint tightens when the arm is tucked and expands when the arm is extended — giving the planner an always-accurate collision boundary with no Nav2 modifications required.

---

## Dependencies

- ROS 2 Humble
- Nav2
- [neo_simulation2](https://github.com/neobotix/neo_simulation2)
- [cs4home_core](https://github.com/Intelligent-Robot-Lab/cs4home_core)

---

## Installation

```bash
cd ~/ros2_ws_coresense/src
git clone https://github.com/KeerthiSagarSN/cs4mt_arm_dynamic_footprint_nav2
cd ..
colcon build --packages-select cs4mt_armfootprint_aware_navigation
source install/setup.bash
```

---

## Usage

Launch the following in order, each in a separate terminal.

**1. Set Gazebo model path and launch simulation**
```bash
export GAZEBO_MODEL_PATH=$(ros2 pkg prefix robotiq_description)/share:$GAZEBO_MODEL_PATH
ros2 launch neo_simulation2 simulation.launch.py my_robot:=mpo_700 world:=neo_workshop arm_type:=ur10
```

**2. Launch navigation with RViz**
```bash
ros2 launch neo_simulation2 navigationwithrviz.launch.py
```

**3. Launch the dynamic arm footprint module**
```bash
ros2 launch cs4mt_armfootprint_aware_navigation cs4mt_armfootprint_aware_navigation.launch.py
```

**4. Configure and activate the lifecycle node**
```bash
ros2 lifecycle set /arm_footprint_cognitive_module configure
ros2 lifecycle set /arm_footprint_cognitive_module activate
```

**5. Launch RQT (optional, for monitoring)**
```bash
rqt
```

---

## Configuration

All parameters are set in `params/params.yaml` — no hardcoded values. Key parameters:

| Parameter | Description |
|---|---|
| `arm_links` | List of URDF link names to include in footprint computation |
| `link_radii` | Per-link disc radius (m) approximating physical cross-section |
| `base_half_extents` | Half-extents `[hx, hy]` of the base platform rectangle (m) |
| `safety_padding` | Outset margin added to the convex hull (m) |
| `update_rate_ms` | Footprint publish period in milliseconds (default: 50 ms = 20 Hz) |

---

## Architecture

The module follows the [cs4home_core](https://github.com/Intelligent-Robot-Lab/cs4home_core) cognitive architecture pattern:

```
Afferent  →  /joint_states subscriber
Core      →  TF lookup + convex hull computation
Efferent  →  /local_costmap/footprint + /global_costmap/footprint publisher
```

---

## Citation

If you use this software, please cite:
```bibtex
@software{sagar2025armfootprintnav2,
  author    = {Sagar, Keerthi},
  title     = {{cs4mt\_arm\_dynamic\_footprint\_nav2}: Real-Time Arm-Aware Footprint Projection for {Nav2}},
  year      = {2025},
  publisher = {GitHub},
  url       = {https://github.com/KeerthiSagarSN/cs4mt_arm_dynamic_footprint_nav2}
}

---

## License

Apache 2.0 — see [LICENSE](LICENSE) for details.
