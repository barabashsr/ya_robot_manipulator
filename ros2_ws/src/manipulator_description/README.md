# Manipulator Description Package

URDF/Xacro description for a rail-mounted robotic manipulator with dynamic storage system.

## Overview

This package contains the robot description (URDF/Xacro) for a storage manipulation system consisting of:

- **Storage System**: Two parallel rows of cabinets with box placement frames
- **Manipulator**: Rail-mounted manipulator (to be added)
- **Dynamic Boxes**: Runtime-spawned box objects (managed by `storage_manager_node`)

## Package Structure

```
manipulator_description/
├── config/
│   └── storage_params.yaml          # Storage system configuration
├── urdf/
│   ├── robot.urdf.xacro             # Top-level robot assembly
│   ├── materials.xacro              # Material definitions
│   ├── storage_system.urdf.xacro    # Storage system entry point
│   ├── cabinet_row.urdf.xacro       # Cabinet row macro
│   ├── cabinet.urdf.xacro           # Single cabinet macro
│   └── box_placement_frames.urdf.xacro  # Box placement frame generation
├── launch/
│   └── display.launch.py            # RViz visualization launch
├── rviz/
│   └── view_robot.rviz              # RViz configuration
└── meshes/
    ├── cabinets/                    # Cabinet mesh files (STL)
    └── boxes/                       # Box mesh files (STL)
```

## Usage

### Build the Package

```bash
cd /home/robo/robo/ya_robot_manipulator/ros2_ws
colcon build --packages-select manipulator_description
source install/setup.bash
```

### Visualize in RViz

```bash
ros2 launch manipulator_description display.launch.py
```

### Generate URDF from Xacro

```bash
xacro src/manipulator_description/urdf/robot.urdf.xacro > robot.urdf
```

## Configuration

Edit `config/storage_params.yaml` to configure:

- Cabinet dimensions and wall thickness
- Cabinet row layout (left/right positions)
- Cabinet sizes per row (e.g., "4x10x10")
- Box placement configurations
- Department configurations
- Materials and colors

## Coordinate System

- **Origin**: Manipulator `base_link`
- **X-axis**: Along cabinet rows (rail direction)
- **Y-axis**: Perpendicular to cabinets (depth)
- **Z-axis**: Vertical (up)

## Box Addressing Convention

Address format: `{side, cabinet_num, box_row, box_col, dept_num}`

- **side**: "left" or "right"
- **cabinet_num**: 1-indexed cabinet number
- **box_row**: 1 = top, M = bottom
- **box_col**: 1 = leftmost (from manipulator view)
- **dept_num**: 1-indexed department within box

Example: `{left, 2, 3, 4, 7}` = Left row, Cabinet 2, Row 3, Column 4, Department 7

## Architecture Details

See `/home/robo/robo/ya_robot_manipulator/docs/storage-spec-v2-1.md` for complete specification (v2.2).

## Status

- [x] Storage system URDF structure
- [x] Box placement frames
- [x] Configuration system
- [ ] Manipulator URDF (TODO)
- [ ] Runtime box spawning node (TODO)
- [ ] Department TF broadcaster (TODO)

## License

Apache-2.0
