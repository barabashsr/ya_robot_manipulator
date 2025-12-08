#!/usr/bin/env python3
"""
Box URDF Generator Utility

Generates URDF XML for dynamically spawned boxes with department child links.
Department frame positions follow: y = offset_y + (dept_num - 1) * step_y

Story 4A.3: Implement Dynamic Box Spawner with Department Frame Generation

AC2: URDF generated with base_link and N department child links
AC7: Department frame Y positions follow formula from storage_params.yaml
"""

import os
from dataclasses import dataclass
from typing import Dict, Optional
import yaml

from ament_index_python.packages import get_package_share_directory


@dataclass
class BoxDimensions:
    """Box physical dimensions and visual properties."""
    width: float   # X dimension
    depth: float   # Y dimension (along box length)
    height: float  # Z dimension
    mass: float = 0.5


@dataclass
class DepartmentConfig:
    """Department layout configuration."""
    num_departments: int
    depth: float      # Individual department depth
    offset_y: float   # Y offset for first department
    step_y: float     # Y step between departments


def load_storage_params() -> Dict:
    """
    Load storage_params.yaml from manipulator_description package.

    Returns:
        Dict with storage system configuration.
    """
    try:
        pkg_share = get_package_share_directory('manipulator_description')
        config_path = os.path.join(pkg_share, 'config', 'storage_params.yaml')
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        raise RuntimeError(f"Failed to load storage_params.yaml: {e}")


def get_cabinet_config(side: str, cabinet_num: int, storage_params: Dict) -> str:
    """
    Get cabinet size configuration string.

    Args:
        side: 'left' or 'right'
        cabinet_num: Cabinet number (1-indexed)
        storage_params: Loaded storage params

    Returns:
        Size string like '4x10x10' (columns x rows x departments)
    """
    cabinet_rows = storage_params.get('cabinet_rows', {})
    side_config = cabinet_rows.get(side, {})
    cabinets = side_config.get('cabinets', [])

    if cabinet_num < 1 or cabinet_num > len(cabinets):
        raise ValueError(f"Invalid cabinet_num {cabinet_num} for side {side}")

    return cabinets[cabinet_num - 1].get('size', '4x10x10')


def parse_cabinet_size(size_str: str) -> tuple:
    """
    Parse cabinet size string.

    Args:
        size_str: Format 'COLSxROWSxDEPTS' e.g., '4x10x10'

    Returns:
        Tuple of (columns, rows, departments)
    """
    parts = size_str.split('x')
    if len(parts) != 3:
        raise ValueError(f"Invalid cabinet size format: {size_str}")
    return int(parts[0]), int(parts[1]), int(parts[2])


def get_box_dimensions(
    side: str,
    cabinet_num: int,
    row: int,
    storage_params: Dict,
    spawner_config: Dict
) -> BoxDimensions:
    """
    Calculate box dimensions based on cabinet configuration.

    Args:
        side: 'left' or 'right'
        cabinet_num: Cabinet number (1-indexed)
        row: Row number (1-indexed)
        storage_params: Loaded storage params
        spawner_config: Box spawner config

    Returns:
        BoxDimensions dataclass
    """
    cabinet_size = get_cabinet_config(side, cabinet_num, storage_params)
    columns, rows, departments = parse_cabinet_size(cabinet_size)

    box_configs = storage_params.get('box_configurations', {})
    dept_configs = storage_params.get('department_configurations', {})

    # Get column config
    col_key = f'columns_{columns}'
    col_config = box_configs.get(col_key, {})
    width = col_config.get('width', 0.06)

    # Get row config for height
    row_key = f'rows_{rows}'
    row_config = box_configs.get(row_key, {})
    height = row_config.get('height', 0.1)

    # Get department config for depth
    dept_key = f'departments_{departments}'
    dept_config = dept_configs.get(dept_key, {})
    depth_per_dept = dept_config.get('depth', 0.02)
    # Total box depth is approximately: offset + departments * step
    offset_y = dept_config.get('offset_y', 0.005)
    step_y = dept_config.get('step_y', 0.024)
    depth = offset_y + departments * step_y

    mass = spawner_config.get('box_mass', 0.5)

    return BoxDimensions(width=width, depth=depth, height=height, mass=mass)


def get_department_config(
    side: str,
    cabinet_num: int,
    storage_params: Dict
) -> DepartmentConfig:
    """
    Get department configuration for a cabinet.

    Args:
        side: 'left' or 'right'
        cabinet_num: Cabinet number (1-indexed)
        storage_params: Loaded storage params

    Returns:
        DepartmentConfig dataclass
    """
    cabinet_size = get_cabinet_config(side, cabinet_num, storage_params)
    _, _, num_departments = parse_cabinet_size(cabinet_size)

    dept_configs = storage_params.get('department_configurations', {})
    dept_key = f'departments_{num_departments}'
    dept_config = dept_configs.get(dept_key, {})

    return DepartmentConfig(
        num_departments=num_departments,
        depth=dept_config.get('depth', 0.02),
        offset_y=dept_config.get('offset_y', 0.005),
        step_y=dept_config.get('step_y', 0.024)
    )


def calculate_department_y_offset(dept_num: int, dept_config: DepartmentConfig) -> float:
    """
    Calculate Y offset for a department link.

    AC7: y = offset_y + (dept_num - 1) * step_y

    Args:
        dept_num: Department number (1-indexed)
        dept_config: Department configuration

    Returns:
        Y offset in meters
    """
    return dept_config.offset_y + (dept_num - 1) * dept_config.step_y


def generate_box_urdf(
    box_id: str,
    box_dims: BoxDimensions,
    dept_config: DepartmentConfig,
    spawner_config: Dict,
    gripper_frame: str,
    include_gazebo_plugin: bool = True
) -> str:
    """
    Generate complete URDF XML string for a box with department links.

    Args:
        box_id: Box identifier (e.g., 'box_l_1_2_3')
        box_dims: Box physical dimensions
        dept_config: Department layout configuration
        spawner_config: Box spawner configuration
        gripper_frame: Gripper frame name for DetachableJoint
        include_gazebo_plugin: Whether to include DetachableJoint plugin

    Returns:
        Complete URDF XML string
    """
    base_link = f"{box_id}_base_link"

    # Get colors from config
    box_color = spawner_config.get('box_color', {})
    dept_color = spawner_config.get('department_marker_color', {})
    dept_radius = spawner_config.get('department_marker_radius', 0.01)

    # Calculate inertia (simple box approximation)
    m = box_dims.mass
    w, d, h = box_dims.width, box_dims.depth, box_dims.height
    ixx = (m / 12.0) * (d**2 + h**2)
    iyy = (m / 12.0) * (w**2 + h**2)
    izz = (m / 12.0) * (w**2 + d**2)

    # Box visual offset: origin at wall (Y=0), box extends in +Y direction
    # This matches address box convention where origin is at the cabinet-facing wall
    box_y_offset = d / 2.0

    # Start URDF
    urdf_lines = [
        f'<?xml version="1.0"?>',
        f'<robot name="{box_id}">',
        f'',
        f'  <!-- Base link: box body (origin at wall, box extends in +Y) -->',
        f'  <link name="{base_link}">',
        f'    <visual>',
        f'      <origin xyz="0 {box_y_offset:.6f} 0" rpy="0 0 0"/>',
        f'      <geometry>',
        f'        <box size="{w:.4f} {d:.4f} {h:.4f}"/>',
        f'      </geometry>',
        f'      <material name="box_material">',
        f'        <color rgba="{box_color.get("r", 0.3):.2f} {box_color.get("g", 0.5):.2f} '
        f'{box_color.get("b", 0.8):.2f} {box_color.get("a", 1.0):.2f}"/>',
        f'      </material>',
        f'    </visual>',
        f'    <collision>',
        f'      <origin xyz="0 {box_y_offset:.6f} 0" rpy="0 0 0"/>',
        f'      <geometry>',
        f'        <box size="{w:.4f} {d:.4f} {h:.4f}"/>',
        f'      </geometry>',
        f'    </collision>',
        f'    <inertial>',
        f'      <origin xyz="0 {box_y_offset:.6f} 0" rpy="0 0 0"/>',
        f'      <mass value="{m:.3f}"/>',
        f'      <inertia ixx="{ixx:.6f}" iyy="{iyy:.6f}" izz="{izz:.6f}" '
        f'ixy="0" ixz="0" iyz="0"/>',
        f'    </inertial>',
        f'  </link>',
        f'',
        f'  <!-- Disable gravity and make static - position controlled by TF sync -->',
        f'  <gazebo reference="{base_link}">',
        f'    <gravity>false</gravity>',
        f'    <static>true</static>',
        f'  </gazebo>',
        f''
    ]

    # Generate department links and joints
    for dept_num in range(1, dept_config.num_departments + 1):
        dept_link = f"{box_id}_dept_{dept_num}_link"
        dept_joint = f"{box_id}_dept_{dept_num}_joint"
        y_offset = calculate_department_y_offset(dept_num, dept_config)

        urdf_lines.extend([
            f'  <!-- Department {dept_num} link -->',
            f'  <link name="{dept_link}">',
            f'    <visual>',
            f'      <geometry>',
            f'        <sphere radius="{dept_radius:.4f}"/>',
            f'      </geometry>',
            f'      <material name="dept_marker_material">',
            f'        <color rgba="{dept_color.get("r", 1.0):.2f} {dept_color.get("g", 0.0):.2f} '
            f'{dept_color.get("b", 0.0):.2f} {dept_color.get("a", 0.8):.2f}"/>',
            f'      </material>',
            f'    </visual>',
            f'  </link>',
            f'',
            f'  <joint name="{dept_joint}" type="fixed">',
            f'    <parent link="{base_link}"/>',
            f'    <child link="{dept_link}"/>',
            f'    <origin xyz="0 {y_offset:.6f} 0" rpy="0 0 0"/>',
            f'  </joint>',
            f''
        ])

    # Note: DetachableJoint plugin removed - using TF-to-Gazebo pose sync instead
    # This is simpler and more reliable than physics-based attachment

    urdf_lines.append('</robot>')

    return '\n'.join(urdf_lines)


def generate_box_urdf_from_address(
    box_id: str,
    side: str,
    cabinet_num: int,
    row: int,
    column: int,
    gripper_frame: str,
    spawner_config: Dict,
    storage_params: Optional[Dict] = None,
    include_gazebo_plugin: bool = True
) -> tuple:
    """
    Generate URDF for a box at a specific storage address.

    Args:
        box_id: Box identifier
        side: 'left' or 'right'
        cabinet_num: Cabinet number (1-indexed)
        row: Row number (1-indexed)
        column: Column number (1-indexed)
        gripper_frame: Gripper frame name
        spawner_config: Box spawner configuration
        storage_params: Storage params (loaded if None)
        include_gazebo_plugin: Include Gazebo DetachableJoint

    Returns:
        Tuple of (urdf_string, department_count)
    """
    if storage_params is None:
        storage_params = load_storage_params()

    box_dims = get_box_dimensions(side, cabinet_num, row, storage_params, spawner_config)
    dept_config = get_department_config(side, cabinet_num, storage_params)

    urdf = generate_box_urdf(
        box_id=box_id,
        box_dims=box_dims,
        dept_config=dept_config,
        spawner_config=spawner_config,
        gripper_frame=gripper_frame,
        include_gazebo_plugin=include_gazebo_plugin
    )

    return urdf, dept_config.num_departments
