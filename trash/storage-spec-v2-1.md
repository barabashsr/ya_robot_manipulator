# Robotic Storage System - URDF/Xacro Specification (v2.2 - URDF Generation Focus)

**Version:** 2.2
**Date:** November 22, 2025
**Scope:** URDF/Xacro generation for static storage system with deterministic manipulator

---

## 1. System Overview & Key Architecture Decisions

### 1.1 Core Concept
The system consists of a **rail-mounted manipulator** positioned between two **parallel rows of storage cabinets**. The key architectural insight is treating **boxes as independent, movable objects** rather than static cabinet children. This enables:

- Boxes to be drawn from cabinets
- Boxes to be manipulated in 3D space
- Departments to be generated on-demand when boxes are accessed
- Significant reduction in static URDF complexity

### 1.2 Coordinate System
- **Origin**: Manipulator base_link
- **X-axis**: Horizontal, along cabinet rows (manipulator rail direction)
- **Y-axis**: Horizontal, perpendicular to cabinet rows (depth)
- **Z-axis**: Vertical (up)

### 1.3 Cabinet Row Layout
- **Left Row**: Y = +400mm from base_link, oriented with opening facing manipulator (toward -Y direction)
- **Right Row**: Y = -400mm from base_link, oriented with opening facing manipulator (toward +Y direction)
- **Orientation**: Both rows parallel to X-axis
- **Spacing**: Cabinets adjacent along X-axis
- **Physical Rotation**: Right row cabinets are rotated 180° around Z-axis (like rotating real furniture to face the opposite side)
- **Box Addressing**: ALWAYS left-to-right, top-to-bottom from the manipulator's view, regardless of row

---

## 2. Component Hierarchy

```
world
├── base_link (manipulator reference frame)
├── manipulator (gripper, selector, etc.)
└── storage_system
    ├── left_cabinet_row
    │   ├── cabinet_1 (static)
    │   │   ├── box_placement_frame_1_1 (virtual frame for box attachment)
    │   │   ├── box_placement_frame_1_2
    │   │   └── ...
    │   ├── cabinet_2 (static)
    │   └── ...
    ├── right_cabinet_row
    │   └── ...
    └── mobile_boxes (created on-demand)
        ├── box_1 (when extracted from cabinet)
        │   ├── department_frame_1
        │   ├── department_frame_2
        │   └── ...
        └── box_2
            └── ...
```

**Key Points:**
- **Cabinets**: Static, remain in URDF
- **Box Placement Frames**: Static frames in cabinet where boxes attach when stored
- **Boxes**: Spawned at runtime when selected, destroyed when returned to cabinet
- **Department Frames**: Generated dynamically when box is extracted

---

## 3. Cabinet Position Formulas (CORRECTED)

### 3.1 Left Row Cabinet Position

For cabinet `i` (1-indexed) in left row:

```
cab_x = row_offset_x + (i - 1) * cabinet_width
cab_y = row_offset_y
cab_z = row_offset_z

Example:
row_offset = {x: 0.0, y: 0.4, z: 0.0}
Cabinet 1: x = 0.0 + 0*0.7 = 0.0m
Cabinet 2: x = 0.0 + 1*0.7 = 0.7m
Cabinet 3: x = 0.0 + 2*0.7 = 1.4m
```

**Cabinet Origin**: Left-bottom-front corner (exterior)
**Cabinet Opening**: Faces +Y direction (toward manipulator)

### 3.2 Right Row Cabinet Position (CORRECTED)

For cabinet `i` (1-indexed) in right row:

```
Important: Right row cabinets are rotated 180° around Z-axis, NOT mirrored!

cab_x = row_offset_x + i * cabinet_width
cab_y = row_offset_y
cab_z = row_offset_z

Example:
row_offset = {x: 0.0, y: -0.4, z: 0.0}
Cabinet 1: x = 0.0 + 1*0.7 = 0.7m
Cabinet 2: x = 0.0 + 2*0.7 = 1.4m
Cabinet 3: x = 0.0 + 3*0.7 = 2.1m
```

**Cabinet Origin**: RIGHT-bottom-front corner (exterior) - rotated 180°
**Cabinet Opening**: Faces -Y direction (toward manipulator)
**Rotation**: 180° around Z-axis relative to left row orientation

**Geometric Reasoning:**
- Left row: cabinets line up along X starting at x=0, each 0.7m wide
- Right row: cabinets are rotated, so first cabinet's RIGHT edge is at x=0.7m
- This ensures symmetric layout without actual mirroring of the 3D model

---

## 4. Box Placement Frames in Cabinet

### 4.1 Purpose
Instead of creating department frames for all boxes in the storage (O(thousands)), we create **box placement frames** in each cabinet. These are:
- Virtual frames marking where a box *can* be placed
- Static during normal operation
- Used as attachment points for boxes via fixed joints/transforms

### 4.2 Box Addressing Convention (CORRECTED v2.2)

**Address Format**: `{side, cabinet_num, box_row, box_col, dept_num}`

**Addressing Rules (From Manipulator's Perspective)**:
- **Columns (box_col)**: 1, 2, 3... = **left to right from manipulator's view**
  - Left row: increasing X (column 1 at leftmost)
  - Right row: decreasing X (column 1 at leftmost when viewed from manipulator)
- **Rows (box_row)**: 1, 2, 3... = **top to bottom** (both rows)
  - Row 1: Highest position in cabinet (largest Z coordinate)
  - Row M: Lowest position in cabinet (smallest Z coordinate)
- **Important**: Addressing is always from the operator/manipulator perspective, matching how a human would number shelves

### 4.3 Box Placement Frame Position (CORRECTED)

For box at row `r`, column `c` in cabinet of size `NxMxD`:

```
CORRECTED FORMULAS (v2.1):

frame_x_local = box_offset_x[N] + (c - 1) * box_step_x[N]
frame_y_local = 0  (at front of cabinet)
frame_z_local = box_offset_z[M] + (M - r) * box_step_z[M]

Note the change: (M - r) instead of (r - 1)
This makes Row 1 = highest position, Row M = lowest position

Global position:
frame_x_global = cabinet_x + frame_x_local
frame_y_global = cabinet_y + frame_y_local  
frame_z_global = cabinet_z + frame_z_local
```

**Frame Origin**: Center of front wall of box position (where box link origin attaches)

### 4.4 Addressing Example

**Cabinet Size**: 4x10x10 (4 columns × 10 rows × 10 departments)

```
Visual representation (front view of cabinet):

         Column:  1    2    3    4
                ┌────┬────┬────┬────┐
Row 1 (top)     │ ■  │ ■  │ ■  │ ■  │  Z = offset_z + 9*step_z  (highest)
                ├────┼────┼────┼────┤
Row 2           │ ■  │ ■  │ ■  │ ■  │  Z = offset_z + 8*step_z
                ├────┼────┼────┼────┤
Row 3           │ ■  │ ■  │ ■  │ ■  │  Z = offset_z + 7*step_z
                ├────┼────┼────┼────┤
...             │ ...│ ...│ ...│ ...│
                ├────┼────┼────┼────┤
Row 10 (bottom) │ ■  │ ■  │ ■  │ ■  │  Z = offset_z + 0*step_z  (lowest)
                └────┴────┴────┴────┘
                X=   X=   X=   X=
              offset offset offset offset
                +0   +step +2step +3step
```

**Address {left, 1, 1, 1, 5}** = Left row, Cabinet 1, Row 1 (TOP), Column 1 (LEFT), Department 5

### 4.5 Box Placement Frame URDF Representation

```xml
<!-- In cabinet.urdf.xacro, for each box position -->
<link name="left_cab1_box_placement_2_5">
  <!-- No geometry, just a frame -->
</link>

<!-- Static joint fixing frame to cabinet -->
<joint name="left_cab1_box_placement_2_5_fixed" type="fixed">
  <parent link="left_cab1"/>
  <child link="left_cab1_box_placement_2_5"/>
  <origin xyz="0.24 0.0 0.201" rpy="0 0 0"/>
</joint>
```

---

## 5. Box as Independent Object

### 5.1 Box Creation (At Runtime)

When manipulator extracts a box from address `{side, cab_num, row, col, dept_num}`:

1. **Query storage state** for box size at that location
2. **Spawn box URDF** with:
   - Link name: `box_{unique_id}`
   - Size determined by cabinet size
   - Department count from size string
   - Initial transform = current box placement frame in cabinet
3. **Create fixed joint** from box placement frame to box link
4. **Generate department frames** on-demand (see section 6)

### 5.2 Box Definition

```xml
<!-- boxes/box.urdf.xacro -->
<xacro:macro name="box" params="
  box_id
  size_name
  box_config
  dept_config
  show_departments">
  
  <link name="box_${box_id}">
    <!-- Visual & collision geometry -->
    <xacro:if value="${use_mesh}">
      <visual>
        <geometry>
          <mesh filename="..." scale="1 1 1"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <mesh filename="..." scale="1 1 1"/>
        </geometry>
      </collision>
    </xacro:if>
    <xacro:unless value="${use_mesh}">
      <visual>
        <geometry>
          <box size="${box_width} ${box_depth} ${box_height}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <box size="${box_width} ${box_depth} ${box_height}"/>
        </geometry>
      </collision>
    </xacro:unless>
  </link>
  
  <!-- Optional: department frames -->
  <xacro:if value="${show_departments}">
    <xacro:box_departments box_id="${box_id}" 
                          num_depts="${num_depts}"
                          dept_config="${dept_config}"/>
  </xacro:if>
</xacro:macro>
```

### 5.3 Box Lifecycle

```
State 1: IN_CABINET
├─ box object: NOT created in URDF
├─ location: address-based (cabinet coordinates known)
├─ department frames: NONE
└─ transform: implicit (in box_placement_frame_X_Y_Z)

    ↓ [Manipulator selects box]

State 2: EXTRACTED
├─ box object: CREATED (spawned at placement frame location)
├─ attached to: manipulator selector (or dynamic frame)
├─ department frames: GENERATED on-demand
└─ transform: follows manipulator

    ↓ [Manipulator item selection & gripper action]

State 3: ITEM_ACCESSED
├─ box object: ACTIVE
├─ attached to: manipulator selector
├─ department frames: ALL generated (for gripper targeting)
└─ transform: manipulator-controlled

    ↓ [Manipulator returns box to cabinet]

State 4: RETURNED_TO_CABINET
├─ box object: DESTROYED (removed from URDF)
├─ location: address-based again
├─ department frames: CLEANED UP
└─ transform: implicit (in box_placement_frame_X_Y_Z)
```

---

## 6. Department Frame Generation (On-Demand)

### 6.1 Purpose
When a box is extracted and we need to access departments:
- Generate frames dynamically
- Attach to moving box link
- Coordinate transformation follows box automatically

### 6.2 Dynamic Department Frame Creation

```python
# Python node: storage_manager.py or department_frame_generator.py

def generate_department_frames(box_id, size_name, box_transform):
    """
    Create TF frames for all departments in extracted box.
    
    Args:
        box_id: unique box identifier
        size_name: "4x10x10", "5x12x14", etc.
        box_transform: current transform of box (from URDF/TF)
    """
    
    # Parse size
    num_cols, num_rows, num_depts = parse_size(size_name)
    
    # Get configs
    box_config = get_box_config(num_cols, num_rows)
    dept_config = get_dept_config(num_depts)
    
    # Create frames for each department
    for d in range(1, num_depts + 1):
        frame_name = f"{box_id}_dept{d}"
        
        # Calculate local position (relative to box origin)
        x_local = 0.0
        y_local = dept_config['offset_y'] + (d-1) * dept_config['step_y']
        z_local = box_config['height']
        
        # Publish TF (transform relative to box link)
        publish_static_transform(
            parent_frame=f"box_{box_id}",
            child_frame=frame_name,
            translation=(x_local, y_local, z_local),
            rotation=(0, 0, 0, 1)  # identity quat
        )
        
        # Also create URDF link/joint for persistence (optional)
        if URDF_PERSISTENT:
            create_urdf_department_link(box_id, d, ...)
```

### 6.3 Department Frame Naming

```
For box_123 extracted from {left, 2, 3, 4, ?}:

Frame names:
  box_123_dept1
  box_123_dept2
  ...
  box_123_dept14  (if 5x12x14)

These frames are attached to "box_123" link via TF broadcaster
When box_123 moves, all department frames move with it
```

### 6.4 Alternative: Add to URDF at Box Creation

Instead of TF broadcaster, add department links to spawned box URDF:

```xml
<!-- In dynamically spawned box URDF -->
<link name="box_123_dept1"/>
<joint name="box_123_dept1_to_box" type="fixed">
  <parent link="box_123"/>
  <child link="box_123_dept1"/>
  <origin xyz="0.0 0.004 0.075" rpy="0 0 0"/>
</joint>

<link name="box_123_dept2"/>
<joint name="box_123_dept2_to_box" type="fixed">
  <parent link="box_123"/>
  <child link="box_123_dept2"/>
  <origin xyz="0.0 0.0211 0.075" rpy="0 0 0"/>
</joint>
```

---

## 7. Coordinate Lookup Strategies

### 7.1 Strategy A: Address-Based Lookup (Static Cabinet Storage)

When box is in cabinet:

```python
def get_department_coordinate(side, cab_num, box_row, box_col, dept_num):
    """
    Get XYZ coordinate without spawning box.
    Used for path planning to extract box.
    
    CORRECTED v2.1: Uses (M - r) formula for top-to-bottom addressing
    """
    
    # Get row offset
    row_offset = config['cabinet_rows'][side]['offset']
    
    # Calculate cabinet position
    cabinet_x = row_offset['x'] + (cab_num - 1) * 0.7
    if side == 'right':
        cabinet_x += 0.7  # CORRECTION: rotate + shift
    cabinet_y = row_offset['y']
    cabinet_z = row_offset['z']
    
    # Get cabinet size
    cabinet_list = config['cabinet_rows'][side]['cabinets']
    size_name = cabinet_list[cab_num - 1]
    num_cols, num_rows, num_depts = parse_size(size_name)
    
    # Get box position in cabinet
    box_col_config = config['box_configurations'][f'columns_{num_cols}']
    box_row_config = config['box_configurations'][f'rows_{num_rows}']
    
    # CORRECTED v2.1: Row addressing top-to-bottom
    box_x_local = box_col_config['offset_x'] + (box_col - 1) * box_col_config['step_x']
    box_z_local = box_row_config['offset_z'] + (num_rows - box_row) * box_row_config['step_z']
    
    # Get department position in box
    dept_config = config['department_configurations'][f'departments_{num_depts}']
    
    dept_y_local = dept_config['offset_y'] + (dept_num - 1) * dept_config['step_y']
    dept_z_offset = box_row_config['height']
    
    # Global coordinates
    coord = {
        'x': cabinet_x + box_x_local,
        'y': cabinet_y + dept_y_local,
        'z': cabinet_z + box_z_local + dept_z_offset,
        'frame': f"{side}_cab{cab_num}_box_placement_{box_row}_{box_col}"
    }
    
    return coord
```

### 7.2 Strategy B: Frame-Based Lookup (Dynamic Box)

When box is extracted:

```python
def get_department_coordinate_dynamic(box_id, dept_num):
    """
    Get XYZ coordinate from TF tree (box is moving).
    """
    
    frame_name = f"box_{box_id}_dept{dept_num}"
    
    try:
        # Query TF
        transform = tf_listener.lookupTransform(
            'base_link',           # target frame
            frame_name,            # source frame
            rospy.Time(0)          # latest
        )
        
        translation = transform[0]  # (x, y, z)
        
        return {
            'x': translation[0],
            'y': translation[1],
            'z': translation[2],
            'frame': frame_name
        }
    except tf.TransformException:
        rospy.logwarn(f"Frame {frame_name} not found")
        return None
```

---

## 8. YAML Configuration

### 8.1 Complete Configuration Structure

```yaml
# storage_params.yaml

# Cabinet dimensions (now configurable)
cabinet_dimensions:
  exterior: {x: 0.7, y: 0.66, z: 1.4}
  wall_thickness: 0.02  # 20mm walls (CORRECTED from 0.2)
  interior: {x: 0.66, y: 0.62, z: 1.36}  # Recalculated: exterior - 2*wall_thickness

# Cabinet row definitions
cabinet_rows:
  left:
    offset: {x: 0.0, y: 0.4, z: 0.0}
    # Note: X offset can be non-zero (for left edge of first cabinet)
    # Note: Z offset can be non-zero (for base height)
    cabinets:
      - size: "4x10x10"
      - size: "4x10x10"
      - size: "4x6x10"
      - size: "5x12x14"
  
  right:
    offset: {x: 0.0, y: -0.4, z: 0.0}
    # Same interpretation, but right row cabinets are rotated 180°
    cabinets:
      - size: "5x12x14"
      - size: "5x8x14"
      - size: "6x14x16"
      - size: "4x10x10"

# Cabinet models grouped by columns x rows (not departments)
cabinet_models:
  "4x10":  # grouping: columns x rows
    mesh:
      visual: "package://robot_storage/meshes/cabinet_4x10_visual.stl"
      collision: "package://robot_storage/meshes/cabinet_4x10_collision.stl"
    use_mesh_collision: true
  
  "4x6":
    mesh:
      visual: "package://robot_storage/meshes/cabinet_4x6_visual.stl"
      collision: "package://robot_storage/meshes/cabinet_4x6_collision.stl"
    use_mesh_collision: true
  
  "5x12":
    mesh:
      visual: "package://robot_storage/meshes/cabinet_5x12_visual.stl"
      collision: "package://robot_storage/meshes/cabinet_5x12_collision.stl"
    use_mesh_collision: true
  
  "5x8":
    mesh:
      visual: "package://robot_storage/meshes/cabinet_5x8_visual.stl"
      collision: "package://robot_storage/meshes/cabinet_5x8_collision.stl"
    use_mesh_collision: true
  
  "6x14":
    mesh:
      visual: "package://robot_storage/meshes/cabinet_6x14_visual.stl"
      collision: "package://robot_storage/meshes/cabinet_6x14_collision.stl"
    use_mesh_collision: true

# Box configurations grouped by columns, rows, departments
box_configurations:
  columns_4:
    width: 0.06
    offset_x: 0.05
    step_x: 0.07
    mesh:
      visual: "package://robot_storage/meshes/box_4col_visual.stl"
      collision: null  # use primitive
    use_mesh_collision: false
  
  columns_5:
    width: 0.055
    offset_x: 0.045
    step_x: 0.065
    mesh:
      visual: "package://robot_storage/meshes/box_5col_visual.stl"
      collision: null
    use_mesh_collision: false
  
  columns_6:
    width: 0.048
    offset_x: 0.04
    step_x: 0.058
    mesh:
      visual: "package://robot_storage/meshes/box_6col_visual.stl"
      collision: null
    use_mesh_collision: false
  
  rows_6:
    height: 0.15
    offset_z: 0.05
    step_z: 0.16
  
  rows_8:
    height: 0.115
    offset_z: 0.045
    step_z: 0.125
  
  rows_10:
    height: 0.09
    offset_z: 0.04
    step_z: 0.1
  
  rows_12:
    height: 0.075
    offset_z: 0.035
    step_z: 0.083
  
  rows_14:
    height: 0.065
    offset_z: 0.03
    step_z: 0.071

department_configurations:
  departments_10:
    depth: 0.02
    offset_y: 0.005
    step_y: 0.024  # includes 4mm wall
  
  departments_14:
    depth: 0.0131
    offset_y: 0.004
    step_y: 0.0171
  
  departments_16:
    depth: 0.011
    offset_y: 0.0035
    step_y: 0.015

# Material definitions
materials:
  cabinet:
    color: {r: 0.7, g: 0.7, b: 0.7, a: 1.0}
  box:
    color: {r: 0.3, g: 0.5, b: 0.8, a: 1.0}
  department_marker:
    color: {r: 1.0, g: 0.0, b: 0.0, a: 0.5}

# Runtime flags
runtime:
  generate_departments_on_demand: true
  use_tf_broadcaster_for_departments: true  # vs URDF links
  department_marker_size: 0.01  # debug visualization
```

---

## 9. URDF/Xacro File Structure

### 9.1 File Organization

```
robot_storage/
├── urdf/
│   ├── world.urdf.xacro              # Top-level assembly
│   ├── manipulator.urdf.xacro        # Manipulator definition
│   ├── storage_system.urdf.xacro     # Storage entry point
│   ├── cabinet_row.urdf.xacro        # Row of cabinets
│   ├── cabinet.urdf.xacro            # Single cabinet
│   ├── box_array.urdf.xacro          # Array of boxes in cabinet
│   ├── box.urdf.xacro                # Single box
│   └── department.urdf.xacro         # Department marker
├── config/
│   ├── storage_params.yaml           # Storage configuration
│   └── manipulator_params.yaml       # Manipulator configuration
└── meshes/
    ├── cabinets/
    │   ├── cabinet_4x10.stl
    │   ├── cabinet_5x12.stl
    │   └── ...
    └── boxes/
        ├── box_4col.stl
        ├── box_5col.stl
        └── ...
```

### 9.2 box_placement_frames.urdf.xacro (CORRECTED)

```xml
<xacro:macro name="box_placement_frames" params="
  side
  cabinet_num
  num_rows
  num_cols
  num_depts
  box_configs
  parent_link">
  
  <!-- Get configs -->
  <xacro:property name="box_col_config" 
    value="${box_configs['columns_' + str(num_cols)]}"/>
  <xacro:property name="box_row_config" 
    value="${box_configs['rows_' + str(num_rows)]}"/>
  
  <!-- Nested loops: rows then columns -->
  <xacro:placement_rows 
    current_row="1" 
    max_rows="${num_rows}"
    num_cols="${num_cols}"
    side="${side}"
    cabinet_num="${cabinet_num}"
    parent_link="${parent_link}"
    box_col_config="${box_col_config}"
    box_row_config="${box_row_config}"/>
    
</xacro:macro>

<!-- Recursion for rows -->
<xacro:macro name="placement_rows" params="
  current_row
  max_rows
  num_cols
  side
  cabinet_num
  parent_link
  box_col_config
  box_row_config">
  
  <xacro:if value="${current_row <= max_rows}">
    <!-- Create frames for all columns in this row -->
    <xacro:placement_cols
      current_col="1"
      max_cols="${num_cols}"
      current_row="${current_row}"
      max_rows="${max_rows}"
      side="${side}"
      cabinet_num="${cabinet_num}"
      parent_link="${parent_link}"
      box_col_config="${box_col_config}"
      box_row_config="${box_row_config}"/>
    
    <!-- Recurse for next row -->
    <xacro:placement_rows
      current_row="${current_row + 1}"
      max_rows="${max_rows}"
      num_cols="${num_cols}"
      side="${side}"
      cabinet_num="${cabinet_num}"
      parent_link="${parent_link}"
      box_col_config="${box_col_config}"
      box_row_config="${box_row_config}"/>
  </xacro:if>
  
</xacro:macro>

<!-- Recursion for columns -->
<xacro:macro name="placement_cols" params="
  current_col
  max_cols
  current_row
  max_rows
  side
  cabinet_num
  parent_link
  box_col_config
  box_row_config">
  
  <xacro:if value="${current_col <= max_cols}">
    <!-- Calculate frame position -->
    <!-- CORRECTED v2.1: Top-to-bottom addressing -->
    <xacro:property name="frame_x" 
      value="${box_col_config['offset_x'] + (current_col - 1) * box_col_config['step_x']}"/>
    <xacro:property name="frame_z" 
      value="${box_row_config['offset_z'] + (max_rows - current_row) * box_row_config['step_z']}"/>
    
    <!-- Create frame link -->
    <link name="${side}_cab${cabinet_num}_box_placement_${current_row}_${current_col}"/>
    
    <!-- Attach to cabinet -->
    <joint name="${side}_cab${cabinet_num}_box_placement_${current_row}_${current_col}_to_cabinet" 
           type="fixed">
      <parent link="${parent_link}"/>
      <child link="${side}_cab${cabinet_num}_box_placement_${current_row}_${current_col}"/>
      <origin xyz="${frame_x} 0.0 ${frame_z}" rpy="0 0 0"/>
    </joint>
    
    <!-- Recurse for next column -->
    <xacro:placement_cols
      current_col="${current_col + 1}"
      max_cols="${max_cols}"
      current_row="${current_row}"
      max_rows="${max_rows}"
      side="${side}"
      cabinet_num="${cabinet_num}"
      parent_link="${parent_link}"
      box_col_config="${box_col_config}"
      box_row_config="${box_row_config}"/>
  </xacro:if>
  
</xacro:macro>
```

**Key Change in v2.1**: 
```xml
<!-- OLD (v2.0): Bottom-to-top addressing -->
<xacro:property name="frame_z" 
  value="${box_row_config['offset_z'] + (current_row - 1) * box_row_config['step_z']}"/>

<!-- NEW (v2.1): Top-to-bottom addressing -->
<xacro:property name="frame_z" 
  value="${box_row_config['offset_z'] + (max_rows - current_row) * box_row_config['step_z']}"/>
```

---

## 10. Example Coordinate Calculation (CORRECTED)

### Given Address
```
{left, 2, 3, 4, 7}
```
- Side: left
- Cabinet: 2
- Box row: 3 (third from top)
- Box column: 4 (fourth from left)
- Department: 7

### Configuration
```yaml
left row offset: {x: 0, y: 0.4, z: 0}
cabinet 2 size: "5x12x14"
columns_5: {offset_x: 0.045, step_x: 0.065}
rows_12: {offset_z: 0.035, step_z: 0.083, height: 0.075}
departments_14: {offset_y: 0.004, step_y: 0.0171}
```

### Calculation Steps (v2.1 CORRECTED)

**1. Cabinet position**
```
cab_x = 0 + (2-1) * 0.7 = 0.7m
cab_y = 0.4m
cab_z = 0m
```

**2. Box position (row 3, col 4) - CORRECTED**
```
box_x_local = 0.045 + (4-1) * 0.065 = 0.045 + 0.195 = 0.24m
box_y_local = 0m
box_z_local = 0.035 + (12 - 3) * 0.083 = 0.035 + 9*0.083 = 0.035 + 0.747 = 0.782m

box_x_global = 0.7 + 0.24 = 0.94m
box_y_global = 0.4 + 0 = 0.4m
box_z_global = 0 + 0.782 = 0.782m
```

**3. Department position (dept 7)**
```
dept_x_local = 0m
dept_y_local = 0.004 + (7-1) * 0.0171 = 0.004 + 0.1026 = 0.1066m
dept_z_local = 0.075m (box height)

dept_x_global = 0.94 + 0 = 0.94m
dept_y_global = 0.4 + 0.1066 = 0.5066m
dept_z_global = 0.782 + 0.075 = 0.857m
```

### Final Coordinates (CORRECTED)
```
Item at {left, 2, 3, 4, 7}:
X = 0.94m
Y = 0.5066m
Z = 0.857m  (Row 3 from top is near the top of cabinet)
```

---

## 11. Runtime Box Management (Conceptual - Out of Scope for v2.2)

### 11.1 Dynamic URDF Modification Approach

**Note**: This section describes runtime behavior conceptually. For v2.2, we focus on **static URDF generation only**.

### 11.2 The Box Lifecycle in Runtime

The system operates through **dynamic URDF modification** in a ROS node (e.g., `storage_manager_node`). When a box is extracted from a cabinet, the node:

1. **Generates box URDF snippet** with links and joints
2. **Publishes updated robot_description** to the parameter server
3. **Broadcasts TF transforms** for department frames
4. **Manages box state** (in cabinet vs attached to selector)

### 11.3 Storage Manager Node (Conceptual)

```python
# CONCEPTUAL PSEUDOCODE - NOT FOR IMMEDIATE IMPLEMENTATION
# This demonstrates the ROS node approach, not Gazebo-specific services

import rospy
from urdf_parser_py.urdf import URDF
import xml.etree.ElementTree as ET

class StorageManagerNode:
    """
    ROS node that manages dynamic box spawning/despawning in the robot URDF.

    Key Responsibilities:
    - Track which boxes are extracted vs in cabinets
    - Generate box URDF snippets on-demand
    - Update robot_description parameter
    - Broadcast department TF frames for active boxes
    - Manage fixed joints between boxes and manipulator selector
    """

    def __init__(self):
        rospy.init_node('storage_manager')

        # Load storage configuration
        self.config = rospy.get_param('storage_params')

        # Track active boxes (extracted from cabinets)
        self.active_boxes = {}  # {box_id: {address, size, state}}

        # Get base robot URDF (without boxes)
        self.base_urdf_str = rospy.get_param('/robot_description')
        self.base_urdf_tree = ET.fromstring(self.base_urdf_str)

        # TF broadcaster for department frames
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Services
        rospy.Service('extract_box', ExtractBox, self.handle_extract_box)
        rospy.Service('return_box', ReturnBox, self.handle_return_box)

    def handle_extract_box(self, req):
        """
        Service handler: Extract box from cabinet.

        Steps:
        1. Generate unique box ID
        2. Create box URDF snippet (link + joint to selector)
        3. Append to robot URDF tree
        4. Publish updated robot_description
        5. Generate department TF frames
        6. Track box as active
        """

        address = req.address  # {side, cab_num, row, col, dept}

        # 1. Generate unique box ID
        box_id = self.generate_box_id()

        # 2. Get box configuration
        cabinet_size = self.get_cabinet_size(address.side, address.cab_num)
        num_cols, num_rows, num_depts = self.parse_size(cabinet_size)

        # 3. Generate box URDF snippet
        box_urdf_snippet = self.generate_box_urdf_snippet(
            box_id=box_id,
            num_cols=num_cols,
            num_rows=num_rows,
            num_depts=num_depts,
            parent_link="selector_link"  # Attach directly to manipulator selector
        )

        # 4. Append box to URDF tree
        urdf_tree = ET.fromstring(self.base_urdf_str)
        for element in box_urdf_snippet:
            urdf_tree.append(element)

        # 5. Publish updated robot_description
        updated_urdf_str = ET.tostring(urdf_tree, encoding='unicode')
        rospy.set_param('/robot_description', updated_urdf_str)

        # 6. Generate department frames (TF broadcast)
        self.generate_department_frames(box_id, num_depts)

        # 7. Track box as active
        self.active_boxes[box_id] = {
            'address': address,
            'size': cabinet_size,
            'state': 'attached_to_selector'
        }

        return ExtractBoxResponse(success=True, box_id=box_id)

    def generate_box_urdf_snippet(self, box_id, num_cols, num_rows, num_depts, parent_link):
        """
        Generate URDF XML elements for a box.

        Returns: List of XML elements [<link>, <joint>]
        """

        box_config = self.get_box_config(num_cols, num_rows)

        # Create box link
        box_link = ET.Element('link', name=f"box_{box_id}")

        # Visual geometry
        visual = ET.SubElement(box_link, 'visual')
        visual_geom = ET.SubElement(visual, 'geometry')
        ET.SubElement(visual_geom, 'box', size=f"{box_config['width']} {box_config['depth']} {box_config['height']}")

        # Collision geometry
        collision = ET.SubElement(box_link, 'collision')
        collision_geom = ET.SubElement(collision, 'geometry')
        ET.SubElement(collision_geom, 'box', size=f"{box_config['width']} {box_config['depth']} {box_config['height']}")

        # Create fixed joint to parent (selector_link)
        box_joint = ET.Element('joint', name=f"box_{box_id}_to_selector", type="fixed")
        ET.SubElement(box_joint, 'parent', link=parent_link)
        ET.SubElement(box_joint, 'child', link=f"box_{box_id}")
        ET.SubElement(box_joint, 'origin', xyz="0 0 0", rpy="0 0 0")

        return [box_link, box_joint]

    def generate_department_frames(self, box_id, num_depts):
        """
        Generate TF frames for all departments in box.

        Broadcasts static transforms from box link to department frames.
        """

        dept_config = self.config['department_configurations'][f'departments_{num_depts}']

        transforms = []
        for d in range(1, num_depts + 1):
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = f"box_{box_id}"
            t.child_frame_id = f"box_{box_id}_dept{d}"

            # Department position relative to box
            t.transform.translation.x = 0.0
            t.transform.translation.y = dept_config['offset_y'] + (d - 1) * dept_config['step_y']
            t.transform.translation.z = dept_config['height_offset']

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            transforms.append(t)

        self.tf_broadcaster.sendTransform(transforms)

    def handle_return_box(self, req):
        """
        Service handler: Return box to cabinet.

        Steps:
        1. Remove box links/joints from URDF
        2. Publish updated robot_description
        3. Remove department TF frames
        4. Remove from active boxes tracking
        """

        box_id = req.box_id

        # 1. Reload base URDF (without the box)
        urdf_tree = ET.fromstring(self.base_urdf_str)

        # Remove any other active boxes and re-add them
        for active_box_id in self.active_boxes:
            if active_box_id != box_id:
                box_info = self.active_boxes[active_box_id]
                num_cols, num_rows, num_depts = self.parse_size(box_info['size'])
                box_snippet = self.generate_box_urdf_snippet(
                    active_box_id, num_cols, num_rows, num_depts, "selector_link"
                )
                for element in box_snippet:
                    urdf_tree.append(element)

        # 2. Publish updated URDF
        updated_urdf_str = ET.tostring(urdf_tree, encoding='unicode')
        rospy.set_param('/robot_description', updated_urdf_str)

        # 3. Remove department frames (handled by TF timeout or explicit removal)
        # Note: Static TF broadcaster doesn't have explicit removal,
        # but frames will disappear when node stops publishing them

        # 4. Remove from tracking
        del self.active_boxes[box_id]

        return ReturnBoxResponse(success=True)
```

### 11.4 Key Implementation Details

**URDF Modification Strategy:**
- **Base URDF**: Contains only cabinets + placement frames (static structure from v2.2)
- **Runtime additions**: Box links/joints added dynamically when extracted
- **Parameter server**: `/robot_description` updated whenever boxes are added/removed
- **Robot state publisher**: Automatically picks up changes and publishes updated TF tree

**Department Frame Strategy:**
- Use **TF2 StaticTransformBroadcaster** for department frames
- Frames are children of `box_{id}` link
- Automatically follow box movement (via TF tree)
- Clean up when box is returned to cabinet

**Box Attachment:**
- Box link created with **fixed joint** to `selector_link`
- When selector moves, box moves with it
- No physics simulation needed (deterministic manipulator)

### 11.5 Integration with Manipulator Control

```python
# In manipulator controller node

def extract_box_sequence(self, address):
    """
    Full sequence to extract a box from cabinet.
    """

    # 1. Move selector to box placement frame
    placement_frame = f"{address.side}_cab{address.cab_num}_box_placement_{address.row}_{address.col}"
    self.move_selector_to_frame(placement_frame)

    # 2. Call storage manager to spawn box
    resp = self.extract_box_service(address)
    box_id = resp.box_id

    # 3. Box is now attached to selector (via fixed joint)
    # Manipulator can move freely, box follows

    # 4. Access department (gripper operates on department frame)
    dept_frame = f"box_{box_id}_dept{address.dept}"
    self.move_gripper_to_frame(dept_frame)

    # 5. Perform pick/place operation
    self.gripper_pick()

    # 6. Return box to cabinet
    self.move_selector_to_frame(placement_frame)
    self.return_box_service(box_id)
```

**Key Point for v2.2**: We generate the **static URDF structure** (cabinets + placement frames). Box spawning is a **runtime operation** performed by the `storage_manager_node` through dynamic URDF modification and TF broadcasting.

---

## 12. Summary of Changes in v2.2

### What Changed from v2.1
1. **Wall thickness corrected**: Changed from 0.2m to 0.02m (20mm)
2. **Interior dimensions recalculated**: Based on corrected wall thickness
3. **Box addressing clarified**: Always from manipulator's perspective (left-to-right, top-to-bottom)
4. **Right row column addressing**: Explicitly noted as decreasing X but numbered left-to-right from view
5. **Runtime scope clarified**: Section 11 marked as conceptual; v2.2 focuses on static URDF generation
6. **Pseudocode corrected**: Replaced non-existent `update_joint_parent()` with Gazebo service calls

### What Changed from v2.0 to v2.1
1. **Box row addressing formula** changed from `(r - 1)` to `(M - r)` to support top-to-bottom addressing
2. **Documentation updated** throughout to clarify Row 1 = top, Row M = bottom
3. **Example calculations updated** to reflect correct Z coordinates
4. **Xacro macros corrected** to use `(max_rows - current_row)` formula

### Why This Matters
- **Correct physical dimensions**: 20mm walls instead of 200mm
- **Intuitive addressing**: Row 1 at top, columns left-to-right from manipulator view
- **Consistent with physical layout**: Matches how humans number shelves in furniture
- **Clear scope**: URDF generation first, runtime behavior later
- **Implementable**: Uses real Gazebo services instead of imaginary functions

### Development Focus for v2.2
**Priority: Static URDF/Xacro Generation**
- Cabinet rows with correct positioning
- Box placement frames in each cabinet
- Correct coordinate calculations
- YAML configuration structure

**Out of Scope (Future Work)**:
- Runtime box spawning
- Gazebo integration
- State persistence
- Collision detection

---

## 13. Quick Reference

### Address Format
```
{side, cabinet_num, box_row, box_col, dept_num}
```

### Size Format
```
{cols}x{rows}x{depts}
```

### Link Name Format
```
{side}_cab{N}_box_placement_{R}_{C}
box_{id}_dept{D}
```

### Global Coordinate Formula (v2.2 CORRECTED)
```
# For box in cabinet (LEFT ROW):
box_x = row_offset_x + (cab_num-1)*0.7 + box_offset_x + (col-1)*box_step_x
box_y = row_offset_y
box_z = row_offset_z + box_offset_z + (num_rows - row)*box_step_z

# For box in cabinet (RIGHT ROW):
# Cabinet is rotated 180°, columns still numbered left-to-right from manipulator view
cabinet_x = row_offset_x + cab_num * 0.7  # +0.7 shift due to rotation
box_x = cabinet_x - (box_offset_x + (col-1)*box_step_x)  # Subtract due to 180° rotation
box_y = row_offset_y
box_z = row_offset_z + box_offset_z + (num_rows - row)*box_step_z

# For department:
dept_x = box_x
dept_y = box_y + dept_offset_y + (dept-1)*dept_step_y
dept_z = box_z + box_height
```

---

**End of Specification (v2.2 - URDF Generation Focus)**
