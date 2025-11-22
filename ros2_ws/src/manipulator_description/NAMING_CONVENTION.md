# URDF Link Naming Convention

## Overview

This document describes the simplified naming scheme for all TF frames in the manipulator system.

## Frame Naming Scheme

### 1. Side Frames
Base frames for left and right cabinet rows:
- **`side_l`** - Left cabinet row base
- **`side_r`** - Right cabinet row base

### 2. Cabinet Frames
Individual cabinet frames:
- **`cab_{side}_{num}`**
  - `side`: `l` (left) or `r` (right)
  - `num`: Cabinet number (1-indexed)

**Examples:**
- `cab_l_1` - First cabinet on left side
- `cab_r_2` - Second cabinet on right side

### 3. Address Frames (Box Placement)
Static frames where boxes can be placed:
- **`addr_{side}_{cab}_{row}_{col}`**
  - `side`: `l` (left) or `r` (right)
  - `cab`: Cabinet number (1-indexed)
  - `row`: Row number (1 = top, M = bottom)
  - `col`: Column number (1 = leftmost from manipulator view)

**Examples:**
- `addr_l_1_2_3` - Left row, cabinet 1, row 2, column 3
- `addr_r_3_1_4` - Right row, cabinet 3, row 1 (top), column 4

### 4. Box Frames (Runtime)
Dynamically created when boxes are extracted:
- **`box_{box_id}`**
  - `box_id`: Unique physical box identifier (from QR code)

**Examples:**
- `box_ABC123` - Physical box with ID ABC123
- `box_XYZ789` - Physical box with ID XYZ789

**Note:** Box IDs are independent of cabinet addresses. The mapping from `box_id` to cabinet address is managed by upper-layer software (database).

### 5. Department Frames (Runtime)
Frames for individual departments within an extracted box:
- **`{box_id}_{dept_num}`**
  - `box_id`: Physical box identifier
  - `dept_num`: Department number within box (1-indexed)

**Examples:**
- `ABC123_7` - Department 7 in box ABC123
- `XYZ789_3` - Department 3 in box XYZ789

## Frame Hierarchy

```
base_link
├── side_l (left row)
│   ├── cab_l_1 (first cabinet)
│   │   ├── addr_l_1_1_1 (row 1, col 1)
│   │   ├── addr_l_1_1_2 (row 1, col 2)
│   │   └── ...
│   ├── cab_l_2 (second cabinet)
│   └── ...
├── side_r (right row)
│   ├── cab_r_1
│   │   ├── addr_r_1_1_1
│   │   └── ...
│   └── ...
└── [Runtime boxes]
    ├── box_ABC123 (extracted box)
    │   ├── ABC123_1 (department 1)
    │   ├── ABC123_2 (department 2)
    │   └── ...
    └── box_XYZ789
        └── ...
```

## Lookup Examples

### Find Address Frame
To find the frame for a specific box position:
```bash
# Left row, cabinet 2, row 3, column 4
ros2 run tf2_ros tf2_echo base_link addr_l_2_3_4
```

### Find Department in Active Box
```bash
# Department 7 in box ABC123
ros2 run tf2_ros tf2_echo base_link ABC123_7
```

### List All Address Frames
```bash
ros2 topic echo /tf_static | grep addr_
```

## Database Schema (Conceptual)

The upper-layer software maintains the box-to-address mapping:

```sql
CREATE TABLE box_locations (
    box_id VARCHAR(50) PRIMARY KEY,  -- Physical box ID from QR
    side CHAR(1),                    -- 'l' or 'r'
    cabinet INT,                     -- Cabinet number
    row INT,                         -- Row number
    col INT,                         -- Column number
    last_updated TIMESTAMP
);
```

**Example entries:**
```
box_id    | side | cabinet | row | col | last_updated
----------|------|---------|-----|-----|-------------
ABC123    | l    | 2       | 3   | 4   | 2025-11-22
XYZ789    | r    | 1       | 1   | 2   | 2025-11-22
```

## Benefits of This Naming Scheme

1. **Simple & Concise**: Short names, easy to type and remember
2. **Easy Lookup**: Address format directly corresponds to physical location
3. **Scannable**: Can use wildcards to find groups (`addr_l_*`, `ABC123_*`)
4. **QR Code Ready**: Box IDs match physical labels
5. **Database Friendly**: Clean mapping from box ID to address
6. **Collision-Free**: Box IDs (alphanumeric) won't collide with address frames (numeric)

## Implementation Status

- [x] Side frames (`side_l`, `side_r`)
- [x] Cabinet frames (`cab_l_1`, etc.)
- [x] Address frames (`addr_l_1_2_3`, etc.)
- [ ] Box frames (runtime - to be implemented in storage_manager_node)
- [ ] Department frames (runtime - to be implemented in storage_manager_node)
