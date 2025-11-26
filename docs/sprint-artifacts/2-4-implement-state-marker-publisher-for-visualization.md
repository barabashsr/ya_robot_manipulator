# Story 2.4: Implement State Marker Publisher for Visualization

Status: done

## Story

As a developer,
I want visual markers in RViz showing system state (magnet engaged, target address, extracted addresses),
So that I can observe system behavior during testing and debugging.

## Acceptance Criteria

1. **Marker Publication (AC1):** The node publishes MarkerArray messages to `/visualization_marker_array` at 10 Hz
2. **Magnet Engaged Marker (AC2):** Red sphere (0.05m diameter, 80% opacity) attached to gripper_magnet_link frame, visible only when `/manipulator/electromagnet/engaged` is TRUE
3. **Target Address Marker (AC3):** Green semi-transparent cube (50% opacity) at target address TF frame, size matches box dimensions from storage_params.yaml, visible when NavigateToAddress or ExtractBox action is active
4. **Extracted Address Markers (AC4):** Red semi-transparent cubes (50% opacity) at addresses with extracted boxes, persist until ReturnBox or PutBox action completes
5. **Namespace (AC5):** Marker namespace is "manipulator_state" for filtering in RViz
6. **RViz Visibility (AC6):** Markers are visible in RViz Displays → MarkerArray
7. **State Subscription (AC7):** Node subscribes to `/manipulator/state` topic for system state updates

## Tasks / Subtasks

- [x] Task 1: Create state_markers.yaml configuration file (AC: 1, 5)
  - [x] 1.1 Create `manipulator_control/config/state_markers.yaml` with marker parameters
  - [x] 1.2 Define marker_namespace, update_rate, colors (RGB + alpha), sizes
  - [x] 1.3 Reference box dimensions from storage_params.yaml (DO NOT duplicate)

- [x] Task 2: Implement StateMarkerPublisher node (AC: 1, 5, 6, 7)
  - [x] 2.1 Create `manipulator_control/src/state_marker_publisher.py`
  - [x] 2.2 Initialize node with 10 Hz timer for marker publication
  - [x] 2.3 Subscribe to `/manipulator/state` topic (or create interim state tracking)
  - [x] 2.4 Create publisher for `/visualization_marker_array` (MarkerArray)
  - [x] 2.5 Load marker configuration from state_markers.yaml
  - [x] 2.6 Load box dimensions from storage_params.yaml for cube marker sizes

- [x] Task 3: Implement Magnet Engaged Marker (AC: 2)
  - [x] 3.1 Subscribe to `/manipulator/electromagnet/engaged` (Bool)
  - [x] 3.2 Create red sphere marker (Marker.SPHERE) with 0.05m diameter
  - [x] 3.3 Set frame_id to `gripper_magnet_link` (or `selector_frame_gripper_link`)
  - [x] 3.4 Set alpha to 0.8 (80% opacity)
  - [x] 3.5 Add/delete marker based on electromagnet state

- [x] Task 4: Implement Target Address Marker (AC: 3)
  - [x] 4.1 Create mechanism to receive target address (parameter, topic, or service)
  - [x] 4.2 Create green cube marker (Marker.CUBE) with 50% opacity
  - [x] 4.3 Set frame_id to address TF frame (e.g., `addr_l_1_2_3`)
  - [x] 4.4 Size marker using box dimensions from storage_params.yaml
  - [x] 4.5 Show marker when navigation/extraction active, hide when complete

- [x] Task 5: Implement Extracted Address Markers (AC: 4)
  - [x] 5.1 Track list of extracted addresses (internal state)
  - [x] 5.2 Create red cube markers (Marker.CUBE) with 50% opacity for each
  - [x] 5.3 Position markers at address TF frames
  - [x] 5.4 Manage marker lifecycle: add on ExtractBox, remove on ReturnBox/PutBox

- [x] Task 6: Update launch file (AC: 1)
  - [x] 6.1 Add state_marker_publisher node to `manipulator_simulation.launch.py`
  - [x] 6.2 Configure as Common node (no condition - runs in both sim and hardware)
  - [x] 6.3 Use 3s delayed start with TimerAction

- [x] Task 7: Create entry point and package updates (AC: 1)
  - [x] 7.1 Add install rule in CMakeLists.txt (ament_cmake package)
  - [x] 7.2 Update package.xml with visualization_msgs dependency
  - [x] 7.3 Build and verify node launches without errors

- [x] Task 8: Manual testing and verification (AC: 1-7)
  - [x] 8.1 Launch Gazebo simulation with state_marker_publisher
  - [x] 8.2 Open RViz and add MarkerArray display for `/visualization_marker_array`
  - [x] 8.3 Manually trigger electromagnet and verify magnet marker appears/disappears
  - [x] 8.4 Set target address and verify green cube marker at correct position
  - [x] 8.5 Simulate extraction and verify red cube markers persist
  - [x] 8.6 **MANDATORY: Developer must personally verify all markers visible in RViz**

## Dev Notes

### Relevant Architecture Patterns

- Reference architecture lines 880-1007 for marker implementation details
- Marker types: Marker.SPHERE for magnet, Marker.CUBE for addresses
- Frame references: `gripper_magnet_link` (or `selector_frame_gripper_link`), `addr_l_1_2_3` format for addresses
- Color scheme: Red (1.0, 0.0, 0.0), Green (0.0, 1.0, 0.0) with alpha for transparency
- Department markers will be added in Epic 5 (Item Picking)

### Configuration Reuse Policy

- **USE:** `manipulator_description/config/storage_params.yaml` - box dimensions for marker sizes
- **USE:** TF frames from URDF (addr_l_1_2_3 format) - address positions
- **CREATE:** `config/state_markers.yaml` - NEW file for marker-specific parameters only
- **DO NOT DUPLICATE:** Box dimensions or address positions - load from existing files

### Source Tree Components

| File | Change Type | Description |
|------|-------------|-------------|
| `manipulator_control/src/state_marker_publisher.py` | New | Main node implementation |
| `manipulator_control/config/state_markers.yaml` | New | Marker visualization parameters |
| `manipulator_control/launch/manipulator_simulation.launch.py` | Modify | Add node to launch |
| `manipulator_control/setup.py` | Modify | Add entry point |

### Launch File Pattern

```python
# Common node (no condition - runs in both sim and hardware)
state_marker_publisher_node = TimerAction(
    period=3.0,
    actions=[
        Node(
            package='manipulator_control',
            executable='state_marker_publisher.py',
            name='state_marker_publisher',
            parameters=[{'use_sim_time': use_sim_time}]
            # No condition - runs in both sim and hardware
        )
    ]
)
```

### Marker Message Structure

```python
from visualization_msgs.msg import Marker, MarkerArray

# Magnet marker
magnet_marker = Marker()
magnet_marker.header.frame_id = "gripper_magnet_link"
magnet_marker.ns = "manipulator_state"
magnet_marker.id = 0
magnet_marker.type = Marker.SPHERE
magnet_marker.action = Marker.ADD  # or Marker.DELETE
magnet_marker.scale.x = magnet_marker.scale.y = magnet_marker.scale.z = 0.05
magnet_marker.color.r = 1.0
magnet_marker.color.a = 0.8

# Address marker
address_marker = Marker()
address_marker.header.frame_id = "addr_l_1_2_3"  # TF frame
address_marker.ns = "manipulator_state"
address_marker.id = 1  # unique per address
address_marker.type = Marker.CUBE
address_marker.action = Marker.ADD
# Size from storage_params.yaml
address_marker.color.g = 1.0  # Green for target
address_marker.color.a = 0.5  # 50% opacity
```

### State Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/manipulator/electromagnet/engaged` | std_msgs/Bool | Magnet state for sphere marker |
| `/manipulator/state` | (custom or placeholder) | Target address, extracted addresses |
| `/visualization_marker_array` | visualization_msgs/MarkerArray | Published markers |

### Testing Standards

- Node must publish at 10 Hz consistently
- Markers must be positioned correctly using TF frames
- Color and opacity must match specification
- Namespace filtering must work in RViz

### Project Structure Notes

- Alignment with unified project structure: Node in `src/`, config in `config/`
- Entry point pattern matches existing nodes (e.g., virtual_limit_switches)
- Launch integration follows tech-spec pattern for Common nodes

### References

- [Source: docs/epics.md#Story 2.4]
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#Detailed Design]
- [Source: docs/prd.md#FR-012: Visual State Markers]
- [Source: manipulator_description/config/storage_params.yaml - box dimensions]

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/2-4-implement-state-marker-publisher-for-visualization.context.xml

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

None

### Completion Notes List

- All 7 ACs verified via topic-level testing and RViz visual confirmation
- Node publishes MarkerArray at 10 Hz
- Magnet marker: red sphere (0.05m, 80% opacity) at left_gripper_magnet frame
- Target marker: green cube (50% opacity) at address TF frame with box dimensions from storage_params.yaml
- Extracted markers: red cubes (50% opacity) at extracted address TF frames
- Namespace "manipulator_state" for RViz filtering

### File List

- `ros2_ws/src/manipulator_control/config/state_markers.yaml` (new)
- `ros2_ws/src/manipulator_control/src/state_marker_publisher.py` (new)
- `ros2_ws/src/manipulator_control/launch/manipulator_simulation.launch.py` (modified)
- `ros2_ws/src/manipulator_control/CMakeLists.txt` (modified)
- `ros2_ws/src/manipulator_control/package.xml` (modified)

