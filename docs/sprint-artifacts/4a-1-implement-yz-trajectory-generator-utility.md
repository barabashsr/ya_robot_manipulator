# Story 4A.1: Implement YZ Trajectory Generator Utility (Parametric Curves)

Status: Approved

## Story

As a developer,
I want a utility to load and execute parametric curve-based YZ trajectories for box insertion/extraction,
So that gripper motion follows smooth, editable curves during box operations.

## Core Concepts

### What This Story Delivers

**Story 4A-1 establishes the trajectory generation foundation for all box handling operations in Epic 4A:**

1. **SVG Source Files** - Editable curve definitions:
   - Design Bezier curves visually in Inkscape
   - SVG paths define insertion/extraction motion profiles
   - Easy to adjust curve shape without code changes
   - Version controlled alongside configuration

2. **SVG-to-YAML Converter** - Development tool:
   - `scripts/svg_to_trajectory.py` converts SVG paths to YAML waypoints
   - Samples Bezier curves at configurable resolution (default 20 points)
   - Maps SVG coordinates to joint space (Y/Z positions in meters)
   - One-time conversion - YAML committed to repo, no runtime SVG dependency

3. **Runtime Trajectory Loader** - Core utility class:
   - `YZTrajectoryGenerator` loads waypoints from YAML
   - Transforms waypoints for cabinet side (left/right sign flip)
   - Applies base position offsets from current gripper position
   - Calculates timing (`time_from_start`) for each waypoint

4. **JointTrajectoryController Integration** - Smooth execution:
   - Builds `JointTrajectory` messages from transformed waypoints
   - Leverages controller's built-in spline interpolation
   - No waypoint-by-waypoint commanding - single trajectory execution
   - Returns success/failure status

### Why Parametric Curves (Not Collision Checking)

- **Predictable**: Same curve shape every time, no runtime variance
- **Editable**: Visual design in Inkscape, adjust curves without code
- **Efficient**: No collision detection computation at runtime
- **Smooth**: JointTrajectoryController interpolates between waypoints
- **Testable**: Known trajectory shape makes validation straightforward

### Trajectory Coordinate System

**SVG files are unitless** - scaling is defined in external config file `trajectory_config.yaml`:

```yaml
# config/trajectory_config.yaml
trajectories:
  extract_left:
    svg_file: trajectories/extract_left.svg
    mapping:
      x_range: [0, 100]        # SVG X range
      y_output: [0.0, 0.4]     # Maps to joint Y (meters)
      y_center: 50             # SVG Y value for Z=0
      z_scale: 0.001           # SVG Y deviation to meters (1 unit = 1mm)
    sampling:
      num_points: 20           # Waypoints to generate
      waypoint_duration: 0.5   # Seconds between waypoints
```

**Conversion Flow:**
```
SVG Space                     Config Mapping                Joint Space
┌─────────────────┐           ┌─────────────────┐           ┌─────────────────┐
│  X: 0 → 100     │  ──────▶  │  x_range, y_out │  ──────▶  │  Y: 0.0 → 0.4m  │
│  Y: 50 = center │           │  y_center, z_sc │           │  Z: offset ±mm  │
└─────────────────┘           └─────────────────┘           └─────────────────┘
```

**Benefits of External Config:**
- SVG designer doesn't need to know magic scale numbers
- Same SVG can be re-scaled without editing
- Clear separation: SVG = shape, Config = units

### Integration in Box Operations

```
ExtractBox Action:
├── Phase 2 (Approach):
│   └── load_trajectory("insertion", side, base_y, base_z)
│       └── execute via JointTrajectoryController
│
└── Phase 5 (Extract):
    └── load_trajectory("extraction", side, base_y, base_z)
        └── execute via JointTrajectoryController

ReturnBox Action:
├── Phase 2 (Insert):
│   └── load_trajectory("insertion", ...)
│
└── Phase 4 (Retract):
    └── load_trajectory("extraction", ...)
```

### File Structure

```
manipulator_control/
├── config/
│   ├── trajectories/
│   │   ├── extract_left.svg           # SVG source (editable in Inkscape)
│   │   └── extract_right.svg          # SVG source (optional, can mirror left)
│   ├── trajectory_config.yaml         # Scaling config (SVG → real units)
│   └── extraction_trajectories.yaml   # Generated waypoints (committed)
├── scripts/
│   └── svg_to_trajectory.py           # Converter tool (reads config)
└── src/
    └── utils/
        └── yz_trajectory_generator.py # Runtime loader
```

## Acceptance Criteria

1. **External Config File (AC1):** `trajectory_config.yaml` exists with mapping parameters (x_range, y_output, y_center, z_scale) and sampling parameters (num_points, waypoint_duration) for each trajectory
2. **SVG Converter (AC2):** Given an SVG file and config entry, when `scripts/svg_to_trajectory.py` is run, then YAML waypoints file is generated using scaling from config (not hardcoded)
3. **YAML Output Format (AC3):** Generated YAML contains `source_svg` reference, `config_used` reference, and `trajectories` dict with path IDs as keys and list of `{y, z}` waypoints
4. **Waypoint Loading (AC4):** Given trajectory name and side, when `load_trajectory()` is called, then waypoints are loaded from YAML and transformed with correct sign flip (right side = negative Y)
5. **Base Position Offset (AC5):** Loaded waypoints have base Y/Z positions added as offsets to relative curve positions
6. **Timing Calculation (AC6):** Loaded waypoints include `time_from_start` calculated from `waypoint_duration` in config
7. **Trajectory Execution (AC7):** Given transformed waypoints, when `execute_trajectory()` is called, then JointTrajectoryController executes smooth motion and method returns success/failure
8. **SVG Source Files (AC8):** SVG source files exist for extraction with paths named "insertion" and "extraction"
9. **Gazebo Validation (AC9):** Trajectories tested in Gazebo produce smooth motion without collisions for 10 different addresses

## Tasks / Subtasks

- [ ] Task 1: Create trajectory config file (AC: 1)
  - [ ] 1.1 Create `config/trajectory_config.yaml`
  - [ ] 1.2 Define `extract_left` entry with:
    - `svg_file: trajectories/extract_left.svg`
    - `mapping.x_range: [0, 100]`
    - `mapping.y_output: [0.0, 0.4]` (meters)
    - `mapping.y_center: 50`
    - `mapping.z_scale: 0.001` (1mm per SVG unit)
    - `sampling.num_points: 20`
    - `sampling.waypoint_duration: 0.5`
  - [ ] 1.3 Add comments explaining each parameter

- [ ] Task 2: Create SVG source files (AC: 8)
  - [ ] 2.1 Create `config/trajectories/` directory structure
  - [ ] 2.2 Create `extract_left.svg` with viewBox="0 0 100 100"
  - [ ] 2.3 Add insertion path: `<path id="insertion" d="M 0,50 C 30,50 70,48 100,50"/>`
  - [ ] 2.4 Add extraction path: `<path id="extraction" d="M 100,50 C 70,52 30,52 0,50"/>`
  - [ ] 2.5 Add XML comments (SVG is unitless, see trajectory_config.yaml for scaling)
  - [ ] 2.6 Test SVG renders correctly in browser/Inkscape

- [ ] Task 3: Implement SVG-to-YAML converter (AC: 2, 3)
  - [ ] 3.1 Create `scripts/svg_to_trajectory.py`
  - [ ] 3.2 Add argparse for: `--config` (trajectory_config.yaml), `--trajectory` (e.g., extract_left), `--output`
  - [ ] 3.3 Load config and extract mapping/sampling for specified trajectory
  - [ ] 3.4 Use `svgpathtools` to parse SVG file from config
  - [ ] 3.5 Sample each path at `num_points` using `path.point(t)` for t in 0..1
  - [ ] 3.6 Apply scaling from config:
    - `y = (point.real - x_range[0]) / (x_range[1] - x_range[0]) * (y_output[1] - y_output[0]) + y_output[0]`
    - `z = (y_center - point.imag) * z_scale`
  - [ ] 3.7 Output YAML with `source_svg`, `config_used`, and `trajectories` dict
  - [ ] 3.8 Add shebang and make executable

- [ ] Task 4: Generate initial trajectory YAML (AC: 3)
  - [ ] 4.1 Install svgpathtools: `pip install svgpathtools`
  - [ ] 4.2 Run converter: `python3 scripts/svg_to_trajectory.py --config config/trajectory_config.yaml --trajectory extract_left -o config/extraction_trajectories.yaml`
  - [ ] 4.3 Verify YAML contains insertion and extraction trajectories with correct scaling
  - [ ] 4.4 Commit generated YAML to repository

- [ ] Task 5: Implement YZTrajectoryGenerator class (AC: 4, 5, 6)
  - [ ] 5.1 Create `src/utils/yz_trajectory_generator.py`
  - [ ] 5.2 Implement `__init__(waypoints_path, config_path)` to load both files
  - [ ] 5.3 Implement `load_trajectory(name, side, base_y, base_z)`:
    - Load waypoints from self.trajectories[name]
    - Get waypoint_duration from config
    - Apply sign = 1.0 if left else -1.0
    - Transform: y = base_y + sign * wp['y'], z = base_z + wp['z']
    - Add time_from_start = i * waypoint_duration
  - [ ] 5.4 Add type hints and docstrings
  - [ ] 5.5 Add logging for trajectory loading

- [ ] Task 6: Implement trajectory execution method (AC: 7)
  - [ ] 6.1 Implement `execute_trajectory(waypoints, trajectory_client)`:
    - Build JointTrajectory message
    - Set joint_names: [selector_frame_gripper_joint, main_frame_selector_frame_joint]
    - Create JointTrajectoryPoint for each waypoint
    - Set positions = [wp['y'], wp['z']]
    - Set time_from_start from waypoint
  - [ ] 6.2 Create FollowJointTrajectory.Goal and send via action client
  - [ ] 6.3 Wait for result and return success/failure boolean
  - [ ] 6.4 Add error handling for timeout and rejected goals

- [ ] Task 7: Create unit tests for converter (AC: 2, 3)
  - [ ] 7.1 Create `test/test_svg_to_trajectory.py`
  - [ ] 7.2 Test: Converter reads scaling from config (not hardcoded)
  - [ ] 7.3 Test: SVG with multiple paths produces multiple trajectories
  - [ ] 7.4 Test: Y/Z scaling matches config values
  - [ ] 7.5 Test: Output YAML includes config_used reference

- [ ] Task 8: Create unit tests for trajectory loader (AC: 4, 5, 6)
  - [ ] 8.1 Create `test/test_yz_trajectory_generator.py`
  - [ ] 8.2 Test: load_trajectory returns correct waypoint count
  - [ ] 8.3 Test: left side produces positive Y values
  - [ ] 8.4 Test: right side produces negative Y values (sign flip)
  - [ ] 8.5 Test: base position offsets applied correctly
  - [ ] 8.6 Test: time_from_start uses waypoint_duration from config

- [ ] Task 9: Gazebo integration testing (AC: 9)
  - [ ] 9.1 Launch simulation with trajectory controller running
  - [ ] 9.2 Create test script that:
    - Initializes YZTrajectoryGenerator with config
    - Creates FollowJointTrajectory action client
    - Loads insertion trajectory for left cabinet
    - Executes trajectory
    - Verifies smooth motion (no jerks, no collisions)
  - [ ] 9.3 Test with 10 different base positions (simulating different addresses)
  - [ ] 9.4 Verify all executions complete successfully
  - [ ] 9.5 Document test results in Dev Agent Record

## Dev Notes

### SVG Design Guidelines

**Path Structure:**
- Use cubic Bezier curves for smooth motion
- `M x,y` = Move to start point
- `C x1,y1 x2,y2 x,y` = Cubic Bezier to end point with control points

**Example Insertion Path (gripper enters cabinet):**
```svg
<path id="insertion" d="M 0,50 C 30,50 70,48 100,50"/>
```
- Start: (0, 50) = gripper at center, no Z offset
- Control 1: (30, 50) = move Y, maintain Z
- Control 2: (70, 48) = approach target, slight Z dip for clearance
- End: (100, 50) = full insertion depth, return to base Z

**Example Extraction Path (gripper exits cabinet with box):**
```svg
<path id="extraction" d="M 100,50 C 70,52 30,52 0,50"/>
```
- Start: (100, 50) = gripper at insertion depth
- Control 1: (70, 52) = begin retract, slight Z lift
- Control 2: (30, 52) = continue retract, maintain lift
- End: (0, 50) = fully extracted, return to base Z

### Joint Mapping

| Waypoint Field | Joint Name | Axis | Purpose |
|----------------|------------|------|---------|
| y | selector_frame_gripper_joint | Y | Into/out of cabinet |
| z | main_frame_selector_frame_joint | Z | Vertical offset for clearance |

### Configuration File Format

```yaml
# config/trajectory_config.yaml
trajectories:
  extract_left:
    svg_file: trajectories/extract_left.svg
    mapping:
      # SVG X-axis mapping to joint Y (gripper depth)
      x_range: [0, 100]        # SVG X coordinate range
      y_output: [0.0, 0.4]     # Joint Y output range (meters)

      # SVG Y-axis mapping to joint Z (vertical offset)
      y_center: 50             # SVG Y value that maps to Z=0
      z_scale: 0.001           # Meters per SVG unit (1 unit = 1mm)

    sampling:
      num_points: 20           # Waypoints to generate from curve
      waypoint_duration: 0.5   # Seconds between waypoints

  extract_right:
    svg_file: trajectories/extract_right.svg  # Or same as left, mirrored at runtime
    mapping:
      x_range: [0, 100]
      y_output: [0.0, 0.4]
      y_center: 50
      z_scale: 0.001
    sampling:
      num_points: 20
      waypoint_duration: 0.5
```

**Generated Waypoints YAML:**
```yaml
# config/extraction_trajectories.yaml (generated, committed)
source_svg: trajectories/extract_left.svg
config_used: trajectory_config.yaml
generated: "2025-11-27T12:00:00"
trajectories:
  insertion:
    - {y: 0.0, z: 0.0}
    - {y: 0.021, z: 0.0}
    - {y: 0.084, z: 0.001}
    # ... more waypoints
  extraction:
    - {y: 0.4, z: 0.0}
    - {y: 0.379, z: -0.002}
    # ... more waypoints
```

### Dependencies

**Development (converter only):**
- `svgpathtools` - SVG path parsing (pip install svgpathtools)
- `numpy` - Linspace for sampling
- `pyyaml` - YAML output

**Runtime:**
- `pyyaml` - YAML loading
- `trajectory_msgs` - JointTrajectory, JointTrajectoryPoint
- `control_msgs` - FollowJointTrajectory action
- `rclpy` - Action client

### Learnings from Previous Story

**From Story 3-6 (Status: review)**

- **Test Framework Established**: Epic 3 test suite at `test/test_epic3_navigation.py` demonstrates ROS2 action client testing patterns
- **Action Client Pattern**: Use `send_goal_async()` + `spin_until_future_complete()` for non-blocking goal handling
- **NFR Compliance**: Position accuracy < 0.02m requirement validated - trajectory execution must maintain this
- **Simulation Timing**: Allow ~30 seconds for Gazebo + controllers to initialize before testing
- **Joint Limits**: Verified limits - selector_frame_gripper_joint: ±0.39m (covers 0.4m extraction depth)

[Source: docs/sprint-artifacts/3-6-create-navigation-test-suite.md#Dev-Agent-Record]

### Project Structure Notes

**New Files:**
- `config/trajectory_config.yaml` - Scaling configuration (SVG → real units)
- `config/trajectories/extract_left.svg` - SVG source (unitless curves)
- `config/extraction_trajectories.yaml` - Generated waypoints (committed)
- `scripts/svg_to_trajectory.py` - Converter tool (reads config)
- `src/utils/yz_trajectory_generator.py` - Runtime utility
- `test/test_svg_to_trajectory.py` - Converter tests
- `test/test_yz_trajectory_generator.py` - Loader tests

**Modified Files:**
- None expected (new utility, not modifying existing code)

### References

- [Source: docs/epics.md#Story 4A.1: Implement YZ Trajectory Generator Utility]
- [Source: docs/sprint-artifacts/tech-spec-epic-4a.md#YZ Trajectory Generation]
- [Source: docs/architecture-ros2-control-v2-CORRECTIONS.md#C. YZ Trajectory Generation]
- [Source: ros2_ws/src/manipulator_control/config/kinematic_chains.yaml]

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-11-27 | Initial draft | SM Agent (Bob) |

## Dev Agent Record

### Context Reference

- `docs/sprint-artifacts/4a-1-implement-yz-trajectory-generator-utility.context.xml`

### Agent Model Used

- claude-opus-4-5-20251101

### Debug Log References

### Completion Notes List

### File List
