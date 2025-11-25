# ya_robot_manipulator - Product Requirements Document

**Author:** BMad
**Date:** 2025-11-24
**Version:** 1.0
**Status:** Draft for Validation

---

## Executive Summary

The **ya_robot_manipulator** is an automated warehouse storage and retrieval system designed for **medical supply warehouses serving ambulance services**. As a Level 3 control system module, it operates autonomously within a larger warehouse automation ecosystem, receiving commands from a Level 2 warehouse management system and executing complex manipulation tasks without human intervention.

This product addresses the critical challenge of **high-density medical supply storage with precision item retrieval** in an environment where:
- **Reliability is paramount** - ambulance services depend on rapid, accurate medication access
- **Space is constrained** - medical facilities require maximum storage density
- **Automation is essential** - minimal human presence required for 24/7 operations
- **Traceability matters** - every item movement must be precise and verifiable

The system serves as a **proof-of-concept to validate control algorithms** before hardware deployment, ensuring that the autonomous manipulation logic is robust and reliable in simulation before commissioning physical equipment.

### What Makes This Special

**Autonomous Box and Item Manipulation in Dense Storage**

Unlike traditional warehouse systems that retrieve entire shelves or bins, ya_robot_manipulator provides **department-level precision** within boxes stored in a dense cabinet array. The system can:

1. **Extract boxes from precise addresses** in a complex (side, cabinet, row, column) coordinate system
2. **Pick individual items from departments within boxes** without requiring full box removal
3. **Dynamically relocate boxes** between addresses for storage optimization
4. **Manage temporary containers** for order fulfillment operations

This three-tier manipulation capability (cabinet → box → item) delivers warehouse density comparable to fixed-slot systems while maintaining the flexibility of dynamic storage allocation.

---

## Project Classification

**Technical Type:** Robotics Control System (ROS2)
**Domain:** Warehouse Automation / Industrial Robotics
**Complexity:** Level 3 (High - Complex domain with specialized hardware control)

### Project Characteristics

- **Multi-axis robotic manipulator** with 9 degrees of freedom
- **Individual joint controllers** (ForwardCommandController per joint, not JointTrajectoryController) enabling flexible single-joint and coordinated multi-joint commands
- **Precision addressing system** using TF frame-based coordinate resolution
- **State machine control** for picker operations using limit switches
- **Dynamic object spawning** for box/item simulation in Gazebo
- **Electromagnet-based grasping** for metal containers
- **Mechanical jaw gripper** for box handling
- **Real-time sensor feedback** from limit switches for state transitions

### Domain Context

**Medical Supply Warehouse Requirements:**
- **Mission-Critical Reliability:** Ambulance services depend on 24/7 medication access - system failures directly impact emergency response capabilities
- **Regulatory Traceability:** Every item movement must be logged and verifiable for medical inventory compliance
- **Zero-Error Picking:** Wrong medication delivery is unacceptable - department-level precision is mandatory
- **Autonomous Operation:** Minimal human presence - system must self-diagnose and report issues clearly
- **Deterministic Behavior:** Same command must produce same result - no probabilistic picking allowed

**Level 3 Control Module Role:**
- **Input:** High-level commands from Level 2 warehouse management system (e.g., "pick item from address addr_l_1_2_3_dept_5")
- **Output:** Execution status, error reporting, system health diagnostics
- **Autonomy:** Operates independently once commanded - no operator intervention during task execution
- **Integration:** REST/MQTT interface ready for Level 2 communication (future phase)

**Critical Level 2/Level 3 Responsibility Boundary:**
- **Level 3 does NOT maintain item location database** - this is Level 2's responsibility
- **Level 2 provides complete item addresses** - Level 3 executes manipulation at those addresses
- **Separation of concerns:** Level 3 focuses on control algorithms and manipulation, Level 2 focuses on inventory intelligence and optimization
- **Interface:** Level 2 sends "pick item from (side, cabinet, row, column, department)" - Level 3 executes without querying item locations

**Hardware Constraints:**
- Stepper motors **without encoders** (position approximated)
- **Limit switches** provide ground truth for critical positions
- Mechanical linkages have play/tolerance
- Real-world physics affect box extraction (friction, weight distribution)

**Proof-of-Concept Goal:**
This simulation-based development validates control algorithms before hardware deployment. Success means algorithms are robust enough to transfer to physical hardware with minimal adjustment.

---

## Success Criteria

### Technical Success Metrics

**Reliability:**
- ≥95% success rate for complete pick-from-storage operations in simulation
- ≥90% success rate when deployed on physical hardware (accounting for mechanical tolerances)
- Zero unrecoverable stuck states (all failures must be detectable and recoverable)

**Performance:**
- Complete pick operation (extract box → pick item → return box) in ≤120 seconds
- Address navigation accuracy within ±5mm of target position
- Department-level picking accuracy within ±2mm of target center

**Safety:**
- All limit switches monitored continuously during motion
- Emergency stop capability at any operation phase
- Safe abort procedures return system to known state

### Operational Success Metrics

**System Availability:**
- Control system ready state within 10 seconds of launch
- Graceful degradation if non-critical sensors fail
- Self-diagnostic capability reports system health status

**Integration Readiness:**
- Level 2 warehouse management system can issue pick commands via defined action interface
- Standard RQt tools sufficient for system monitoring and debugging
- Configuration changes (new cabinet layouts) require only YAML updates, no code changes

### Proof-of-Concept Success Metrics

**Algorithm Validation (Primary Goal):**
- ✅ **Simulation Reliability:** ≥95% success rate across 100+ test operations validates algorithm robustness
- ✅ **State Machine Completeness:** Zero unrecoverable stuck states - all error conditions handled gracefully
- ✅ **Hardware Readiness:** Clean separation of simulation-specific vs. hardware-agnostic code enables direct transfer to physical system

**System Capability Demonstration:**
- ✅ **Address Space Coverage:** Successfully navigate to all valid addresses across 8 cabinets (left/right rows, 4 cabinets each)
- ✅ **Department-Level Precision:** Pick from correct departments within boxes (10-16 departments per box)
- ✅ **Complex Workflows:** Complete end-to-end operations (extract → pick → return, box relocation, loading station operations)

**Investment Decision Criteria:**
- ✅ **Technical Feasibility:** Demonstrate autonomous operation without manual intervention
- ✅ **Integration Readiness:** Action interface suitable for Level 2 system integration
- ✅ **Configuration Flexibility:** Support multiple cabinet layouts via YAML configuration only

---

## Product Scope

### MVP - Minimum Viable Product (Complete Algorithm Validation in Simulation)

The MVP delivers **all autonomous manipulation capabilities in Gazebo simulation**, corresponding to architecture Phases 0-5. This represents algorithm readiness for hardware deployment.

**Core Manipulation Capabilities:**
1. ✅ Navigate to any warehouse address using (side, cabinet, row, column) coordinates
2. ✅ Extract boxes from storage addresses with YZ trajectory planning and electromagnet attachment
3. ✅ Return boxes to original storage addresses
4. ✅ **Relocate boxes** to different addresses (PutBox with validation)
5. ✅ Pick individual items from departments within extracted boxes using state machine
6. ✅ **Container management** - retrieve and place containers using jaw gripper
7. ✅ **Loading station operations** - move boxes to external access points with optional relocation

**Simulation-Specific Infrastructure (Gazebo only):**
1. ✅ Virtual limit switch simulation (18 switches - all joints min/max positions)
2. ✅ Electromagnet attachment/detachment in Gazebo with proximity checking
3. ✅ Dynamic box spawning in Gazebo (physics simulation)

**Core Algorithms (simulation + hardware):**
1. ✅ YZ trajectory generator for collision-free cabinet insertion/extraction

**Visualization Infrastructure (RViz - simulation + hardware):**
1. ✅ Department frame generation and TF broadcasting (computed from box address + storage params)
2. ✅ Visual markers for full system state (magnet status, target addresses, extracted addresses, department locations)

**Complete Action Set (MVP):**

**High-Level Workflows:**
- `PickItemFromStorage` - Complete item retrieval (container → navigate → extract → pick → return)
- `MoveBoxToLoad` - Box loading workflow with optional relocation

**Mid-Level Box Operations:**
- `ExtractBox` - Remove box from cabinet
- `ReturnBox` - Return box to original address
- `PutBox` - Place box in different address (with validation)

**Mid-Level Container & Navigation:**
- `GetContainer` / `PlaceContainer` - Container retrieval and placement
- `ManipulateContainer` - Container jaw control
- `NavigateToAddress` - Move to cabinet address

**Mid-Level Picking:**
- `PickItem` - State machine with limit switches

**Low-Level Control:**
- `MoveJoint` - Single joint position command
- `MoveJointGroup` - Coordinated multi-joint motion

**Utilities & Services:**
- Address resolver (TF lookup service)
- Address validator (empty check, width compatibility)
- YZ trajectory generator
- Box spawn/despawn manager
- State marker publisher

**Configuration & Observability:**
- All parameters in YAML (storage, controllers, switches, containers, load positions)
- Address resolution via TF frames (no hardcoded coordinates)
- Standard RQt tools for monitoring and control
- Complete logging and diagnostic reporting

**Success Criteria for MVP:**
- ✅ Complete pick-from-storage workflow: ≥95% success rate over 100 operations
- ✅ Box relocation operations: validate address checking and width matching
- ✅ Loading station workflows: successful round-trip with relocation
- ✅ State machine robustness: zero unrecoverable stuck states
- ✅ All 8 cabinets accessible with correct positioning
- ✅ Department-level picking accuracy maintained across all operations
- ✅ System logs provide clear diagnostic information for all failures
- ✅ **Algorithms ready for hardware deployment** - clean hardware abstraction layer

### Growth Features (Hardware Deployment Phase)

**Hardware Integration:**
1. ✅ **Real Limit Switch Integration** - GPIO-based or industrial sensor protocol (replace virtual switches)
2. ✅ **Physical Electromagnet Control** - Relay/SSR driver interface for real magnet control
3. ✅ **Hardware Calibration Procedures** - Joint calibration, limit switch verification, trajectory tuning
4. ✅ **Real-World Trajectory Optimization** - Adjust speeds, accelerations, safety margins based on physical constraints
5. ✅ **Mechanical Tolerance Handling** - Compensation for play in linkages, friction variations

**Hardware Testing & Validation:**
1. ✅ Individual joint control verification on physical hardware
2. ✅ Limit switch functionality testing (all 18 switches)
3. ✅ Electromagnet attach/detach reliability testing
4. ✅ Address navigation accuracy measurement and tuning
5. ✅ State machine execution on hardware (compare to simulation)
6. ✅ Complete pick workflow on physical system (target: ≥90% success rate)

**Hardware-Specific Configuration:**
1. ✅ Hardware config layer separate from simulation config
2. ✅ Launch files for hardware vs. simulation modes
3. ✅ Hardware diagnostics and health monitoring
4. ✅ Maintenance mode for manual operation/testing

**Success Criteria for Growth:**
- ✅ All algorithms transfer to hardware without logic changes
- ✅ Hardware pick operations achieve ≥90% success rate
- ✅ System operates autonomously for 8-hour test period
- ✅ Hardware diagnostics detect and report failures clearly

### Vision (Future System Integration)

**Level 2 Warehouse Management Integration:**
1. REST API bridge for high-level command interface
2. MQTT interface for real-time status streaming
3. Order fulfillment workflow integration (batch picking, prioritization)
4. Inventory tracking synchronization (item locations, stock levels)
5. Error reporting and recovery coordination with Level 2

**Advanced Sensing & Verification:**
1. Computer vision for item verification (confirm correct item picked)
2. Barcode/RFID scanning during pick operations (traceability)
3. Weight sensing for grasp validation (detect empty picks)
4. Adaptive grasping based on item properties (fragile items, varying sizes)

**Medical Domain Enhancements:**
1. Temperature monitoring for temperature-sensitive medications
2. Expiration date tracking and FIFO enforcement
3. Controlled substance handling (special logging, dual verification)
4. Emergency priority handling (stat medication requests)

---

## Functional Requirements

### FR-001: Warehouse Address Navigation

**Category:** Core Navigation

The system shall navigate the manipulator to any valid warehouse address specified by (side, cabinet_num, row, column) coordinates.

**Details:**
- Side: "left" or "right" cabinet row
- Cabinet numbers: 1-4 per side
- Row/column ranges: Per cabinet configuration in `storage_params.yaml`
- Navigation uses X-axis (rail) and Z-axis (vertical selector) positioning
- Position accuracy: ±2cm from target address TF frame

**MVP:** Yes
**Dependencies:** TF frame tree must include all address frames from URDF

---

### FR-002: Address Coordinate Resolution

**Category:** Core Navigation

The system shall resolve warehouse addresses to world-frame coordinates by looking up TF frames, not by storing hardcoded position tables.

**Details:**
- Address frame naming: `addr_{side_abbrev}_{cabinet}_{row}_{col}`
- Example: `addr_l_1_2_3` = Left row, Cabinet 1, Row 2, Column 3
- Service interface: `GetAddressCoordinates` returns (x, y, z) pose
- Frames defined in URDF, loaded at system startup

**MVP:** Yes
**Dependencies:** URDF must be loaded with complete address frame definitions

---

### FR-003: Box Extraction from Storage

**Category:** Core Box Handling

The system shall extract boxes from storage addresses using coordinated YZ motion and electromagnet attachment.

**Details:**
- Navigate to address (FR-001)
- Generate safe insertion trajectory (Y+ into cabinet, avoid frame collision)
- Extend gripper using YZ trajectory to reach box depth
- Engage electromagnet
- Generate extraction trajectory (Y- out of cabinet)
- Retract gripper with box attached
- Spawn box representation in simulation with department frames
- Mark address as "extracted" (visual red marker)

**MVP:** Yes
**Dependencies:** Electromagnet simulation, YZ trajectory generation, box spawn manager

---

### FR-004: Box Return to Original Address

**Category:** Core Box Handling

The system shall return extracted boxes to their original storage addresses.

**Details:**
- Navigate to original address
- Generate insertion trajectory (approach and extend into cabinet)
- Execute YZ trajectory to insertion depth
- Disengage electromagnet (release box)
- Generate and execute retraction trajectory
- Despawn box representation from simulation
- Remove "extracted" visual marker

**MVP:** Yes
**Dependencies:** Box must be currently held (magnet engaged)

---

### FR-005: Item Picking with State Machine

**Category:** Core Item Picking

The system shall pick individual items from departments within extracted boxes using a state-machine approach driven by limit switch feedback.

**Details:**
- **State Flow:** IDLE → APPROACH → OPEN_JAW → EXTEND → CLOSE → RETRACT → LIFT → SUCCESS
- Navigate picker to department position (not box center)
- Monitor limit switches for state transitions:
  - `picker_jaw_opened` → transition from OPEN_JAW
  - `picker_jaw_extended` → transition from EXTEND
  - `picker_jaw_closed` → transition from CLOSE (item grasped)
  - `picker_jaw_retracted` → transition from RETRACT
- Timeout per state: 10 seconds (configurable)
- Failure recovery: Safe abort returns to IDLE state

**MVP:** Yes
**Dependencies:** Virtual limit switch simulation, department frame generation

---

### FR-006: Department Frame Generation (RViz Visualization)

**Category:** Visualization (works for simulation AND hardware)

The system shall generate and broadcast TF frames for each department within boxes based on known box parameters, enabling RViz visualization of department locations.

**Details:**
- Frame naming: `{box_id}_dept_{dept_num}` (dept_num: 1 to N)
- Department positions calculated from `storage_params.yaml`:
  - `department_depth`: spacing between departments
  - `department_offset_y`: offset from box origin
- Frames positioned at department **centers** (pick target locations)
- Visual markers (red spheres) published to `/visualization_marker_array` at each department center with text labels
- TF broadcast rate: 10 Hz
- **Hardware Note:** On real hardware, department frames are computed from known box address + storage parameters (no physical box model needed in Gazebo)

**MVP:** Yes
**Dependencies:** Storage parameters loaded, box address known

---

### FR-007: Limit Switch Simulation

**Category:** Simulation Infrastructure

The system shall simulate limit switches for all manipulator joints based on joint position thresholds.

**Details:**
- **18 total switches:** 9 joints × 2 switches each (min and max position)
- Special picker switches:
  - `picker_jaw_opened`, `picker_jaw_closed`
  - `picker_jaw_extended`, `picker_jaw_retracted`
- Switch trigger positions defined in `limit_switches.yaml`
- Trigger tolerance: ±0.01m (1cm)
- Publish as `Bool` topics: `/manipulator/end_switches/{switch_name}`
- Update rate: 50 Hz

**MVP:** Yes
**Dependencies:** Subscribe to `/joint_states` from ros2_control

---

### FR-008: Electromagnet Simulation

**Category:** Simulation Infrastructure

The system shall simulate electromagnet attachment/detachment in Gazebo via a service interface.

**Details:**
- Service: `ToggleElectromagnet` (activate: true/false)
- When activated:
  - Check proximity to box (within 5cm)
  - Call Gazebo attach service to fix box to gripper link
  - Publish magnet state: `/manipulator/electromagnet/engaged` (Bool)
- When deactivated:
  - Call Gazebo detach service
  - Publish magnet disengaged
- Visual indicator: Red sphere marker on gripper link when engaged

**MVP:** Yes
**Dependencies:** Gazebo attach/detach services available

---

### FR-009: Dynamic Box Spawning and Despawning (Gazebo Simulation Only)

**Category:** Simulation Infrastructure

The system shall spawn box models dynamically in Gazebo when boxes are extracted for physics simulation, and despawn them when returned.

**Details:**
- **Simulation Only:** This feature is Gazebo-specific for physics simulation
- Service interface: `SpawnBox`, `DespawnBox`
- Box model generation:
  - Dimensions from `storage_params.yaml` based on cabinet configuration
  - SDF/URDF model with collision and visual geometry
  - Mass: 0.5kg (configurable)
- Spawn location: Address TF frame position
- Box ID format: `box_{side_abbrev}_{cabinet}_{row}_{col}`
- Track active boxes (prevent duplicate spawns)
- **Hardware Note:** On real hardware, boxes are physical objects - no spawning needed. Department frame generation (FR-006) works independently of this.

**MVP:** Yes
**Dependencies:** Gazebo spawn/delete entity services

---

### FR-010: YZ Trajectory Generation for Safe Insertion/Extraction

**Category:** Motion Planning

The system shall generate collision-free trajectories in the YZ plane (world frame) for box insertion and extraction operations.

**Details:**
- **Insertion Trajectory:**
  1. Align Z height outside cabinet
  2. Check top frame clearance (lower if needed for safety margin)
  3. Move Y+ into cabinet to target depth
  4. Return to target Z height
- **Extraction Trajectory:**
  1. Check current Z vs. top frame clearance
  2. Lower if collision risk
  3. Retract Y- to safe position outside cabinet
- Safety margin: 2cm clearance from cabinet frame
- Waypoint-based execution (position, speed, description)
- Configurable approach speed (default: 0.05 m/s for insertion)

**MVP:** Yes
**Dependencies:** Cabinet dimensions from `storage_params.yaml`

---

### FR-011: Low-Level Joint Control

**Category:** Control Primitives

The system shall provide action interfaces for controlling individual joints and coordinated joint groups.

**Details:**
- **MoveJoint Action:**
  - Target: Single joint by name
  - Parameters: target_position, max_velocity (optional)
  - Feedback: current_position, progress_percent
  - Result: success, final_position
- **MoveJointGroup Action:**
  - Target: Named joint group (e.g., "selector_gripper" for YZ motion)
  - Parameters: target_positions (array), max_velocity
  - Coordination: Simultaneous motion of all joints in group
  - Result: success, all joints reached targets

**MVP:** Yes
**Dependencies:** Individual ForwardCommandControllers for each joint

---

### FR-012: Visual State Markers (RViz Visualization)

**Category:** Observability (works for simulation AND hardware)

The system shall publish RViz markers indicating current system state and active operations for visualization in RViz.

**Details:**
- **Magnet Engaged:** Red sphere on gripper magnet link (when electromagnet active)
- **Target Address:** Green box marker at target address frame (during navigation)
- **Extracted Address:** Red box marker at empty address (after extraction, indicates box currently held)
- **Department Markers:** Red spheres at department centers with text labels (visualizes pick locations within box)
- Publish to `/visualization_marker_array` topic
- Update rate: 10 Hz
- **Hardware Note:** Works identically on real hardware - visualizes system state in RViz regardless of whether running in simulation or on physical manipulator

**MVP:** Yes
**Dependencies:** Subscribe to manipulator state, electromagnet state, active operations

---

### FR-013: Configuration from YAML Files

**Category:** System Configuration

The system shall load all configuration from YAML files without hardcoding values in source code.

**Details:**
- **Primary Configs (manipulator_description):**
  - `ros2_control.xacro` - Joint limits, initial values
  - `manipulator_controllers.yaml` - Controller names and types
  - `storage_params.yaml` - Cabinet dimensions, department counts
- **Secondary Configs (manipulator_control):**
  - `action_servers.yaml` - Timeouts, default speeds
  - `limit_switches.yaml` - Switch trigger positions
  - `container_storage.yaml` - Predefined container locations
  - `load_positions.yaml` - External access point definitions (loading stations)
- Changes to cabinet layout require only YAML edits, no code recompilation

**MVP:** Yes
**Dependencies:** None

---

### FR-014: Box Relocation (PutBox)

**Category:** Advanced Box Handling

The system shall place an extracted box into any empty address with compatible width, enabling storage optimization.

**Details:**
- Validate target address:
  - Must be empty (not currently occupied)
  - Box width must match cabinet column count
- Navigate to target address
- Execute YZ insertion trajectory
- Disengage electromagnet (release box)
- Execute YZ extraction trajectory
- Despawn box from simulation
- Mark target address as occupied

**MVP:** Yes
**Dependencies:** Address validator utility, box currently held by manipulator

---

### FR-015: Box Loading Station Operations

**Category:** Advanced Box Handling

The system shall move boxes to predefined loading stations for external access (inspection, operator access, integration with other systems) and optionally return to storage.

**Details:**
- Extract box from source address
- Navigate to named load position (from `load_positions.yaml`)
- Lower selector to configured load height
- **Optional:** Wait for external signal or timeout
- **Optional:** Return to storage:
  - Same address as source, OR
  - Different address (uses PutBox for relocation)
- Feedback phases: "extracting", "moving_to_load", "at_load_position", "returning", "completed"

**MVP:** Yes
**Dependencies:** ExtractBox, PutBox (if relocation), load position configuration

---

### FR-016: Container Jaw Manipulation

**Category:** Container Handling

The system shall open and close container jaws synchronously for gripping containers.

**Details:**
- Synchronized control of left and right container jaw joints
- Target opening width parameter (jaw-to-jaw distance)
- Calculate symmetric positions: left = -width/2, right = +width/2
- Software mimic for simulation (hardware uses mechanical mimic)
- Monitor jaw positions for verification

**Implementation Note:** Software synchronization required only in Gazebo simulation where mimic joints are not supported by the dartsim physics engine. Real hardware uses mechanical mimic linkage between jaw joints - no software coordination needed, jaws move together automatically.

**MVP:** Yes
**Dependencies:** Individual controllers for left/right jaw joints

---

### FR-017: Container Retrieval and Placement

**Category:** Container Handling

The system shall retrieve containers from predefined storage locations and return them after use.

**Details:**
- **GetContainer Action:**
  1. Open jaws wider than container width
  2. Navigate to container location (X, Z positioning)
  3. Lower selector to container height
  4. Close jaws to grip container
  5. Lift selector (container hangs on jaws)
  6. Verify jaw positions indicate gripping
- **PlaceContainer Action:**
  1. Navigate to placement location
  2. Lower selector until container rests on support
  3. Open jaws to release
  4. Raise selector away from container

**MVP:** Yes
**Dependencies:** ManipulateContainer, predefined container positions in config

---

### FR-018: Address Validation Utilities

**Category:** Safety and Validation

The system shall validate addresses before box placement operations to prevent errors.

**Details:**
- **Empty Check:** Verify target address is not currently occupied
- **Width Compatibility:** Verify box column count matches cabinet column count
- Load cabinet configurations from `storage_params.yaml`:
  - Left row cabinets: 1(4cols), 2(4cols), 3(4cols), 4(5cols)
  - Right row cabinets: 1(5cols), 2(5cols), 3(6cols), 4(4cols)
- Return validation result: success/failure with error message

**MVP:** Yes
**Dependencies:** Storage configuration loaded, occupancy tracking

---

### FR-019: Complete Pick-from-Storage Workflow

**Category:** High-Level Workflow

The system shall execute a complete autonomous picking operation from a single high-level action goal.

**Details:**
- **PickItemFromStorage Action:**
  - Goal: address (side, cabinet, row, column, department_num), container_id
  - Workflow:
    1. Get container (if needed)
    2. Navigate to box address
    3. Extract box
    4. Navigate to picking position (adjust for picker reach)
    5. Pick item from specified department
    6. Place item in container
    7. Return box to cabinet
  - Error handling: Safe abort returns box if extraction succeeded
  - Feedback: current operation phase, progress percent, elapsed time

**MVP:** Yes
**Dependencies:** All mid-level actions (GetContainer, NavigateToAddress, ExtractBox, PickItem, ReturnBox, ManipulateContainer)

---

### FR-020: RQt Tool Integration

**Category:** Observability and Control

The system shall be fully controllable and observable using standard RQt tools without requiring custom GUI development initially.

**Details:**
- **Standard Tools Used:**
  - `rqt_action` - Send action goals, monitor feedback/results
  - `rqt_publisher` - Publish to topics (joint commands, manual electromagnet control)
  - `rqt_service_caller` - Call services (spawn box, address resolution)
  - `rqt_topic` - Monitor topics (joint states, limit switches)
  - `rqt_tf_tree` - Visualize TF frames (addresses, departments)
  - `rqt_console` - View logs and debug messages
- Create reusable RQt perspective file for development workflow
- **Future:** Custom RQt plugin only if standard tools prove insufficient during Phase 5 testing

**MVP:** Yes
**Dependencies:** Standard RQt package installation

---

## Non-Functional Requirements

### Performance

**NFR-001: Operation Timing**
- Complete PickItemFromStorage operation: ≤120 seconds (extract → pick → return)
- NavigateToAddress operation: ≤15 seconds per address
- ExtractBox operation: ≤30 seconds
- PickItem operation: ≤25 seconds (state machine execution)

**NFR-002: Position Accuracy**
- Address navigation: ±2cm from target TF frame
- Department-level picking: ±5mm from department center
- Limit switch trigger tolerance: ±1cm from configured threshold

**NFR-003: System Responsiveness**
- Control system initialization: ≤10 seconds from launch to ready
- Action feedback publish rate: ≥2 Hz during active operations
- TF frame broadcast rate: 10 Hz for department frames

### Reliability

**NFR-004: Operation Success Rate**
- Simulation environment: ≥95% success rate for complete pick operations
- Target for hardware deployment: ≥90% (accounting for mechanical tolerances)

**NFR-005: Fault Tolerance**
- Zero unrecoverable stuck states (all failures detectable and recoverable)
- Safe abort capability from any operation phase
- Timeout protection on all actions (configurable per action type)

**NFR-006: State Observability**
- All action states published as feedback
- All limit switch states published continuously at 50 Hz
- System health status available via service query

### Scalability

**NFR-007: Configuration Flexibility**
- Support 4 cabinets per side (8 total cabinets) with varying dimensions
- Handle cabinet configurations from 4x6x10 up to 6x14x16 (columns × rows × departments)
- Add new cabinet layouts via YAML configuration only (no code changes)

**NFR-008: Extensibility**
- Modular action server architecture supports adding new high-level workflows
- Utility functions isolated for reuse across action servers
- Clear separation between simulation-specific and hardware-agnostic code

### Integration

**NFR-009: Level 2 Interface Readiness**
- Action interface definitions stable and documented
- Message/service definitions versioned and backward-compatible
- Bridge node architecture prepared (REST/MQTT support planned for post-MVP)

**NFR-010: ROS2 Standards Compliance**
- Use ros2_control framework for all joint control
- Follow ROS2 naming conventions for topics, services, actions
- Compatible with Gazebo Fortress/Harmonic simulation environments

### Maintainability

**NFR-011: Configuration Management**
- Single source of truth for all parameters (YAML files)
- No magic numbers in code (all thresholds, speeds, timeouts configurable)
- Configuration hierarchy documented (primary vs. secondary configs)

**NFR-012: Logging and Debugging**
- Structured logging with severity levels (DEBUG, INFO, WARN, ERROR)
- Action-specific namespaces for filtering logs
- Diagnostic messages include context (joint names, positions, target values)

**NFR-013: Code Organization**
- Package structure follows ROS2 conventions
- Clear separation: actions/, simulation/, utils/, gui/
- Each action server in separate module with base class inheritance

---

## Epic Overview (MVP Scope)

The following epics represent the complete MVP implementation, delivering all simulation-validated algorithms ready for hardware deployment. These align with architecture development Phases 0-5.

**Epic 1: Package Setup & Interface Definitions (Phase 0)**
- Package structure creation
- All action/service/message interface definitions
- Build system configuration
- Development environment validation

**Epic 2: Simulation Foundation & Joint Control (Phase 1)**
- Virtual limit switch node (18 switches)
- Controller interface utility
- MoveJoint action server
- State marker publisher (basic)
- Launch files for simulation helpers

**Epic 3: Address Navigation System (Phase 2)**
- Address resolver utility with TF lookups
- GetAddressCoordinates service
- MoveJointGroup action server
- NavigateToAddress action
- Address marker visualization

**Epic 4: Box Handling & YZ Trajectories (Phase 3)**
- YZ trajectory generator utility
- Address validator utility
- Electromagnet simulator node
- Box spawn manager (basic, no departments yet)
- ExtractBox action with YZ trajectory
- ReturnBox action
- PutBox action (box relocation)
- MoveBoxToLoad action (loading stations)

**Epic 5: Item Picking & Department Frames (Phase 4)**
- Box spawn manager enhancement (department frames)
- Department TF broadcasting
- Department visual markers with labels
- ManipulateContainer action (jaw control)
- GetContainer and PlaceContainer actions
- PickItem action (state machine with switches)

**Epic 6: High-Level Workflows & System Integration (Phase 5)**
- PickItemFromStorage composite action
- Error handling and recovery logic
- Complete state marker suite
- System integration testing
- Reliability validation (100+ operations)
- Documentation and troubleshooting guides

---

## Growth Phase Overview (Hardware Deployment)

The Growth phase focuses on transferring validated algorithms to physical hardware.

**Epic 7: Hardware Interface Layer**
- Real limit switch integration (GPIO/industrial protocol)
- Physical electromagnet control interface
- Hardware-specific configuration layer
- Launch files for hardware mode
- Hardware diagnostics node

**Epic 8: Hardware Calibration & Testing**
- Joint calibration procedures
- Limit switch verification
- Trajectory tuning for physical constraints
- Individual action testing on hardware
- Complete workflow validation on hardware

**Epic 9: Hardware Operations & Maintenance**
- 8-hour autonomous operation test
- Failure mode testing and recovery
- Maintenance mode implementation
- Operator documentation
- Hardware troubleshooting procedures

---

## References

### Source Documents
- `docs/architecture-ros2-control-v2-CORRECTIONS.md` - Technical architecture and implementation details
- `ros2_ws/src/manipulator_description/urdf/` - URDF definitions for manipulator and warehouse
- `ros2_ws/src/manipulator_description/config/storage_params.yaml` - Cabinet and storage configuration

### Technical Standards
- ROS2 Humble - Primary framework
- ros2_control - Joint control framework
- Gazebo Fortress/Harmonic - Simulation environment
- TF2 - Coordinate frame transformations

### Related Systems
- Level 2 Warehouse Management System (future integration)
- External loading systems at loading stations (future integration)

---

_This PRD captures the complete vision for ya_robot_manipulator as an autonomous warehouse picking system. The phased approach (MVP → Growth → Vision) enables iterative development with clear success criteria at each stage._

_The functional requirements extracted from the technical architecture provide a product-focused view of system capabilities, organized by business value rather than implementation details._

_Created through reverse-engineering analysis of architecture document by AI Product Manager._
