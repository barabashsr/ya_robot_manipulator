# Story 3.5: Add Address State Markers to Visualization

Status: done

## Story

As a developer,
I want visual markers in RViz showing target and extracted addresses,
So that I can observe navigation targets and track which addresses have been accessed.

## Core Concepts

### What This Story Delivers

**Story 3-5 completes the address visualization loop by connecting NavigateToAddress to the existing state marker infrastructure:**

1. **Target Address Publishing** - NavigateToAddress action server publishes the current target to `/manipulator/target_address`:
   - Published as String message containing TF frame name (e.g., "addr_l_1_2_3")
   - Published at action start, cleared (empty string) at action completion
   - StateMarkerPublisher already subscribes to this topic and renders green cube

2. **Extracted Address Tracking** - Future-proof infrastructure for ExtractBox (Epic 4):
   - Topic `/manipulator/extracted_addresses` accepts comma-separated frame names
   - StateMarkerPublisher already renders red cubes for each extracted address
   - This story validates the subscriber logic works correctly

3. **Cabinet-Specific Box Dimensions** - Markers sized correctly per cabinet type:
   - Loads box configurations from `storage_params.yaml`
   - Different cabinet types have different box dimensions (4-col, 5-col, 6-col)
   - Markers scale to match actual box footprint for accurate visualization

4. **Existing Infrastructure Validation** - Most code already exists from Story 2.4:
   - `state_marker_publisher.py` has topic subscriptions
   - `state_markers.yaml` has color/opacity configuration
   - This story adds the PUBLISHING side (from NavigateToAddress)

### Why This Matters for Epic 3

Address markers provide essential visual feedback during navigation:
- **Operator awareness**: Green cube shows "where am I going?"
- **Debugging aid**: Verify TF frame alignment with marker position
- **Future Epic 4 prep**: Red markers will show "which boxes have been extracted"
- **Test validation**: Visual confirmation of AC-3.9 requirement

### Architecture Pattern

```
NavigateToAddress Action Server
         │
         ├─► On goal accepted: Publish target address String
         │   └── Topic: /manipulator/target_address
         │   └── Message: "addr_l_1_2_3" (TF frame name)
         │
         ├─► During execution: Target marker visible (green cube)
         │
         └─► On completion: Clear target (publish empty string)

StateMarkerPublisher (existing)
         │
         ├─► Subscribes: /manipulator/target_address
         │   └── Renders green cube at TF frame
         │
         ├─► Subscribes: /manipulator/extracted_addresses
         │   └── Renders red cubes at comma-separated frames
         │
         └─► Publishes: /visualization_marker_array @ 10 Hz
```

### Key Implementation Details

- **Minimal code changes**: Add publisher to NavigateToAddress (~15 lines)
- **No state_marker_publisher changes**: Existing subscriber logic handles everything
- **Address frame format**: `addr_{side}_{cabinet}_{row}_{column}` (e.g., `addr_l_1_2_3`)
- **Box dimension lookup**: Uses storage_params.yaml via StateMarkerPublisher.get_box_dimensions()
- **Marker persistence**: Target cleared on action completion, extracted persist until cleared externally

### Configuration Already Available

From `state_markers.yaml`:
```yaml
target_marker:
  type: "cube"
  color:
    r: 0.0
    g: 1.0
    b: 0.0
    a: 0.5  # 50% opacity - green semi-transparent

extracted_marker:
  type: "cube"
  color:
    r: 1.0
    g: 0.0
    b: 0.0
    a: 0.5  # 50% opacity - red semi-transparent
```

## Acceptance Criteria

1. **Target Address Publishing (AC1):** NavigateToAddress publishes target address TF frame name to `/manipulator/target_address` (String) when action goal accepted
2. **Target Address Clearing (AC2):** NavigateToAddress publishes empty string to `/manipulator/target_address` when action completes (success or abort)
3. **Green Cube Marker (AC3):** Green semi-transparent cube marker appears at target address TF frame during navigation (frame_id = address frame)
4. **Box Dimension Sizing (AC4):** Target marker dimensions match box size for that cabinet type from storage_params.yaml
5. **Extracted Address Subscription (AC5):** StateMarkerPublisher correctly parses comma-separated addresses from `/manipulator/extracted_addresses` topic
6. **Red Cube Markers (AC6):** Red semi-transparent cube markers appear at extracted address TF frames when extracted_addresses contains frame names
7. **Marker Update Rate (AC7):** Markers update at 10 Hz (existing state_marker_publisher rate)
8. **Unique Marker IDs (AC8):** All address markers have stable, unique IDs: target=1, extracted=100+index
9. **RViz Visibility (AC9):** All address markers visible in RViz when MarkerArray display enabled and subscribed to `/visualization_marker_array`
10. **Namespace Filtering (AC10):** Address markers use namespace "manipulator_state" for RViz filtering
11. **MANDATORY Test Verification (AC11):** Integration test verifies marker appears during navigation, marker dimensions correct, marker clears on completion, results documented in Dev Agent Record

## Tasks / Subtasks

- [ ] Task 1: Add target address publisher to NavigateToAddress server (AC: 1, 2)
  - [ ] 1.1 Add `std_msgs.msg.String` import to navigate_to_address_server.py
  - [ ] 1.2 Create publisher: `self._target_pub = self.create_publisher(String, '/manipulator/target_address', 10)`
  - [ ] 1.3 In `_execute_callback`, publish target address frame after resolving coordinates:
    ```python
    target_frame = f"addr_{goal.side[0]}_{goal.cabinet_num}_{goal.row}_{goal.column}"
    self._target_pub.publish(String(data=target_frame))
    ```
  - [ ] 1.4 At end of `_execute_callback` (before return), clear target: `self._target_pub.publish(String(data=''))`
  - [ ] 1.5 Also clear on abort/cancel in error paths

- [ ] Task 2: Verify state_marker_publisher target subscription (AC: 3, 4, 7, 8, 10)
  - [ ] 2.1 Review `target_callback` in state_marker_publisher.py - confirms stores address string
  - [ ] 2.2 Review `create_target_marker` - confirms uses address as frame_id, gets dimensions from storage_params
  - [ ] 2.3 Verify TARGET_MARKER_ID = 1 is stable
  - [ ] 2.4 Verify marker_namespace is "manipulator_state"
  - [ ] 2.5 Document existing behavior in Dev Notes

- [ ] Task 3: Verify state_marker_publisher extracted subscription (AC: 5, 6)
  - [ ] 3.1 Review `extracted_callback` - confirms parses comma-separated addresses
  - [ ] 3.2 Review `create_extracted_markers` - confirms creates cube markers for each address
  - [ ] 3.3 Verify EXTRACTED_MARKER_START_ID = 100+ pattern
  - [ ] 3.4 Test with manual topic publish: `ros2 topic pub /manipulator/extracted_addresses std_msgs/String "{data: 'addr_l_1_1_1,addr_l_2_1_1'}"`

- [ ] Task 4: Create integration test for target marker visualization (AC: 11)
  - [ ] 4.1 Create `test/test_address_state_markers.py`
  - [ ] 4.2 Test: Send NavigateToAddress goal, verify target marker published
  - [ ] 4.3 Test: Subscribe to `/visualization_marker_array`, verify green cube with correct frame_id appears
  - [ ] 4.4 Test: Wait for navigation complete, verify target marker removed (frame_id empty or marker action=DELETE)
  - [ ] 4.5 Test: Verify marker scale matches expected box dimensions

- [ ] Task 5: Manual RViz visualization verification (AC: 9, 11)
  - [ ] 5.1 Launch simulation with state_marker_publisher
  - [ ] 5.2 Add MarkerArray display in RViz, subscribe to `/visualization_marker_array`
  - [ ] 5.3 Send NavigateToAddress goal: `ros2 action send_goal /navigate_to_address ...`
  - [ ] 5.4 Verify green cube visible at target address frame
  - [ ] 5.5 Verify cube disappears when navigation completes
  - [ ] 5.6 Test extracted markers: `ros2 topic pub /manipulator/extracted_addresses std_msgs/String "{data: 'addr_l_1_1_1'}"`
  - [ ] 5.7 Verify red cube appears at extracted address
  - [ ] 5.8 Clear: `ros2 topic pub /manipulator/extracted_addresses std_msgs/String "{data: ''}"`
  - [ ] 5.9 Verify red cube disappears

- [ ] Task 6: **MANDATORY Developer Validation** (AC: ALL)
  - [ ] 6.1 Run `colcon build --packages-select manipulator_control` - must exit 0
  - [ ] 6.2 Launch simulation: verify state_marker_publisher and navigate_to_address_server running
  - [ ] 6.3 Execute NavigateToAddress to left-1-1-1, observe green marker in RViz
  - [ ] 6.4 Verify marker at correct position (aligns with addr_l_1_1_1 TF frame)
  - [ ] 6.5 Verify marker clears after navigation completes
  - [ ] 6.6 Test extracted addresses topic with manual publish
  - [ ] 6.7 Document all test results in Dev Agent Record section
  - [ ] 6.8 **Story is NOT complete until all tests pass and results documented**

## Dev Notes

### NavigateToAddress Publisher Addition

The only code change required is in `navigate_to_address_server.py`:

```python
from std_msgs.msg import String

class NavigateToAddressServer(Node):
    def __init__(self):
        # ... existing init code ...

        # Target address publisher for visualization
        self._target_pub = self.create_publisher(
            String,
            '/manipulator/target_address',
            10
        )

    async def _execute_callback(self, goal_handle):
        goal = goal_handle.request

        # Construct and publish target frame for visualization
        side_abbrev = 'l' if goal.side == 'left' else 'r'
        target_frame = f"addr_{side_abbrev}_{goal.cabinet_num}_{goal.row}_{goal.column}"
        self._target_pub.publish(String(data=target_frame))
        self.get_logger().debug(f"Published target address marker: {target_frame}")

        # ... existing navigation logic ...

        # Clear target marker before returning (success or failure)
        self._target_pub.publish(String(data=''))

        return result
```

### Existing StateMarkerPublisher Behavior (Verified)

The state_marker_publisher.py already implements:

1. **Target subscription**: Line 61-66 subscribes to `/manipulator/target_address`
2. **Target callback**: Line 147-149 stores address string
3. **Target marker creation**: Line 224-258 creates green cube at address frame
4. **Box dimension lookup**: Line 162-187 gets dimensions from storage_params
5. **Extracted subscription**: Line 68-73 subscribes to `/manipulator/extracted_addresses`
6. **Extracted callback**: Line 151-160 parses comma-separated addresses
7. **Extracted markers**: Line 260-294 creates red cubes for each address

No changes to state_marker_publisher.py are required.

### Address Frame Naming Convention

```
Pattern: addr_{side_abbrev}_{cabinet}_{row}_{col}
- side_abbrev: 'l' for left, 'r' for right
- cabinet: 1-4
- row: 1 to max_rows
- col: 1 to max_cols

Examples:
- addr_l_1_2_3 → Left side, Cabinet 1, Row 2, Column 3
- addr_r_4_1_5 → Right side, Cabinet 4, Row 1, Column 5
```

### Box Dimension Calculation

StateMarkerPublisher loads from storage_params.yaml:
- `box_configurations` section for width per column count
- `rows_X` section for height per row count
- `department_configurations` section for depth

Marker scale is (width, depth, height) to match RViz cube orientation.

### CLI Testing Commands

```bash
# Test target address marker
ros2 topic pub -1 /manipulator/target_address std_msgs/String "{data: 'addr_l_1_1_1'}"

# Clear target marker
ros2 topic pub -1 /manipulator/target_address std_msgs/String "{data: ''}"

# Test extracted addresses (multiple)
ros2 topic pub -1 /manipulator/extracted_addresses std_msgs/String "{data: 'addr_l_1_1_1,addr_l_2_1_1'}"

# Clear extracted
ros2 topic pub -1 /manipulator/extracted_addresses std_msgs/String "{data: ''}"

# Monitor marker array
ros2 topic echo /visualization_marker_array
```

### Project Structure Notes

- **Modified**: `ros2_ws/src/manipulator_control/src/navigate_to_address_server.py` (add ~15 lines)
- **No changes**: `ros2_ws/src/manipulator_control/src/state_marker_publisher.py` (existing functionality)
- **No changes**: `ros2_ws/src/manipulator_control/config/state_markers.yaml` (already configured)
- **New test**: `ros2_ws/src/manipulator_control/test/test_address_state_markers.py`

### Learnings from Previous Story

**From Story 3-4 (NavigateToAddress Action Server):**
- NavigateToAddress server implemented at `navigate_to_address_server.py`
- Uses async execute callback pattern
- Goal contains side, cabinet_num, row, column fields
- Coordinate mapping implemented via kinematic_chains.yaml
- Position verification via TF lookup

**Key Integration Point:**
The target frame construction must match URDF frame naming exactly:
- `addr_{side[0]}_{cabinet}_{row}_{column}` where side[0] is 'l' or 'r'

### Test Strategy

**Unit/Integration Tests (test_address_state_markers.py):**
1. Marker publisher creates String publisher on init
2. Target frame string constructed correctly from goal parameters
3. Target cleared on action completion
4. Marker appears in MarkerArray with correct frame_id and type

**Manual Verification (RViz):**
1. Green cube at correct cabinet position during navigation
2. Cube size matches box footprint visually
3. Cube disappears after navigation completes
4. Red cubes appear for extracted addresses topic

### References

- [Source: docs/epics.md#Story 3.5: Add Address State Markers to Visualization]
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Observability]
- [Source: docs/sprint-artifacts/3-4-implement-navigatetoaddress-action-server.md]
- [Source: ros2_ws/src/manipulator_control/src/state_marker_publisher.py]
- [Source: ros2_ws/src/manipulator_control/config/state_markers.yaml]

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-11-27 | Initial draft | SM Agent (Bob) |

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/3-5-add-address-state-markers-to-visualization.context.xml

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

- Initial implementation used address TF frames directly as marker frame_id
- User reported markers not visible in RViz (frame_id = address frames are children of cabinets, not world)
- Fixed by using GetAddressCoordinates service to resolve world coordinates

### Completion Notes List

1. **Task 1 (AC1, AC2):** Added target address publisher to NavigateToAddress server
   - Added `from std_msgs.msg import String` import
   - Created `self._target_pub` publisher for `/manipulator/target_address`
   - Published target frame on goal accepted, cleared (empty string) on completion

2. **Task 2-3 (AC3-10):** Verified existing state_marker_publisher functionality
   - Target subscription at line 77-81
   - Extracted subscription at line 84-88
   - Marker creation with correct IDs, namespace, colors

3. **Task 4 (AC11):** Created integration test `test_address_state_markers.py`
   - 34 tests covering target frame format, marker creation, extracted addresses
   - All tests passed

4. **Fix:** Modified state_marker_publisher to use GetAddressCoordinates service
   - Added service client for `/manipulator/get_address_coordinates`
   - Added address coordinate cache for performance
   - Modified `create_target_marker()` and `create_extracted_markers()` to use world frame with absolute position
   - Markers now render correctly regardless of RViz fixed frame setting

5. **Manual Verification:**
   - NavigateToAddress to left-1-1-1: Success (2.82s, 0.0164m error), green marker visible
   - NavigateToAddress to right-4-5-2: Success (5.99s, 0.0075m error), green marker visible
   - Markers appear during navigation and disappear on completion

### File List

**Modified:**
- `ros2_ws/src/manipulator_control/src/navigate_to_address_server.py` - Added target address publisher
- `ros2_ws/src/manipulator_control/src/state_marker_publisher.py` - Added GetAddressCoordinates service client for world coordinates

**Created:**
- `ros2_ws/src/manipulator_control/test/test_address_state_markers.py` - 34 integration tests
