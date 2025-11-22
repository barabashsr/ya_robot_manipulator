# Box Color Management Strategy

## Overview

Box colors indicate different states during operation:
- **Light Blue** (default): Box is present at this address
- **Green**: Target address - manipulator is moving to this box
- **Scarlet Red**: Empty address - no box present

## Implementation Strategy

### Option 1: URDF Static Colors (Current Implementation)
**Purpose**: Visualization and testing only

The URDF shows all boxes in light blue color when `show_address_boxes: true` in config.

**Pros:**
- Simple to implement
- Good for testing URDF structure
- Shows all address positions clearly

**Cons:**
- Cannot change colors at runtime
- All boxes same color
- Not suitable for production

**Usage:**
```yaml
# storage_params.yaml
visualization:
  show_address_boxes: true  # Show all boxes in light blue
```

### Option 2: RViz Interactive Markers (Recommended for Runtime)
**Purpose**: Dynamic color indication during operation

Use a ROS2 node that publishes `visualization_msgs/MarkerArray` to overlay colored markers on the URDF.

**Pros:**
- Colors change dynamically
- Independent from URDF
- Can show/hide individual boxes
- Supports transparency, blinking, etc.

**Cons:**
- Requires separate node
- Slight performance overhead (minimal)

**Implementation:**

```python
#!/usr/bin/env python3
"""
Box State Visualizer Node

Publishes colored markers for boxes based on their state.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class BoxStateVisualizer(Node):
    def __init__(self):
        super().__init__('box_state_visualizer')

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/box_state_markers',
            10
        )

        # Box state database
        # Format: {address: {'state': 'occupied'|'empty'|'target', 'box_id': 'ABC123'}}
        self.box_states = {}

        # Subscribe to box state updates
        # self.create_subscription(BoxState, '/box_states', self.update_box_state, 10)

        # Timer to publish markers
        self.create_timer(0.1, self.publish_markers)  # 10Hz

    def update_box_state(self, address, state, box_id=None):
        """
        Update state of a box at given address.

        Args:
            address: tuple (side, cab, row, col) e.g., ('l', 1, 2, 3)
            state: 'occupied', 'empty', or 'target'
            box_id: Physical box ID (if occupied)
        """
        self.box_states[address] = {
            'state': state,
            'box_id': box_id
        }

    def publish_markers(self):
        """Publish marker array with colored boxes."""

        marker_array = MarkerArray()

        for i, (address, info) in enumerate(self.box_states.items()):
            side, cab, row, col = address
            state = info['state']

            marker = Marker()
            marker.header.frame_id = f"addr_{'l' if side == 'left' else 'r'}_{cab}_{row}_{col}"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "box_states"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position relative to address frame
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0  # Adjust based on box size
            marker.pose.position.z = 0.0  # Adjust based on box size
            marker.pose.orientation.w = 1.0

            # Size (get from config)
            marker.scale.x = 0.06  # box width
            marker.scale.y = 0.2   # box depth
            marker.scale.z = 0.09  # box height

            # Color based on state
            if state == 'occupied':
                marker.color = ColorRGBA(r=0.53, g=0.81, b=0.92, a=0.8)  # Light blue
            elif state == 'empty':
                marker.color = ColorRGBA(r=1.0, g=0.14, b=0.0, a=0.6)    # Scarlet
            elif state == 'target':
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)     # Green

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = BoxStateVisualizer()

    # Example: Set some box states
    node.update_box_state(('left', 1, 1, 1), 'occupied', 'ABC123')
    node.update_box_state(('left', 1, 1, 2), 'empty')
    node.update_box_state(('left', 1, 2, 1), 'target', 'XYZ789')

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Option 3: Hybrid Approach (Best for Production)

**Phase 1: Development/Testing**
- Use URDF static boxes (`show_address_boxes: true`)
- All boxes visible in light blue
- Good for verifying placement and sizing

**Phase 2: Runtime Operation**
- Set `show_address_boxes: false` in URDF
- Run `box_state_visualizer` node
- Publishes markers only for boxes that exist
- Colors indicate state dynamically

**Switching:**
```yaml
# Development
visualization:
  show_address_boxes: true

# Production
visualization:
  show_address_boxes: false  # Hide URDF boxes, use markers instead
```

## RViz Configuration

To see interactive markers:
1. Add `MarkerArray` display in RViz
2. Set topic to `/box_state_markers`
3. Enable the display

## Database Integration

The visualizer node reads box states from database:

```python
def sync_from_database(self):
    """Sync box states from upper-level database."""

    # Query: SELECT side, cabinet, row, col, box_id FROM box_locations
    results = self.db.query_all_boxes()

    for row in results:
        address = (row['side'], row['cabinet'], row['row'], row['col'])
        if row['box_id']:
            self.update_box_state(address, 'occupied', row['box_id'])
        else:
            self.update_box_state(address, 'empty')
```

## Performance Considerations

**URDF Static Boxes:**
- Generated once at startup
- No runtime overhead
- All boxes always visible

**Interactive Markers:**
- Updated at 10Hz (configurable)
- Only active boxes rendered
- Minimal CPU/GPU impact
- Recommended: Update only on state change, not every frame

## Recommendation

**For your use case (QR box tracking):**

1. **Development**: Use URDF static boxes to verify system structure
2. **Production**:
   - Disable URDF boxes
   - Create `box_state_visualizer_node`
   - Subscribe to box location database updates
   - Publish markers with state-based colors
   - Update markers only when box states change

This keeps the URDF clean and separates visualization logic from robot description.
