# PlayStation Controller Layout for Manipulator

```
                        ┌─────────────────────────────────────┐
                        │                                     │
         L1 (ENABLE) ───┤  ●                           ●     │
                        │                                     │
         L2 (Jaws) ─────┤  ●                           ●     │── R1 (TURBO)
                        │                                     │
                        │                                     │── R2 (unused)
                        │                                     │
                        │                                     │
                        │          ┌───────────┐              │
                        │          │           │              │
                        │          │    PS     │              │
                        │          │           │              │
                        │          └───────────┘              │
                        │                                     │
                        │  ╭─────╮                  ╭─────╮   │
                        │  │  ▲  │  D-Pad           │  △  │   │── Triangle (Picker jaw open)
                        │  │◄ ● ►│  Picker          │ □ ● ○ │  │
                        │  │  ▼  │  H/V             │  ✕  │   │── Cross (Picker jaw close)
                        │  ╰─────╯                  ╰─────╯   │
                        │                                     │
                        │   ╱─╲      Left Stick      ╱─╲     │
                        │  │ ● │     Base+Slider    │ ● │    │── Right Stick
                        │   ╲─╱      X/Y             ╲─╱     │   Selector+Gripper X/Y
                        │                                     │
                        └─────────────────────────────────────┘
```

## Control Mapping

### Left Analog Stick
- **Y-axis (up/down)**: Base Railway (0.0 - 4.0 m)
- **X-axis (left/right)**: Picker Slider (0.0 - 0.12 m)

### Right Analog Stick
- **Y-axis (up/down)**: Selector Vertical (0.0 - 1.5 m)
- **X-axis (left/right)**: Gripper Y-axis (-0.4 - 0.4 m)

### D-Pad (Discrete Control)
- **Up/Down**: Picker Vertical (0.0 - 0.3 m)
- **Left/Right**: Picker Horizontal (-0.3 - 0.3 m)

### Shoulder Buttons
- **L1**: **ENABLE** (must hold for any control to work!)
- **R1**: TURBO (2x speed multiplier)
- **L2**: Container Jaws open/close (-0.2 - 0.2 m, controls both jaws)
- **R2**: (unused)

### Face Buttons
- **Triangle (△)**: Picker jaw extend to 0.2 m (hold action)
- **Cross (✕)**: Picker jaw retract to 0.0 m (hold action)
- **Square (□)**: Reserved for future action
- **Circle (○)**: Reserved for future action

### System Buttons
- **Share**: Reserved for future action
- **Options**: Reserved for future action
- **PS**: (system use)

## Usage Notes

1. **Always hold L1** to enable control - this is a safety feature
2. **Hold R1** for turbo mode (faster movement)
3. **Analog sticks** provide proportional velocity control
4. **D-Pad** provides discrete stepped control
5. **Buttons** provide hold-to-move actions for picker jaw

## Technical Details

### Deadzone
- Analog sticks: 0.1 (10% deadzone)
- D-Pad: 0.5 (filters only noise)

### Update Rate
- 20 Hz (50ms period)

### Velocity Scales
- Base railway: 0.5 m/s
- Selector vertical: 0.3 m/s
- Gripper Y-axis: 0.3 m/s
- Container jaws: 0.2 m/s
- Picker vertical: 0.2 m/s
- Picker horizontal: 0.25 m/s
- Picker slider: 0.15 m/s
- Picker jaw: 0.2 m/s

### Turbo Multiplier
- Normal: 1.0x
- Turbo (R1 held): 2.0x

## Customization

To change mappings, edit:
```
src/joy_control/config/manipulator_joy_config.yaml
```

To test which axis/button is which on your controller:
```bash
ros2 run joy_control test_joy_axes.py
```

## Safety

The **L1 enable button** acts as a dead-man switch:
- Release L1 → All movement stops immediately
- Commands only published while L1 is held
- Prevents accidental movements
