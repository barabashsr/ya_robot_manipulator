# Virtual limit switch state for picker jaw operations
string switch_name       # Name of limit switch (e.g., "picker_jaw_limit", "extend_limit")
bool triggered           # True if limit switch is currently triggered (contact made)
float64 trigger_position # Position where limit switch triggers (radians or meters)
float64 current_position # Current joint position (radians or meters)
