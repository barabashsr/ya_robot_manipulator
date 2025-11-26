#!/usr/bin/env python3
"""
Address Resolver Utility for Warehouse Navigation System.

Resolves warehouse addresses (side, cabinet, row, column) to world-frame
(x, y, z) coordinates via TF2 lookup. Provides validation against cabinet
configurations loaded from storage_params.yaml.

Story 3.1 - Epic 3: Address Navigation System
"""

from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from tf2_ros import ConnectivityException
from rclpy.duration import Duration
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
import yaml
import os


class AddressResolver:
    """Resolves warehouse addresses to world-frame coordinates via TF lookup."""

    # Side abbreviation mapping (AC2)
    SIDE_ABBREV = {
        'left': 'l',
        'right': 'r'
    }

    # Reference frame for coordinate output (AC9)
    # Primary: 'world' per AC9, fallbacks for simulation compatibility
    DEFAULT_REFERENCE_FRAME = 'world'
    FALLBACK_REFERENCE_FRAMES = ['base_link', 'storage_system_base']

    def __init__(self, node, tf_buffer=None, reference_frame=None):
        """
        Initialize AddressResolver with TF buffer and cabinet configurations.

        Args:
            node: ROS2 node for logging and TF listener
            tf_buffer: Optional mock buffer for testing (AC10)
            reference_frame: Optional override for reference frame (default: 'world' per AC9)
        """
        self._node = node
        self._logger = node.get_logger()
        self._reference_frame = reference_frame or self.DEFAULT_REFERENCE_FRAME

        # TF setup - use provided buffer or create real one (AC10)
        if tf_buffer is not None:
            self._tf_buffer = tf_buffer
            self._tf_listener = None  # No listener needed for mock
        else:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, node)

        # Load cabinet configurations from storage_params.yaml (AC8)
        self._cabinet_configs = self._load_cabinet_configs()

    def _load_cabinet_configs(self) -> dict:
        """
        Load cabinet configurations from storage_params.yaml.

        Returns:
            Dict mapping (side, cabinet_num) to {'columns': int, 'rows': int, 'departments': int}
        """
        pkg_path = get_package_share_directory('manipulator_description')
        config_path = os.path.join(pkg_path, 'config', 'storage_params.yaml')

        with open(config_path, 'r') as f:
            params = yaml.safe_load(f)

        configs = {}

        # Parse left_row and right_row sections
        for side in ['left', 'right']:
            if side in params.get('cabinet_rows', {}):
                cabinets = params['cabinet_rows'][side].get('cabinets', [])
                for idx, cabinet in enumerate(cabinets):
                    cabinet_num = idx + 1  # 1-indexed
                    size_str = cabinet.get('size', '')
                    # Parse "CxRxD" format (columns x rows x departments)
                    parts = size_str.split('x')
                    if len(parts) == 3:
                        cols, rows, depts = int(parts[0]), int(parts[1]), int(parts[2])
                        configs[(side, cabinet_num)] = {
                            'columns': cols,
                            'rows': rows,
                            'departments': depts
                        }

        self._logger.info(f'Loaded {len(configs)} cabinet configurations')
        return configs

    def _construct_frame_name(self, side: str, cabinet: int, row: int, column: int) -> str:
        """
        Construct TF frame name from address components.

        Args:
            side: 'left' or 'right'
            cabinet: Cabinet number (1-4)
            row: Row number (1-based)
            column: Column number (1-based)

        Returns:
            Frame name in format 'addr_{l|r}_{cabinet}_{row}_{column}'
        """
        abbrev = self.SIDE_ABBREV.get(side.lower(), side[0])
        return f'addr_{abbrev}_{cabinet}_{row}_{column}'

    def validate_address(self, side: str, cabinet: int, row: int, column: int) -> tuple:
        """
        Validate address against cabinet configurations.

        Args:
            side: 'left' or 'right'
            cabinet: Cabinet number (1-4)
            row: Row number (1-based)
            column: Column number (1-based)

        Returns:
            Tuple (valid: bool, error_msg: str)
            - (True, '') if valid
            - (False, error_message) if invalid
        """
        # Validate side (AC2)
        side_lower = side.lower()
        if side_lower not in ('left', 'right'):
            return (False, f"Side must be 'left' or 'right', got: {side}")

        # Validate cabinet range (AC3)
        if cabinet < 1 or cabinet > 4:
            return (False, f"Cabinet {cabinet} does not exist on {side_lower} side")

        # Look up cabinet config
        config = self._cabinet_configs.get((side_lower, cabinet))
        if config is None:
            return (False, f"Cabinet {cabinet} does not exist on {side_lower} side")

        # Validate row (AC4)
        max_rows = config['rows']
        if row < 1 or row > max_rows:
            return (False, f"Row {row} exceeds cabinet {cabinet} max of {max_rows} rows")

        # Validate column (AC5)
        max_cols = config['columns']
        if column < 1 or column > max_cols:
            return (False, f"Column {column} exceeds cabinet {cabinet} max of {max_cols} columns")

        return (True, '')

    def get_address_coordinates(self, side: str, cabinet: int, row: int, column: int) -> tuple:
        """
        Resolve address to (x, y, z) coordinates via TF lookup.

        Args:
            side: 'left' or 'right'
            cabinet: Cabinet number (1-4)
            row: Row number (1-based)
            column: Column number (1-based)

        Returns:
            Tuple (x, y, z, success, error_msg)
            - (x, y, z, True, '') if successful
            - (0.0, 0.0, 0.0, False, error_message) if failed
        """
        # First validate the address (AC1)
        valid, error_msg = self.validate_address(side, cabinet, row, column)
        if not valid:
            return (0.0, 0.0, 0.0, False, error_msg)

        # Construct frame name (AC1, AC2)
        frame_name = self._construct_frame_name(side, cabinet, row, column)

        # TF lookup with 1.0 second timeout (AC6, AC9)
        # Try primary reference frame first, fallback if not available
        frames_to_try = [self._reference_frame]
        if self._reference_frame == self.DEFAULT_REFERENCE_FRAME:
            frames_to_try.extend(self.FALLBACK_REFERENCE_FRAMES)

        last_error = None
        for ref_frame in frames_to_try:
            try:
                transform = self._tf_buffer.lookup_transform(
                    ref_frame,  # Target frame (AC9)
                    frame_name,  # Source frame
                    Time(),  # Latest available
                    timeout=Duration(seconds=1.0)  # AC6
                )

                # Extract coordinates from transform
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z

                # Log if using fallback frame
                if ref_frame != self.DEFAULT_REFERENCE_FRAME:
                    self._logger.debug(
                        f"Using fallback frame '{ref_frame}' instead of '{self.DEFAULT_REFERENCE_FRAME}'"
                    )

                return (x, y, z, True, '')

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                last_error = e
                continue

            except Exception as e:
                last_error = e
                continue

        # All frames failed
        error_msg = f"TF lookup failed for frame {frame_name}: {str(last_error)}"
        self._logger.warning(error_msg)
        return (0.0, 0.0, 0.0, False, error_msg)

    def get_cabinet_config(self, side: str, cabinet: int) -> dict:
        """
        Get cabinet configuration (columns, rows, departments).

        Args:
            side: 'left' or 'right'
            cabinet: Cabinet number (1-4)

        Returns:
            Dict with keys 'columns', 'rows', 'departments' or None if invalid
        """
        side_lower = side.lower()
        return self._cabinet_configs.get((side_lower, cabinet))
