# manipulator_utils Python package
"""
manipulator_utils - Utility modules for rail-mounted robotic manipulator.

Modules:
    address_resolver: Resolves warehouse addresses to world-frame coordinates via TF.
"""

from manipulator_utils.address_resolver import AddressResolver

__all__ = ['AddressResolver']
