"""Provides the units registry for the rest of the robot code"""
import pint  # type: ignore

UNIT_REGISTRY = pint.UnitRegistry()

# Define units
ft = UNIT_REGISTRY.foot
m = UNIT_REGISTRY.meter
s = UNIT_REGISTRY.second
rad = UNIT_REGISTRY.radian
deg = UNIT_REGISTRY.degree
rpm = UNIT_REGISTRY.rpm
