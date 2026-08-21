"""NavProMini SDK — HTTP + WebSocket API server for the robot.

The package installs as a regular (not namespace) package: setup.py lists it
explicitly, so it works either way, but an implicit namespace package silently
changes how imports resolve when anything else shares the name. Being explicit
here costs one file and removes that class of surprise.
"""
