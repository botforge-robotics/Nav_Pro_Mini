#!/usr/bin/env python3
"""Door adapter note for NavProMini RMF sim.

There is **no Python door adapter** in this package (same as Open-RMF demos).

Who owns doors
--------------

* **Hardware / sim adapter**: Gazebo ``libdoor`` (``rmf_building_sim_gz_plugins``)
  publishes ``/door_states`` and handles open/close physics for ``door_L1``.
* **Supervisor**: ``door_supervisor`` from ``rmf_fleet_adapter`` (started with
  RMF core in ``rmf_web.launch.py``).

Do **not** run a parallel DoorState stub — dual publishers leave doors stuck
``MOVING`` and robots wait forever.

This console script only prints the layout (for discoverability). Launch file
``launch/include/adapters/door_adapter.launch.py`` documents the same.
"""

from __future__ import annotations


def main() -> None:
    print(__doc__)
    print(
        'Runtime: Gazebo libdoor + rmf_fleet_adapter door_supervisor '
        '(no navpromini door node).'
    )


if __name__ == '__main__':
    main()
