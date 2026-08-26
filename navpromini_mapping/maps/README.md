Save SLAM maps into this folder.

## Pose graph files (`.posegraph` / `.data`)

Saving a map now also serializes slam_toolbox's pose graph beside the image,
via `ros2 run navpromini_mapping serialize_posegraph <map_base>` (wired into
both map-saver launch files, best-effort).

Keep these alongside the `.pgm`/`.yaml`. A `.pgm` is a picture of the map and
cannot be extended — without the graph, the only way to absorb a changed
environment is to re-map from scratch, which produces a **new origin** and
silently invalidates everything stored in map frame: the dock pose
(`~/.navpromini_dock_pose.json`) and every Flutter bookmark. With the graph you
can instead continue the existing map in the same frame
(`slam_toolbox` `DeserializePoseGraph`), and those poses stay valid.

The graph cannot be recovered after the fact, which is why it is saved now
rather than when it is first needed.

## Start mapping with the robot DOCKED

`slam_toolbox` puts the map origin at the robot's pose when mapping starts, so
docking the robot first makes the origin the docked pose — and then the dock's
location in map frame is the same numbers on every map, forever.

`dock_manager_node` relies on this: with no dock pose saved it assumes the dock
face is `dock_origin_offset_m` (0.13m, the robot's radius) behind the origin
with yaw 0. That is why a re-map no longer invalidates docking. An explicitly
placed dock bookmark still overrides it, so nothing is lost if a map was made
without docking first — but then the dock must be re-placed after every re-map,
which is the situation this default exists to avoid.

Set `assume_dock_at_map_origin:=false` if a map is deliberately made elsewhere.
