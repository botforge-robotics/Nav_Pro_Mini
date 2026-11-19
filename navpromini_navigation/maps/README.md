# Nav2 Maps Directory

Stores pre-generated 2D occupancy grid maps (`.yaml` and `.pgm`) for Nav2 autonomous navigation and localization.

Available simulation maps:
- `playground.yaml` / `playground.pgm`: 22m x 22m mobile manipulation arena.
- `cafe.yaml` / `cafe.pgm`: Cafe environment map.

Maps can be generated directly from Gazebo SDF worlds using:
```bash
ros2 run navpromini_mapping sdf_map_generator --world playground
```
