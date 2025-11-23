# Scene Configuration Files

This directory contains YAML scene configuration files for the PyBullet robotics simulation.

## Scene File Format

Each scene file is a YAML document with the following structure:

```yaml
metadata:
  name: "Scene Name"
  description: "Brief description of the scene"

objects:
  - name: "object_name"
    type: "cube"
    position: [x, y, z]  # Position in world coordinates
    color: [r, g, b, a]  # RGBA values (0.0 to 1.0)
    scale: 1.0           # Scale factor

task:
  description: "Natural language description of the task to perform"
```

## Fields

### Metadata
- `name`: Human-readable name for the scene
- `description`: Brief description of what the scene contains

### Objects
Each object in the `objects` list must have:
- `name`: Unique identifier for the object (used in task descriptions and LLM commands)
- `type`: Object type (currently only "cube" is supported)
- `position`: 3D coordinates [x, y, z] in meters
- `color`: RGBA color values [red, green, blue, alpha] where each value is 0.0-1.0
- `scale`: Scaling factor (1.0 = default size)

### Task
- `description`: Natural language description of the task for the LLM to execute (batch mode only)

## Usage

### Batch Mode
```bash
python main.py --scene default
python main.py --scene example_stacking
```

### Interactive Mode
```bash
streamlit run main_interactive.py -- --scene default
```

If no `--scene` argument is provided, the default scene will be loaded.

## Available Scenes

- `default.yaml`: Original five-cube configuration with stacking task
- `example_stacking.yaml`: Simple three-cube linear arrangement

## Creating Custom Scenes

1. Create a new YAML file in this directory (e.g., `my_scene.yaml`)
2. Follow the format shown above
3. Run with: `python main.py --scene my_scene`

### Tips
- Object names should be descriptive and match what you reference in task descriptions
- Position coordinates are in meters relative to the world origin
- The robot's workspace is approximately x: [-0.5, 0.7], y: [-0.7, 0.7], z: [0, 0.8]
- Use z=0.05 for objects on the ground plane
- RGBA colors: [1, 0, 0, 1] = red, [0, 1, 0, 1] = green, [0, 0, 1, 1] = blue
