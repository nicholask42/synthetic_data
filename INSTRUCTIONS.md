# Synthetic Data Generation in Isaac Sim Using Replicator

This guide walks you through generating synthetic data using **NVIDIA Isaac Sim** and the **Replicator** extensions, using an existing **USD scene file**. You will:

1. Load your USD scene
2. Enable Replicator extensions
3. Add cameras and annotators
4. Randomize lighting
5. Randomize position of a selected object
6. Randomize colors or materials of that object
7. Render and save annotations + images

---

## 1. Launch Isaac Sim and Enable Replicator

1. Start Isaac Sim.
2. Go to **Window → Extensions**.
3. Search for and enable:
   - `omni.replicator.core`
   - `omni.replicator.isaac` (optional but useful for Isaac sensors)
   - `omni.syntheticdata`

These will give you access to the Replicator API.

---

## 2. Load Your Existing USD Scene

1. Go to **File → Open**.
2. Select your existing `.usd` / `.usda` / `.usdc` file.
3. Verify that the object you want to randomize has a **unique prim path**, for example:

```
/World/TargetObject
```

This will be used in the script.

---

## 3. Use the Existing Scripts

Ready-to-use scripts are provided in `sim/scripts/`. Open the desired script in **Window → Script Editor** and run it directly. See the [README](README.md) for a description of each script and its output directory.

If you want to write a custom script from scratch, continue with the steps below.

---

## 4. Add a Camera for Data Capture

In the Script Editor later, you will include something like:

```python
import omni.replicator.core as rep

camera = rep.create.camera(
    position=(0, 0, 1.5),
    look_at=(0, 0, 0)
)
```

You may also choose to use cameras that already exist in your USD scene.

---

## 5. Set Up Render Products and Annotators

Replicator supports:
- RGB
- Depth
- Bounding boxes
- Semantic segmentation
- Instance segmentation

Example:

```python
render_product = rep.create.render_product(camera, resolution=(1280, 720))
annotators = ["rgb", "depth", "semantic_segmentation", "instance_segmentation"]
```

---

## 6. Randomize Lighting Conditions

You can vary:
- Light intensity
- Temperature
- Position
- Type (distant, dome, sphere, etc.)

Example randomizer:

```python
with rep.new_layer():
    light = rep.create.light(
        light_type="Sphere",
        intensity=rep.distribution.uniform(500, 3000),
        temperature=rep.distribution.uniform(3000, 9000),
        position=rep.distribution.uniform((-2, -2, 2), (2, 2, 4))
    )
```

---

## 7. Randomize Object Position

The medicine bottle's prim path in this scene is `/World/med_bottle`. Adjust if your scene differs:

```python
target_obj = rep.get.prims(path_pattern="/World/med_bottle")

with rep.new_layer():
    rep.modify.pose(
        target_obj,
        position=rep.distribution.uniform((-0.3, -0.3, 0), (0.3, 0.3, 0.2)),
        rotation=rep.distribution.uniform((0, 0, 0), (0, 360, 0))
    )
```

---

## 8. Randomize Object Color / Material

There are two main approaches:

### Option A — Randomize Base Color

```python
with rep.new_layer():
    rep.modify.attribute(
        target_obj,
        attribute="primvars:displayColor",
        value=rep.distribution.uniform((0,0,0), (1,1,1))
    )
```

### Option B — Randomize Material Assignment

```python
materials = rep.create.material_omnipbr(
    diffuse=rep.distribution.uniform((0,0,0), (1,1,1)),
    roughness=rep.distribution.uniform(0.1, 0.9),
)

with rep.new_layer():
    rep.modify.material(target_obj, materials)
```

---

## 9. Define the Writer (Where Files Go)

Example writing RGB + annotations to a folder:

```python
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="results/med_color",
    rgb=True,
    depth=True,
    semantic_segmentation=True,
)
writer.attach([render_product])
```

---

## 10. Full Example Script

Paste this into the Isaac Sim Script Editor and run:

```python
import omni.replicator.core as rep

# Load your scene is done manually via UI

# 1. Camera
target_camera = rep.create.camera(position=(0, -1.0, 0.8), look_at=(0, 0, 0))
render_product = rep.create.render_product(target_camera, (1280, 720))

# 2. Target object
target_obj = rep.get.prims(path_pattern="/World/med_bottle")

# 3. Randomizers
with rep.new_layer():
    # Lighting
    rep.create.light(
        light_type="Sphere",
        intensity=rep.distribution.uniform(500, 3000),
        temperature=rep.distribution.uniform(3000, 9000),
        position=rep.distribution.uniform((-2, -2, 2), (2, 2, 4))
    )

with rep.new_layer():
    # Object position
    rep.modify.pose(
        target_obj,
        position=rep.distribution.uniform((-0.3, -0.3, 0), (0.3, 0.3, 0.2)),
        rotation=rep.distribution.uniform((0, 0, 0), (0, 360, 0))
    )

with rep.new_layer():
    # Object color
    rep.modify.attribute(
        target_obj,
        attribute="primvars:displayColor",
        value=rep.distribution.uniform((0,0,0), (1,1,1))
    )

# 4. Writer
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="results/med_color",
    rgb=True,
    depth=True,
    semantic_segmentation=True,
)
writer.attach([render_product])

# 5. Trigger generation
rep.orchestrator.run(num_frames=200)
```

---

## 11. Run and Inspect Output

1. Press **Run Script** in the Script Editor.
2. Watch the viewport update as variations are generated.
3. Inspect your export directory for:
   - RGB images
   - Depth maps
   - Segmentation masks
   - Metadata JSON files

---

## 12. Next Steps

- Add multiple cameras
- Add domain randomization (textures, clutter objects, HDRI domes)
- Randomize materials more realistically
- Generate COCO-format annotations
- Export to KITTI, BDD, etc.

If you'd like, I can generate a **customized version of this script** that fits your exact USD file and object names.
